/** KORADCONTROL
 * Author: Max Heisinger <mail@maximaximal.com>
 */

/* Includes */

#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <boost/asio.hpp>
#include <boost/program_options.hpp>

namespace as = boost::asio;
namespace po = boost::program_options;

#define VERSION "0.1.0"

/* Protocol messages */

#define REQUEST_STATUS "STATUS?"
#define REQUEST_ID "*IDN?"
#define REQUEST_SET_VOLTAGE "VSET1?"
#define REQUEST_ACTUAL_VOLTAGE "VOUT1?"
#define REQUEST_SET_CURRENT "ISET1?"
#define REQUEST_ACTUAL_CURRENT "IOUT1?"

#define SET_VOLTAGE "VSET1:"
#define SET_CURRENT "ISET1:"
#define SET_OUTPUT_ON "OUT1"
#define SET_OUTPUT_OFF "OUT0"
#define SET_OVP "OVP"// Over Voltage Protection
#define SET_OCP "OCP"// Over Current Protection

#define TIMEOUT_WRITE 1000
#define TIMEOUT_READSOME 100

static bool verbose = false;

/* Callback functions for results */
struct Status
{
  Status()
    : _padding1(false)
    , cv_cc(false)
    , ovp_ocp(false)
    , out(false)
    , _padding2(false)
  {}
  bool _padding1 : 4;
  bool cv_cc : 1;
  bool ovp_ocp : 1;
  bool out : 1;
  bool _padding2 : 1;
};

std::ostream&
operator<<(std::ostream& o, const Status& s)
{
  return o << "("
           << "CV/CC=" << s.cv_cc << ",OVP/OCP=" << s.ovp_ocp
           << ",OUT=" << s.out << ")";
}

using IntCB = std::function<void(int)>;
using StrCB = std::function<void(const std::string&)>;
using StatusCB = std::function<void(Status)>;
using EmptyCB = std::function<void()>;

class Request
{
  public:
  explicit Request(const std::string& query, size_t responseLen)
    : query(query)
    , expectedRespnseLen(responseLen)
  {}
  virtual ~Request() {}

  virtual void parseResponse(std::string resp)
  {
    std::cerr << "Response ignored." << std::endl;
  }

  void receiveBytes(const char* data, size_t bytes)
  {
    std::copy(data, data + bytes, std::back_inserter(m_response));
  }

  bool readFinished()
  {
    if(m_response.size() == expectedRespnseLen) {
      auto resp = responseAsString();
      if(verbose) {
        std::cerr << "    <-- " << resp << std::endl;
      }
      parseResponse(resp);
      return true;
    } else {
      m_response.clear();
      return false;
    }
  }

  std::string responseAsString()
  {
    return std::string(m_response.begin(), m_response.end());
  }

  const std::string query;
  const size_t expectedRespnseLen;

  private:
  std::vector<char> m_response;
};

class VIntRequest : public Request
{
  public:
  explicit VIntRequest(const std::string& query, size_t responseLen, IntCB cb)
    : Request(query, responseLen)
    , cb(cb)
  {}

  virtual void parseResponse(std::string resp) override
  {
    assert(resp.find('.') == 2);
    resp.erase(2, 1);
    int voltage = std::atoi(resp.c_str()) * 10;
    cb(voltage);
  }

  IntCB cb;
};

class IIntRequest : public Request
{
  public:
  explicit IIntRequest(const std::string& query, size_t responseLen, IntCB cb)
    : Request(query, responseLen)
    , cb(cb)
  {}

  virtual void parseResponse(std::string resp) override
  {
    resp.erase(resp.find('.'), 1);
    int current = std::atoi(resp.c_str());
    cb(current);
  }

  IntCB cb;
};

class StrRequest : public Request
{
  public:
  explicit StrRequest(const std::string& query, size_t responseLen, StrCB cb)
    : Request(query, responseLen)
    , cb(cb)
  {}

  virtual void parseResponse(std::string resp) override { cb(resp); }

  StrCB cb;
};

/** Wrapper class for power supply */
class KA3005P
{
  public:
  explicit KA3005P(as::io_service& ioService, const std::string& serialPortPath)
    : m_serialPortPath(serialPortPath)
    , m_serialPort(ioService)
    , m_readTimer(ioService)
    , m_writeTimer(ioService)
  {}

  void open()
  {
    if(m_serialPort.is_open())
      m_serialPort.close();
    std::cerr << "Initiating KA3005P on port " << m_serialPortPath << "..."
              << std::endl;
    try {
      m_serialPort.open(m_serialPortPath);
      m_serialPort.set_option(as::serial_port_base::baud_rate(9600));
      m_serialPort.set_option(as::serial_port_base::character_size(8));
      m_serialPort.set_option(
        as::serial_port_base::stop_bits(as::serial_port_base::stop_bits::one));
      m_serialPort.set_option(
        as::serial_port_base::parity(as::serial_port_base::parity::none));
      m_serialPort.set_option(as::serial_port_base::flow_control(
        as::serial_port_base::flow_control::hardware));

      // m_deadlineTimer.expires_from_now(boost::posix_time::milliseconds(300));
      // m_deadlineTimer.wait();

      readSome();
    } catch(const boost::system::system_error& e) {
      std::cerr << "Could not init serial port! Encountered error: " << e.what()
                << ". Terminating program." << std::endl;
      ::exit(EXIT_FAILURE);
    }
  }

  void init() { open(); }

  void startReadTimeout()
  {
    m_readTimer.expires_from_now(
      boost::posix_time::milliseconds(TIMEOUT_READSOME));
    m_readTimer.async_wait(
      std::bind(&KA3005P::readTimeout, this, std::placeholders::_1));
  }

  void startWriteTimeout()
  {
    m_writeTimer.expires_from_now(
      boost::posix_time::milliseconds(TIMEOUT_WRITE));
    m_writeTimer.async_wait(std::bind(
      &KA3005P::stop, this, std::placeholders::_1, TIMEOUT_WRITE, "write"));
  }

  void cancelReadTimeout() { m_readTimer.cancel(); }
  void cancelWriteTimeout() { m_writeTimer.cancel(); }

  void stop(const boost::system::error_code& ec,
            int timeout,
            const std::string& message)
  {
    if(ec == boost::asio::error::operation_aborted) {
      return;
    }
    std::cerr << "Timeout for " << message << " of " << timeout
              << " seconds expired! Terminating program." << std::endl;
    ::exit(EXIT_FAILURE);
  }

  void readSome()
  {
    m_serialPort.async_read_some(
      as::mutable_buffer(m_readBuffer.data(), m_readBuffer.size()),
      std::bind(&KA3005P::readHandler,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
    startReadTimeout();
  }

  void readTimeout(const boost::system::error_code& ec)
  {
    if(ec == boost::asio::error::operation_aborted) {
      return;
    }

    if(m_request) {
      auto req = m_request;
      m_request.reset();
      if(!req->readFinished()) {
        execRequestResponse(req);
      }
    }

    readSome();
  }

  void readHandler(const boost::system::error_code& ec, size_t bytes)
  {
    cancelReadTimeout();

    if(ec) {
      std::cerr << "Error during read! Error: " << ec.message() << std::endl;
    } else {

      if(m_request) {
        m_request->receiveBytes(m_readBuffer.data(), bytes);
      }
    }

    readSome();
  }

  void updateStatus(StatusCB cb)
  {
    queryStatus([this, cb](const std::string& status) {
      assert(status.size() == 1);
      const Status statusStruct = *reinterpret_cast<const Status*>(&status[0]);
      m_status = statusStruct;
      cb(statusStruct);
    });
  }

  void updateOutputCurrent(IntCB cb)
  {
    queryOutI([this, cb](int current) {
      m_outputCurrent = current;
      cb(current);
    });
  }

  void updateOutputVoltage(IntCB cb)
  {
    queryOutV([this, cb](int voltage) {
      m_outputVoltage = voltage;
      cb(voltage);
    });
  }

  void updateSetCurrent(IntCB cb)
  {
    querySetI([this, cb](int current) {
      m_setCurrent = current;
      cb(current);
    });
  }

  void updateSetVoltage(IntCB cb)
  {
    querySetV([this, cb](int voltage) {
      m_setVoltage = voltage;
      cb(voltage);
    });
  }

  void queryOutV(IntCB cb)
  {
    std::unique_ptr<VIntRequest> req =
      std::make_unique<VIntRequest>(REQUEST_ACTUAL_VOLTAGE, 5, cb);
    execRequestResponse(std::move(req));
  }

  void querySetV(IntCB cb)
  {
    std::unique_ptr<VIntRequest> req =
      std::make_unique<VIntRequest>(REQUEST_SET_VOLTAGE, 5, cb);
    execRequestResponse(std::move(req));
  }

  void queryOutI(IntCB cb)
  {
    std::unique_ptr<IIntRequest> req =
      std::make_unique<IIntRequest>(REQUEST_ACTUAL_CURRENT, 5, cb);
    execRequestResponse(std::move(req));
  }

  void querySetI(IntCB cb)
  {
    std::unique_ptr<IIntRequest> req =
      std::make_unique<IIntRequest>(REQUEST_SET_CURRENT, 6, cb);
    execRequestResponse(std::move(req));
  }

  void queryID(StrCB cb)
  {
    std::unique_ptr<StrRequest> req =
      std::make_unique<StrRequest>(REQUEST_ID, 16, cb);
    execRequestResponse(std::move(req));
  }

  void queryStatus(StrCB cb)
  {
    std::unique_ptr<StrRequest> req =
      std::make_unique<StrRequest>(REQUEST_STATUS, 1, cb);
    execRequestResponse(std::move(req));
  }

  void sendSetVoltage(StrCB cb)
  {
    assert(m_targetVoltage > 0);
    assert(m_targetVoltage <= 31000);
    std::string cmd = std::to_string(m_targetVoltage / 10);
    cmd = std::string(4 - cmd.length(), '0') + cmd;
    cmd.insert(2, ".");
    cmd = SET_VOLTAGE + cmd;

    std::unique_ptr<StrRequest> req = std::make_unique<StrRequest>(cmd, 0, cb);
    this->execRequestResponse(std::move(req));
  }

  void setVoltage(int mV, EmptyCB cb)
  {
    m_setVoltageCoro = as::coroutine();
    m_targetVoltage = mV;
    setVoltageInternal(cb);
  }

#include <boost/asio/yield.hpp>
  void setVoltageInternal(EmptyCB endCB)
  {
    auto s = std::bind(&KA3005P::setVoltageInternal, this, endCB);

    reenter(m_setVoltageCoro)
    {
      do {
        yield this->sendSetVoltage(s);
        yield this->updateSetVoltage(s);
      } while(m_setVoltage != m_targetVoltage && !m_exit);
      endCB();
    }
  }
#include <boost/asio/unyield.hpp>

  void sendSetCurrent(StrCB cb)
  {
    assert(m_targetCurrent > 0);
    assert(m_targetCurrent <= 5100);
    std::string cmd = std::to_string(m_targetCurrent);
    cmd = std::string(4 - cmd.length(), '0') + cmd;
    cmd.insert(1, ".");
    cmd = SET_CURRENT + cmd;

    std::unique_ptr<StrRequest> req = std::make_unique<StrRequest>(cmd, 0, cb);
    this->execRequestResponse(std::move(req));
  }

  void setCurrent(int mA, EmptyCB cb)
  {
    m_setCurrentCoro = as::coroutine();
    m_targetCurrent = mA;
    setCurrentInternal(cb);
  }

#include <boost/asio/yield.hpp>
  void setCurrentInternal(EmptyCB endCB)
  {
    auto s = std::bind(&KA3005P::setCurrentInternal, this, endCB);

    reenter(m_setCurrentCoro)
    {
      do {
        yield this->sendSetCurrent(s);
        yield this->updateSetCurrent(s);
      } while(m_setCurrent != m_targetCurrent && !m_exit);
      endCB();
    }
  }
#include <boost/asio/unyield.hpp>

  void execRequestResponse(std::shared_ptr<Request> req)
  {
    flushBuffer();

    startWriteTimeout();
    as::async_write(
      m_serialPort,
      as::buffer(req->query),
      [this, req](const boost::system::error_code& ec, size_t bytes) mutable {
        cancelWriteTimeout();
        if(verbose) {
          std::cerr << "--> " << req->query << std::endl;
        }

        if(ec) {
          std::cerr << "Error during write! Error: " << ec.message()
                    << ". Terminating program." << std::endl;
          ::exit(EXIT_FAILURE);
        }
        if(req->expectedRespnseLen == 0) {
          m_writeTimer.expires_from_now(boost::posix_time::milliseconds(200));
          m_writeTimer.async_wait([req](const boost::system::error_code& ec) {
            req->parseResponse("");
          });
        } else {
          m_request = req;
        }
      });
  }

  void flushBuffer()
  {
    ::tcflush(m_serialPort.lowest_layer().native_handle(), TCIOFLUSH);
  }

  const Status& getStatus() { return m_status; }
  int getOutputVoltage() { return m_outputVoltage; }
  int getOutputCurrent() { return m_outputCurrent; }

  void executeSetOutput(EmptyCB cb)
  {
    std::unique_ptr<StrRequest> req = std::make_unique<StrRequest>(
      m_targetOutputOn ? SET_OUTPUT_ON : SET_OUTPUT_OFF, 0, std::bind(cb));
    execRequestResponse(std::move(req));
  }
  void turnOutputOn(EmptyCB cb)
  {
    m_setOutputCoro = as::coroutine();
    m_targetOutputOn = true;
    setOutputInternal(cb);
  }
  void turnOutputOff(EmptyCB cb)
  {
    m_setOutputCoro = as::coroutine();
    m_targetOutputOn = false;
    setOutputInternal(cb);
  }

#include <boost/asio/yield.hpp>
  void setOutputInternal(EmptyCB endCB)
  {
    auto s = std::bind(&KA3005P::setOutputInternal, this, endCB);
    reenter(m_setOutputCoro)
    {
      do {
        yield this->executeSetOutput(s);
        yield this->updateStatus(s);
      } while(m_status.out != m_targetOutputOn && !m_exit);
      endCB();
    }
  }
#include <boost/asio/unyield.hpp>

  void exit() { m_exit = true; }

  private:
  std::string m_serialPortPath;

  as::serial_port m_serialPort;
  as::deadline_timer m_readTimer;
  as::deadline_timer m_writeTimer;
  std::array<char, 255> m_readBuffer;

  std::shared_ptr<Request> m_request;

  uint32_t m_outputVoltage = 0;
  uint32_t m_outputCurrent = 0;
  uint32_t m_setVoltage = 0;
  uint32_t m_setCurrent = 0;

  uint32_t m_targetVoltage = 0;
  uint32_t m_targetCurrent = 0;

  bool m_targetOutputOn = false;

  as::coroutine m_setVoltageCoro;
  as::coroutine m_setCurrentCoro;
  as::coroutine m_setOutputCoro;

  Status m_status = Status();
  bool m_exit = false;
};

using PSU = KA3005P;

class App
{
  public:
  App()
    : m_signals(m_ioService, SIGINT)
    , m_waitTimer(m_ioService)
  {}

  int run(int argc, char* argv[])
  {
    po::options_description desc("Allowed Options");

    // clang-format off
    desc.add_options()
        ("help", "produce help message")
        ("version", "produce version number")

        ("device", po::value<std::string>()->default_value("/dev/ttyACM0"), "serial port to power supply")
        ("volt", po::value<int>()->default_value(5000), "(targeted) output voltage [mV]")
        ("current", po::value<int>()->default_value(1500), "(targeted) output current [mA]")

        ("poll-interval", po::value<int>()->default_value(1000), "time to wait between polls of current and voltage [ms]")

        ("verbose,v", po::bool_switch(&verbose), "verbose output of messages")
    ;
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if(vm.count("help")) {
      std::cout << desc << std::endl;
      return EXIT_SUCCESS;
    }
    if(vm.count("version")) {
      std::cout << VERSION << std::endl;
      return EXIT_SUCCESS;
    }

    m_device = vm["device"].as<std::string>();

    m_targetVoltage = vm["volt"].as<int>();
    m_targetCurrent = vm["current"].as<int>();
    m_pollInterval = vm["poll-interval"].as<int>();

    m_signals.async_wait(std::bind(
      &App::signalHandler, this, std::placeholders::_1, std::placeholders::_2));

    start();

    return EXIT_SUCCESS;
  }

  void start()
  {
    m_psu = std::make_unique<PSU>(m_ioService, m_device);
    m_ioService.post(std::bind(&PSU::init, m_psu.get()));
    m_ioService.post(std::bind(&App::step, this));

    m_ioService.run();
  }

  template<typename CB>
  void waitForMilliseconds(int ms, CB cb)
  {
    m_waitTimer.expires_from_now(std::chrono::milliseconds(ms));
    m_waitTimer.async_wait(cb);
  }

#include <boost/asio/yield.hpp>

  void step()
  {
    auto s = std::bind(&App::step, this);
    reenter(m_appCoro)
    {
      yield m_psu->setVoltage(m_targetVoltage, s);
      yield m_psu->setCurrent(m_targetCurrent, s);
      yield m_psu->turnOutputOn(s);

      printCSVHeader();

      while(!m_exit) {
        yield this->waitForMilliseconds(m_pollInterval, s);

        yield m_psu->updateOutputCurrent(s);
        yield m_psu->updateOutputVoltage(s);

        printCSVLine();
      }

      yield m_psu->turnOutputOff(s);
      m_psu->exit();
    }
  }

  void stop()
  {
    auto s = std::bind(&App::stop, this);

    reenter(m_stopCoro)
    {
      if(m_psu) {
        m_psu->turnOutputOff(s);
        yield this->waitForMilliseconds(1000, s);
      }
      m_ioService.stop();
    }
  }

#include <boost/asio/unyield.hpp>

  void signalHandler(const boost::system::error_code& ec, int signalNumber)
  {
    if(ec) {
      std::cerr << "Error when catching signal! Error: " << ec.message()
                << std::endl;
      return;
    }
    if(signalNumber == SIGINT) {
      std::cerr << "SIGINT captured. Turning off PSU." << std::endl;
      if(m_psu) {
        m_exit = true;
        step();
      }
      stop();
    }
  }

  void printCSVHeader()
  {
    std::cout << "MS_SINCE_START;OUTPUT_VOLTAGE;OUTPUT_CURRENT" << std::endl;
  }
  void printCSVLine()
  {
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - m_startTime)
                   .count()
              << ";" << m_psu->getOutputVoltage() << ";"
              << m_psu->getOutputCurrent() << std::endl;
  }

  private:
  as::io_service m_ioService;
  as::signal_set m_signals;

  std::unique_ptr<PSU> m_psu;

  std::string m_device;

  int m_targetVoltage;// in mV
  int m_targetCurrent;// in mA
  int m_pollInterval = 1000;

  as::coroutine m_appCoro;
  as::coroutine m_stopCoro;

  bool m_exit = false;

  std::chrono::steady_clock::time_point m_startTime =
    std::chrono::steady_clock::now();
  as::steady_timer m_waitTimer;
};

int
main(int argc, char* argv[])
{
  App app;
  return app.run(argc, argv);
}
