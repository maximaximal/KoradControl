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
#define SET_OUTPUT "OUT"
#define SET_OVP "OVP"// Over Voltage Protection
#define SET_OCP "OCP"// Over Current Protection

#define TIMEOUT 7

/* Callback functions for results */
using IntCB = std::function<void(int)>;
using StrCB = std::function<void(const std::string&)>;

class Request
{
  public:
  explicit Request(const std::string& query, size_t responseLen)
    : query(query)
    , responseLen(responseLen)
  {}

  virtual void parseResponse(const std::string& resp)
  {
    std::cerr << "Response ignored." << std::endl;
  }

  const std::string query;
  const size_t responseLen;
};

class IntRequest : public Request
{
  public:
  explicit IntRequest(const std::string& query, size_t responseLen, IntCB cb)
    : Request(query, responseLen)
    , cb(cb)
  {}

  virtual void parseResponse(const std::string& resp)
  {
    std::cerr << "Response: \"" << resp << "\"" << std::endl;
    if(resp == "") {
      cb(0);
    } else {
      std::cerr << resp << std::endl;
    }
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

  virtual void parseResponse(const std::string& resp)
  {
    std::cerr << "Response: \"" << resp << "\"" << std::endl;
    cb(resp);
  }

  StrCB cb;
};

/** Wrapper class for power supply */
class KA3005P
{
  public:
  explicit KA3005P(as::io_service& ioService, const std::string& serialPortPath)
    : m_ioService(ioService)
    , m_serialPortPath(serialPortPath)
    , m_serialPort(ioService)
    , m_deadlineTimer(ioService)
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
        as::serial_port_base::flow_control::none));

      // char buf[2] = "\n";
      // as::write(m_serialPort, as::const_buffer(buf, 1));

      m_deadlineTimer.expires_from_now(boost::posix_time::milliseconds(300));
      m_deadlineTimer.wait();
    } catch(const boost::system::system_error& e) {
      std::cerr << "Could not init serial port! Encountered error: " << e.what()
                << ". Terminating program." << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  void init()
  {
    open();

    queryID([this](const std::string&) {
      queryStatus([this](const std::string&) {
        queryOutV([this](int i) {

        });
      });
    });
  }

  void startTimeout()
  {
    m_deadlineTimer.expires_from_now(boost::posix_time::seconds(TIMEOUT));
    m_deadlineTimer.async_wait(
      std::bind(&KA3005P::stop, this, std::placeholders::_1));
  }

  void cancelTimeout() { m_deadlineTimer.cancel(); }

  void stop(const boost::system::error_code& ec)
  {
    if(ec == boost::asio::error::operation_aborted) {
      return;
    }
    std::cerr << "Timeout of " << TIMEOUT
              << " seconds expired! Terminating program." << std::endl;
    exit(EXIT_FAILURE);
  }

  void queryOutV(IntCB cb)
  {
    std::unique_ptr<IntRequest> req =
      std::make_unique<IntRequest>(REQUEST_ACTUAL_VOLTAGE, 2, cb);
    execRequestResponse(std::move(req));
  }

  void queryID(StrCB cb)
  {
    std::unique_ptr<StrRequest> req =
      std::make_unique<StrRequest>(REQUEST_ID, 15, cb);
    execRequestResponse(std::move(req));
  }

  void queryStatus(StrCB cb)
  {
    std::unique_ptr<StrRequest> req =
      std::make_unique<StrRequest>(REQUEST_STATUS, 1, cb);
    execRequestResponse(std::move(req));
  }

  void execRequestResponse(std::shared_ptr<Request> req)
  {
    m_serialPort.async_read_some(
      as::mutable_buffer(m_readBuffer.data(), req->responseLen),
      [this, req](const boost::system::error_code& ec, size_t bytes) mutable {
        cancelTimeout();
        std::cerr << "Received Answer to " << req->query << " Bytes: " << bytes
                  << std::endl;
        if(ec) {
          std::cerr << "Error during read! Error: " << ec.message()
                    << ". Terminating program." << std::endl;
          exit(EXIT_FAILURE);
        }
        m_deadlineTimer.expires_from_now(boost::posix_time::milliseconds(200));
        m_deadlineTimer.wait();
        req->parseResponse(
          std::string(m_readBuffer.begin(), m_readBuffer.begin() + bytes));
        // flushBuffer();
      });

    startTimeout();
    // flushBuffer();
    as::async_write(
      m_serialPort,
      as::buffer(req->query),
      [this, req](const boost::system::error_code& ec, size_t bytes) mutable {
        cancelTimeout();
        if(ec) {
          std::cerr << "Error during write! Error: " << ec.message()
                    << ". Terminating program." << std::endl;
          exit(EXIT_FAILURE);
        }
        std::cerr << "Wrote " << req->query << std::endl;
        if(req->responseLen == 0) {
          std::cerr << "Immediate End " << req->query << std::endl;
          req->parseResponse("");
        } else {
          // Wait for 0.015s
          std::cerr << "Waiting then wait for answer." << req->query
                    << std::endl;
          m_deadlineTimer.expires_from_now(boost::posix_time::milliseconds(15));
          m_deadlineTimer.wait();

          std::cerr << "Waiting for answer." << req->query << std::endl;
          startTimeout();
        }
      });
  }

  void flushBuffer()
  {
    ::tcflush(m_serialPort.lowest_layer().native_handle(), TCIOFLUSH);
  }

  private:
  as::io_service& m_ioService;
  std::string m_serialPortPath;

  as::serial_port m_serialPort;
  as::deadline_timer m_deadlineTimer;
  std::array<char, 255> m_readBuffer;
};

using PSU = KA3005P;

class App
{
  public:
  App()
    : m_signals(m_ioService, SIGINT)
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

    m_signals.async_wait(std::bind(
      &App::signalHandler, this, std::placeholders::_1, std::placeholders::_2));

    start();

    return EXIT_SUCCESS;
  }

  void start()
  {
    m_psu = std::make_unique<PSU>(m_ioService, m_device);
    m_ioService.post(std::bind(&PSU::init, m_psu.get()));

    m_ioService.run();
  }

  void stop() { m_ioService.stop(); }

  void signalHandler(const boost::system::error_code& ec, int signalNumber)
  {
    if(ec) {
      std::cerr << "Error when catching signal! Error: " << ec.message()
                << std::endl;
      return;
    }
    if(signalNumber == SIGINT) {
      std::cerr << "SIGINT captured. Turning off PSU." << std::endl;
      stop();
    }
  }

  private:
  as::io_service m_ioService;
  as::signal_set m_signals;

  std::unique_ptr<PSU> m_psu;

  std::string m_device;

  int m_targetVoltage;// in mV
  int m_targetCurrent;// in mA
};

int
main(int argc, char* argv[])
{
  App app;
  return app.run(argc, argv);
}
