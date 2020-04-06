KoradControl
============

This simple project is about controlling a Korad KA005P power supply from
a PC. It is implemented in a single C++ file and is only built for the
following use-case:

  1. Enter parameters into the CLI
  2. Start KoradControl, which then sets up the power supply, checks correct settings and starts outputting power
  3. Watch live readings in the stdout in CSV format
  4. Press CTRL+C to exit KoradControl, which also turns off power

Logging Measurements
--------------------

A handy way to use this tool is logging current measurements into timestamped CSV files. These
measurements can then be opened using LibreOffice or any other CSV consuming program (Gnuplot, Matplotlib, R, etc.).
To use KoradControl for this task, consider the following procedure:

``` shell
# Create output directory
mkdir log

# Run koradcontrol (press CTRL+C to exit & turn off power)
./koradcontrol --volt <target voltage in mV> --current <target current in mA> > log/$(date +%F_%T).csv
```

The file will then look like this (no load was connected):

``` csv
MS_SINCE_START;OUTPUT_VOLTAGE;OUTPUT_CURRENT
1521;9000;0
1865;9000;0
2264;9000;0
2686;9000;0
2953;9000;0
3347;9000;0
3685;9000;0
3958;9000;0
4417;9000;0
```

Acknowledgements
----------------

Thanks to Tamagotono and [her/his project](https://github.com/Tamagotono/Korad-KA6003P-Software),
I got the foundation for the communication protocol right.
