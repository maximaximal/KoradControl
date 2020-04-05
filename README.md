KoradControl
============

This simple project is about controlling a Korad KA005P power supply from
a PC. It is implemented in a single C++ file and is only built for the
following use-case:

  1. Enter parameters into the CLI
  2. Start KoradControl, which then sets up the power supply, checks correct settings and starts outputting power
  3. Watch live readings in the stdout in CSV format
  4. Press CTRL+C to exit KoradControl, which also turns off power
 
Acknowledgements
----------------

Thanks to Tamagotono and [her/his project](https://github.com/Tamagotono/Korad-KA6003P-Software),
I got the foundation for the communication protocol right.
