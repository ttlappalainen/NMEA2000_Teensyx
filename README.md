# NMEA2000_Teensyx

This library provides a Teensy 3.1/3.2/3.5/3.6/4.0/4.1 CAN driver for the NMEA2000 library.

Library is under tests. I prefer to start to use it also for Teensy 3.1/3.2/3.5/3.6 for
non critical devices in testing purposes so that possible errors will be catched. Final
goal is to use this as default library for all Teensy boards.

See https://github.com/ttlappalainen/NMEA2000.


## Usage


    #include <NMEA2000.h>
    #include <N2kMessages.h>
    #include <NMEA2000_Teensyx.h>

    tNMEA2000_Teensyx NMEA2000;

    void setup() {
      NMEA2000.Open();
    }

    void loop() {
	  NMEA2000.ParseMessages();
    }

See the [NMEA2000/Examples](https://github.com/ttlappalainen/NMEA2000/tree/master/Examples) for more examples. They are all compatible with this library.

## Changes
    26.10.2022
    
    - Added possibility to select CAN alternative pins

    23.08.2020
	
	- Initial commit

## License

    The MIT License

    Copyright (c) 2020 Timo Lappalainen

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
