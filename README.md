# NMEA2000_Teensyx

This library provides a Teensy CAN driver for the NMEA2000 library.

See https://github.com/ttlappalainen/NMEA2000.


## Usage


    #include <NMEA2000.h>
    #include <N2kMessages.h>
    #include <NMEA2000_Teensyx.h>

    tNMEA2000_Teensyx NMEA2000;

    void setup() {
      NMEA2000.open();
    }

    void loop() {
	  NMEA2000.ParseMessages();
    }

See the [NMEA2000](https://github.com/ttlappalainen/NMEA2000) for more examples. They are all compatible with this library.


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
