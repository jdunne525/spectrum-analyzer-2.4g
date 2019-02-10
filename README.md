Wifi Spectrum Analyzer 2.4GHz
===========

This project is an Arduino based 2.4GHz scanner built around a TI CC2500 module and an ESP32 with an OLED display.
The LCD displays the selected frequency bands and corresponding RSSI (signal strength) measured on each frequency.

This project is based on source code obtained from Simon Castle.  Credit goes to him and the sources he referenced in designing the original version of this scanner.

Starting from Simon Castle's code I did several things for my purposes.
(I was trying to identify the source of a 2.4GHz range wifi disturbance in my house every 5 seconds
which turned out to be some kind of periodic frequency scan coming from the router itself.)

* Switched to an esp32 with built in oled display (Hiletgo wifi kit)
* Updated from u8g lib to u8g2
* Added "spike" mode to zoom into a certain range (code configured range)
* Spike mode also shows the max rssi over a 5 second interval. (for locating the Signal source)
* Minor refactoring during development
* Added horizontal axis tick marks to aid in identifying frequencies 

![ScreenShot](https://github.com/jdunne525/spectrum-analyzer-2.4g/raw/master/Scanner%20pic.jpg?raw=true)
