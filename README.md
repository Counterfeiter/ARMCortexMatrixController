China RGB-LED-Matirx Display
==============

Use cheap chines LED RGB modules to control the PWM of each individual LED by a CortexM4. Normally a FPGA is required for this work load. But you could also use the CortexM external parallel memory interface to get fast refresh rates.
The Tiva-C boards from TI acts as WS2801 protocol receiver, so each WS2801 controller could be used. To Tiva boards could be chained. This gives controll over 2048 RGB-LEDs.

License
--------------
MIT

Software
--------------
Code Composer Studio 6.1.2 (TI) -> Demo-Board License

Hardware
--------------
Two Tiva-C Dev-Boards with ethernet

ToDo
--------------
Ethernet receiver -> but to less resources free?!

Use
--------------
- please read my blogposts: 
https://sebastianfoerster86.wordpress.com/2014/06/02/2048-pixel-rgb-led-matrix-basic/
https://sebastianfoerster86.wordpress.com/2014/06/04/2048-pixel-rgb-led-matrix-ws2801-emulator/
https://sebastianfoerster86.wordpress.com/2014/06/10/2048-pixel-rgb-led-matrix-tivas-udma/
https://sebastianfoerster86.wordpress.com/2014/06/11/2048-pixel-rgb-led-matrix-led-boards-und-tiva-epi/

Video
--------------
[![Video of the running system](https://img.youtube.com/vi/uE1VEO-WsN8/0.jpg)](https://www.youtube.com/watch?v=uE1VEO-WsN8)