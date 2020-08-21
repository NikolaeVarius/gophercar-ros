# TFmini

https://www.sparkfun.com/products/14588

## Connecting
Red Wire -> 5V PWR
Black Wire -> GND
White Wire -> UART_2_TX (Pin 8)
Green Wire -> UART_2_RX (Pin 10)

1) Connect Red wire of TFMini Lidar with jetson nano pin 2 (5v)*.

2) Connect Black wire of TFMini Lidar with jetson nano pin 6 (GND)*.

3) Connect WHITE wire of TFMini Lidar with jetson nano pin 8 (GPIO 14)*.

4) Connect Green wire of TFMini Lidar with jetson nano pin 10 (GPIO 15)*.

         TFMini Pinout     |    Jetson nano Pinout
       ==============================================
            Red            |        5V(Pin 2)
            Black          |       GND(Pin 6)
            White          |      GPIO 14(Pin 8)
            Green          |      GPIO 15(Pin10)
       ==============================================   
       
Command line procedure:-
    sudo nano “your file name”
    ls
    sudo python tfmini1.py