# esp32_quadruped
## Summary

esp32 quadruped robot with arduino

Using 3D-printer an build it !

![After assemble](https://s2.loli.net/2022/02/06/FHeAd84jrylX2bf.jpg)

## Details

### About Anruino

1. Prepare the ESP32 development environment

2. install lib: Adafruit_SSD1306, MPU6050 (by Electronic Cats), Adafruit_PWM_Servo_Driver_Library
3. Modify YOUR SSID and PASSWD in the code
4. Connect USB , hold the pin0 and upload
5.  Reset it and open the monitor, get this device IP, (esp32 work with station mode)
6.  Open browser with IP, you may see the camera image.

![Contrl with web view](https://s2.loli.net/2022/02/06/RUhLdfgjGIVbCHv.jpg)

### About pcb

you may make a PCB circuit board proofing, and you may perpar :

1. servo : mg90s *10*
2. esp32-cam board
3. ov2640 (fpc 15cm at least)
4. battery: 18650* 2
5. parallel connection battery holder
6. 5V fan （2.5cm*2.5cm ）

PCB circuit board BOM:

1. power: FP6276*2
2. gyroscope : MPU6050
3. servo control IC：pca9685
4. USB to UART IC : ch340N
5. charge IC: TP4056
6. Thermal Camer : MLX90640 （Not necessary）
7. OLED : SSD1306 （Not necessary）
8. External antenna （Not necessary）

Screw：

1. M1.7*8   x50
2. M4*10    x4
3.  M3*10   x10
4. M3*35   x4
5. M3 Nut  x2
6. and so on ~

### About 3D object:

https://www.thingiverse.com/thing:5232362
