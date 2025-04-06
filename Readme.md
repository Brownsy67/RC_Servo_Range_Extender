
DIY Servo Range Extender. 

Takes an input PWM servo signal and maps it to the range of 500us-2500us with a centre of 1500us. Be mindful that your servo can operate in this range, if not, adjust the travel or endpoints of your Radio/Transmitter lower and then work up towards the max degree of travel in both directions. 

This is for use on Arduino NANO, UNO, MINI, MICRO, or Atmega8A. Any boards that use the Atmega328P, or the Atmega32U4. 
I salvaged a couple Atemga8A's and have used those. 

1CH, 3CH, and 6CH versions.

!!!!!!!!!!!!!!!!!!!!!!!---------READ THIS----------!!!!!!!!!!!!!!!!!!!!!!!!

 I am kinda lazy, so I have not drawn up schematics or pictures with with exact wiring instructions(Currently working on this). Also,  I am considering that you know how you flash a sketch with the Arduino IDE. 

You can power the Arduino with 5v to the 5v pin on the Arduino, or you can wire the + power wire to the VIN pin, but it is usually better to provide no less than 7-12 volts on the VIN (although this is the safer bet if you are going to be using it it a few differnet models with different voltages). Check the power ratings for the pins on the board you are using.

You would wire every wire from the receiver to the Arduino or board, +/ and the input pin, then with the other output connector that you hook the servo up to, you would wire the +/- to the same power you used for input, but wire the output signal wire to the output pin for whatever channel. 


Compile and flash with the Arduino IDE, or PlatformIO in VSCode. 


INPUT/OUTPUT PINS are at the top of each sketch/code along with the pin assignments for the Button and LED. The Sketch's for the Atmgea328P use the LED_BUILTIN attached to pin13, and for the Atmega32U4 TXLED IO30 is used, so a LED is not needed. You only need the LED (connected with a resistor) for the calibration process. It is not needed for normal operation.

If you are going to use a salvaged Atmega8A, I highly recommend making sure that you research the steps required to flash and use the MCU. I salvaged mine from some old crappy brushless ESC's for drones/airplanes. 
*** You will get an error message when compiling these versions or the Atmega8A, ignore it, the firmware will still compile and flash. 


*** It will look like this, but again, it is fine. 

In file included from C:\Users\shmoo\AppData\Local\Arduino15\packages\MiniCore\hardware\avr\3.0.3\cores\MCUdude_corefiles/Arduino.h:30:0,
                 from C:\Users\shmoo\AppData\Local\Temp\.arduinoIDE-unsaved202534-13576-1hd92gl.g8f2\sketch_apr4a\sketch_apr4a.ino:1:
C:\Users\shmoo\AppData\Local\Temp\.arduinoIDE-unsaved202534-13576-1hd92gl.g8f2\sketch_apr4a\sketch_apr4a.ino: In function 'void PCINT_vect()':
C:\Users\shmoo\AppData\Local\Temp\.arduinoIDE-unsaved202534-13576-1hd92gl.g8f2\sketch_apr4a\sketch_apr4a.ino:88:5: warning: 'PCINT_vect' appears to be a misspelled 'signal' handler, missing '__vector' prefix [-Wmisspelled-isr]
Sketch uses 3308 bytes (42%) of program storage space. Maximum is 7808 bytes.
Global variables use 109 bytes (10%) of dynamic memory, leaving 915 bytes for local variables. Maximum is 1024 bytes.

***


---------------Calibration Steps----------------

Do this with servo(s) disconnected. 

The calibration order is CH1 - (3 or 6), MIN, then MAX, then finally MID, for each indiviual channel.

Hold the connected button for 2 seconds and wait for 2 rapid blinks every 1 second. The rapid blinking lets you know you are in calibration mode.

 Hold the value for the selected channel and range value you are trying to calibrate i.e. MIN on whatever channel you are trying to calibrate, and then hold the button again, the LED will blink 3 times to show that the value has saved and it is ready for the next value in the series. 

Do this for for the MIN, MAX, and MID values of each Channel.When you have calibrated all the values for a channel/input, the LED will blink quickly 5 times to signal that the calibration is complete for that channel/input. If you arent using a channel, just press the button to move through MIN, MAX, and MID for each unused channel (if you have flashed a version with more than 1 input/output). The default values of 988, 2012, and 1500 will be used for any channel without valid calibration data already saved for it. 

The LED will blink 3 long blinks and then stay lit to signal that calibration is complete. Power cycle or reset the board. You can now use it as intended. Confirm that it works as expected. You may need to perform calibration twice just to make sure that the values are recorded properly. 

Calibration data will persist through power cycles and resets. You only need to calibrate it once. If you change the input method or device, you will want to calibrate it again. 

*Calibration data is saved to EEPROM, there is a maximum amount of write/erase cycles, 1000 I think, so try not to be all willy nilly with the calibrations.*

Calibration data persists on the EEPROM through firmware updates, you won't need to calibrate again after doing it once unless you change input devices. 

-------------------------------------------------



