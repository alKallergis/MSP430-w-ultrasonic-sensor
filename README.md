# MSP430-w-ultrasonic-sensor
msp430g2553 interfaced with an ultrasonic distance sensor, a 16x2 LCD to display the distance,and a servo which moves according to this distance.

This is a test project incorporating an msp430g2553,a 16x2 LCD, a servo and an ultrasonic sensor. Distance measurements
are taken from the ultrasonic sensor by triggering it in the Trigger pin and then waiting for a high-edge triggered
interrupt from the Echo pin. Timer 1 is used for time measurements in this part.
Also timer 0 is used to create the appropriate PWM signal for the servo on P1.2. TA0CCR1 is changed in a loop
with values from the distance sensor,thus the servo moves as the distance changes.
The distance is also shown on  the LCD.
