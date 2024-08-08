/*
Whit Whittall
Verified to function on 12/12/2023
The drive(spd) function drives the two 37mm DC motors on my self-balancing robot project. The code does not reverse the left motor, so the left and
right motor directions must be reversed by swapping the pins on the motor. There will be some trial and error to verify that the motors are operating
in the proper direction. I will perform that trial and error on the fully integrated Self_Balancing_Robot_Runme.ino script.

----------------------instructions for testing this code----------------------------
My 37mm Pololu DC gearmotors with encoders are meant to run on 12V power, which they get through the motor driver motor supply voltage. The motor driver regulates motor speed by 
PWMing the motor supply voltage, thereby creating the effect of modulating motor supply voltage. At voltages lower than 12V the motor speed is reduced, though I believe the 
motors are rated to operate all the way down to about 3V. A lipo battery is required to test motor function, because motor supply voltage is not connected to any voltage supply
pin on the arduino. Therefore, the motors will not be given any power unless a lipo is plugged in. I beleive it is safe to operate an arduino with a lipo plugged in at the same
time it is connected to a computer through USB. https://forum.arduino.cc/t/concerns-over-using-usb-and-an-independent-power-supply-at-same-time/168091 states it is safe to use
Vin/barrel jack and USB at the same time as long as the Vin/barrel jack is higher than 7.5V. This says to me it will be safest to test this code with a 3-cell lipo as the motor
voltage source.

spd = 26 is the lowest speed I could see the motor turning at. This pololu motor has some real solid low duty cycle performance, and that was at a 5V motor voltage.
I think the motor may operate at an even lower voltage
*/

//---------------------------------------------MOT STUFF--------------------------------------------

// define pins to control right motor
const int R1 = 6;   //AI1
const int R2 = 7;   //AI2
const int pwmR = 11;  //PWMA

// define pins to control left motor
const int L1 = 5;   //BI1
const int L2 = 4;   //BI2
const int pwmL = 10;  //PWMB

// specific to test program
int spd = 25; // adjust to control motor speed

void setup() {
  // put your setup code here, to run once:
  //---------------------------------------------MOT STUFF--------------------------------------------

  // set motor control pins as outputs
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(pwmL, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  drive(spd);
}

void drive(int motSpd) {
// expects motSpd to be between -255 and 255

  if (motSpd > 0) {   // drive wheels forward
    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
    digitalWrite(L1, HIGH);
    digitalWrite(L2, LOW);
    //Serial.println("1-1");
  }
  if (motSpd < 0) {   // drive wheels backward
    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
  }
  if (motSpd == 0) {  // stop wheels
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
    //Serial.println("0-1");
  }
  analogWrite(pwmR, abs(motSpd));
  analogWrite(pwmL, abs(motSpd));
  //Serial.println(mapSpd);
}