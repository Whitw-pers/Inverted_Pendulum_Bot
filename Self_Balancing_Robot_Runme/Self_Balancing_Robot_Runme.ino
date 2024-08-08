/*
Whit Whittall
Self Balancing Robot V1 Runme code. This runs a single PID controller to control a single degree of freedom of this 2 DoF system. 

12/12/2023 – demonstrated semi successfully. PID controller operates (still not tuned however) and drives motor. Motor directions are verified to be correct. There is one bug- the PID
controller output will occasionally get "stuck" on a single value and the motor will begin turning at a constant rate. The system stops responding to changes of angle of the IMU. I 
have yet to determine where in the system the source of this error is or the nature of this error, be it a logic error, a power problem, or otherwise. Weirdly, I haven't observed 
this error in any individual subsystem tests– this is a new error observed first upon integration.

Remaining tasks for Self Balancing Robot V1 are: 
- troubleshoot the integrated system to prevent the "sticking" error
- assemble robot
- tune PID controller

Improvements I'd like to impliment on V2 are:
- a mid-level PID controller that changes the reference for this PID controller to oppose the current direction of motion. This requires a few things
  - implimentation of at least one motor encoder (altho positional data can also be gathered via the IMU)
  - making the PID controller a class so multiple PID objects can be initialized OR
  - writing a second PID function with different variables which is less cool :(
- buy lighter wheels with (reduce moment of inertia) to improve the response speed of the robot base
- buy lower-geared motors which have finer positional control
*/

//--------------------------------------------IMU STUFF---------------------------------------------
#include <Wire.h>                 // required to communicate with the IMU over I2C
#include "SparkFun_ISM330DHCX.h"  // IMU library

SparkFun_ISM330DHCX myISM; 

// Structs for X,Y,Z data
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData; 

// Kalman filter variables
const float Q_angle = 0.001;    // tune to improve filter performance
const float Q_bias = 0.003;     // tune to improve filter performance
const float R_measure = 0.03;   // tune to improve filter performance

const float dt = 0.01;  // Time step in seconds, should match delay statement in main loop

float angle = 0;
float rate = 0;
float bias = 0;
float P[2][2] = {{0, 0}, {0, 0}};

float ax = 0;
float ay = 0;
float az = 0;
float gx = 0;
float gy = 0;
float gz = 0;

//--------------------------------------------PID STUFF---------------------------------------------

float kp = 8, ki = 0.03, kd = 3;          // this tuning is unstable but balances on its own for a few secs
/*

Possible sources of instability/things I should try to improve:
- estimate the execute speed of this code to determine how high I can increase the sample rate on my IMU
  - increasing controller frequency may lead to better controller performance
- get torquier motors- 500 rpm still sounds like plenty and may have finer motor control
- increase height (and therefore moment of inertia) of bot
- get lighter wheels for faster controller response time
- perhaps do some actual math analysis to determine the ballpark for the controller terms

*/
//float kp = 0.25, ki = 0.08, kd = 0.15;  // much slower tuning parameters for verification
float lastErr = 0, iTerm;
unsigned long lastTime = 0;

//---------------------------------------------MOT STUFF--------------------------------------------

// robot is oriented correctly when the USB in and barrel jack of the arduino are on the left side of the robot.
// left and right directions commented here assume that the USB in and barrel jack are on the left side of the robot.
// In this orientation- most critically- the "ISM330DHCX MMC5983MA 9DoF" silkscreen on the IMU should appear right-side-up to the operator.

// define pins to control left motor
const int R1 = 6;   //AI1
const int R2 = 7;   //AI2
const int pwmR = 11;  //PWMA

// define pins to control right motor
const int L1 = 5;   //BI1
const int L2 = 4;   //BI2
const int pwmL = 10;  //PWMB

void setup() {
  // put your setup code here, to run once:

  //--------------------------------------------IMU STUFF---------------------------------------------

	Wire.begin();

	Serial.begin(115200);

	if( !myISM.begin() ){
		Serial.println("Did not begin.");
		while(1);
	}

	// Reset the device to default settings. This if helpful is you're doing multiple
	// uploads testing different settings. 
	myISM.deviceReset();

	// Wait for it to finish reseting
	while( !myISM.getDeviceReset() ){ 
		delay(1);
	} 

	Serial.println("Reset.");
	Serial.println("Applying settings.");
	delay(100);
	
	myISM.setDeviceConfig();
	myISM.setBlockDataUpdate();
	
	// Set the output data rate and precision of the accelerometer
	myISM.setAccelDataRate(ISM_XL_ODR_104Hz); // this accelerometer and gyro data rate limits the frequency the controller can operate at
	myISM.setAccelFullScale(ISM_4g); 

	// Set the output data rate and precision of the gyroscope
	myISM.setGyroDataRate(ISM_GY_ODR_104Hz);
	myISM.setGyroFullScale(ISM_500dps); 

	// Turn on the accelerometer's filter and apply settings. 
	myISM.setAccelFilterLP2();
	myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

	// Turn on the gyroscope's filter and apply settings. 
	myISM.setGyroFilterLP1();
	myISM.setGyroLP1Bandwidth(ISM_MEDIUM);

  //--------------------------------------------PID STUFF---------------------------------------------



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
  //----------------------- verified------------------------------
	// Check if both gyroscope and accelerometer data is available.
	if(myISM.checkStatus()){

    myISM.getAccel(&accelData);
		myISM.getGyro(&gyroData);

    ax = accelData.xData;
    ay = accelData.yData;
    az = accelData.zData;
    gx = gyroData.xData;
    gy = gyroData.yData;
    gz = gyroData.zData;

    angle = kalmanX(ay, az, gx);

    // my own PID fxn call
    float ref = 0;
    //float uOut = pidSat(ref, angle, kp, ki, kd);
    float uOut = pidCalc(ref, angle, kp, ki, kd);

    int Output = (int) uOut;  // typecast for use in drive fxn later
    Output = constrain(Output, -255, 255);

    Serial.println(Output);
    // Output should be between -255 and 255 and respond quickly when returning from the saturation limit

    drive(Output);
  }

  delay(10);
}

//--------------------------------------------------CUSTOM FXNS------------------------------------------------------
// float pidCalc(ref, state, kp, ki, kd)
// void drive(motSpd)
// float kalmanX(ay, az, gx)

float pidCalc(float ref, float state, float kp, float ki, float kd) {
  // PID algorithm with constraints to prevent integrator windup
  
  // how long since last calculated
  unsigned long now = millis();
  float dTime = (float)(now - lastTime);

  if (dTime > 10) {

    // compute working vars
    float err = ref - state;
    iTerm += ki * (err * dTime);
    iTerm = constrain(iTerm, -255, 255); // setting limits on intTerm so it doesn't accumulate beyond the bounds of reason
    float dErr = (err - lastErr) / dTime;

    // compute PID output
    float uOut = (kp * err) + iTerm + (kd * dErr);

    // remember some vars
    lastErr = err;
    lastTime = now;
    
    return uOut;
  }
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
  //Serial.println(motSpd);
}


float kalmanX(float ay, float az, float gx) {
// calculates angle of IP robot from accelerometer data and gyroscope data
// accelerometer data is noisy but does not drift
// gyro data is precise but drifts
// by combining both we get a precise reading that does not drift

  float newAngle  = atan2(ay, az) * RAD_TO_DEG;
  float newRate = gx / 500.0;  // Gyroscope data in degrees per second 

  // Prediction step
  rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure; // estimate error
  float K[2];  // Kalman gain
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Update step
  float y = newAngle - angle; // determine error between accel estimate and gyro estimate

  angle += K[0] * y;
  bias += K[1] * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}
