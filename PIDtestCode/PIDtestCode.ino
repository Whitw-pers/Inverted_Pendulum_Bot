/*
Whit Whittall
Verified 12/11/2023
The function pidNoWindup(ref, state, kp, ki, kd) is a tunable PID controller that determines the controller output for a direct-acting system with a constant or changing reference 
when given the state of the system. The function includes a basic anti-windup scheme which prevents integrator windup by preventing the integrator term from exceeding certain
bounds. These bounds are hardcoded into the function. Practical improvements to this controller algorithm for future use would be to include certain settings which are not
hardcoded but can be set by the user before calling the function. These settings to be set beforehand would be whether the system is direct acting or reverse acting and the output
limit of the controller (determined by the saturation point of the actuators or any other limit the engineer my consider). Other improvements would be to move the controller
variable initialization into the function itself or inside a library if I wanted to really clean this up. Otherwise, see 
http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ for other improvements to clean up this PID algorithm.
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

//float kp=24, ki=8, kd=15;           // change tuning parameters
float kp = 0.2, ki = 0.08, kd = 0.15;  // much slower turning parameters for verification
float lastErr = 0, iTerm;
unsigned long lastTime = 0;

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
	myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
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


}


void loop() {
  // put your main code here, to run repeatedly:

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
    float uOut = pidNoWindup(ref, angle, kp, ki, kd);

    int Output = (int) uOut;  // typecast for use in drive fxn later
    Output = constrain(Output, -255, 255);

    Serial.println(Output);
    // Output should be between -255 and 255

  }

  delay(10);
}

// pid uses an error = ref - state passed through a function f(err, kp, ki, kd) that computes an output u
// all my datatypes are float, so I will need to check I'm declaring the right types at the top
// may need to typecast the output of this fxn into an int
float pidSat(float ref, float state, float kp, float ki, float kd) {
  
  // how long since last calculated
  unsigned long now = millis();
  float dTime = (float)(now - lastTime);

  if (dTime > 10) {

    // compute working vars
    float err = ref - state;
    iTerm += ki * (err * dTime);
    float dErr = (err - lastErr) / dTime;

    // compute PID output
    float uOut = (kp * err) + iTerm + (kd * dErr);

    // remember some vars
    lastErr = err;
    lastTime = now;
    
    return uOut;

  }
}

float pidNoWindup(float ref, float state, float kp, float ki, float kd) {
  // PID algorithm with constraints to prevent integrator windup
  // the Kalman filter runs in seconds but the PID controller runs in ms. It seems to operate smoother that way
  
  // how long since last calculated
  unsigned long now = millis();
  float dTime = (float)(now - lastTime);
  // NEED TO PUT THIS IN THE RUNME PROGRAM
  // OR USE dt to calculate PID output... knowings its always a bit inaccurate
  //dTime = dTime/1000;   // dTime is initially in units of ms, put into units of seconds

  if (dTime > 10) { // if converting dTime to seconds use 0.010

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

// calculates angle from accelerometer data and gyroscope data
// accelerometer data is noisy but does not drift
// gyro data is precise but drifts
// by combining both we get a precise reading that does not drift
float kalmanX(float ay, float az, float gx) {
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
