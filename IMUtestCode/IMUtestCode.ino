// Whit Whittall
// Code tested and verified to work with ISM330DHCX IMU on 12/7/2023

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

void setup(){

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

}

void loop(){

	// Check if both gyroscope and accelerometer data is available.
	if( myISM.checkStatus() ){

		myISM.getAccel(&accelData);
		myISM.getGyro(&gyroData);

    ax = accelData.xData;
    ay = accelData.yData;
    az = accelData.zData;
    gx = gyroData.xData;
    gy = gyroData.yData;
    gz = gyroData.zData;

    angle = kalmanX(ay, az, gx);
    //angle = accelOnly(ay, az, gx);
    //angle = gyroOnly(ay, az, gx);

    Serial.println(angle);

	}

	delay(10);  // adjust based on sampling rate (104Hz)
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

// accelOnly() and gyroOnly() are used for demonstrating the problem with each individual sensor and why the Kalman filter is necessary

float accelOnly(float ay, float az, float gx) {
  float newAngle  = atan2(ay, az) * RAD_TO_DEG;
  return newAngle;
}

float gyroOnly(float ay, float az, float gx) {
  float newRate = gx / 500.0;  // Gyroscope data in degrees per second 
  angle += dt * newRate;

  return angle;
}