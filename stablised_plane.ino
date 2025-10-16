// Include the PPM library
#include "ppm.h"
#include "Servo.h"
#include "ITG3200.h"
#include "ArduPID.h"

#define dbg(var) Serial.print(#var); Serial.print(":"); Serial.print(var); Serial.print(" ")

// Rotation around the forward axis is called roll, and is changed by the aelirons
ArduPID aerController;
Servo   aerServo;
double  targetRoll;
double  currentRoll;
double  aerOutput;
double  aerP = 1;
double  aerI = 0;
double  aerD = 0;

// Rotation around the east-west axis is called pitch and is changed by the elevator
ArduPID eleController;
Servo   eleServo;
double  targetPitch;
double  currentPitch;
double  eleOutput;
double  eleP = 1;
double  eleI = 0;
double  eleD = 0;

// Rotation around the up-down axis is called yaw and is changed by the rudder
ArduPID rudController;
Servo   rudServo;
double  targetYaw;
double  currentYaw;
double  rudOutput;
double  rudP = 1;
double  rudI = 0;
double  rudD = 0;

// Throttle is simply throttle :).
double throttle;

// PPM channel layout (update for your situation)
#define THROTTLE 3
#define ROLL 1
#define PITCH 2
#define YAW 4
#define SWITCH3WAY_1 5
#define BUTTON 6
#define SWITCH3WAY_2 7  // trim-pot for left/right motor mix  (face trim)
#define POT 8           // trim-pot on the (front left edge trim)

// Loop interval time
const long ppmInterval = 100;
const long gyroInterval = 100;
unsigned long previousGyroMillis = 0;
unsigned long previousPpmMillis = 0;

ITG3200 gyro;

void setup() {
  // Start the serial port to display data
  Serial.begin(9600);

  aerController.begin(&currentRoll, &aerOutput, &targetRoll, aerP, aerI, aerD);
  aerServo.attach(10);

  eleController.begin(&currentPitch, &eleOutput, &targetPitch, eleP, eleI, eleD);
  eleServo.attach(9);

  aerController.begin(&currentYaw, &aerOutput, &targetYaw, aerP, aerI, aerD);
  aerServo.attach(8);

  // Start the PPM receiver on PIN A0
  ppm.begin(A0, false);
  
  // Gyro is on generic i2c so no explictit pin needed.
  gyro.init();
  gyro.zeroCalibrate(200,10);//sample 200 times to calibrate and it will take 200*10ms

  // Set initial zero of angle of plane
  currentYaw = 0;
  currentRoll = 0;
  currentPitch = 0;
    // Set initial zero of angle of plane
  targetYaw = 0;
  targetRoll = 0;
  targetPitch = 0;
}

void loop() {
  float gx, gy, gz; // Angular momentum in deg/sec in each axis

  // Interval at which the PPM is updated
  unsigned long currentMillis = millis();

  // If we have passed the cycle for reading the PPM, then read in a new channel
  // of input
  if ((currentMillis - previousPpmMillis) >= ppmInterval) {
    previousPpmMillis = currentMillis;

    // Acquiring all the channels values
    //aerSetpoint = map(ppm.read_channel(ROLL), 1000, 2000, 0, 180);
    //eleSetpoint = map(ppm.read_channel(PITCH), 1000, 2000, 0, 180);
    targetRoll = ppm.read_channel(ROLL);
    targetPitch = ppm.read_channel(PITCH);
    targetYaw = ppm.read_channel(YAW);
    throttle = ppm.read_channel(THROTTLE);

    dbg(throttle);
    dbg(targetRoll);
    dbg(targetPitch);
    dbg(targetYaw);
  }

  // If we have passed the cycle for reading the velocity, then read in and reintegrate
  // our target angle
  if ((currentMillis - previousGyroMillis) >= gyroInterval) {
    double ratio = (((double)currentMillis/1000 - (double)previousGyroMillis/1000));


    gyro.getAngularVelocity(&gx,&gy, &gz);
    currentYaw += gx * ratio;
    currentRoll += gy * ratio;
    currentPitch += gz * ratio;

    dbg(currentMillis);
    dbg(previousGyroMillis);
    dbg(ratio);
    dbg(currentYaw);
    dbg(currentRoll);
    dbg(currentPitch);
    dbg(gx);
    dbg(gy);
    dbg(gz);

    previousGyroMillis = currentMillis;
  }



  Serial.println();
  //aerInput = map(gx, -1000, 1000, -90, 90);
  //eleInput = map(gz, -1000, 1000, -90, 90);
  


  aerController.compute();
  eleController.compute();
  //aerController.debug(&Serial, "aerController", PRINT_INPUT |     // Can include or comment out any of these terms to print
    //                                                      PRINT_OUTPUT |  // in the Serial plotter
      //                                                    PRINT_SETPOINT | PRINT_BIAS | PRINT_P | PRINT_I | PRINT_D);

  aerServo.write(aerOutput);
  eleServo.write(eleOutput);
}




// Returns what angle (from 0 to 180) the aeriolon should be set at based on the detected angle of the plane
// based on the target angle. 90 is neutral, 0 is full title left, 180 is full tilt right
//int aerOutput(int currentRoll, int targetRoll) {
  //return 90 - (currentRoll - targetRoll);
//}
//int eleOutput(int currentPitch, int targetPitch) {
 // return 90 - (currentPitch - targetPitch);
//}