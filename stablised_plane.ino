#include "ppm.h"
#include "Servo.h"
#include "ITG3200.h"
#include "ArduPID.h"

/* What pins things are on. All output pins must be PWN supported! */
#define PPM_PIN A0
#define AER_PIN 9
#define ELE_PIN 10
#define RUD_PIN 11
#define THROT_PIN 3

#define dbg(var) \
  Serial.print(#var); \
  Serial.print(":"); \
  Serial.print(var); \
  Serial.print(" ")

// Rotation around the forward axis is called roll, and is changed by the aelirons
ArduPID aerController;
Servo aerServo;
double targetRoll;
double currentRoll;
double aerOutput;
double aerP = 2;
double aerI = 0.5;
double aerD = 0.5;

// Rotation around the east-west axis is called pitch and is changed by the elevator
ArduPID eleController;
Servo eleServo;
double targetPitch;
double currentPitch;
double eleOutput;
double eleP = 2;
double eleI = 0.5;
double eleD = 0.5;

#ifdef HAVE_RUDDER
// Rotation around the up-down axis is called yaw and is changed by the rudder
ArduPID rudController;
Servo rudServo;
double targetYaw;
double currentYaw;
double rudOutput;
double rudP = 2;
double rudI = 0.5;
double rudD = 0.5;
#endif

// Throttle needed is simply throttle :).
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
const long servoInterval = 200;
unsigned long previousGyroMillis = 0;
unsigned long previousPpmMillis = 0;
unsigned long previousServoMillis = 0;

ITG3200 gyro;

void setup() {
  // Start the serial port to display data
  Serial.begin(9600);

  aerController.begin(&currentRoll, &aerOutput, &targetRoll, aerP, aerI, aerD);
  aerServo.attach(10);

  eleController.begin(&currentPitch, &eleOutput, &targetPitch, eleP, eleI, eleD);
  eleServo.attach(9);

#ifdef HAVE_RUDDER
  rudController.begin(&currentYaw, &aerOutput, &targetYaw, aerP, aerI, aerD);
  rudServo.attach(8);
#endif

  // Start the PPM receiver on PIN A0
  ppm.begin(A0, false);

  // Gyro is on generic i2c so no explictit pin needed.
  gyro.init();
  gyro.zeroCalibrate(200, 10);  //sample 200 times to calibrate and it will take 200*10ms

// Set initial zero of angle of plane
#ifdef HAVE_RUDDER
  currentYaw = 0;
  targetYaw = 0;
#endif
  currentRoll = 0;
  currentPitch = 0;
  // Set initial zero of angle of plane

  targetRoll = 0;
  targetPitch = 0;
}

void loop() {
  float gx, gy, gz;  // Angular momentum in deg/sec in each axis

  // Interval at which the PPM is updated
  unsigned long currentMillis = millis();

  // If we have passed the cycle for reading the PPM, then read in a new channel
  // of input
  if ((currentMillis - previousPpmMillis) >= ppmInterval) {
    previousPpmMillis = currentMillis;

    // Acquiring all the channels values
    //aerSetpoint = map(ppm.read_channel(ROLL), 1000, 2000, 0, 180);
    //eleSetpoint = map(ppm.read_channel(PITCH), 1000, 2000, 0, 180);
    targetRoll = map(ppm.read_channel(ROLL), 1000, 2000, -90, 90);
    targetPitch = map(ppm.read_channel(PITCH), 1000, 2000, -90, 90);
#ifdef HAVE_RUDDER
    targetYaw = map(ppm.read_channel(YAW), 1000, 2000, -90, 90);
#endif
    throttle = ppm.read_channel(THROTTLE);

    dbg(throttle);
    dbg(targetRoll);
    dbg(targetPitch);
#ifdef HAVE_RUDDER
    dbg(targetYaw);
#endif
  }

  // If we have passed the cycle for reading the velocity, then read in and reintegrate
  // our target angle
  if ((currentMillis - previousGyroMillis) >= gyroInterval) {
    double ratio = (((double)currentMillis / 1000 - (double)previousGyroMillis / 1000));


    gyro.getAngularVelocity(&gx, &gy, &gz);
#ifdef HAVE_RUDDER
    currentYaw += gx * ratio;
#endif
    currentRoll += gy * ratio;
    currentPitch += gz * ratio;

    dbg(currentMillis);
    dbg(previousGyroMillis);
    dbg(ratio);
#ifdef HAVE_RUDDER
    dbg(currentYaw);
#endif
    dbg(currentRoll);
    dbg(currentPitch);
    dbg(gx);
    dbg(gy);
    dbg(gz);

    previousGyroMillis = currentMillis;
  }

  if ((currentMillis - previousServoMillis) >= servoInterval) {
    int throttleOut;

    // Map thtrottle from PPM in 0-1000 to 8 bit value
    throttleOut = map(throttle, 1000, 2000, 0, 254);
    // First set the throttle, nice and simple
    analogWrite(THROT_PIN, throttleOut);

    aerController.compute();
    eleController.compute();
    
    aerServo.write(aerOutput);
    eleServo.write(eleOutput);

    dbg(throttleOut);
    dbg(aerOutput);
    dbg(eleOutput);

#ifdef HAVE_RUDDER
    rudController.compute();
    rudServo.write(rudOutput);
#endif


    //aerController.debug(&Serial, "aerController", PRINT_INPUT |     // Can include or comment out any of these terms to print
    //                                                      PRINT_OUTPUT |  // in the Serial plotter
    //                                                    PRINT_SETPOINT | PRINT_BIAS | PRINT_P | PRINT_I | PRINT_D);

  }
  Serial.println(); // Needed to end all the debug macros
}