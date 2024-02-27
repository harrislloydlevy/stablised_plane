// Include the PPM library
#include "ppm.h"
#include "Servo.h"
#include "GY_85.h"
#include "ArduPID.h"

#define dbg(var) Serial.print(#var); Serial.print(":"); Serial.print(var); Serial.print(" ")

ArduPID airleonController;

double Aerlionsetpoint;
double Aerlioninput;
double Aerlionoutput;
double p = 1;
double i = 0;
double d = 0;
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
const long interval = 50;
unsigned long previousMillis = 0;

Servo aeroleonServo;
GY_85 GY85;

void setup() {
  // Start the serial port to display data
  Serial.begin(57600);
  airleonController.begin(&Aerlioninput, &Aerlionoutput, &Aerlionsetpoint, p, i, d);
  // Start the PPM function on PIN A0
  ppm.begin(4, false);
  aeroleonServo.attach(10);
  GY85.init();
}

void loop() {
  // Interval at which the PPM is updated
  unsigned long currentMillis = millis();

  short roll;
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Acquiring all the channels values

    Aerlionsetpoint = map(ppm.read_channel(ROLL), 1000, 2000, 0, 180);
  }

  int* compassReadings = GY85.readFromCompass();
  int cx = GY85.compass_x(compassReadings);
  int cy = GY85.compass_y(compassReadings);
  int cz = GY85.compass_z(compassReadings);

  dbg(cx);
  dbg(cy);
  dbg(cz);


  float* gyroReadings = GY85.readGyro();
  float gx = GY85.gyro_x(gyroReadings);
  float gy = GY85.gyro_y(gyroReadings);
  float gz = GY85.gyro_z(gyroReadings);
  float gt = GY85.temp(gyroReadings);

  dbg(gx);
  dbg(gy);
  dbg(gz);


  Serial.println();
  Aerlioninput = map(cx, -1000, 1000, -90, 90);
  


  /* Serial.print(" CurrentRoll:");
  Serial.print(cx);
  Serial.print(" TargetRoll:");
  Serial.print(roll);
  Serial.print(" AerilonAngle:");
  Serial.print(aerilonAngle);
  Serial.println();
  */

  /*
  Serial.print(" cy:");
  Serial.print(cy);
  Serial.print(" cz:");
  Serial.print(cz);

  Serial.print("\t  gyro");
  Serial.print(" gx:");
  Serial.print(gx);*/
  airleonController.compute();
  //airleonController.debug(&Serial, "airleonController", PRINT_INPUT |     // Can include or comment out any of these terms to print
    //                                                      PRINT_OUTPUT |  // in the Serial plotter
      //                                                    PRINT_SETPOINT | PRINT_BIAS | PRINT_P | PRINT_I | PRINT_D);
  

  // Make delay between readings
  delay(100);


  aeroleonServo.write(Aerlionoutput);

}
// Returns what angle (from 0 to 180) the aeriolon should be set at based on the detected angle of the plane
// based on the target angle. 90 is neutral, 0 is full title left, 180 is full tilt right
int getTargetAerilonAngle(int currentRoll, int targetRoll) {
  return 90 - (currentRoll - targetRoll);
}