/* This example uses the line sensors on the 3pi+ 32U4 to follow
a black line on a white background, using a PID-based algorithm.
It works well on courses with smooth, 6" radius curves and can
even work with tighter turns, including sharp 90 degree corners.
This example has been tested with robots using 30:1 MP motors.
Modifications might be required for it to work well on different
courses or with different motors. */

#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>


using namespace Pololu3piPlus32U4;

// Change next line to this if you are using the older 3pi+
// with a black and green LCD display:
// LCD display;
LCD display;

Buzzer buzzer;
LineSensors lineSensors;
Motors motors;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
bool useEmitters = true;
int16_t lastError = 0;

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

/* Configuration for specific 3pi+ editions: the Standard, Turtle, and
Hyper versions of 3pi+ have different motor configurations, requiring
the demo to be configured with different parameters for proper
operation.  The following functions set up these parameters using a
menu that runs at the beginning of the program.  To bypass the menu,
you can replace the call to selectEdition() in setup() with one of the
specific functions.
*/

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 would let the motors go at top speed, but
// the value of 200 here imposes a speed limit of 50%.

// Note that making the 3pi+ go faster on a line following course
// might involve more than just increasing this number; you will
// often have to adjust the PID constants too for it to work well.
uint16_t maxSpeed = 10;
int16_t minSpeed = 0;

// This is the speed the motors will run when centered on the line.
// Set to zero and set minSpeed to -maxSpeed to test the robot
// without.
uint16_t baseSpeed = maxSpeed;
uint16_t calibrationSpeed;

// PID configuration: This example is configured for a default
// proportional constant of 1/4 and a derivative constant of 1, which
// seems to work well at low speeds for all of our 3pi+ editions.  You
// will probably want to use trial and error to tune these constants
// for your particular 3pi+ and line course, especially if you
// increase the speed.

uint16_t proportional = 200; // coefficient of the P term * 256
uint16_t derivative = 500; // coefficient of the D term * 256
uint16_t integral;
unsigned long startTime;
unsigned long myTime;
unsigned long driveTime = 5000;
uint16_t biggestError = 0;
int16_t errorArray[100];
int16_t error;
int16_t position;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
bool dashFound = false;
bool select = false;
bool confirm = false;
PololuMenu<typeof(display)> menu;
uint16_t timeDelay = 10;

void selectEco()
{
  maxSpeed = 200;
  minSpeed = 0;
  baseSpeed = 200;
  calibrationSpeed = 50;
  proportional = 64; 
  derivative = 256; 
}

void selectCorsa()
{
  maxSpeed = 400;
  minSpeed = 0;
  baseSpeed = 400;
  calibrationSpeed = 60;
  proportional = 110; 
  derivative = 1600;
  integral = 0;
}

void selectStrada()
{
  maxSpeed = 310;
  minSpeed = 0;
  baseSpeed = 310;
  calibrationSpeed = 60;
  proportional = 90;
  derivative = 610; 
}

void changeSpeed(){
  delay(100);
  select = false;
  display.clear();
  display.print(F("Speed :"));
  display.gotoXY(0,1);
  display.print(maxSpeed);

  while(!select){
    if(buttonA.getSingleDebouncedPress()){
      maxSpeed = maxSpeed - 10;
      if(maxSpeed > 400){
        maxSpeed = 400;
      }
      display.clear();
      display.print("Speed :");
      display.gotoXY(0,1);
      display.print(maxSpeed);
    } 

    if(buttonC.getSingleDebouncedPress()){
      maxSpeed = maxSpeed + 10;
      if(maxSpeed > 400){
        maxSpeed = 400;
      }
      display.clear();
      display.print("Speed :");
      display.gotoXY(0,1);
      display.print(maxSpeed);
    }

    if(buttonB.getSingleDebouncedPress()){
      select = true;
    }
  }
}

void changeKp(){
  delay(100);
  select = false;
  display.clear();
  display.print(F("KP :"));
  display.gotoXY(0,1);
  display.print(proportional);

  while(!select){
    if(buttonA.getSingleDebouncedPress()){
      proportional = proportional - 5;
      display.clear();
      display.print("KP :");
      display.gotoXY(0,1);
      display.print(proportional);
    } 

    if(buttonC.getSingleDebouncedPress()){
      proportional = proportional + 5;
      display.clear();
      display.print("KP :");
      display.gotoXY(0,1);
      display.print(proportional);
    }

    if(buttonB.getSingleDebouncedPress()){
      select = true;
    }
  }
}

void changeKd(){
  delay(100);
  select = false;
  display.clear();
  display.print(F("KD :"));
  display.gotoXY(0,1);
  display.print(derivative);

  while(!select){
    if(buttonA.getSingleDebouncedPress()){
      derivative = derivative - 5;
      display.clear();
      display.print("KD :");
      display.gotoXY(0,1);
      display.print(derivative);
    } 

    if(buttonC.getSingleDebouncedPress()){
      derivative = derivative + 5;
      display.clear();
      display.print("KD :");
      display.gotoXY(0,1);
      display.print(derivative);
    }

    if(buttonB.getSingleDebouncedPress()){
      select = true;
    }
  }
}

void manualMode(){
  select = false;
  display.clear();
  display.print(F("Select"));
  
  static const PololuMenuItem items[] = {
    { F("Speed"), changeSpeed},
    { F("KP"), changeKp},
    { F("KD"), changeKd},
  };

  menu.setItems(items, 3);
  menu.setDisplay(display);
  menu.setButtons(buttonA, buttonB, buttonC);
  while(!menu.select());

  display.clear();
  display.print("Finish?");
  display.gotoXY(0,1);
  display.print("A:Y C:N");

  bool startAsking= true;
  while(startAsking){
    if(buttonA.getSingleDebouncedPress()){
    confirm = true;
    startAsking = false;
    } else if(buttonC.getSingleDebouncedPress()){
    confirm = false;
    startAsking = false;
    }
  }

  if(!confirm){
    manualMode();
  }
  calibrationSpeed = 60;
  minSpeed = 0;
  baseSpeed = maxSpeed;

}

void autoMode(){
  select = false;
  display.clear();
  display.print(F("Select"));

  static const PololuMenuItem items[] = {
    { F("ECO"), selectEco},
    { F("STRADA"), selectStrada},
    { F("CORSA"), selectCorsa},
  };

  menu.setItems(items, 3);
  menu.setDisplay(display);
  menu.setButtons(buttonA, buttonB, buttonC);
  while(!menu.select());

  display.gotoXY(0,1);
  display.print("OK!  ...");
}

void selectMode(){
    display.clear();
    display.print(F("A:Manual"));
    display.gotoXY(0,1);
    display.print(F("C:Auto"));

    while(!select){
      if(buttonA.isPressed() || buttonC.isPressed()){
        if(buttonA.isPressed()){
        manualMode();
        select = true;
        } else if(buttonC.isPressed()){
        autoMode();
        select = true;
        }
      }
    }
    select = false;
}

// Sets up special characters in the LCD so that we can display
// bar graphs.
void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  display.loadCustomCharacter(levels + 0, 0);  // 1 bar
  display.loadCustomCharacter(levels + 1, 1);  // 2 bars
  display.loadCustomCharacter(levels + 2, 2);  // 3 bars
  display.loadCustomCharacter(levels + 3, 3);  // 4 bars
  display.loadCustomCharacter(levels + 4, 4);  // 5 bars
  display.loadCustomCharacter(levels + 5, 5);  // 6 bars
  display.loadCustomCharacter(levels + 6, 6);  // 7 bars
}

void printBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, (char)255};
  display.print(barChars[height]);
}

void calibrateSensors()
{
  display.clear();

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for(uint16_t i = 0; i < 80; i++)
  {
    if (i > 20 && i <= 60)
    {
      motors.setSpeeds(-(int16_t)calibrationSpeed, calibrationSpeed);
    }
    else
    {
      motors.setSpeeds(calibrationSpeed, -(int16_t)calibrationSpeed);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

// Displays the estimated line position and a bar graph of sensor
// readings on the LCD. Returns after the user presses B.
void showReadings()
{
  display.clear();

  while(!buttonB.getSingleDebouncedPress())
  {
    uint16_t position = lineSensors.readLineBlack(lineSensorValues);
    display.gotoXY(0, 0);
    display.print(position);
    display.print("    ");
    display.gotoXY(0, 1);
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
      uint8_t barHeight = map(lineSensorValues[i], 0, 1000, 0, 8);
      printBar(barHeight);
    }

    delay(50);
  }
}

void setup()
{
  Serial.begin(9600);
  loadCustomCharacters();
  selectMode();

  // Wait for button B to be pressed and released.
  display.clear();
  display.print(F("Press B"));
  display.gotoXY(0, 1);
  display.print(F("to calib"));
  while(!buttonB.getSingleDebouncedPress());

  calibrateSensors();

  Serial.print("Time: ");
  Serial.print("\tPosition: ");
  Serial.print("\n");

  showReadings();

  display.clear();
  display.print(F("Go!"));
  startTime = millis();
}

void printReadingsToSerial()
{
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d %c\n",
    sensorValues[0],
    sensorValues[1],
    sensorValues[2],
    sensorValues[3],
    sensorValues[4],
    useEmitters ? 'E' : 'e'
  );
  Serial.print(buffer);
}

void runWithPD(){
  // Our "error" is how far we are away from the center of the
  // line, which corresponds to position 2000.
  error = position - 2000;

  // Get motor speed difference using proportional and derivative
  // PID terms (the integral term is generally not very useful
  // for line following).
  int16_t speedDifference = error * (int32_t)proportional / 256  + (error - lastError) * (int32_t)derivative / 256 ;
  lastError = error;

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int16_t leftSpeed = (int16_t)baseSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)baseSpeed - speedDifference;

  // Constrain our motor speeds to be between 0 and maxSpeed.
  // One motor will always be turning at maxSpeed, and the other
  // will be at maxSpeed-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you
  // might want to allow the motor speed to go negative so that
  // it can spin in reverse.
  leftSpeed = constrain(leftSpeed, minSpeed, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, (int16_t)maxSpeed);

  motors.setSpeeds(leftSpeed, rightSpeed);
}

void loop()
{
  position = lineSensors.readLineBlack(lineSensorValues);
  lineSensors.readCalibrated(sensorValues, useEmitters ? LineSensorsReadMode::On : LineSensorsReadMode::Off);

  // Search for dash line 
  if(sensorValues[0] < 300 && sensorValues[1] < 200 && sensorValues[2] <200 && sensorValues[3] < 200 && sensorValues[4] < 200 ){
    dashFound = true;
  }

  // If dash is found 
  if(dashFound){
    while (dashFound) {
    motors.setSpeeds(50, 50);
    delay(timeDelay); 
    lineSensors.readCalibrated(sensorValues, useEmitters ? LineSensorsReadMode::On : LineSensorsReadMode::Off);
      if (sensorValues[0] > 200 || sensorValues[1] > 200 || sensorValues[2] >200 || sensorValues[3] > 200 || sensorValues[4] > 200) {
      dashFound = false;
      }
    }
    position = lineSensors.readLineBlack(lineSensorValues);
    runWithPD();
  }else {
    runWithPD();
  }
}

