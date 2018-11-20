// ================================================================================
// Output pin assignments to control the A4988 stepper motor drivers.  The step
// inputs are triggered on a rising edge, with a minimum 1 microsecond HIGH and
// LOW pulse widths.
#define X_AXIS_STEP_PIN 2
#define X_AXIS_DIR_PIN 5

#define Y_AXIS_STEP_PIN 3
#define Y_AXIS_DIR_PIN 6

#define STEPPER_ENABLE_PIN 8  // active-low (i.e. LOW turns on the drivers)

#define MAX_SPEED 8000
#define ACCELERATION 2750
#define BAUD_RATE 115200

#define SKID_TIMES 60
#define DELAY 300

// POSITION EXTREMES
#define LEFT_EXTREME
#define RIGHT_EXTREME
#define CLOSE_EXTREME
#define FAR_EXTREME

// POSITION FACTORS
#define X_DISTANCE_FACTOR
#define Y_DISTANCE_FACTOR /// ??? wtf experiment and find out

// SPEED TOLERANCE
#define SPEED_THRESHOLD

// LINEAR_TRANSITION_TOLERANCE
#define X_TRANSITION_THRESHOLD
#define Y_TRANSITION_THRESHOLD

// ================================================================================
// Import the third-party AccelStepper library.
#include <AccelStepper.h>

// Declare three AccelStepper objects to manage the output timing.
AccelStepper xaxis(AccelStepper::DRIVER, X_AXIS_STEP_PIN, X_AXIS_DIR_PIN);
AccelStepper yaxis(AccelStepper::DRIVER, Y_AXIS_STEP_PIN, Y_AXIS_DIR_PIN);

// ================================================================================
/// Configure the hardware once after booting up.  This runs once after powering
/// up the board, pressing reset, or connecting to the console serial port.
void setup(void)
{
  // set up the CNC Shield I/O
  digitalWrite(STEPPER_ENABLE_PIN, HIGH); // initialize drivers in disabled state
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);

  xaxis.setMaxSpeed(MAX_SPEED);
  xaxis.setAcceleration(ACCELERATION / 4.0);

  yaxis.setMaxSpeed(MAX_SPEED);
  yaxis.setAcceleration(ACCELERATION / 4.0);

  // set up the serial port for debugging output
  Serial.begin(BAUD_RATE);

  // enable the drivers, the motors will remain constantly energized
  digitalWrite(STEPPER_ENABLE_PIN, LOW);

  //pixy.init();

  move(0, 0, true);
  delay(1000);
}
/****************************************************************/
/// Call the run() function for each stepper driver object, which will
/// recalculate speeds and generate step pulses as needed.
void poll_steppers(void){
  xaxis.run();
  yaxis.run();
}
/// Return true if any one of the drivers are still moving.
bool is_moving(void)
{
  return xaxis.isRunning() || yaxis.isRunning();
}

/// Move a relative displacement at the current speed, blocking until the move is done.
void move(long x, long y, bool toMode)
{
  if (toMode) {
    xaxis.moveTo(x);
    yaxis.moveTo(y);
  } else {
      xaxis.move(x); 
      yaxis.move(y);
  }
  
  do {
    poll_steppers();
  } while(is_moving());
}

void track(int degree, char dir) {
  if (dir == 'B') {
    move(degree, -degree, false);
  } else if (dir == 'F') {
    move(-degree, degree, false);
  }
}

void turnClockwise(int degree) {
  move(degree, degree, false);  
}

void turnAntiClockwise(int degree) {
  move(-1 * degree, -1 * degree, false);
}

void skid(int degree) {
  int dir = 0;
  for (int i = 0; i < SKID_TIMES; i += 1) {
      if (dir == 0) {
        turnClockwise(degree + i / 5.0);
        dir = 1;
        delay(10);
      } else {
          turnAntiClockwise(degree + i / 5.0);
          dir = 0; 
          delay(10);
      }  
  }
}

char getCommand() {
  Serial.println("Enter command: ");
  while(!Serial.available()) {
    delay(10);
  } 
  return Serial.read();
}

char getDirection() {
  Serial.println("Enter direction: ");
  while(!Serial.available()) {
    delay(10);  
  }
  return Serial.read();
}

int getDegree() {
  Serial.println("Enter degree: ");
  while(!Serial.available()) {
    delay(10);
  }
  return Serial.parseInt();
}



/****************************************************************/
/// Run one iteration of the main loop.  The Arduino system will call this
/// function over and over forever.  This implementation is a script which will
/// generate a series of movements, waiting for each to complete.
void loop(){
    
//    unsigned long lastTime = millis();
//  
//    pixy.ccc.getBlocks();
//  
//    // If there are detect blocks, print them!
//    if (pixy.ccc.numBlocks) {
//      Serial.print("Detected ");
//      Serial.println(pixy.ccc.numBlocks);
//      for (int i=0; i<pixy.ccc.numBlocks; i++)
//      {
//        Serial.print("  block ");
//        Serial.print(i);
//        Serial.print(": ");
//        Serial.println(pixy.ccc.blocks[i].m_age);
//        Serial.print("Frame time: ");
//        Serial.println(millis() - lastTime);
//      }
//    }  
//  
  switch (getCommand()) {
    case 'L':
      turnAntiClockwise(getDegree());
      break; 
    case 'R':
      turnClockwise(getDegree());
      break;
    case 'T':
      track(getDegree(), getDirection());
      break;
    case 'S':
      skid(getDegree());
      break;
    default:
      Serial.println("Command not recognized");   
  }
    //track(360);
}
/****************************************************************/
