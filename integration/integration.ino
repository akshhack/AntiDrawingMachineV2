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

// POSITION FACTORS - DETERMINE THESE
#define X_DISTANCE_FACTOR 1.0
#define Y_DISTANCE_FACTOR 3.75

// SPEED TOLERANCE
#define SPEED_THRESHOLD_UPPER 40
#define SPEED_THRESHOLD_LOWER 15

// LINEAR_TRANSITION_TOLERANCE
#define X_TOLERANCE 1.0
#define Y_TOLERANCE 1.0

// TIME BETWEEN FRAMES
#define FRAME_TIME 16.5 // these are in millisecond

// BAUD
#define BAUD_RATE 115200

// QUEUE SPECS
#define QUEUE_SIZE 5

#include <Pixy2.h>
#include <AccelStepper.h>

// Declare three AccelStepper objects to manage the output timing.
AccelStepper xaxis(AccelStepper::DRIVER, X_AXIS_STEP_PIN, X_AXIS_DIR_PIN);
AccelStepper yaxis(AccelStepper::DRIVER, Y_AXIS_STEP_PIN, Y_AXIS_DIR_PIN);

Pixy2 pixy;

typedef struct {
  uint16_t x;
  uint16_t y;  
} PenPosition;

typedef struct {
  double vx;
  double vy;  
} PenVelocity;

typedef struct {
  PenPosition* pos;
  PenVelocity* vel;
  
  PenPosition* prevPositions[QUEUE_SIZE];
} PenMotion;

typedef struct {
  bool isHorizontal; // in X
  bool isVertical; // in Y
  bool isFast; // velocity magnitude
  bool isSlow; // velocity magnitude
  char dir; // direction N, E, S, W
} PenState;

PenPosition posInit = {0, 0};
PenVelocity velInit = {0.0, 0.0};
PenMotion penMotion = {&posInit, &velInit, {&posInit, &posInit, &posInit}};
PenState penState = {false, false, false, false, 'N'};
bool isSetup = false;
int setupCounter = 0;

void setup() {
  digitalWrite(STEPPER_ENABLE_PIN, HIGH); // initialize drivers in disabled state
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);

  xaxis.setMaxSpeed(MAX_SPEED);
  xaxis.setAcceleration(ACCELERATION / 4.0);

  yaxis.setMaxSpeed(MAX_SPEED);
  yaxis.setAcceleration(ACCELERATION / 4.0);
  pixy.init();
  Serial.begin(BAUD_RATE);

  digitalWrite(STEPPER_ENABLE_PIN, LOW);

  move(0, 0, true); // initialize to origin
  
  delay(1000);
}

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
/// toMode as a boolean indicates if the move is to be made absolute or relative. false for
/// relative and true for absolute
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

/*
 *  track: tracks the paper linearly, foward or backward to some degree
 *  @param: degree: moves paper by degrees forward or backward
 *  @param: dir: indicates which direction is the paper to be tracked
 *               - forward (F) or backward (B)
 */
void track(int degree, char dir) {
  if (dir == 'B') {
    move(degree, -degree, false);
  } else if (dir == 'F') {
    move(-degree, degree, false);
  }
}

/*
 * turnClockwise: moves paper by degrees clockwise
 * @param: degree: moves paper by degrees clockwise
 */
void turnClockwise(int degree) {
  move(degree, degree, false);  
}

/*
 * turnAntiClockwise: moves paper by degrees anticlockwise
 * @param: degree: moves paper by degrees anticlockwise
 */
void turnAntiClockwise(int degree) {
  move(-1 * degree, -1 * degree, false);
}

/*
 * skid: skids the paper SKID number of times as defined by the constant
 *       at the start of the file, by degrees
 * @param: degree: degree by which the paper is skid
 */
void skid(int degree) {
  int dir = 0; // helps flip skid direction
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

/* PenState handlers */
void resetPenState() {
  penState = {false, false, false, false};
}

void decideHorizontal() {
  uint16_t firstPosition = penMotion.prevPositions[0]->x;
  bool isHorizontal = true;
  // check if all positions are in tolerance of the firstPosition
  for (int i = 0; i < QUEUE_SIZE; i += 1) {
    uint16_t currentPos = penMotion.prevPositions[i]->x;
    if (currentPos < firstPosition - X_TOLERANCE || 
        currentPos > firstPosition + X_TOLERANCE) {
      isHorizontal = false;
      break;  
    } 
  }
  penState.isHorizontal = isHorizontal;
}

void decideVertical() {
  uint16_t firstPosition = penMotion.prevPositions[0]->y;
  bool isVertical = true;
  // check if all positions are in tolerance of the firstPosition
  for (int i = 0; i < QUEUE_SIZE; i += 1) {
    uint16_t currentPos = penMotion.prevPositions[i]->y;
    if (currentPos < firstPosition - Y_TOLERANCE || 
        currentPos > firstPosition + Y_TOLERANCE) {
      isVertical = false;
      break;  
    } 
  }
  penState.isVertical = isVertical;
}

void decideFast() {
  double penSpeed = sqrt(penMotion.vel->vx * penMotion.vel->vx + penMotion.vel->vy * penMotion.vel->vy);
  
  penState.isFast = (penSpeed > SPEED_THRESHOLD_UPPER);
}

void decideSlow() {
  double penSpeed = sqrt(penMotion.vel->vx * penMotion.vel->vx + penMotion.vel->vy * penMotion.vel->vy);

  penState.isSlow = (penSpeed < SPEED_THRESHOLD_LOWER);  
}

void decideDirection() {
  double mag = sqrt(penMotion.vel->vx * penMotion.vel->vx + penMotion.vel->vy * penMotion.vel->vy);

  double vy_abs = abs(penMotion.vel->vy);
  double vx_abs = abs(penMotion.vel->vx);

  double angle = atan(vy_abs / vx_abs);

  if (penMotion.vel->vx > 0 && penMotion.vel->vy > 0) {
    // NE  
    if (angle < PI / 4) {
      penState.dir = 'E';
    } else {
        penState.dir = 'N';
    }
  } else if (penMotion.vel->vx > 0 && penMotion.vel->vy < 0) {
    // SE
    if (angle < PI / 4) {
      penState.dir = 'E';
    } else {
        penState.dir = 'S';
    }
  } else if (penMotion.vel->vx < 0 && penMotion.vel->vy > 0) {
    // NW
    if (angle < PI / 4) {
      penState.dir = 'W';
    } else {
        penState.dir = 'N';
    }
  } else {
    // SW
    if (angle < PI / 4) {
      penState.dir = 'W';
    } else {
        penState.dir = 'S';
    }
  }
}

void frameListSetup(uint16_t x, uint16_t y) {
  if (setupCounter < QUEUE_SIZE - 1) {
    penMotion.prevPositions[setupCounter]->x = x;
    penMotion.prevPositions[setupCounter]->y = y;
    setupCounter += 1; 
  } else {
      isSetup = true;  
  }
}

/*
 * addBlockToFrameList: adds block to frame list
 */
void addBlockToFrameList(uint16_t x, uint16_t y) {
    for (int i = 0; i < QUEUE_SIZE - 1; i += 1) {
      penMotion.prevPositions[i]->x = penMotion.prevPositions[i+1]->x;
      penMotion.prevPositions[i]->y = penMotion.prevPositions[i+1]->y;
    }

    penMotion.prevPositions[QUEUE_SIZE - 1]->x = x;
    penMotion.prevPositions[QUEUE_SIZE - 1]->y = y; 
}

void updatePenPosition() {
  uint16_t x = penMotion.prevPositions[QUEUE_SIZE - 1]->x;
  uint16_t y = penMotion.prevPositions[QUEUE_SIZE - 1]->y;

  penMotion.pos->x = x;
  penMotion.pos->y = y;
}

void updatePenVelocities() {
  uint16_t mostRecentX = penMotion.prevPositions[QUEUE_SIZE - 1]->x;
  uint16_t mostRecentY = penMotion.prevPositions[QUEUE_SIZE - 1]->y;

  double velocitySumX = 0.0;
  double velocitySumY = 0.0;

  // get average of all velocities
  for (int i = QUEUE_SIZE - 2; i >= 0; i -= 1) {
    uint16_t x = penMotion.prevPositions[i]->x;
    uint16_t y = penMotion.prevPositions[i]->y; 

    double velocityX = (mostRecentX * 1.0 - x) / (QUEUE_SIZE - 1 - i);
    double velocityY = (mostRecentY * 1.0 - y) / (QUEUE_SIZE - 1 - i);

    velocitySumX += velocityX;
    velocitySumY += velocityY;
  }

  penMotion.vel->vx = velocitySumX / (QUEUE_SIZE - 1);
  penMotion.vel->vy = velocitySumY / (QUEUE_SIZE - 1);
}

void decideMove() {
  resetPenState();

  decideHorizontal();
  decideVertical();
  decideFast();
  decideSlow();
  decideDirection();

  switch (penState.dir) {
    case 'N':
      // CHECK FOR LINEARITY
      if (penState.isHorizontal) {
        // turn or jiggle  
        skid(10);
      } else if (penState.isVertical) {
        // turn or jiggle 
        skid(10); 
      }

      // CHECK FOR SPEED
      if (penState.isFast) {
        // slow down
        track(50, 'F');
      } else if (penState.isSlow) {
        // speed up
        track(50, 'B');
      }
    break;
    case 'E':
      if (penState.isHorizontal) {
        // turn or jiggle  
        turnClockwise(30);
      } else if (penState.isVertical) {
        // turn or jiggle  
        turnClockwise(30);
      }
  
      // CHECK FOR SPEED
      if (penState.isFast) {
        turnClockwise(60);
        // slow down
      } else if (penState.isSlow) {
        turnAntiClockwise(60);
        // speed up
      }
    break;
    case 'S':
      if (penState.isHorizontal) {
        // turn or jiggle  
        skid(10);
      } else if (penState.isVertical) {
        // turn or jiggle  
        skid(10);
      }

      // CHECK FOR SPEED
      if (penState.isFast) {
        // slow down
        track(50, 'B');
      } else if (penState.isSlow) {
        // speed up
        track(50, 'F');
      }
    break;
    case 'W':
      if (penState.isHorizontal) {
        // turn or jiggle 
        turnAntiClockwise(30); 
      } else if (penState.isVertical) {
        // turn or jiggle 
        turnAntiClockwise(30); 
      }

      // CHECK FOR SPEED
      if (penState.isFast) {
        // slow down
        turnAntiClockwise(60);
      } else if (penState.isSlow) {
        // speed up
        turnClockwise(60);
      }
    break;  
    default:
      Serial.println("decideMove: direction not recognized");
  }
}

void loop() {
//  Serial.println(penMotion.pos->x);
  pixy.ccc.getBlocks();
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks) {
    uint16_t x = pixy.ccc.blocks[0].m_x;
    uint16_t y = pixy.ccc.blocks[0].m_y;

    Serial.println(pixy.ccc.blocks[0].m_x); // only take most relevant block
    Serial.print("Y value = ");
    Serial.println(pixy.ccc.blocks[0].m_y);

    if (!isSetup) {
      frameListSetup(x, y);  
    } else {
        addBlockToFrameList(x, y);
  
        updatePenPosition();
      
        updatePenVelocities();
  
        decideMove();  
    }
  }  
}
