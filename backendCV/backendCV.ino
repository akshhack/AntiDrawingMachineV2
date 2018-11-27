// POSITION FACTORS - DETERMINE THESE
#define X_DISTANCE_FACTOR 1.0
#define Y_DISTANCE_FACTOR 1.5

// SPEED TOLERANCE
#define SPEED_THRESHOLD_UPPER 0.5 
#define SPEED_THRESHOLD_LOWER 0.1

// LINEAR_TRANSITION_TOLERANCE
#define X_TOLERANCE 4.0
#define Y_TOLERANCE 4.0

// TIME BETWEEN FRAMES
#define FRAME_TIME 16.5 // these are in millisecond

// BAUD
#define BAUD_RATE 115200

// QUEUE SPECS
#define QUEUE_SIZE 10

#define DELAY 10000

#include <Pixy2.h>

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
PenPosition posInit1 = {0, 0};
PenPosition posInit2 = {0, 0};
PenPosition posInit3 = {0, 0};
PenPosition posInit4 = {0, 0};
PenPosition posInit5 = {0, 0};
PenPosition posInit6 = {0, 0};
PenPosition posInit7 = {0, 0};
PenPosition posInit8 = {0, 0};
PenPosition posInit9 = {0, 0};
PenPosition posInit10 = {0, 0};

PenVelocity velInit = {0.0, 0.0};
PenMotion penMotion = {&posInit, &velInit, {&posInit1, &posInit2, &posInit3,
                                            &posInit4, &posInit5, &posInit6,
                                            &posInit7, &posInit8, &posInit9,
                                            &posInit10}};
PenState penState = {false, false, false, false, 'N'};
int setupCounter = 0;
int moveCounter = 0;

void setup() {
  pixy.init();
  Serial.begin(BAUD_RATE);
}

/* PenState handlers */
void resetPenState() {
  penState = {false, false, false, false};
}

void decideHorizontal() {
  uint16_t firstPosition = penMotion.prevPositions[0]->y;
  bool isHorizontal = true;
  // check if all positions are in tolerance of the firstPosition
  for (int i = 0; i < QUEUE_SIZE; i += 1) {
    uint16_t currentPos = penMotion.prevPositions[i]->y;
    if (currentPos < firstPosition - Y_TOLERANCE || 
        currentPos > firstPosition + Y_TOLERANCE) {
      isHorizontal = false;
      break;  
    } 
  }
  penState.isHorizontal = isHorizontal;
}

void decideVertical() {
  uint16_t firstPosition = penMotion.prevPositions[0]->x;
  bool isVertical = true;
  // check if all positions are in tolerance of the firstPosition
  for (int i = 0; i < QUEUE_SIZE; i += 1) {
    uint16_t currentPos = penMotion.prevPositions[i]->x;
    if (currentPos < firstPosition - X_TOLERANCE || 
        currentPos > firstPosition + X_TOLERANCE) {
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
  
  double vy_abs = abs(penMotion.vel->vy);
  double vx_abs = abs(penMotion.vel->vx);
  
  if (penMotion.vel->vx > 0 && penMotion.vel->vy > 0) {
    // NE  
    if (vy_abs > vx_abs) {
      penState.dir = 'N';  
    } else {
        penState.dir = 'E';  
    }
  } else if (penMotion.vel->vx > 0 && penMotion.vel->vy < 0) {
    // SE
    if (vy_abs > vx_abs) {
      penState.dir = 'S';  
    } else {
        penState.dir = 'E';  
    }
  } else if (penMotion.vel->vx < 0 && penMotion.vel->vy > 0) {
    // NW
    if (vy_abs > vx_abs) {
      penState.dir = 'N';  
    } else {
        penState.dir = 'W';  
    }
  } else {
    // SW
    if (vy_abs > vx_abs) {
      penState.dir = 'S';  
    } else {
        penState.dir = 'W';
    }
  }
}

void frameListSetup(uint16_t x, uint16_t y) {
  if (setupCounter < QUEUE_SIZE) {
    penMotion.prevPositions[setupCounter]->x = x;
    penMotion.prevPositions[setupCounter]->y = y;
    setupCounter += 1; 
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

    double velocityX = (mostRecentX * 1.0 - x) / ((QUEUE_SIZE - 1 - i) * FRAME_TIME);
    double velocityY = (mostRecentY * 1.0 - y) / ((QUEUE_SIZE - 1 - i) * FRAME_TIME);

    velocitySumX += velocityX;
    velocitySumY += velocityY;
  }

  penMotion.vel->vx = velocitySumX / (QUEUE_SIZE - 1);
  penMotion.vel->vy = velocitySumY / (QUEUE_SIZE - 1);

  Serial.print("Velocity X component: ");
  Serial.println(penMotion.vel->vx);

  Serial.print("Velocity Y component: ");
  Serial.println(penMotion.vel->vy);
}

void decideMove() {
  resetPenState();

  decideHorizontal();
  decideVertical();
  decideFast();
  decideSlow();
  decideDirection();

  if (penState.isHorizontal == true) {
    Serial.println("Pen is horizontal");  
  } 

  if (penState.isVertical == true) {
    Serial.println("Pen is vertical");  
  } 

  if (penState.isFast == true) {
    Serial.println("Pen is fast");  
  }

  if (penState.isSlow == true) {
    Serial.println("Pen is slow");  
  }

  // pen direction
  Serial.print("Pen direction is: ");
  Serial.println(penState.dir);

  switch (penState.dir) {
    case 'N':
      // CHECK FOR LINEARITY
      if (penState.isHorizontal) {
        // turn or jiggle  
      } else if (penState.isVertical) {
        // turn or jiggle  
      }

      // CHECK FOR SPEED
      if (penState.isFast) {
        // slow down
      } else if (penState.isSlow) {
        // speed up
      }
    break;
    case 'E':
      if (penState.isHorizontal) {
        // turn or jiggle  
      } else if (penState.isVertical) {
        // turn or jiggle  
      }
  
      // CHECK FOR SPEED
      if (penState.isFast) {
        // slow down
      } else if (penState.isSlow) {
        // speed up
      }
    break;
    case 'S':
      if (penState.isHorizontal) {
        // turn or jiggle  
      } else if (penState.isVertical) {
        // turn or jiggle  
      }

      // CHECK FOR SPEED
      if (penState.isFast) {
        // slow down
      } else if (penState.isSlow) {
        // speed up
      }
    break;
    case 'W':
      if (penState.isHorizontal) {
        // turn or jiggle  
      } else if (penState.isVertical) {
        // turn or jiggle  
      }

      // CHECK FOR SPEED
      if (penState.isFast) {
        // slow down
      } else if (penState.isSlow) {
        // speed up
      }
    break;  
    default:
      Serial.println("decideMove: direction not recognized");
  }
}

void printFrameList() {
  for (int i = QUEUE_SIZE - 1; i >= 0; i -= 1) {
    Serial.print("Frame #: ");
    Serial.println(i + 1);

    Serial.print("X position: ");
    Serial.println(penMotion.prevPositions[i]->x);
    Serial.print("Y position: ");
    Serial.println(penMotion.prevPositions[i]->y);
  }  
}

void loop() {
//  Serial.println(penMotion.pos->x);
  //Serial.println("Getting blocks");
  pixy.ccc.getBlocks();
  //Serial.println("Got blocks");
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks) {
    uint16_t x = pixy.ccc.blocks[0].m_x * X_DISTANCE_FACTOR;
    uint16_t y = pixy.ccc.blocks[0].m_y * Y_DISTANCE_FACTOR;


    Serial.print("X value = ");
    Serial.println(pixy.ccc.blocks[0].m_x); // only take most relevant block
    Serial.print("Y value = ");
    Serial.println(pixy.ccc.blocks[0].m_y);

    if (setupCounter < QUEUE_SIZE) {
      //Serial.println("Setting up");
      frameListSetup(x, y);  
    } else {
        //Serial.println("Set up complete");
        addBlockToFrameList(x, y);

        //printFrameList();
  
        updatePenPosition();
      
        updatePenVelocities();

        if (moveCounter % QUEUE_SIZE == 0) {
          decideMove();
          moveCounter = 0; 
          delay(DELAY); 
        }

        moveCounter += 1;
    }
  }  
}
