// POSITION EXTREMES
#define LEFT_EXTREME
#define RIGHT_EXTREME
#define CLOSE_EXTREME
#define FAR_EXTREME
// all above not needed, I think

// POSITION FACTORS - DETERMINE THESE
#define X_DISTANCE_FACTOR 
#define Y_DISTANCE_FACTOR 3.75

// SPEED TOLERANCE
#define SPEED_THRESHOLD

// LINEAR_TRANSITION_TOLERANCE
#define X_TRANSITION_THRESHOLD
#define Y_TRANSITION_THRESHOLD

// TIME BETWEEN FRAMES
#define FRAME_TIME 16.5 // these are in millisecond

// BAUD
#define BAUD_RATE 115200

// QUEUE SPECS
#define QUEUE_SIZE 3

#include <Pixy2.h>
#include <cQueue.h>

Pixy2 pixy;
Queue_t q;

typedef struct {
  uint16_t x;
  uint16_t y;  
} PenPosition;

typedef struct {
  float vx;
  float vy;  
} PenVelocity;

typedef struct {
  PenPosition* pos;
  PenVelocity* vel;
  
  PenPosition* prevPositions[QUEUE_SIZE];
} PenMotion;

PenPosition posInit = {0, 0};
PenVelocity velInit = {0.0, 0.0};
PenMotion penMotion = {&posInit, &velInit, {&posInit, &posInit, &posInit}};
bool isSetup = false;
int setupCounter = 0;

void setup() {
  pixy.init();
  Serial.begin(BAUD_RATE);
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

  float velocitySumX = 0.0;
  float velocitySumY = 0.0;

  // get average of all velocities
  for (int i = QUEUE_SIZE - 2; i >= 0; i -= 1) {
    uint16_t x = penMotion.prevPositions[i]->x;
    uint16_t y = penMotion.prevPositions[i]->y; 

    float velocityX = (mostRecentX * 1.0 - x) / (QUEUE_SIZE - 1 - i);
    float velocityY = (mostRecentY * 1.0 - y) / (QUEUE_SIZE - 1 - i);

    velocitySumX += velocityX;
    velocitySumY += velocityY;
  }

  penMotion.vel->vx = velocitySumX / (QUEUE_SIZE - 1);
  penMotion.vel->vy = velocitySumY / (QUEUE_SIZE - 1);
}

void decideMove() {
  
}

void loop() {
//  Serial.println(penMotion.pos->x);
  pixy.ccc.getBlocks();
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks) {
    uint16_t x = pixy.ccc.blocks[0].m_x;
    uint16_t y = pixy.ccc.blocks[0].m_y;

    if (!isSetup) {
      frameListSetup(x, y);  
    } else {
        addBlockToFrameList(x, y);
  
        updatePenPosition();
      
        updatePenVelocities();
  
        decideMove();  
    }

    Serial.println(pixy.ccc.blocks[0].m_x); // only take most relevant block
    Serial.print("Y value = ");
    Serial.println(pixy.ccc.blocks[0].m_y);
    
  }  
}
