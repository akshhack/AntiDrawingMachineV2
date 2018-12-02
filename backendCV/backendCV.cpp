#include <Arduino.h>
#include <Pixy2.h>
#include "backendCV.h"

// Constructor definition
BackendCV::BackendCV(int queue_size) {
  // setup class data fields
  posInit = {0, 0};
  velInit = {0.0, 0.0};
  penMotion = {&posInit, &velInit};

  penState = {false, false, false, false, 'N'};

  moveCounter = 0;

  pixy.init();

  // not needed now as we are constructing the code
  // // set up first QUEUE_SIZE # of frames
  // for (int setupCounter = 0; setupCounter < QUEUE_SIZE; setupCounter += 1) {
  //   pixy.ccc.getBlocks();
  //
  //   if (pixy.ccc.numBlocks) {
  //       uint16_t x = pixy.ccc.blocks[0].m_x * X_DISTANCE_FACTOR;
  //       uint16_t y = pixy.ccc.blocks[0].m_y * Y_DISTANCE_FACTOR;
  //
  //       BackendCV::frameListSetup(x, y);
  //   }
  // }
}

void BackendCV::resetPenState() {
  penState = {false, false, false, false, 'N'};
}

void BackendCV::addBlockToFrameList() {
  for (int i = 0; i < QUEUE_SIZE - 1; i += 1) {
      penMotion.prevPositions[i].x = penMotion.prevPositions[i+1].x;
      penMotion.prevPositions[i].y = penMotion.prevPositions[i+1].y;
  }

  penMotion.prevPositions[QUEUE_SIZE - 1].x = x;
  penMotion.prevPositions[QUEUE_SIZE - 1].y = y;
}

void BackendCV::updatePenPosition() {
  uint16_t x = penMotion.prevPositions[QUEUE_SIZE - 1].x;
  uint16_t y = penMotion.prevPositions[QUEUE_SIZE - 1].y;

  penMotion.pos->x = x;
  penMotion.pos->y = y;
}

void BackendCV::updatePenVelocities() {
  uint16_t mostRecentX = penMotion.prevPositions[QUEUE_SIZE - 1].x;
  uint16_t mostRecentY = penMotion.prevPositions[QUEUE_SIZE - 1].y;

  double velocitySumX = 0.0;
  double velocitySumY = 0.0;

  // get average of all velocities
  for (int i = QUEUE_SIZE - 2; i >= 0; i -= 1) {
    uint16_t x = penMotion.prevPositions[i].x;
    uint16_t y = penMotion.prevPositions[i].y;

    double velocityX = (mostRecentX * 1.0 - x) / ((QUEUE_SIZE - 1 - i) * FRAME_TIME);
    double velocityY = (mostRecentY * 1.0 - y) / ((QUEUE_SIZE - 1 - i) * FRAME_TIME);

    velocitySumX += velocityX;
    velocitySumY += velocityY;
  }

  penMotion.vel->vx = velocitySumX / (QUEUE_SIZE - 1);
  penMotion.vel->vy = velocitySumY / (QUEUE_SIZE - 1);
}

PenState* BackendCV::getState() {
  while (moveCounter % QUEUE_SIZE != 0) {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks) {
      uint16_t x = pixy.ccc.blocks[0].m_x * X_DISTANCE_FACTOR;
      uint16_t y = pixy.ccc.blocks[0].m_y * Y_DISTANCE_FACTOR;

      BackendCV::addBlockToFrameList(x, y);
      BackendCV::updatePenPosition();
      BackendCV::updatePenVelocities();

      moveCounter += 1;
    }
  }

  // decide and return move state
  BackendCV::decideMove();
  moveCounter = 0;
  return &penState;
}

// move decide helpers
void BackendCV::decideMove() {
  BackendCV::resetPenState();

  BackendCV::decideHorizontal();
  BackendCV::decideVertical();
  BackendCV::decideFast();
  BackendCV::decideSlow();
  BackendCV::decideDirection();
}

void BackendCV::decideVertical() {
  uint16_t firstPosition = penMotion.prevPositions[0].x;
  bool isVertical = true;
  // check if all positions are in tolerance of the firstPosition
  for (int i = 0; i < QUEUE_SIZE; i += 1) {
    uint16_t currentPos = penMotion.prevPositions[i].x;
    if (currentPos < firstPosition - X_TOLERANCE ||
        currentPos > firstPosition + X_TOLERANCE) {
      isVertical = false;
      break;
    }
  }
  penState.isVertical = isVertical;
}

void BackendCV::decideHorizontal() {
  uint16_t firstPosition = penMotion.prevPositions[0].y;
  bool isHorizontal = true;
  // check if all positions are in tolerance of the firstPosition
  for (int i = 0; i < QUEUE_SIZE; i += 1) {
    uint16_t currentPos = penMotion.prevPositions[i].y;
    if (currentPos < firstPosition - Y_TOLERANCE ||
        currentPos > firstPosition + Y_TOLERANCE) {
      isHorizontal = false;
      break;
    }
  }
  penState.isHorizontal = isHorizontal;
}

void BackendCV::decideSlow() {
  double penSpeed = sqrt(penMotion.vel->vx * penMotion.vel->vx + penMotion.vel->vy * penMotion.vel->vy);
  penState.isSlow = (penSpeed < SPEED_THRESHOLD_LOWER);
}

void BackendCV::decideFast() {
  double penSpeed = sqrt(penMotion.vel->vx * penMotion.vel->vx + penMotion.vel->vy * penMotion.vel->vy);
  penState.isFast = (penSpeed > SPEED_THRESHOLD_UPPER);
}

void BackendCV::decideDirection() {
  double vy_abs = abs(penMotion.vel->vy);
  double vx_abs = abs(penMotion.vel->vx);
  if (vy_abs > vx_abs) {
    if (penMotion.vel->vy <= 0.0) {
      penState.dir = 'S';
    } else if (penMotion.vel->vy > 0.0){
        penState.dir = 'N';
    }
  } else if (vx_abs > vy_abs) {
       if (penMotion.vel->vx <= 0.0) {
          penState.dir = 'W';
       } else if (penMotion.vel->vx > 0.0) {
          penState.dir = 'E';
       }
  }
}
