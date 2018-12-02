#ifndef backendCV_h
#define backendCV_h

#include <Arduino.h>
#include <Pixy2.h>

// POSITION FACTORS - DETERMINE THESE
#define X_DISTANCE_FACTOR 1.0*3.33
#define Y_DISTANCE_FACTOR 2.5*7 // I changed this from 5 to 7

// SPEED TOLERANCE
#define SPEED_MINIMUM 0.02
#define SPEED_THRESHOLD_UPPER 0.8
#define SPEED_THRESHOLD_LOWER 0.1

// LINEAR_TRANSITION_TOLERANCE
#define X_TOLERANCE 10.0
#define Y_TOLERANCE 10.0

// TIME BETWEEN FRAMES
#define FRAME_TIME 16.5 // these are in millisecond

// QUEUE_SIZE
#define QUEUE_SIZE 25

/* Data structs */
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

  PenPosition prevPositions[QUEUE_SIZE];
} PenMotion;

typedef struct {
  bool isHorizontal; // in X
  bool isVertical; // in Y
  bool isFast; // velocity magnitude
  bool isSlow; // velocity magnitude
  char dir; // direction N, E, S, W
} PenState;

class BackendCV {
	public:
		BackendCV();
		PenState* getState(); // updates and returns pointer to penState variable.
	private:
		// data fields
    Pixy2 pixy;
		PenMotion penMotion;
		PenState penState;
		int moveCounter;
		// methods
		void init(); // all set up functionality will now go into this function
		void resetPenState();
		void addBlockToFrameList(uint16_t x, uint16_t y);
		void updatePenPosition();
		void updatePenVelocities();
    void decideMove();
    void decideHorizontal();
    void decideVertical();
    void decideFast();
    void decideSlow();
    void decideDirection();
};
#endif
