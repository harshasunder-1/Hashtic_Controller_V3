#ifndef STOPCONTROLLER_H_
#define STOPCONTROLLER_H_

#include "stdint.h"


#define NO_ACTION 1 
#define COASTING 2
#define REGEN 3
#define NO_BRAKE 4
#define MECHBRAKE 5
#define SLAMBRAKE 6
#define DISENGAGE_BRAKE 7

#define NOTEVER 1
#define IMMEDIATE 2
#define AT_150 150
#define AT_200 200
#define AT_300 300

#define NOT_FINISHED 1
#define FINISHED 2

typedef struct {
  uint8_t stop_bool;
  
  uint8_t action1;
  uint8_t action2;
  uint16_t action2AtRPM;

  uint8_t action1_Done;
  uint8_t action2_Done;

}stopController;


void setStopAction(stopController *sc,uint8_t action1,uint8_t action2, uint16_t action2AtRPM);
void applyStopAction1(stopController *sc);
void applyStopAction2(stopController *sc,uint16_t absCurrentRPM);
void resetStopController(stopController *sc);



















#endif