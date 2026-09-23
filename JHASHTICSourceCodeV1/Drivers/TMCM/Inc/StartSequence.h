#ifndef START_SEQ_H_
#define START_SEQ_H_

#include "StateMachine.h"


//ON GROUND(800,2000,2400,2800,5500,5,50,600)
//LIFTED (500,800,1000,1200,1000,5,50,200)

#define T_RAMP_DONE 500 //time to ramp up the Torque
#define T_STALL_CHECK 800 // time before which we should see RPM_MIN movement
#define T_HANDOVER_MAX 1000 // time in which we should see RPM_HANDOVER movement
#define T_PCM_HANDOVER  2000 // time in which PCM should respond and take over

#define START_TORQUE_IQ 800//command to start the torque
#define RPM_STALL_MIN 5 // RPM below which we consider it a stall
#define RPM_HANDOVER 50 
#define RPM_STEADYSTATE 600

enum StartState { IDLE_WAITING, START_SEQUENCE, SPEED_LOOP_SYNC, WAIT_PCM,TRANSITION_DONE };
typedef enum { NO_ERROR, START_STALL,SPEED_HANDOVER_TIMEOUT, PCM_HANDOVER_TIMEOUT,PCM_HANDOVER_BAD_RPM}ErrorState;

typedef struct{
  uint8_t currentState;
  ErrorState errorState;
  
  uint16_t globalStartSeqTimer;
  
  uint8_t PCM_startCommand;
  uint16_t PCM_startRPM;
  int8_t PCM_ContinousConnectionMade;
 
}startSeq;

extern startSeq ssq;

void initializeStartSeqParams( startSeq *ssq_);
ErrorState ExecStartSeq(startSeq *ssq_,systemState *ss);
ErrorState ExecStartSeqWithPCM(startSeq *ssq_,systemState *ss);

void resetStartSeqParams( startSeq *ssq);
void resetStartSeqErrorState( startSeq *ssq);


#endif
