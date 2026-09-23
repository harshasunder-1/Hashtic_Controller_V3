#include "StopController.h"
#include "StateMachine.h"
#include "FDCAN.h"


void resetStopController(stopController *sc){
  sc->stop_bool = 0;
  sc->action1 = 0;
  sc->action2 = 0;
  sc->action2AtRPM = 0;
  sc->action1_Done = NOT_FINISHED;
  sc->action2_Done = NOT_FINISHED;
}

void setStopAction(stopController *sc,uint8_t action1,uint8_t action2, uint16_t action2AtRPM){
  sc->stop_bool = 1;
  sc->action1 = action1;
  sc->action2 = action2;
  sc->action2AtRPM = action2AtRPM;
  sc->action1_Done = NOT_FINISHED;
  sc->action2_Done = NOT_FINISHED;
}


void applyStopAction1(stopController *sc){
  /* in all action1 act immediately.action1 can be coasting,regen,disengageBrake or MechBrake
     For safety, everytime we apply the mech brake, we always redo coasting.
  */
  
  if (sc->action1_Done == NOT_FINISHED){
      if (sc->action1 == COASTING){
        TMCM_SpeedLoop_TurnOff();
      }
      else if (sc->action1 == REGEN){
        applyRegen(ss.motorDirection);
      }
      else if (sc->action1 == MECHBRAKE){
        TMCM_SpeedLoop_TurnOff();
        FDCAN_SendControlledBrakeMsg();
        incrementBrakeState(&ss);
      }
      else if (sc->action1 == SLAMBRAKE){
        TMCM_SpeedLoop_TurnOff();
        FDCAN_SendSlamBrakeMsg();
        incrementBrakeState(&ss);
      }
      else if (sc->action1 == DISENGAGE_BRAKE){
        FDCAN_SendStopBrakeMsg();
        decrementBrakeState(&ss);
      }
    sc->action1_Done = FINISHED;
  }
}
       

void applyStopAction2(stopController *sc,uint16_t absCurrentRPM){
  /* If theres an action2 it applies at a certain rpm, after action1 is done.
     For instance, action1=coasting, action2=mech brake when RPM falls to 300, or 150.
     Or action1=regen, action2=coasting at x RPM .or action1=regen, action2=mech brake at Y RPM 
  */
  
  if (sc->action2_Done == NOT_FINISHED){  
    if (sc->action2 == NO_ACTION){
      sc->action2_Done = FINISHED;
    }
    else{
      if (absCurrentRPM < sc->action2AtRPM){
          if (sc->action2 == COASTING){
            TMCM_SpeedLoop_TurnOff();
          }
          else if (sc->action2 == MECHBRAKE){
            TMCM_SpeedLoop_TurnOff();
            FDCAN_SendControlledBrakeMsg();
            incrementBrakeState(&ss);
          }
          sc->action2_Done = FINISHED;
        } 
      }
  }
}