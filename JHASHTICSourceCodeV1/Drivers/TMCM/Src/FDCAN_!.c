 /*
* FDCAN.c
*
*  Created on: Mar 5, 2023
*      Author: Jonathan
*/

#include "FDCAN.h"
#include "mc_type.h"
#include "motorcontrol.h"
#include "PRECHARGE.h"
#include "StateMachine.h"
#include "ControlFns.h"
#include "TemperatureLogic.h"
#include "Configuration.h"
#include "EncFaults.h"
#include "StartSequence.h"
#include <stdlib.h>

extern FDCAN_HandleTypeDef hfdcan2;
extern FOCVars_t FOCVars[1];
extern systemState ss;
extern continousControlTimer ccT;
extern Temp t;
extern brakeCtrl b;
extern logController logC;
//CAN variables here
uint32_t functionID,source_address, destination_address;

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
extern FDCAN_RxHeaderTypeDef   RxHeader;
extern uint8_t RxData[8];
FDCAN_FilterTypeDef sFilterConfig;

extern int16_t hTargetSpeedUserDefined;
extern FOCVars_t FOCVars[1];
extern uint8_t PCOn;
uint8_t inputsOk,podOK;
extern float k;
extern uint8_t CircleLimitationState;

extern uint8_t cc_stopMsg_oneTime;
extern uint8_t cc_turnOff;

void FDCAN_TxInit(void)
{
  if(HAL_FDCAN_Start(&hfdcan2)!= HAL_OK)
  {
    //Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    //Error_Handler();
  }
  TxHeader.Identifier = 0x0E090102;//This is our identifier
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}

void FDCAN_RxFilterInit(void)
{
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK; //FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00;//destination address of flyer 0x00000200 (uint32_t)S.CAN_ID<<8 // 2 - is just a number
  sFilterConfig.FilterID2 = 0x7FFFFFFF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    //Error_Handler();
  }
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
  
  //HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0);
  RxData[0]=RxData[1]=RxData[2]=RxData[3]=RxData[4]=0;
}


uint8_t CheckCanInputs(CAN_SS_Input *s){
    uint8_t IsSpeedUnderLimit = 0,IsRampUnderLimit = 0,IsDirectionOk=0;
    float rampTemp=0;
      
    if(s->targetRPM > 100 && s->targetRPM < 2000){
        IsSpeedUnderLimit = 1;
      }
      
    rampTemp = (float)s->targetRPM /s->rampupTime ;
    if(rampTemp <=0.15f && rampTemp > 0.015f){ //min ramptime = 2s , max ramptime = 20s
      IsRampUnderLimit = 1;
    }
    
    if ((s->direction == 0xFF) || (s->direction == 0xAA)){
      IsDirectionOk = 1;
    }
    
    if ((IsSpeedUnderLimit == 1 ) && (IsRampUnderLimit == 1) &&(IsDirectionOk == 1)){
      return 1;
    }else{
      return 0;
    }
}



uint8_t CheckCanInputsContinuous(CAN_Continuous_Input *s){
    uint8_t IsDirectionOk=0,startInputsOK=0;
    
    if ((s->direction == 0xFF) || (s->direction == 0xAA)){
      IsDirectionOk = 1;
    }
    
    startInputsOK = 1; 
 /*   if ((s->controlQty == 0) && (s->indexID == 0)){
      startInputsOK = 1;
    }*/
    
    if (IsDirectionOk==1 && startInputsOK==1){
      return 1;
    }else{
      return 0;
    }
}





void FDCAN_CC_TMCM_sendRunTimeData(void)
{ 
  TxHeader.Identifier = 0x7801020;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
      
  TxData[0]= ss.indexID >> 8;
  TxData[1]= ss.indexID;  
  TxData[2]= (hTargetSpeedUserDefined)>>8;
  TxData[3]= hTargetSpeedUserDefined;
  TxData[4]= ss.currentRPM>>8;
  TxData[5]= ss.currentRPM;
  TxData[6]= FOCVars[0].Iqdref.q>>8;
  TxData[7]= FOCVars[0].Iqdref.q;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}

void FDCAN_CC_TMCM_sendRPM_To_CCM(void)
{ 
  TxHeader.Identifier = 0x18353020;//30 dst - is brake, 20 s-src is tmcm
  TxHeader.DataLength = FDCAN_DLC_BYTES_2;
  
  TxData[0]= ss.currentRPM>>8;
  TxData[1]= ss.currentRPM;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}



void FDCAN_CC_TMCM_sendStatusData(void){
  TxHeader.Identifier = 0x7811020;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  
  TxData[0]= t.motorTempC;
  TxData[1]= t.mosfetTempC;
  TxData[2]= ss.DC_VOLTAGE/2;
  if (ss.CustomFaults != NO_FAULTS){
    TxData[3] = ss.CustomFaults;
  }else{
  TxData[3]= ss.MCSDK_PreFault;
  }
  uint16_t travelDist = (uint16_t)(ss.travelledDist * 100.0f); //for 200m, itll be 20000 , well below 65000, max dist is 650 mtrs.
  TxData[4]= travelDist >> 8;
  TxData[5]= travelDist;
  TxData[6]= ss.phaseVoltageRMS>>8;
  TxData[7]= ss.phaseVoltageRMS;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}


void FDCAN_SendStopBrakeMsg(void)
{
  TxHeader.Identifier =(0x18343020);  //src 0x30-tmcm, dst 0x20 - brake board 
  TxHeader.DataLength = FDCAN_DLC_BYTES_4;
  TxData[0]= 0;
  TxData[1]= 0;
  TxData[2]= 0;
  TxData[3]= 0;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}

void FDCAN_SendControlledBrakeMsg(void)
{
  TxHeader.Identifier =(0x18343020);  //src 0x30-tmcm, dst 0x20 - brake board 
  TxHeader.DataLength = FDCAN_DLC_BYTES_4;
  TxData[0]= 0;
  TxData[1]= 0;
  TxData[2]= 1;
  TxData[3]= 40; // braking strength , can be 30,40,50
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}
void FDCAN_SendSlamBrakeMsg(void)
{
  TxHeader.Identifier =(0x18343020);  //src 0x30-tmcm, dst 0x20 - brake board 
  TxHeader.DataLength = FDCAN_DLC_BYTES_4;
  uint16_t rpm = 300;
  TxData[0]= rpm>>8;
  TxData[1]= rpm;
  TxData[2]=1;
  TxData[3]=0;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}


void FDCAN_SendStartSeqAckMsg(uint8_t msgType,uint16_t currentRPM)
{
  TxHeader.Identifier =(0x7791020); 
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxData[0]= msgType;
  TxData[1]= currentRPM >> 8;
  TxData[2]= currentRPM;
  TxData[3]= 0x00;
  TxData[4]= 0x00;
  TxData[5]= 0x00;
  TxData[6]= 0x00;
  TxData[7]= 0x00;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}


void FDCAN_SendPCMAckMsg(uint8_t msgType)
{
  TxHeader.Identifier =(0x7791020); 
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxData[0]= msgType;
  TxData[1]= 0x00;
  TxData[2]= 0x00;
  TxData[3]= 0x00;
  TxData[4]= 0x00;
  TxData[5]= 0x00;
  TxData[6]= 0x00;
  TxData[7]= 0x00;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}

void FDCan_SendStopLoggingMsg(void){
  TxHeader.Identifier =(0x7831020); 
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxData[0]= 0x00;
  TxData[1]= 0x00;
  TxData[2]= 0x00;
  TxData[3]= 0x00;
  TxData[4]= 0x00;
  TxData[5]= 0x00;
  TxData[6]= 0x00;
  TxData[7]= 0x00;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
  
}

void FDCAN_TMCM_StopFrame(uint8_t errorReason,int16_t errorVal){
  TxHeader.Identifier =(0x7821020); 
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxData[0]= errorReason;
  TxData[1]= errorVal >> 8;
  TxData[2]= errorVal;
  TxData[3]= 0x00;
  TxData[4]= 0x00;
  TxData[5]= 0x00;
  TxData[6]= 0x00;
  TxData[7]= 0x00;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
  
}


uint8_t deltaID=0;
uint16_t dRPM=0;

void FDCAN_parseForMotor(void){ // This gets toggled inside the interrupt. Whenever a reception happened on the CAN

  functionID=((RxHeader.Identifier)&0xFF0000)>>16;
  source_address=(RxHeader.Identifier)&0xFF;
  

  switch (functionID) {
    
  case STOP_COMMANDS:
    
    canSSIp.motorAction = RxData[0];
    
    if(canSSIp.motorAction == STOP_COASTING){ 
      cc_turnOff = 1; //setStopAction(COASTING,NO_BRAKE);
                      
    }
        
    else if (canSSIp.motorAction == SS_STOP_REGEN){
      // stop motor with regen braking
      applyRegen(ss.motorDirection); 
      ss.cc_ramp = CC_RAMPDOWN;  //whenever we regen we have to put state as cc_rampdown, and set cc_State as runingOL
      ss.cc_state = CC_RUNNING_OL;
      FDCAN_SendPCMAckMsg(1);  //setStopAction(REGEN,NO_BRAKE);
    }
    
    else if (canSSIp.motorAction == SS_STOP_COAST_MECH_BRAKE){
      // coasting and mech braking
      cc_turnOff = 1;
      ss.cc_ramp = CC_RAMPOFF;
      ss.brakeState++;
      FDCAN_SendControlledBrakeMsg();
      FDCAN_SendPCMAckMsg(1);   //setStopAction(COASTING,ENGAGE_BRAKE,AT_300);
      
    }
    
    else if (canSSIp.motorAction == SS_DISENGAGE_MECHBRAKE){
      FDCAN_SendStopBrakeMsg(); //break disengage
      ss.brakeState--;
      FDCAN_SendPCMAckMsg(1);    //setStopAction(NONE,DISENGAGE_BRAKE);
    }
    
    else if (canSSIp.motorAction == SS_ENGAGE_MECHBRAKE){
      // coasting and mech braking
      cc_turnOff = 1;
      ss.runType=NO_RUN;
      FDCAN_SendControlledBrakeMsg();
      ss.brakeState++;
      FDCAN_SendPCMAckMsg(1);   //setStopAction(COASTING,ENGAGE_BRAKE,IMMEDIATE);

    }
    
    else if (canSSIp.motorAction == SS_REGEN_MECH_BRAKE){
      // stop motor with regen braking and mech braking
      applyRegen(ss.motorDirection);
      ss.cc_ramp = CC_RAMPDOWN;  //whenever we regen we have to put state as cc_rampdown and state as running ol
      ss.cc_state = CC_RUNNING_OL;
      
      FDCAN_SendControlledBrakeMsg();
      ss.brakeState++;
      FDCAN_SendPCMAckMsg(1);    //setStopAction(REGEN,ENGAGE_BRAKE,IMMEDIATE);
    }
    else if(canSSIp.motorAction == RESET_TMCM){
      FDCAN_SendPCMAckMsg(1);
      HAL_Delay(10);
      HAL_NVIC_SystemReset(); // reset Motor!Not working
    }
    else{
    }
    break;
        
  case PRECHARGE_FUNCTIONID: //

    //pcv.Precharge_Stage = PRECHARGE_START;
    //PCOn = RxData[0];
    ss.targetRPM =  0;
    ss.indexID  = 0;
    ss.brakeDistance = 0;
    ss.travelledDist = 0;
    
    //reset logging.
    logC.startTickForDelayedStop = 0; // will reset timer in WaitmSecForStopping
    logC.logStopMsgSent = 0;
    ss.loggingOn = 0;
    
    ss.cc_state = CC_IDLE;
    ss.CustomFaults = NO_FAULTS; //remove faults when u get precharge
    ss.customFaultErrorVal = 0;
    b.brakeCounter = 0;

    //reset start seq
    resetStartSeqParams( &ssq);
    resetStartSeqErrorState( &ssq);
       
    PID_HandleInit(&PIDSpeedHandle_M1);   
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);
    
    FDCAN_SendPCMAckMsg(1);
    break;
    
  case CONTINUOUS_DATA:
    
    canContIp.motorAction = RxData[0];
    canContIp.direction = RxData[1];
    canContIp.controlQty = ((RxData[3]<<8)|(RxData[2]));
    canContIp.indexID =  ((RxData[5]<<8)|(RxData[4]));
    
    //stop looking at these commands if you go into error state.Here only start and continuous commands
    //other stop commands comes from a diff function id.
    if (ss.cc_state == CC_ERROR){
       return;
    }
    
    if (canContIp.motorAction == MOTORON){ 
      /*if motor is already running : turn off , else get direction, check if other data is empty , then send ACK.
        Prepare for continous values, by starting a timer , and resetting the index values to zero. can turn on PWM with Zero duty
        check temperatures within limits, encoder reading is coming properly also.*/
        podOK = CheckPodState(&ss);  // checks if motor state is not run, checks if pod is not moving, checks if encoder is OK
        inputsOk =  CheckCanInputsContinuous(&canContIp);
        if (podOK && inputsOk){
          ss.indexID = 0;
          ss.targetRPM = 0;
          if(canContIp.direction==0xFF){ // REVERSE FROM APP     
            if (c.positionInPod == LEFT_SIDE){ss.podDirection = -1;} // For signFor CW = 1,left side motor needs to rotate counter clockwise.
            if (c.positionInPod == RIGHT_SIDE){ss.podDirection = 1;} // For signFor CW = 1,right side motor needs to rotate clockwise.
          }
          else if(canContIp.direction==0xAA){ // FORWARD FROM APP
            if (c.positionInPod == LEFT_SIDE){ss.podDirection = 1;}  // opposite for above
            if (c.positionInPod == RIGHT_SIDE){ss.podDirection = -1;}
          }
          
          ss.motorDirection = ss.podDirection;
          
          // first time, targetRPM and brake distance is sent in control Qty and index ID, from the next time  we get actual controlQty and indexID
          ss.targetRPM = canContIp.controlQty;
          ss.brakeDistance = canContIp.indexID;
          ss.CL_DeltaRPMThreshold =  Calculate_CLDeltaRPMThreshold(ss.targetRPM);
          
          ss.loggingOn = 1;
          
          ss.CustomFaults = NO_FAULTS;
          ss.cc_state = CC_RUNNING_CL; 
          ss.cc_ramp = CC_RAMPUP;
          ss.runType=CC;
          ss.travelledDist = 0;
          
          DisableEncoderFltChking(&encFlts);
          ResetEncFaults(&encFlts);
          EnableEncoderFltChking(&encFlts);
          
          FDCAN_SendPCMAckMsg(1);
                    
          //start the StartSeq
          ssq.PCM_startCommand = 1;
            
        }else{
          //send one error msg
          FDCAN_SendPCMAckMsg(2);
          cc_turnOff = 1;
        }        
    }
       
    
    else if (canContIp.motorAction == CC_DATA_IP){ 
      // When we first come in, we have to transition from the start seq mode, and we also need to start 
      // the timer to keep checking if the intervals btw data is 100ms.
      // after that first time, we keep checking if inde is incrementing properly and change in rpm requested is within bounds.
          
      if (ssq.currentState == WAIT_PCM){
        dRPM = abs(canContIp.controlQty - ss.currentAbsRpm);
        if (dRPM < 50){
          ssq.PCM_ContinousConnectionMade = 1;
          
          //start can interval Timers
          ccT.PCM_timer = 0;
          ccT.timerOnBool = 1;
          
          ss.targetRPM = canContIp.controlQty;
          ss.indexID = canContIp.indexID;
          
          int16_t finalTargetSpeed = (ss.targetRPM / 6) * ss.motorDirection;
          
          // to minimize stair case feel, we give a ramptime as close to 100ms interval in which these rpms arrive.
          MC_ProgramSpeedRampMotor1(finalTargetSpeed, 90); 
          
        }else{
          ssq.PCM_ContinousConnectionMade = -1; // will stop the start sequence with PCM_HANDOVER_BAD_RPM fault in while loop ssq section
        }   
      } // closes WAIT_PCM
 
      else if (ssq.currentState == TRANSITION_DONE){
       
        ccT.PCM_timer = 0; //reset Timer   
        deltaID = canContIp.indexID - ss.indexID;
        dRPM = abs(canContIp.controlQty - ss.targetRPM);
        
        if (canContIp.controlQty < (ss.targetRPM)){ //in ramp up till we see a msg where the new value is less than the old value. MOVE THIS elsewhere?
          ss.cc_ramp = CC_RAMPDOWN;
        }
                
        if ((deltaID <= 5) && (dRPM < 50)){
          ss.targetRPM = canContIp.controlQty;
          ss.indexID = canContIp.indexID;
          int16_t finalTargetSpeed = (ss.targetRPM / 6) * ss.motorDirection;
          MC_ProgramSpeedRampMotor1(finalTargetSpeed, 90);
        }else{            
          ss.CustomFaults = PCM_BAD_CAN_MSG;
          ss.cc_state = CC_ERROR;
        }
      } // closes else ssq_
      
    } // if data is contIP data
    

  default:
    break;
  }
}
