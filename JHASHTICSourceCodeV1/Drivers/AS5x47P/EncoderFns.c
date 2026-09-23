/*
 * EncoderSetup.c
 *
 *  Created on: 22-Mar-2023
 *      Author: harsha
 */

#include "AS5x47P.h"
#include "EncoderFns.h"

Diaagc diag;
Errfl errfl;


// Checking whether the magnet field intensity is proper. 
uint8_t AS5047_checkEncoderHealth(void){
	diag.raw = AS5047_SPI_Read(DIAGC_READ_FRAME,0);
	if((diag.values.magh == 1) || (diag.values.magl == 1) || (diag.values.cof == 1)){
		return 1;
	}else{
		return 0;
	}
}

//need to read to clear error bit in AS5047
uint8_t AS5047_checkEncoderError(void){
    errfl.raw = AS5047_SPI_Read(ERRFL_READ_FRAME,0);
    if (errfl.values.frerr == 1){return AS5047_FRAME_ERROR;}
    else if (errfl.values.invcomm == 1){return AS5047_INVALID_COMMAND_ERROR;}
    else if(errfl.values.parerr == 1){return AS5047_PARITY_ERROR;}
    else {return AS5047_NO_ERROR;}
}


// Setting up the encoder to work in ABI mode. ABI will be turned on. There is also a provision for getting a PWM output across W_Hall. This is turned off.
void SetupABIwithoutPWM(void){
  Settings1 settings1;
  settings1.values.factorySetting = 1; // do not change. 
  settings1.values.not_used = 0; // do not change
  settings1.values.dir = 0;  // By definition A leads B for CW direction. for us seen from the front, rotating in a CW direction gives A leading B.
  settings1.values.uvw_abi = 0; // 0-ABI with W pin as PWM, 1-UVW_HALL with I pin as PWM
  settings1.values.daecdis = 0; 
  settings1.values.abibin = 1; // ABI-decimal or binary.
  settings1.values.dataselect = 0; //1 is cordic Angle, 0 is dynamic angle compensation. Remove for very slow speeds.
  settings1.values.pwmon = 0; //sets pwm on W pin if 1.

  AS5047_writeRegister(SETTINGS1_REG, settings1.raw);

  Settings2 settings2;
  settings2.values.abires = 0; // with abibin sets the resolution
  settings2.values.uvwpp = 4; // 2 pole pairs - 0b010 
  AS5047_writeRegister(SETTINGS2_REG,settings2.raw);
}


void SetupHallwithoutPWM(void){
  Settings1 settings1;
  settings1.values.factorySetting = 1; // do not change. 
  settings1.values.not_used = 0; // do not change
  settings1.values.dir = 0;  // By definition A leads B for CW direction. for us seen from the front, rotating in a CW direction gives A leading B.
  settings1.values.uvw_abi = 1; // 0-ABI with W pin as PWM, 1-UVW_HALL with I pin as PWM
  settings1.values.daecdis = 0; 
  settings1.values.abibin = 1; // ABI-decimal or binary.
  settings1.values.dataselect = 0; //1 is cordic Angle, 0 is dynamic angle compensation. Remove for very slow speeds.
  settings1.values.pwmon = 0; //sets pwm on W pin if 1.

  AS5047_writeRegister(SETTINGS1_REG, settings1.raw);

  Settings2 settings2;
  settings2.values.abires = 1; // with abibin sets the resolution
  settings2.values.uvwpp = 4; // 2 pole pairs - 0b010 
  AS5047_writeRegister(SETTINGS2_REG,settings2.raw);
}

//Just a callback to ensure whether the registers are set correctly. 
uint8_t Check_ABI_SetCorrectly(Settings1 settings1, Settings2 settings2){
  if ((settings1.values.uvw_abi == 0) && (settings1.values.abibin == 1 ) && (settings1.values.pwmon == 0 ) && ( settings1.values.dir == 0)
      && (settings2.values.abires == 0)  && ( settings2.values.uvwpp == 4)){
        return 1;
      }
  else{
    return 0;
  }
}

uint8_t Check_Hall_SetCorrectly(Settings1 settings1, Settings2 settings2){
  if ((settings1.values.uvw_abi == 1) && (settings1.values.abibin == 1 ) && (settings1.values.pwmon == 0 ) && ( settings1.values.dir == 0)
      && (settings2.values.abires == 1)  && ( settings2.values.uvwpp == 4)){
        return 1;
      }
  else{
    return 0;
  }
}

//Main function call to be used in main.c
Settings1 settings1Reg;
Settings2 settings2Reg;
uint8_t abiSettingsOK;
uint8_t setupMotorEncoder_inABI_Mode(void){
	SetupABIwithoutPWM();
	settings1Reg.raw = AS5047_SPI_Read(SETTINGS1_READ_FRAME, 0);
	settings2Reg.raw = AS5047_SPI_Read(SETTINGS2_READ_FRAME, 0);
	abiSettingsOK = Check_ABI_SetCorrectly(settings1Reg,settings2Reg);
	return abiSettingsOK;
}

uint8_t setupMotorEncoder_inHall_Mode(void){
	SetupHallwithoutPWM();
	settings1Reg.raw = AS5047_SPI_Read(SETTINGS1_READ_FRAME, 0);
	settings2Reg.raw = AS5047_SPI_Read(SETTINGS2_READ_FRAME, 0);
	abiSettingsOK = Check_Hall_SetCorrectly(settings1Reg,settings2Reg);
	return abiSettingsOK;
}

uint16_t zeroPos;
uint8_t updateEncoderZeroPosition(uint16_t zeroValue){
	
	zeroPos = AS5047_ReadZeroValue();
	if (zeroPos != zeroValue){
		AS5047_WriteZeroValue(zeroValue); //function must check if it got back the same value it wrote.
		zeroPos = AS5047_ReadZeroValue(); //to check if this value is same as what we wrote
		if (zeroPos != zeroValue){
			return 0;
		}else{
			return 1;
		}
	  }
	return 1;
}


uint8_t AS5047_EnableMagErrors(void){
	ZPOSL_frame ZPOS_L;
	ZPOS_L.raw = AS5047_readRegister(ZPOSL_REG,0);
	if ((ZPOS_L.values.comp_h_error == 0 ) || (ZPOS_L.values.comp_l_error == 0)){
		ZPOS_L.values.comp_h_error = 1;
		ZPOS_L.values.comp_l_error = 1;
		AS5047_writeRegister(ZPOSL_REG, ZPOS_L.raw);

		//check if it was written properly
		ZPOS_L.raw = 0;
		ZPOS_L.raw = AS5047_readRegister(ZPOSL_REG,0);
		if ((ZPOS_L.values.comp_h_error == 0 ) || (ZPOS_L.values.comp_l_error == 0)){
			return 0;
		}else{
			return 1;
		}
	}
	return 1;
}

uint16_t getEncoderStartPosition(void){
	uint16_t startingAngle16Bit=0;
	//float startingAngleMech=0;
	int startingCNT_val = 0;
	startingAngle16Bit = GetAveragedAngleReading(10);  // take the average of 10 readings and then take that as the starting angle.
	//startingAngleMech = startingAngle16Bit * SPI_RDNG_TO_MECH_ANGLE;
	startingCNT_val = (int)((startingAngle16Bit * SPI_RDNG_TO_ENC_CNT)+0.5f); // rounded
	return startingCNT_val;
}

float getEncoderAngleFromABI(TIM_HandleTypeDef *htim){
	float angleMech_encCnt = 0;
	uint16_t encoderCNT = 0;
	encoderCNT = htim->Instance->CNT;
	angleMech_encCnt = encoderCNT * ENC_CNT_TO_MECH_ANGLE;
	return angleMech_encCnt;
}

//make continuous Read = 1 if your reading in the while loop etc.
float getEncoderAngleFromSPI(uint8_t continuousRead){
	uint16_t angleData = 0;
	float angleMech = 0;
	ReadDataFrame readdataframe;
	Angle angle;
	readdataframe.raw = AS5047_SPI_Read(ANGLE_READ_FRAME,continuousRead);//AS5047_readRegister(ANGLE_REG,1);
	angle.raw = readdataframe.values.data;
	angleData =  angle.values.cordicang;
	angleMech = angleData*360.0/16384;
	return angleMech;
}

ReadDataFrame readdataframe;	
int16_t ENC_getRawReadingFromSPI(void){
  Angle angle;
  uint16_t angleData=0;
  readdataframe.raw  = 0;
  readdataframe.raw = AS5047_SPI_Read(ANGLE_READ_FRAME,0);//AS5047_readRegister(ANGLE_REG,1);
  if (readdataframe.values.ef == 1){
    return -1;
  }else{
    angle.raw = readdataframe.values.data;
    angleData =  angle.values.cordicang;
    return angleData;
  }
}


//TO USE
uint8_t ENC_CompareSPI_ABI(uint16_t ABIrdng, uint16_t SPIrdng,uint16_t threshold_in_Angle){
    // Scale your raw encoder values to the full 16-bit range (0 to 65535)
    uint16_t ABI_elAngle_int = (ABIrdng % 409) * 160;   // (65536 / 409) ˜ 160
    uint16_t spi_elAngle_int = (SPIrdng % 3276) * 20;   // (65536 / 3276) ˜ 20

    // The hardware automatically wraps this to the shortest path (-180 to 180)! by making a uint16_t int16_t!
    int16_t delta_int = (int16_t)(ABI_elAngle_int - spi_elAngle_int);
    
    if ((delta_int < -threshold_in_Angle) || (delta_int > threshold_in_Angle)){
      return 1;
    }
    
    return 0;
    
}


int16_t ENC_makeRaw_mechS16(uint16_t angleReading){   
  int16_t mechS16 = angleReading * 4;
  return mechS16;
}
  


uint8_t SectorIdentification(uint16_t ElAngle)
{
  uint8_t Sector;
  if(ElAngle > 0 && ElAngle <= 60)
    Sector = 1;
  else if(ElAngle > 60 && ElAngle <= 120)
    Sector = 2;
  else if(ElAngle > 120 && ElAngle <= 180)
    Sector = 3;
  else if(ElAngle > 180 && ElAngle <= 240)
    Sector = 4;
  else if(ElAngle > 240 && ElAngle <= 300)
    Sector = 5;
  else if(ElAngle > 300 && ElAngle <= 359)
    Sector = 6;
  
  return Sector;
}

