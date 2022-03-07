/*
 * ADE9000_Calibration.h
 *
 *  Created on: 6 mar 2022
 *      Author: M.Cristina Giannini
 */

#ifndef INC_ADE9000_CALIBRATION_H_
#define INC_ADE9000_CALIBRATION_H_

#include <ADE9000RegMap.h>
#include <ADE9000_API.h>
#include <ADE9000CalibrationInputs.h>


/*Constant Definitions***/
#define ADE90xx_FDSP 8000   			/*ADE9000 FDSP: 8000sps, ADE9078 FDSP: 4000sps*/
/*Energy Accumulation Settings*/
#define ADE9000_EP_CFG 0x0011			/*Enable energy accumulation, accumulate samples at 8ksps*/
										/*latch energy accumulation after EGYRDY*/
										/*If accumulation is changed to half line cycle mode, change EGY_TIME*/
#define ADE9000_EGY_TIME 0x1F3F 				/*Accumulate 8000 samples*/

/*Full scale Codes referred from Datasheet.Respective digital codes are produced when ADC inputs are at full scale. Donot Change. */
#define ADE9000_RMS_FULL_SCALE_CODES  52702092
#define ADE9000_WATT_FULL_SCALE_CODES 20694066
#define ADE9000_RESAMPLED_FULL_SCALE_CODES  18196
#define ADE9000_PCF_FULL_SCALE_CODES  74532013

/*Size of array reading calibration constants from EEPROM*/
#define CALIBRATION_CONSTANTS_ARRAY_SIZE 13
#define ADE9000_EEPROM_ADDRESS 0x54			//1010xxxy xxx---> 100(A2,A1,A0 defined by hardware). y (1: Read, 0:Write)
#define EEPROM_WRITTEN 0x01

/*Address of registers stored in EEPROM.Calibration data is 4 bytes*/
#define ADDR_CHECKSUM_EEPROM 0x0800		 // Simple checksum to verify data transmission errors. Add all the registers upto CPGAIN. The lower 32 bits should match data starting @ADDR_CHECKSUM_EEPROM
#define ADDR_EEPROM_WRITTEN_BYTE 0x0804  //1--> EEPROM Written, 0--> Not written. One byte only

#define ADDR_AIGAIN_EEPROM 0x0004
#define ADDR_BIGAIN_EEPROM 0x000C
#define ADDR_CIGAIN_EEPROM 0x0010
#define ADDR_NIGAIN_EEPROM 0x0014
#define ADDR_AVGAIN_EEPROM 0x0018
#define ADDR_BVGAIN_EEPROM 0x001C
#define ADDR_CVGAIN_EEPROM 0x0020
#define ADDR_APHCAL0_EEPROM 0x0024
#define ADDR_BPHCAL0_EEPROM 0x0028
#define ADDR_CPHCAL0_EEPROM 0x002C
#define ADDR_APGAIN_EEPROM 0x0030
#define ADDR_BPGAIN_EEPROM 0x0034
#define ADDR_CPGAIN_EEPROM 0x0038

/*Function declerations*/
void ADE9000_calibrate();
void ADE9000_iGain_calibrate(int32_t *,int32_t *,int32_t *,int32_t *, int);     //Current gain calibration function
void ADE9000_vGain_calibrate(int32_t *,int32_t *,int32_t *,int32_t *, int);     //Voltage gain calibration function
void ADE9000_pGain_calibrate(int32_t *,int32_t *,int32_t *,int, float);   //Power gain calibration function
void ADE9000_phase_calibrate(int32_t *,int32_t *,int32_t *,int32_t *, int);     //Phase calibration function
void calibrationEnergyRegisterSetup();                                          //Setup of Energy registers used in calibration. Donot Edit
void getPGA_gain();
int8_t isRegisterPositive(int32_t);
void storeCalConstToEEPROM();
void ADE9000_Calibration(void);


#define IGAIN_CAL_REG_SIZE 4
extern int32_t xIgain_registers[IGAIN_CAL_REG_SIZE];   //order [AIGAIN, BIGAIN, CIGAIN, NIGAIN]
extern int32_t xIgain_register_address[IGAIN_CAL_REG_SIZE];   //order [AIGAIN, BIGAIN, CIGAIN, NIGAIN]
extern int32_t xIrms_registers[IGAIN_CAL_REG_SIZE];
extern int32_t xIrms_registers_address[IGAIN_CAL_REG_SIZE];

#define VGAIN_CAL_REG_SIZE 3
extern int32_t xVgain_registers[VGAIN_CAL_REG_SIZE];   //order [AVGAIN, BVGAIN, CVGAIN]
extern int32_t xVgain_register_address[VGAIN_CAL_REG_SIZE];  //order [AVGAIN, BVGAIN, CVGAIN]
extern int32_t xVrms_registers[VGAIN_CAL_REG_SIZE];
extern int32_t xVrms_registers_address[VGAIN_CAL_REG_SIZE];

#define PHCAL_CAL_REG_SIZE 3
extern int32_t xPhcal_registers[PHCAL_CAL_REG_SIZE];   //order [APHCAL, BPHCAL, CPHCAL]
extern int32_t xPhcal_register_address[PHCAL_CAL_REG_SIZE];  //order [APHCAL, BPHCAL, CPHCAL]
extern int32_t xWATTHRHI_registers[PHCAL_CAL_REG_SIZE];  //Active energy registers
extern int32_t xWATTHRHI_registers_address[PHCAL_CAL_REG_SIZE];
extern int32_t xVARHRHI_registers[PHCAL_CAL_REG_SIZE];
extern int32_t xVARHRHI_registers_address[PHCAL_CAL_REG_SIZE];

#define PGAIN_CAL_REG_SIZE 3
extern int32_t xPgain_registers[PGAIN_CAL_REG_SIZE];   //order [APGAIN, BPGAIN, CPGAIN]
extern int32_t xPgain_register_address[PGAIN_CAL_REG_SIZE];  //order [AVGAIN, BVGAIN, CVGAIN, NVGAIN]
//The Power gain calibration reads active energy registers. The content and address arrays are defined in the PHCAL section above

//Global variables
#define EGY_REG_SIZE 3
extern int8_t calCurrentPGA_gain;
extern int8_t calVoltagePGA_gain;
extern int32_t accumulatedActiveEnergy_registers[EGY_REG_SIZE];
extern int32_t accumulatedReactiveEnergy_registers[EGY_REG_SIZE];
extern uint32_t calibrationDataToEEPROM[CALIBRATION_CONSTANTS_ARRAY_SIZE];

#define SPI_SPEED 5000000
#define CS_PIN 8
#define PM_1 4
#define IRQ0_INTERRUPT_PIN 2
#define INT_MODE FALLING
#define ACCUMULATION_TIME 5                 //accumulation time in seconds when EGY_TIME=7999, accumulation mode= sample based
#define EGY_INTERRUPT_MASK0 0x00000001      //Enable EGYRDY interrupt

enum CAL_STATE
{
  CAL_START,
  CAL_VI_CALIBRATE,
  CAL_PHASE_CALIBRATE,
  CAL_PGAIN_CALIBRATE,
  CAL_STORE,
  CAL_STOP,
  CAL_RESTART,
  CAL_COMPLETE
};

extern enum CAL_STATE CUR_STATE;   //current state is start


#endif /* INC_ADE9000_CALIBRATION_H_ */
