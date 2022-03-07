 /*Calibrates ADE9000*/
/*The calibration inputs are stored in the ADE9000CalibrationInputs.h file. The phase and parameter being calibrated is input through the serial console*/
/*Calbration constants are computed and stored in the EEPROM */
/*Caibration should be done with the end application settings. If any  parameters(GAIN,High pass corner,Integrator settings) are changed, the device should be recalibrated*/
/*This application assumues the PGA_GAIN among all current channels is same.Also, the PGA_GAIN among all voltage channels should be same*/

#include <ADE9000_Calibration.h>

int32_t xIgain_registers[IGAIN_CAL_REG_SIZE];   //order [AIGAIN, BIGAIN, CIGAIN, NIGAIN]
int32_t xIgain_register_address[IGAIN_CAL_REG_SIZE]=
         {ADDR_AIGAIN, ADDR_BIGAIN, ADDR_CIGAIN, ADDR_NIGAIN};   //order [AIGAIN, BIGAIN, CIGAIN, NIGAIN]
int32_t xIrms_registers[IGAIN_CAL_REG_SIZE];
int32_t xIrms_registers_address[IGAIN_CAL_REG_SIZE]= {ADDR_AIRMS, ADDR_BIRMS, ADDR_CIRMS, ADDR_NIRMS};


int32_t xVgain_registers[VGAIN_CAL_REG_SIZE];   //order [AVGAIN, BVGAIN, CVGAIN]
int32_t xVgain_register_address[VGAIN_CAL_REG_SIZE]={ADDR_AVGAIN, ADDR_BVGAIN, ADDR_CVGAIN};   //order [AVGAIN, BVGAIN, CVGAIN]
int32_t xVrms_registers[VGAIN_CAL_REG_SIZE];
int32_t xVrms_registers_address[VGAIN_CAL_REG_SIZE]= {ADDR_AVRMS, ADDR_BVRMS, ADDR_CVRMS};

int32_t xPhcal_registers[PHCAL_CAL_REG_SIZE];   //order [APHCAL, BPHCAL, CPHCAL]
int32_t xPhcal_register_address[PHCAL_CAL_REG_SIZE]={ADDR_APHCAL0, ADDR_BPHCAL0, ADDR_CPHCAL0};   //order [APHCAL, BPHCAL, CPHCAL]
int32_t xWATTHRHI_registers[PHCAL_CAL_REG_SIZE];  //Active energy registers
int32_t xWATTHRHI_registers_address[PHCAL_CAL_REG_SIZE]= {ADDR_AWATTHR_HI, ADDR_BWATTHR_HI, ADDR_CWATTHR_HI};
int32_t xVARHRHI_registers[PHCAL_CAL_REG_SIZE];
int32_t xVARHRHI_registers_address[PHCAL_CAL_REG_SIZE]= {ADDR_AVARHR_HI, ADDR_BVARHR_HI, ADDR_CVARHR_HI};

int32_t xPgain_registers[PGAIN_CAL_REG_SIZE];   //order [APGAIN, BPGAIN, CPGAIN]
int32_t xPgain_register_address[PGAIN_CAL_REG_SIZE]={ADDR_APGAIN, ADDR_BPGAIN, ADDR_CPGAIN};   //order [AVGAIN, BVGAIN, CVGAIN, NVGAIN]
//The Power gain calibration reads active energy registers. The content and address arrays are defined in the PHCAL section above

//Global variables
int8_t calCurrentPGA_gain=0;
int8_t calVoltagePGA_gain=0;
int32_t accumulatedActiveEnergy_registers[EGY_REG_SIZE];
int32_t accumulatedReactiveEnergy_registers[EGY_REG_SIZE];
uint32_t calibrationDataToEEPROM[CALIBRATION_CONSTANTS_ARRAY_SIZE];


enum CAL_STATE CUR_STATE = CAL_START;   //current state is start

void ADE9000_Calibration(){
	  calibrationEnergyRegisterSetup();
	  getPGA_gain();
	// ade9000.writeByteToEeprom(ADDR_EEPROM_WRITTEN_BYTE,~(EEPROM_WRITTEN)); //clear calibration done status
	  HAL_Delay(1000);
	  ADE9000_calibrate();
}


void ADE9000_calibrate()
{
  float calPf ;
  char serialReadData;
  static int8_t calChannel = 0; //the channel being calibrated
  static int8_t channelCalLength = 1; //the length

  switch(CUR_STATE)
  {
      case CAL_START:       //Start
      printf("Starting calibration process. Select one of the following {Start (Y/y) OR Abort(Q/q) OR Restart (R/r)}:\r\n");
      scanf("%c",&serialReadData);
      if(serialReadData == 'Y' || serialReadData == 'y')
        {
           printf("Enter the Phase being calibrated:{Phase A(A/a) OR Phase B(B/b) OR Phase C(C/c) OR Neutral(N/n) OR All Phases(D/d)}:\r\n");
           scanf("%c",&serialReadData);
           if(serialReadData == 'A' || serialReadData == 'a')
              {
                printf("Calibrating Phase A:\r\n");
                calChannel=0;
                channelCalLength=1;
              }
           else if(serialReadData == 'B' || serialReadData == 'b')
              {
                printf("Calibrating Phase B:\r\n");
                calChannel=1;
                channelCalLength=1;
              }
           else if(serialReadData == 'C' || serialReadData == 'c')
              {
                printf("Calibrating Phase C:\r\n");
                calChannel=2;
                channelCalLength=1;
              }
           else if(serialReadData == 'N' || serialReadData == 'n')
              {
               // calChannel=0;
              //  channelCalLength=1;
              }
           else if(serialReadData == 'D' || serialReadData == 'd')
              {
                printf("Calibrating all phases:\r\n");
                calChannel=0; //Start from channel A
                channelCalLength=3;
              }
           else
              {
                printf("Wrong input\r\n");
                serialReadData=' ';
                break;
              }

           CUR_STATE=CAL_VI_CALIBRATE;
           printf("Starting calibration with %d Vrms and %d Arms\r\n",NOMINAL_INPUT_VOLTAGE,NOMINAL_INPUT_CURRENT);
           serialReadData=' ';
           break;
        }
      else
        {
           if(serialReadData == 'Q' || serialReadData == 'q')
              {
                CUR_STATE=CAL_STOP;
                printf("Aborting calibration\r\n");
               serialReadData=' ';
               break;
              }
            else if(serialReadData == 'R' || serialReadData == 'r')
              {
                 CUR_STATE=CAL_RESTART;
                  serialReadData=' ';
                  break;
              }
           else
             {
                printf("Wrong input\r\n");
                serialReadData=' ';
                break;
             }
        }

      break;

      case CAL_VI_CALIBRATE:   //Calibrate
      ADE9000_iGain_calibrate(&xIgain_registers[calChannel],&xIgain_register_address[calChannel], &xIrms_registers[calChannel], &xIrms_registers_address[calChannel], channelCalLength);       //Calculate xIGAIN
      ADE9000_vGain_calibrate(&xVgain_registers[calChannel], &xVgain_register_address[calChannel], &xVrms_registers[calChannel], &xVrms_registers_address[calChannel], channelCalLength);       //Calculate xVGAIN
      printf("Current gain calibration completed\r\n");
      printf("Voltage gain calibration completed\r\n");
      CUR_STATE=CAL_PHASE_CALIBRATE;
      break;

      case CAL_PHASE_CALIBRATE:
      printf("Perform Phase calibration: Yes(Y/y) OR No (N/n): \r\n");
      scanf("%c",&serialReadData);
      if(serialReadData == 'Y' || serialReadData == 'y')
        {
            printf("Ensure Power factor is 0.5 lagging such that Active and Reactive energies are positive: Continue: Yes(Y/y) OR Restart (R/r): \r\n");
            scanf("%c",&serialReadData);
            if(serialReadData == 'Y' || serialReadData == 'y')
              {
            	ADE9000_phase_calibrate(&xPhcal_registers[calChannel],&xPhcal_register_address[calChannel], &accumulatedActiveEnergy_registers[calChannel], &accumulatedReactiveEnergy_registers[calChannel], channelCalLength);     //Calculate xPHCAL
               printf("Phase calibration completed\r\n");
              }
            else
              {
                if(serialReadData == 'R' || serialReadData == 'r')
                  {
                    CUR_STATE=CAL_RESTART;
                    break;
                  }
                  else
                  {
                    printf("Wrong Input\r\n");
                    break;
                  }
              }
          }
      else
        {
          if(serialReadData == 'N' || serialReadData == 'n')
            {
                printf("Skipping phase calibration \r\n");
            }
          else
            {
              printf("Wrong input\r\n");
              break;
             }
        }
      CUR_STATE = CAL_PGAIN_CALIBRATE;
      break;

      case CAL_PGAIN_CALIBRATE:
      printf("Starting Power Gain calibration\r\n");
      printf("Enter the Power Factor of inputs for xPGAIN calculation: 1(1) OR CalibratingAnglePF(0): \r\n");
      scanf("%c",&serialReadData);
      if(serialReadData == '1')
        {
           calPf=1;
        }
      else
        {
          if(serialReadData == '0')
            {
              calPf=CAL_ANGLE_RADIANS(CALIBRATION_ANGLE_DEGREES);
            }
          else
          {
             printf("Wrong input\r\n");
             break;
          }

        }
      ADE9000_pGain_calibrate(&xPgain_registers[calChannel],&xPgain_register_address[calChannel],&accumulatedActiveEnergy_registers[calChannel],channelCalLength, calPf);
      printf("Power gain calibration completed \r\n");
      //printf("Calibration completed. Storing calibration constants to EEPROM \r\n");
      CUR_STATE = CAL_STORE;
      break;

      case CAL_STORE:     //Store Constants to EEPROM
      //storeCalConstToEEPROM();
      //Serial.println("Calibration constants successfully stored in EEPROM. Exit Application");
      CUR_STATE = CAL_COMPLETE;
      break;

      case CAL_STOP:    //Stop calibration
      printf("Calibration stopped. Restart Arduino to recalibrate\r\n");
      CUR_STATE = CAL_COMPLETE;
      break;

      case CAL_RESTART:    //restart
      printf("Restarting calibration\r\n");
      CUR_STATE = CAL_START;
      break;

      case CAL_COMPLETE:
      break;

      default:
      break;

  }

}


void ADE9000_iGain_calibrate(int32_t *igainReg, int32_t *igainRegAddress, int32_t *iRmsReg, int32_t *iRmsRegAddress, int arraySize)
{
  float temp;
  int32_t actualCodes;
  int32_t expectedCodes;
  int i;

  temp=ADE9000_RMS_FULL_SCALE_CODES*CURRENT_TRANSFER_FUNCTION*calCurrentPGA_gain*NOMINAL_INPUT_CURRENT *sqrt(2);
  expectedCodes= (int32_t) temp;  //Round off
  printf("Expected IRMS Code: %x\r\n",expectedCodes);
  for (i=0; i < arraySize ;i++)
    {
      actualCodes = ADE9000_SPI_Read_32(iRmsRegAddress[i]);
      temp= (((float)expectedCodes/(float)actualCodes)-1)* 134217728;  //calculate the gain.
      igainReg[i] = (int32_t) temp; //Round off
      printf("Channel %d\r\n",i+1);
      printf("Actual IRMS Code: %x ",actualCodes);
      printf("Current Gain Register: %x \r\n",igainReg[i]);

    }
}

void ADE9000_vGain_calibrate(int32_t *vgainReg, int32_t *vgainRegAddress, int32_t *vRmsReg, int32_t *vRmsRegAddress, int arraySize)
{
  float temp;
  int32_t actualCodes;
  int32_t expectedCodes;
  int i;

  temp=ADE9000_RMS_FULL_SCALE_CODES*VOLTAGE_TRANSFER_FUNCTION*calVoltagePGA_gain*NOMINAL_INPUT_VOLTAGE*sqrt(2);
  expectedCodes= (int32_t) temp;  //Round off
  printf("Expected VRMS Code: %x \r\n",expectedCodes);
  for (i=0; i < arraySize ;i++)
    {
      actualCodes = ADE9000_SPI_Read_32(vRmsRegAddress[i]);
      temp= (((float)expectedCodes/(float)actualCodes)-1)* 134217728;  //calculate the gain.
      vgainReg[i] = (int32_t) temp; //Round off
      printf("Channel %d \r\n",i+1);
      printf("Actual VRMS Code: %x \r\n",actualCodes);
      printf("Voltage Gain Register: %x \r\n",vgainReg[i]);
    }
}

void ADE9000_phase_calibrate(int32_t *phcalReg,int32_t *phcalRegAddress,int32_t *accActiveEgyReg,int32_t *accReactiveEgyReg, int arraySize)
{
  printf("Computing phase calibration registers...\r\n");
  HAL_Delay((ACCUMULATION_TIME+1)*1000); //delay to ensure the energy registers are accumulated for defined interval
  float errorAngle;
  float errorAngleDeg;
  float omega;
  double temp;
  int32_t actualActiveEnergyCode;
  int32_t actualReactiveEnergyCode;
  int i;
  omega = (float)2 *(float)3.14159*(float) INPUT_FREQUENCY /(float)ADE90xx_FDSP;


  for (i=0; i < arraySize ;i++)
    {
        actualActiveEnergyCode = accActiveEgyReg[i];
        actualReactiveEnergyCode = accReactiveEgyReg[i];
        errorAngle = (double)-1 * atan( ((double)actualActiveEnergyCode*(double)sin(CAL_ANGLE_RADIANS(CALIBRATION_ANGLE_DEGREES))-(double)actualReactiveEnergyCode*(double)cos(CAL_ANGLE_RADIANS(CALIBRATION_ANGLE_DEGREES)))/((double)actualActiveEnergyCode*(double)cos(CAL_ANGLE_RADIANS(CALIBRATION_ANGLE_DEGREES))+(double)actualReactiveEnergyCode*(double)sin(CAL_ANGLE_RADIANS(CALIBRATION_ANGLE_DEGREES))));
        temp = (((double)sin((double)errorAngle-(double)omega)+(double)sin((double)omega))/((double)sin(2*(double)omega-(double)errorAngle)))*134217728;
        phcalReg[i]= (int32_t)temp;
        errorAngleDeg = (float)errorAngle*180/3.14159;
        printf("Channel %d \r\n",i+1);
        printf("Actual Active Energy Register: %x \r\n",actualActiveEnergyCode);
        printf("Actual Reactive Energy Register: %x \r\n",actualReactiveEnergyCode);
        printf("Phase Correction (degrees): %f \r\n",errorAngleDeg);
        printf("Phase Register: %x \r\n ",phcalReg[i]);
    }



}

void ADE9000_pGain_calibrate(int32_t *pgainReg, int32_t *pgainRegAddress, int32_t *accActiveEgyReg, int arraySize, float pGaincalPF)
{
  printf("Computing power gain calibration registers...\r\n");
  HAL_Delay((ACCUMULATION_TIME+1)*1000); //delay to ensure the energy registers are accumulated for defined interval
  int32_t expectedActiveEnergyCode;
  int32_t actualActiveEnergyCode;
  int i;
  float temp;
  temp = ((float)ADE90xx_FDSP * (float)NOMINAL_INPUT_VOLTAGE * (float)NOMINAL_INPUT_CURRENT * (float)CALIBRATION_ACC_TIME * (float)CURRENT_TRANSFER_FUNCTION *(float)calCurrentPGA_gain* (float)VOLTAGE_TRANSFER_FUNCTION *(float)calVoltagePGA_gain* (float)ADE9000_WATT_FULL_SCALE_CODES * 2 * (float)(pGaincalPF))/(float)(8192);
  expectedActiveEnergyCode = (int32_t)temp;
  printf("Expected Active Energy Code: %x \r\n ",expectedActiveEnergyCode);

  for (i=0; i < arraySize ;i++)
    {
      actualActiveEnergyCode = accActiveEgyReg[i];

      temp= (((float)expectedActiveEnergyCode/(float)actualActiveEnergyCode)-1)* 134217728;  //calculate the gain.
      pgainReg[i] = (int32_t) temp; //Round off
      printf("Channel %d \r\n",i+1);
      printf("Actual Active Energy Code: %x \r\n",actualActiveEnergyCode);
      printf("Power Gain Register: %x\r\n ",pgainReg[i]);
    }
}

void calibrationEnergyRegisterSetup()
{
  uint16_t epcfgRegister;
  ADE9000_SPI_Write_32(ADDR_MASK0,EGY_INTERRUPT_MASK0);   //Enable EGYRDY interrupt
  ADE9000_SPI_Write_16(ADDR_EGY_TIME,EGYACCTIME);   //accumulate EGY_TIME+1 samples (8000 = 1sec)
  epcfgRegister =  ADE9000_SPI_Read_16(ADDR_EP_CFG);   //Read EP_CFG register
  epcfgRegister |= CALIBRATION_EGY_CFG;                //Write the settings and enable accumulation
  ADE9000_SPI_Write_16(ADDR_EP_CFG,epcfgRegister);
  HAL_Delay(2000);
  ADE9000_SPI_Write_32(ADDR_STATUS0,0xFFFFFFFF);
  //attachInterrupt(digitalPinToInterrupt(IRQ0_INTERRUPT_PIN),updateEnergyRegisterFromInterrupt,INT_MODE);
}


void getPGA_gain()
{
  int16_t pgaGainRegister;
  int16_t temp;
  pgaGainRegister = ADE9000_SPI_Read_16(ADDR_PGA_GAIN);  //Ensure PGA_GAIN is set correctly in SetupADE9000 function.
  printf("PGA Gain Register is: %x \r\n",pgaGainRegister);
  temp =    pgaGainRegister & (0x0003);  //extract gain of current channel
  if (temp == 0)  // 00-->Gain 1: 01-->Gain 2: 10/11-->Gain 4
      {
        calCurrentPGA_gain =1;
      }
  else
      {
        if(temp==1)
        {
         calCurrentPGA_gain =2;
        }
        else
        {
         calCurrentPGA_gain =4;
        }
      }
  temp =    (pgaGainRegister>>8) & (0x0003); //extract gain of voltage channel
  if (temp == 0)
      {
        calVoltagePGA_gain =1;
      }
  else
      {
        if(temp==1)
        {
         calVoltagePGA_gain =2;
        }
        else
        {
         calVoltagePGA_gain =4;
        }
      }
}

/*
void storeCalConstToEEPROM()
{
  //Arrange the data as formatted in 'ADE9000_Eeprom_CalibrationRegAddress' array.
  int8_t i;
  uint32_t temp;
  uint32_t checksum=0; //  adds all the gain and phase calibration registers. The truncated 32 bit data is stored as checksum in EEPROM.

  for(i=0;i<IGAIN_CAL_REG_SIZE;i++) //arrange current gain calibration registers
     {
       calibrationDataToEEPROM[i]=xIgain_registers[i];

     }
  for(i=0;i<VGAIN_CAL_REG_SIZE;i++) //arrange voltage gain calibration registers
     {
       calibrationDataToEEPROM[i+IGAIN_CAL_REG_SIZE]=xVgain_registers[i];

     }
  for(i=0;i<PHCAL_CAL_REG_SIZE;i++) //arrange phase calibration registers
     {
       calibrationDataToEEPROM[i+IGAIN_CAL_REG_SIZE+VGAIN_CAL_REG_SIZE]=xPhcal_registers[i];
     }
  for(i=0;i<PGAIN_CAL_REG_SIZE;i++) //arrange phase calibration registers
     {
       calibrationDataToEEPROM[i+IGAIN_CAL_REG_SIZE+VGAIN_CAL_REG_SIZE+PHCAL_CAL_REG_SIZE]=xPgain_registers[i];
     }

  for(i=0;i<CALIBRATION_CONSTANTS_ARRAY_SIZE;i++)
     {
       checksum +=calibrationDataToEEPROM[i];
     }




  for(i=0;i<CALIBRATION_CONSTANTS_ARRAY_SIZE;i++)
     {
       ade9000.writeWordToEeprom(ADE9000_Eeprom_CalibrationRegAddress[i],calibrationDataToEEPROM[i]);
       delay(10);
     }
  for(i=0;i<CALIBRATION_CONSTANTS_ARRAY_SIZE;i++)
     {
       temp= ade9000.readWordFromEeprom(ADE9000_Eeprom_CalibrationRegAddress[i]);
       delay(10);
       Serial.println(temp,HEX);
     }
  ade9000.writeWordToEeprom(ADDR_CHECKSUM_EEPROM,checksum);           //Save checksum to EEPROM
  ade9000.writeByteToEeprom(ADDR_EEPROM_WRITTEN_BYTE,EEPROM_WRITTEN); //Save calibration status in EEPROM


}



void updateEnergyRegisterFromInterrupt()
{
  int8_t i;
  static int8_t count=0;
  static int32_t intermediateActiveEgy_Reg[EGY_REG_SIZE]={0};
  static int32_t intermediateReactiveEgy_Reg[EGY_REG_SIZE]={0};
  uint32_t temp;
  temp = ade9000.SPI_Read_32(ADDR_STATUS0);
  temp&=EGY_INTERRUPT_MASK0;
  if (temp==EGY_INTERRUPT_MASK0)
  {
      ade9000.SPI_Write_32(ADDR_STATUS0,0xFFFFFFFF);
      for(i=0;i<EGY_REG_SIZE;i++)
      {
        intermediateActiveEgy_Reg[i]+=ade9000.SPI_Read_32(xWATTHRHI_registers_address[i]);  //accumulate the registers
        intermediateReactiveEgy_Reg[i]+=ade9000.SPI_Read_32(xVARHRHI_registers_address[i]);   //accumulate the registers
      }

      if (count == (ACCUMULATION_TIME-1))  //if the accumulation time is reached, update the final values to registers
        {
          for(i=0;i<EGY_REG_SIZE;i++)
              {
                accumulatedActiveEnergy_registers[i] = intermediateActiveEgy_Reg[i];
                accumulatedReactiveEnergy_registers[i] = intermediateReactiveEgy_Reg[i];
                intermediateActiveEgy_Reg[i]=0;  // Reset the intermediate registers
                intermediateReactiveEgy_Reg[i]=0;   //Reset the intermediate registers
              }
          count=0;   //Reset counter
          return;   //exit function
        }
     count++;
     return;
  }


}
*/



