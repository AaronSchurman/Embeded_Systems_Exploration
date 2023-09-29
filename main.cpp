#include <Arduino.h>
#include <Controllino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Modbus.h>
#include <ModbusIP.h>

/*
PLC Registers for the Controllino MEGA

Digital Outputs (COILS):
1 : D12 : Liquid Level LED 1
2 : D13 : Liquid Level LED 2
3 : D14 : Liquid Level LED 3
4 : D15 : Liquid Level LED 4
5 : D16 : Liquid Level LED 5
6 : D17 : Unassigned
7 : D18 : Unassigned
8 : D19 : Unassigned
9 : D20 : Solenoid valve
10 : D21 : Peristaltic pump
11 : D22 : Unassigned
12 : D23 : Unassigned
13 : R0 : Unassigned
14 : R1 : Unassigned
15 : R2 : Unassigned
16 : R3 : Unassigned
17 : R4 : Unassigned
18 : R5 : Unassigned
19 : R6 : Unassigned
20 : R7 : Unassigned
21 : R8 : Unassigned
22 : R9 : Unassigned
23 : R10 : Unassigned
24 : R11 : Unassigned
25 : R12 : Unassigned
26 : R13 : Unassigned
27 : R14 : Unassigned
28 : R15 : Unassigned
29 : D1 (X1-13) : MOTOR Step Direction
30 : N/A : STOP (0) and START(1) motor

Digital Inputs
10001 : IN0 : liquid level photo transistor 1
10002 : IN1 : liquid level photo transistor 2
10003 : A10 : liquid level photo transistor 3
10004 : A11 : liquid level photo transistor 4
10005 : A12 : liquid level photo transistor 5
10006 : A13 : Unassigned
10007 : A14 : Unassigned
10008 : A15 : Unassigned
10009 : I16 : Unassigned
10010 : I17 : Unassigned
10011 : I18 : Unassigned

Analog Outputs (PWM)
20004 : N/A : 0-93 : RPM Settting of stepper diver

Analog Inputs
30001 : A0 : 0-1024 : liquid level photo transistor 1
30002 : A1 : 0-1024 : liquid level photo transistor 2
30003 : A2 : 0-1024 : liquid level photo transistor 3
30004 : A3 : 0-1024 : liquid level photo transistor 4
30005 : A4 : 0-1024 : liquid level photo transistor 5
30006 : A5 : 0-1024 : Unassigned
30007 : A6 : 0-1024 : Unassigned
30008 : A7 : 0-1024 : Unassigned
30009 : A8 : 0-1024 : Unassigned
30010 : A9 : 0-1024 : Unassigned

PWM for pump:
NOTE: the stepper driver is set to 8 microsteps.  MAX RPM is 200.
See excel spreadsheet in the github repo titled: PWM_calc_spreadsheet_3M_pump

2021-1-6 UPDATE
* Was getting modbus comm errors from labview.  Assuming it was because the
PLC is dropping off of the ethernet IP addess so added a renewal every 5
minutes to ensure stays on.  This was used in previous projects such as the
MECHROOM_PLC code:
(https://github.com/mjschurman/MechRoomPLC/blob/master/arduino_ethernetAliveTest/
arduino_ethernetAliveTest.ino).
See that code for details.

*/

// *********************** Declare Const and Vars *************************

// defines
#define num_o_DO 30
#define num_o_AO 1  \\ not used, here from legacy code
#define num_o_DI 11
#define num_o_AI 10
#define MOTOR_Direction_Pin CONTROLLINO_D1
#define MOTOR_ENABLE_Pin CONTROLLINO_D0
#define motorStepPin CONTROLLINO_D3
#define millisBtwUpdateConnection 300000 // milliseconds between periodic
                                       // update of connection of mnodbus
                                       // due to occasional drop off of the
                                       // modbus from the ethernet

// Variables
unsigned long timeDiff;
unsigned long counterStart = 0;  // start of measurment timer for TCP re-connect
//The media access control (ethernet hardware) address for the shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// The IP address for the shield
//byte ip[] = { 192, 168, 1, 211}; 
byte ip[] = { 10,1,10,102};

// Constants for PINS and Modbus Regs
const int PLC_DO[] = {CONTROLLINO_D12,CONTROLLINO_D13,CONTROLLINO_D14,
                      CONTROLLINO_D15,CONTROLLINO_D16,CONTROLLINO_D17,
                      CONTROLLINO_D18,CONTROLLINO_D19,CONTROLLINO_D20,
                      CONTROLLINO_D21,CONTROLLINO_D22,CONTROLLINO_D23,
                      CONTROLLINO_R0,CONTROLLINO_R1,CONTROLLINO_R2,
                      CONTROLLINO_R3,CONTROLLINO_R4,CONTROLLINO_R5,
                      CONTROLLINO_R6,CONTROLLINO_R7,CONTROLLINO_R8,
                      CONTROLLINO_R9,CONTROLLINO_R10,CONTROLLINO_R11,
                      CONTROLLINO_R12,CONTROLLINO_R13,CONTROLLINO_R14,
                      CONTROLLINO_R15, CONTROLLINO_D1, CONTROLLINO_D0};

const int coilRegs[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28, 29, 30};

const int PLC_DI[] = {CONTROLLINO_IN0,CONTROLLINO_IN1,CONTROLLINO_A10,CONTROLLINO_A11,
                      CONTROLLINO_A12,CONTROLLINO_A13,CONTROLLINO_A14,CONTROLLINO_A15,
                      CONTROLLINO_I16,CONTROLLINO_I17,CONTROLLINO_I18};

const int DI_Regs[] = {10001,10002,10003,10004,10005,10006,10007,10008,10009,10010,10011};


const int AO_Regs[] = {20004};


const int PLC_AI[] = {CONTROLLINO_A0,CONTROLLINO_A1,CONTROLLINO_A2,CONTROLLINO_A3,
                      CONTROLLINO_A4,CONTROLLINO_A5,CONTROLLINO_A6,CONTROLLINO_A7,
                      CONTROLLINO_A8,CONTROLLINO_A9};

const int AI_Regs[] = {30001,30002,30003,30004,30005,30006,30007,30008,30009,30010};

// PWM timer settings for motor
// 6 to 198 RPM in 2 RPM increments with prescaler set to 1
const int motoRotationSpeeds[94] = {25000,21429,18750,16667,15000,13636,12500,11538,
                                    10714,10000,9375,8824,8333,7895,7500,7143,6818,
                                    6522,6250,6000,5769,5556,5357,5172,5000,4839,4688,
                                    4545,4412,4286,4167,4054,3947,3846,3750,3659,
                                    3571,3488,3409,3333,3261,3191,3125,3061,3000,
                                    2941,2885,2830,2778,2727,2679,2632,2586,2542,
                                    2500,2459,2419,2381,2344,2308,2273,2239,2206,
                                    2174,2143,2113,2083,2055,2027,2000,1974,1948,
                                    1923,1899,1875,1852,1829,1807,1786,1765,1744,
                                    1724,1705,1685,1667,1648,1630,1613,1596,1579,
                                    1563,1546,1531,1515};


// ADC related vars
const int num_to_average = 16; // number of ADC measurements to average together
volatile int adcVals[10], numOfADC;
bool ADC_readyFlag = 0;


// *********************** ModbusIP object *************************
ModbusIP mb;


// *********************** FUNCTIONS *************************

void setupMotorPWM(){
  /*
  This code is based on:
  https://github.com/mjschurman/CoffeeProjectCode/tree/main/PLC%20Code/PWMTest_controllino


  note that the frequency of the PWM is set by:
  PWM_freq = clockFreq / (2 * N * TOP)
    where:
    PWM_freq: the PWM frequency
    clockFreq: crystal freq (16 MHz)
    N: Prescaler used
    TOP: in this case the ICR1 Register is being used

  The DUTY CYCLE OF THE PWM is set by:
  The Ratio of OCR1A and ICR1 Register value
  
  
  
  */
  TCCR3A=_BV(COM3A1)|_BV(COM3B1); /* set Fast PWM Mode */
  TCCR3B=_BV(WGM33)|_BV(CS30); /* Activate PWM Phase, frequency correction Mode */
    // CS30 Bits set prescaler to 1

}

void setNewMotorRotationSpeed(){
  /*
  see the following notes in evernote:
  https://www.evernote.com/shard/s145/nl/16869466/2b2d325e-c5fd-d9f0-1283-dd74ff920715?title=Taking%20the%20Roaster%20to%2011
  https://www.evernote.com/shard/s145/nl/16869466/efcfd525-05ca-48d0-9e8b-8862e516157f?title=Arduino%20PWM%20:%20Generate%20Fix%20and%20Variable%20Frequency%20Duty%20Cycle%20Signal
  https://www.evernote.com/shard/s145/nl/16869466/b5c569c2-2e2f-3666-a0eb-00bc73b76041?title=Calculations%20for%20Controlling%20the%20Stepper%20for%20the%20Pump
  https://www.evernote.com/shard/s145/nl/16869466/56eead90-2912-ac9d-1afd-e087d232382c?title=Low%20Voltage%20Driver%20for%20Pump%20Stepper

  This controls the rotation speed of the motor assuming
    1) full steps
    2) 200 steps / 1 revolution
    3) set to microsatep 8
  Valid settings are located in the variable: motoRotationSpeeds
  
  */
  int myVal = mb.Hreg(2004);
  // make sure register 20004 value is valid!
  if (myVal > 93){
    myVal = 93;
    mb.Hreg(2004, 93);
  } 
  ICR3 = motoRotationSpeeds[myVal];
  OCR3A =int(motoRotationSpeeds[myVal]/4);  // This sets the duty cycle
  // set motor direction
  digitalWrite(MOTOR_Direction_Pin, mb.Coil(29));
  //enable motor
  if(mb.Coil(30)){
    digitalWrite(MOTOR_ENABLE_Pin, true);
    TCCR3A=_BV(COM3A1)|_BV(COM3B1); /* set Fast PWM Mode */
  }
  else{
    digitalWrite(MOTOR_ENABLE_Pin, false);
    TCCR3A = 0;
  }
  
}




void updateTheDO(){
  for (int i = 0; i <= num_o_DO-1; i++){
    digitalWrite(PLC_DO[i], mb.Coil(coilRegs[i]));
  }
}

void updateTheDI(){
for (int i = 0; i <= num_o_DI-1; i++){
    mb.Ists(DI_Regs[i], digitalRead(PLC_DI[i]));
  }
}

// setup the ADC for continuous aquisition
void setupADC(){
  /*
  This sets up the ADC to continuously collect data
  starting with ADC0.  See freeRunADC_test project
  in the PLC code folder on github for more details:
  https://github.com/mjschurman/CoffeeProjectCode/tree/main/PLC%20Code/freeRunADC_test
  */
  ADCSRA = 0; // reset the register
  // ADCSRA |= 1<<ADPS2; // set prescaler to 16
  ADCSRA = _BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0); //set prescaler to 128
  ADCSRA |= 1<<ADIE; // enable interrupts
  ADCSRA |= 1<<ADEN; // enable ADC
  ADCSRA |= 1<<ADSC; // start ADC
  ADMUX=0x40; // set to channel 0
}

ISR(ADC_vect)
  {
  // The following composes the 10 bit number
  uint8_t theLow = ADCL;
  int theTenBitResult = ADCH<<8 | theLow;
  // enable switching between analog ports 0 - 9
  // NOTE:  NEED TO TOGGLE ADCSRB MUX5 BIT TO GET TO ADC chan 8
  // AND 9 SO THERE NEEEDS TO BE SOME LOGIC TO CATCH THAT FOR
  // CASE 0X40 AND 0X41 BELOW!!!!!
  switch (ADMUX)
  {
    case 0x40:
      // ADCSRB MUX5 bit controls if the MUX starts at ADC0
      // or ADC8.  IF MUX5 is 0 and ADMUX=0x40, then pointing at ADC0
      if(ADCSRB==0){
        adcVals[0]+= (int)theTenBitResult;
      }
      else{
        adcVals[8]+= (int)theTenBitResult;
      }
      ADMUX = 0x41;
      ADCSRA |= 1<<ADSC; // start ADC conversion
      break;
    case 0x41:
      // ADCSRB MUX5 bit controls if the MUX starts at ADC0
      // or ADC8.  IF MUX5 is 0 and ADMUX=0x41, then pointing at ADC1
      if(ADCSRB==0){
      adcVals[1]+= (int)theTenBitResult;
      ADMUX = 0x42;
      ADCSRA |= 1<<ADSC; // start ADC conversion
      }
      else{
        adcVals[9]+= (int)theTenBitResult;
        ADMUX = 0x40;  //back to 0 for ADC0
        ADCSRB ^= (_BV(MUX5)); //flip bit back to 0 for ADC0
        if(numOfADC==num_to_average-1){
          numOfADC = 0;
          ADC_readyFlag = true;
          cli();  // shut down interrupts
        }
        else{
          numOfADC+=1;
          ADCSRA |= 1<<ADSC; // start ADC conversion
        }
      }
      break;
    case 0x42:
      adcVals[2]+= (int)theTenBitResult;
      ADMUX = 0x43;
      ADCSRA |= 1<<ADSC; // start ADC conversion
      break;
    case 0x43:
      adcVals[3]+= (int)theTenBitResult;
      ADMUX = 0x44;
      ADCSRA |= 1<<ADSC; // start ADC conversion
      break;
    case 0x44:
      adcVals[4]+= (int)theTenBitResult;
      ADMUX = 0x45;
      ADCSRA |= 1<<ADSC; // start ADC conversion
      break;
    case 0x45:
      adcVals[5]+= (int)theTenBitResult;
      ADMUX = 0x46;
      ADCSRA |= 1<<ADSC; // start ADC conversion
      break;
    case 0x46:
      adcVals[6]+= (int)theTenBitResult;
      ADMUX = 0x47;
      ADCSRA |= 1<<ADSC; // start ADC conversion
      break;
    case 0x47:
      adcVals[7]+= (int)theTenBitResult;
      ADMUX = 0x40; // back to 0 for ADC8
      ADCSRB = _BV(MUX5); // flip bit to TRUE to switch to ADC 8 and 9
      ADCSRA |= 1<<ADSC; // start ADC conversion
      break;
    default:
      //Default code
      break;
  }
}

void updateTheAI_regs(){
  for (int i = 0; i <= num_o_AI-1; i++){
    mb.Ireg(AI_Regs[i], adcVals[i]>>4);  // The >>4 divides by 16
    adcVals[i]=0;
    // mb.Ireg(AI_Regs[i], 1023);
    ADC_readyFlag = false;
  }
  sei();  // renable interrupts
  ADCSRA |= 1<<ADSC; // start ADC conversion
}




void setup() {
  Serial.begin(9600);
// ************************ Setup the Ethernet for MEGA ***********************
   
    //Config Modbus IP 
    mb.config(mac, ip);

// ************ Setup Digital Output Pins and coil registers ************
  for (int i = 0; i <= num_o_DO-1; i++){
    pinMode(PLC_DO[i], OUTPUT);
    mb.addCoil(coilRegs[i]);
  }
  // ************ Setup Analog Output Pins and AO registers ************
  // for (int i = 0; i <= num_o_AO-1; i++){
  //   // pinMode(PLC_AO[i], OUTPUT);  NOTE no analog outputs for this application
  //   mb.addHreg(AO_Regs[i]);
  // }
  mb.addHreg(2004);
  pinMode(motorStepPin, OUTPUT);

    // ************ Setup Analog Input Pins and AI registers ************
  for (int i = 0; i <= num_o_AI-1; i++){
    pinMode(PLC_AI[i], INPUT);
    mb.addIreg(AI_Regs[i]);
  }

  // ************************ Setup ADC *****************************
  setupADC();

  // *********************** setup PWM ******************************
  setupMotorPWM();

  //************************ Enable Interrupt ***********************
  sei(); // enable ISR
  counterStart = millis();


}

void loop() {
  timeDiff = millis();

  // check to see if need to renew IP connection
  if((timeDiff-counterStart) > millisBtwUpdateConnection){
    //Config Modbus IP 
    mb.config(mac, ip);
    counterStart = millis();
  }
  mb.task();  // Update any MODBUS registers
  updateTheDO();
  updateTheDI();
  if(ADC_readyFlag==true){
    updateTheAI_regs();
  }
  // timeDiff = millis()-timeDiff;
  // Serial.println(timeDiff);
  setNewMotorRotationSpeed();
}