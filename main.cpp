
/*
  Driver board firmware
  Copyright Aaron C. Schurman
  uses the library: 
  https://github.com/emelianov/modbus-esp8266

  Target: ATMEGA328P

  Pins used ------------------------------------------------------
  Pin number      Function            Connected to
  Digital 0       RX                  RS485 Tranceiver (SN75LBC184P)
  Digital 1       TX                  RS485 Tranceiver (SN75LBC184P)
  Digital 2       DO                  SSR Swtich
  Digital 3       PWM                 PWM COntrol pin Driver 3, LDD-1500H
  Digital 5       PWM                 PWM COntrol pin Driver 2, LDD-1500H
  Digital 6       PWM                 PWM COntrol pin Driver 1, LDD-1500H
  Digital 9       PWM                 PWM Control pin Driver 4, LDD-1500H
  Digital 7       DO                  Header J5, pin 1, blue LED
  Digital 8       DO                  Header J5, pin 2, red LED
  Analog 0        Temp sensor         Header J5, pin 4
  Analog 1        Temp sensor         Header J5, pin 3

  RS485 Tranceiver notes ----------------------------------------
  * Still working on which tranciever and set up to use

 
MODBUS Registers -----------------------------------------------------
Register    Type        Read/Write      Connected to              Value
100         0x04        R/W             PWM, D6, Driver1          0 - 255
101         0x04        R/W             PWM, D5, Driver2          0 - 255
102         0x04        R/W             PWM, D3, Driver3          0 - 255
103         0x04        R/W             PWM, B1, Driver4          0 - 255 
200         0x03        R               temp sensor 1, A0, j5-4   0 - 1024  need to divide by 10 for C
201         0x03        R               temp sensor 2, A1, J5-3   0 - 1024  need to divide by 10 for C
202         0x04        R               {tempFlag information}    0 - 3 
300         0x05        R/W             dig-out, D7,J5-1, blue    0,1
301         0x05        R/W             dig-out, D8,J5-2, red     0,1
302         0x05        R/w             dig-out, SSR              0,1
400         0x05        R/W             Board over temperature Flag   0,1

tempFlag Ireg Value Lookup table ---------------------------------
Value       Meaning
0           No temp overload 
1           Temp sensor 1 (reg value 200) over temp
2           Temp sensor 2 (reg value 201) over temp
3           Both temp sensors over temp


*/

#include <Arduino.h>
#include <ModbusRTU.h>
// *********************** ModbusIP object *************************
ModbusRTU mb;

// *********************** Constant Vraiable Decleration *************************
//Slave ID 
#define SLAVE_ID 1

// Modbus Registers Offsets (0-9999)

// Holding registers 
const int Driver1 = 100;
const int Driver2 = 101;
const int Driver3 = 102;
const int Driver4 = 103;
const int tempOver_Setting = 110;

// Input Registers
const int TempSense1 = 200;
const int TempSense2 = 201;
const int tempOver   = 202;

// Coils
const int dig_out_red   = 300;
const int dig_out_blue  = 301;
const int SSR_fan       = 302;
const int tempOver_Flag = 400;


//pins on board arduino values

// Coils
const int Pin_DriverRED  = 7;
const int Pin_DriverBLUE = 8;
const int Pin_SSR        = 2;

// Input Registers
const int anologueRead_1_Pin = A0;
const int anologueRead_2_Pin = A1;

// Holding registers 
const int DriverPin1 = 6;
const int DriverPin2 = 5;
const int DriverPin3 = 3;
const int DriverPin4 = 9;


const int num_to_average = 16; //number of ADC values taken to average
volatile int adc1, adc2, numOfADC; //buckets fpr ADC values to be added into

bool ADC_readyFlag = 0; //tells when ADC is done taking the values it needs and the information can be given to the Iregs
bool TEMP_OVERLOAD = 0; // Tells if the temperature is overloaded

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

  ADMUX=0x40;

}

//Interupt service routine used to collect ADC values
ISR(ADC_vect)

  {

  // The following composes the 10 bit number

  uint8_t theLow = ADCL;

  int theTenBitResult = ADCH<<8 | theLow;

  // enable switching between analog ports 0 - 4

  switch (ADMUX)

  {

    case 0x40:

      adc1+= (int)theTenBitResult;

      ADMUX = 0x41;

      ADCSRA |= 1<<ADSC; // start ADC conversion

      break;

    case 0x41:

      adc2+= (int)theTenBitResult;

      ADMUX = 0x40;

      if(numOfADC==num_to_average-1){

        numOfADC = 0;

        ADC_readyFlag = true;

        cli();  // shut down interrupts

      }

      else{

        numOfADC+=1;

        ADCSRA |= 1<<ADSC; // start ADC conversion

      }

      break;

    default:

      //Default code

      break;

  }

}

//functin converts ADC value to degree Celcius
int getTemp(int val){
  int temp = 0;
  float ADC_val = (float) val;
   //calculations to turn ADC to volts and convert to milivolts
  float milivolts_read = ((ADC_val * 2.5)/1023)*1000;
  //calculation to turn milivolts read to temp in C found here - https://www.ti.com/lit/ds/symlink/tmp236.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1654003425437&ref_url=https%253A%252F%252Fwww.ti.com%252Fgeneral%252Fdocs%252Fsuppproductinfo.tsp%253FdistId%253D10%2526gotoUrl%253Dhttps%253A%252F%252Fwww.ti.com%252Flit%252Fgpn%252Ftmp236
  temp = (milivolts_read - 500)/10 ;
  return (int) (temp*10);
}


// update the AI regs with ADC values averaged and put through the temperature conversion function
void updateTheAI_regs(){
  mb.Ireg(TempSense1, getTemp(adc1>>4));  // The >>4 divides by 16
  mb.Ireg(TempSense2, getTemp(adc2>>4));  // The >>4 divides by 16
  
  //put temp in variable
  int temp1 = getTemp(adc1>>4);
  int temp2 = getTemp(adc2>>4);

  //check if thre is a temp overload
  if((temp1 >= ((int) mb.Hreg(tempOver_Setting) * 10)) & (temp2 >= ((int)mb.Hreg(tempOver_Setting)* 10))){
    TEMP_OVERLOAD = true; // set temp overload to true
    mb.Ireg(tempOver, 3);  // put a 3 in the Ireg for temp overload
    mb.Coil(tempOver_Flag,1); // raise the temo overload flag
    }
  else if (temp1 >= ((int)mb.Hreg(tempOver_Setting)*10)){
    TEMP_OVERLOAD = true;
    mb.Ireg(tempOver, 1);  // put a 1 in the Ireg for temp overload
    mb.Coil(tempOver_Flag,1);
   }
  else if (temp2 >= ((int)mb.Hreg(tempOver_Setting) * 10)){
    TEMP_OVERLOAD = true;
    mb.Ireg(tempOver, 2);  // put a 2 in the Ireg for temp overload
    mb.Coil(tempOver_Flag,1);
   }
  //reset ADC value buckets to zero
  adc1 = 0;
  adc2 = 0;
  //reset the ADC ready flag to its not ready state
  ADC_readyFlag = false;

  sei();  // renable interrupts

  ADCSRA |= 1<<ADSC; // start ADC conversion

}

void setup() {
    Serial.begin(115200, SERIAL_8N1);
    Serial.setTimeout(1);
   // ************************ Setup the slave ID *********************** 
    //mb.begin(&Serial, RXTX_PIN);  
    mb.begin(&Serial, 4);
    //set baudrate
    mb.setBaudrate(115200);
    //set slave
    mb.slave(SLAVE_ID);
    //setup ADC function
    setupADC();
    
    // add pwm registers for drivers - Use addHreg() for analog outputs
    // set initial values to zero!
    mb.addHreg(Driver1,0); //Driver 1
    mb.addHreg(Driver2,0); //Driver 2 
    mb.addHreg(Driver3,0); //Driver 3
    mb.addHreg(Driver4,0); //Driver 4

    mb.addHreg(tempOver_Setting,65); // Max allowable temperature for the board. Set initaily to 65
    //Add coils setting initial values to 0
    mb.addCoil(dig_out_red,0); // Red LED
    mb.addCoil(dig_out_blue,0); // Blue LED
    mb.addCoil(SSR_fan,1); // SSR Fan
    mb.addCoil(tempOver_Flag,0); // Flag for temp overload

    // add ADC registers - use Use addIreg() for analog Inputs
    mb.addIreg(TempSense1);
    mb.addIreg(TempSense2);
    mb.addIreg(tempOver);
}

void loop() {
  // Call once inside loop() - all magic here--------------------------------------------
  //This takes care of all of the updates and comands
  //It will be constanty listening for new comands from the master
  mb.task();
  if(ADC_readyFlag == true){
    updateTheAI_regs();
  }

  //check if temp is below the temp overload 65
  if(mb.Coil(tempOver_Flag) == 0){
    //continuously write the register values to the LED's if temp is below flag
    digitalWrite(Pin_DriverRED, mb.Coil(dig_out_red));
    digitalWrite(Pin_DriverBLUE, mb.Coil(dig_out_blue));
    digitalWrite(Pin_SSR, mb.Coil(SSR_fan));

    //send values in Hreg to driver pin
    analogWrite(DriverPin1,mb.Hreg(Driver1));
    analogWrite(DriverPin2,mb.Hreg(Driver2));
    analogWrite(DriverPin3,mb.Hreg(Driver3));
    analogWrite(DriverPin4,mb.Hreg(Driver4));

    mb.Ireg(tempOver,0); // EVERYTHING IS OKAY
  }
  // TEMP ABOVE FLAG SHUT EVERYTHING OFF
  else{
    digitalWrite(Pin_DriverRED, 0);
    digitalWrite(Pin_DriverBLUE, 0);
    analogWrite(DriverPin1,0);
    analogWrite(DriverPin2,0);
    analogWrite(DriverPin3,0);
    analogWrite(DriverPin4,0);
    digitalWrite(Pin_SSR, 0); // not sure if we want to kill the SSR if temp is above flag too
  }
}