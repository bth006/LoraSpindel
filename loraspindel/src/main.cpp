/// if running off 2 AA cells directly then the BOD voltage shoiuld be changed to 1.8v 
///  Burn E:FE    (1.8v BOD)   avrdude -P /dev/ttyACM0 -b 19200 -c avrisp -p m328p -U efuse:w:0xfe:m

///PINS used
//LORA RA01 module SPI 11 12(MISO)  13      DIO0 2,   reset 9,  NSS 10,
//cap sensors 17 14 15 16
// not connected 0(rx) 1(Tx)   3 4 5 6 7 8    A4 A5 A6 A7 

//#define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
//test
#ifdef DEBUG    //Macros are usually in all capital letters.
  #define DPRINT(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
  #define DPRINTln(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
  #define DPRINT(...)     //now defines a blank line
  #define DPRINTln(...)   //now defines a blank line
#endif

#include <MemoryFree.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "RadioSettings.h"
#include "LowPower.h"
#include "avr/power.h" //to adjust clock speed
#include "RunningMedian.h"

//Function prototypes
int readVcc(void);
double GetTemp(void);
byte batteryVoltageCompress (int batvoltage);
byte temperatureCompress (double temperature);
long GetCapacitance(int repeats, int OUT_PIN, int IN_PIN);
void waitForAck();
void UnusedPinsPullup();

//varables
const byte nodeID=1;//must be unique for each device
boolean ackReceived =0;
const int sleepDivSixteen =54; //sleep time divided by 16 (seconds)  36=10minutes, 54=15minutes
RH_RF95 rf95(10, 2); // Select, interupt. Singleton instance of the radio driver
static const RH_RF95::ModemConfig radiosetting = {
    BW_SETTING<<4 | CR_SETTING<<1 | ImplicitHeaderMode_SETTING,
    SF_SETTING<<4 | CRC_SETTING<<2,
    LowDataRateOptimize_SETTING<<3 | ACGAUTO_SETTING<<2};
struct payloadDataStruct{
  byte nodeID;
  byte rssi =0; //not used anymore
  byte voltage;
  byte temperature;
  byte capsensor1Lowbyte;
  byte capsensor1Highbyte;
  byte capsensor2Lowbyte;
  byte capsensor2Highbyte;
  byte capsensor3Lowbyte;
  byte capsensor3Highbyte;
}txpayload;
byte tx_buf[sizeof(txpayload)] = {0};
byte RSSI =0;
//------------------------------------------------------------------------------
void setup()
{
clock_prescale_set(clock_div_2); // divides the Atmel clock by 2 to save power and allow to run at lower voltages

ACSR = B10000000;// Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one. //TH saved 100 micro A
power_twi_disable(); // TWI (I2C)
delay(500);

UnusedPinsPullup();//set unused pined to INPUTPULLUP to save power

//LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);

  txpayload.nodeID=nodeID;
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW); delay(10);
  digitalWrite(9, HIGH);  //reset pin

  delay(500);
  
  Serial.begin(115200);//note if the main clock speed is slowed the baud will change
  DPRINTln("booting");
 
  while (!Serial) ; // Wait for serial port to be available
  if (!rf95.init()){
    DPRINTln("init failed");
    delay(1000);
    rf95.init();//try again
  }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:

rf95.setTxPower(20, false);//was 17
DPRINTln("init ok");
rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);// BW =125kHz, CRC4/8, sf 4096
delay(10);
rf95.setModemRegisters(&radiosetting);//this is where we apply our custom settings from RadioSettings.h
rf95.printRegisters(); //th
delay(500);
UnusedPinsPullup();//set unused pined to INPUTPULLUP to save power
DPRINTln("sleep n setup");delay(50);
LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
DPRINTln("wake n setup");
 Serial.print("ACSR=");Serial.println(ACSR,BIN);
Serial.print("PORTD=");Serial.println(PORTD,BIN);
Serial.print("PORTB=");Serial.println(PORTB,BIN);
Serial.print("PORTC=");Serial.println(PORTC,BIN);
Serial.print("ADCSRA=");Serial.println(ADCSRA,BIN);
}

//------------------------------------------------------------------------------
void loop()
{
  
  //put radio and Atmega to sleep
  rf95.sleep();//FIFO data buffer is cleared when the device is put in SLEEP mode
  //therefore any acks destined for other devices should be cleared
  delay(10);


//pinMode (9, INPUT);// if the reset pin is held high on the RA-01 it will somtimes draw hundreds of micro amps
//pinMode(12, INPUT_PULLUP);// mISO
//digitalWrite(10, LOW);//Spi CHIP SELECT
  DPRINTln(" sleep");delay(10);
  for (int i=0; i < sleepDivSixteen; i++){
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);//sleep atmega
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
  }

  //digitalWrite(10, HIGH);//Spi CHIP SELECT
  delay(50);DPRINTln("waking");
  
  DPRINT(millis());
  long capTotal;

//capsensor 1
GetCapacitance(1, 17, 14);//prime
capTotal=GetCapacitance(6, 17, 14);
capTotal+=GetCapacitance(6, 17, 14);
//capTotal+=GetCapacitance(6, 17, 14);
capTotal=capTotal/2;//get mean
DPRINT(" mainloop freeMemory()=");
DPRINTln(freeMemory());
  DPRINT("  ");DPRINT(millis());

  DPRINT(" capacitive sensor 1");DPRINTln(capTotal);
  DPRINT(" capLobyte ");DPRINT( (byte)(capTotal%256));
  DPRINT(" caphibyte "); DPRINT ((byte)(capTotal>>8));

//start building txpayload
  if ((capTotal >=0) & (capTotal <= 65535)){
  txpayload.capsensor1Lowbyte=(byte)(capTotal%256);
  txpayload.capsensor1Highbyte=(byte)(capTotal>>8);
  }
  else {txpayload.capsensor1Lowbyte=0;txpayload.capsensor1Highbyte=0;}
  delay(50);
//next sensor
  GetCapacitance(1, 17, 15);//prime
  capTotal=GetCapacitance(6, 17, 15);
  capTotal+=GetCapacitance(6, 17, 15);
  //capTotal+=GetCapacitance(6, 17, 15);
  capTotal=capTotal/2;//get mean


  //start building txpayload
    if ((capTotal >=0) & (capTotal <= 65535)){
    txpayload.capsensor2Lowbyte=(byte)(capTotal%256);
    txpayload.capsensor2Highbyte=(byte)(capTotal>>8);
    }
    else {txpayload.capsensor2Lowbyte=0;txpayload.capsensor2Highbyte=0;}

delay(50);
//next sensor
  GetCapacitance(1, 17, 16);//prime
  capTotal=GetCapacitance(6, 17, 16);
  capTotal+=GetCapacitance(6, 17, 16);
  //capTotal+=GetCapacitance(6, 17, 16);
  capTotal=capTotal/2;//get mean


  //start building txpayload
    if ((capTotal >=0) & (capTotal <= 65535)){
    txpayload.capsensor3Lowbyte=(byte)(capTotal%256);
    txpayload.capsensor3Highbyte=(byte)(capTotal>>8);
    }
    else {txpayload.capsensor3Lowbyte=0;txpayload.capsensor3Highbyte=0;}

  delay(10);
  if (ackReceived==true) {RSSI=abs(rf95.lastRssi());}
   else{RSSI=0;}
  txpayload.rssi = RSSI;
  delay(10);
  int battery= readVcc();
  DPRINT("batvoltage=");DPRINT(battery);
  txpayload.voltage=batteryVoltageCompress(battery);
  txpayload.temperature=temperatureCompress(GetTemp());
  memcpy(tx_buf, &txpayload, sizeof(txpayload) );
  byte zize=sizeof(txpayload);
  DPRINT(" sizeof data = ");DPRINT(sizeof(txpayload));

//send packet
  rf95.send((uint8_t *)tx_buf, zize);
  ackReceived =0;// clear previous ack.  Not used unless waitForAck() is called
  delay(500);
  rf95.waitPacketSent(5000);//timeout after 5 seconds
  DPRINTln("packet sent");
  //waitForAck();// not used to save battery.  Can be enabled if wanted
  delay(20);

  
}




double GetTemp(void)
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;

  // The returned temperature is in degrees Celsius.
  return (t);
}

byte batteryVoltageCompress (int batvoltage) {
//compress voltage reading to 1 byte
if ((batvoltage < 1300) or (batvoltage >= 3340)) batvoltage =1300;
int result2;
result2 =  (batvoltage - 1300L)/8;
return (byte)(result2);
}

byte temperatureCompress (double temperature) {
//compress temperature to 1 byte
// allowable temperature range is 0 to 51 degree C
if ((temperature > 51) or (temperature < 0)) temperature =0;
double result2;
result2 =  temperature *5;
return (byte)(result2);
}

int readVcc(void) // Returns actual value of Vcc (x 1000)
   {
    // For 168/328 boards only
    const long InternalReferenceVoltage = 1102;  // Adjust this value to your boards specific internal BG voltage x1000
       // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
       // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);

    delay(50);  // Let mux settle a little to get a more stable A/D conversion
       // Start a conversion
    ADCSRA |= _BV( ADSC );
       // Wait for it to complete
    while( ( (ADCSRA & (1<<ADSC)) != 0 ) );
       // Scale the value
    int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L); // calculates for straight line value
    return results;

   }

   long GetCapacitance(int repeats, int OUT_PIN, int IN_PIN)
   {
  //clock_prescale_set(clock_div_1);
     //const int OUT_PIN = A4;//A8=4, A7=6
     //const int IN_PIN = A5;
    RunningMedian samples = RunningMedian(repeats);
     const float IN_STRAY_CAP_TO_GND = 24.48;
     const float IN_CAP_TO_GND  = IN_STRAY_CAP_TO_GND;
     const int MAX_ADC_VALUE = 1023;

   float capacitance;

   pinMode(OUT_PIN, OUTPUT);
   pinMode(IN_PIN, OUTPUT);

   for (int i=0; i < repeats; i++){

   pinMode(IN_PIN, INPUT);
   //delay(1);
   digitalWrite(OUT_PIN, HIGH);
   int val = analogRead(IN_PIN);
   digitalWrite(OUT_PIN, LOW);
   samples.add(val);
   if (val < 1023)//was 1000
   {
     pinMode(IN_PIN, OUTPUT);

     capacitance = (float)val * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val);

     DPRINT(F("Cap Value = "));
     DPRINT(capacitance, 3);
     DPRINT(F(" pF ("));
     DPRINT(val);
     DPRINTln(F(") "));


   }

   //while (millis() % 1000 != 0);
   delay(100);
   }
   //DPRINT(" freeMemory()=");
   //DPRINTln(freeMemory());
     float temp = samples.getMedian();
     capacitance = 10*(float)temp * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - temp);
     return (long)capacitance;
   }


void waitForAck(){
// Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  DPRINT("waiting to rx, millis = ");DPRINT(millis());
  unsigned long temptimer = millis();

  //DPRINT("available "); DPRINTln(rf95.available());
  if (rf95.waitAvailableTimeout(10000))//was10000
  {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len))
   {
      DPRINT(" got reply , time for ack=");DPRINT(millis()-temptimer); DPRINT(" millis "); DPRINT(millis());
      //rf95.printBuffer("Got:", buf, len);
      DPRINT(" local com voltage = ");DPRINT(txpayload.voltage);
      DPRINT("    local RSSI = ");DPRINT(rf95.lastRssi(), DEC);
      DPRINT("    local snr = ");DPRINT(rf95.lastSNR(), DEC);
      DPRINT(" rx=");DPRINT(buf[0], DEC);DPRINT(" nodeid=");DPRINT(buf[1], DEC);
      //check message is an ack for this node
      if ((buf[0]==170) and (buf[1]==nodeID)) {
        ackReceived=true;
        DPRINTln(" ackReceived=true");
      }

    }
    else
    {
      DPRINTln("recv failed");
    }
  }
  else
  {
    DPRINTln("No reply");}
rf95.recv(buf, &len);// clear buffer just in case}
}

void UnusedPinsPullup(){

  /*pinMode (0, INPUT_PULLUP);    // to save power.  serial rx pin
  pinMode (3, INPUT_PULLUP); 
  pinMode (4, INPUT_PULLUP); 
  pinMode (5, INPUT_PULLUP); 
  pinMode (6, INPUT_PULLUP); 
  pinMode (7, INPUT_PULLUP); 
  pinMode (8, INPUT_PULLUP); 
  pinMode (A4, INPUT_PULLUP); 
  pinMode (A5, INPUT_PULLUP); 
  pinMode (A6, INPUT_PULLUP); 
  pinMode (A7, INPUT_PULLUP); */

pinMode (0, INPUT_PULLUP);
pinMode(3, OUTPUT); digitalWrite(3, LOW);
pinMode(4, OUTPUT); digitalWrite(4, LOW);
pinMode(5, OUTPUT); digitalWrite(5, LOW);
pinMode(6, OUTPUT); digitalWrite(6, LOW);
pinMode(7, OUTPUT); digitalWrite(7, LOW);

pinMode(A4, OUTPUT); digitalWrite(A4, LOW);
pinMode(A5, OUTPUT); digitalWrite(A5, LOW);
pinMode(A6, OUTPUT); digitalWrite(A6, LOW);
pinMode(A7, OUTPUT); digitalWrite(A7, LOW);

/*

  for (int i = 3; i <= 8; i++)
    {
    pinMode (i, INPUT_PULLUP);    // to save power
    //digitalWrite (i, LOW);  //     ditto
    }
    

  for (int i = 18; i <= 21; i++)
    {
    pinMode (i, INPUT_PULLUP);    // to save power
    //digitalWrite (i, LOW);  //     ditto
    }*/
}