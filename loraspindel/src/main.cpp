/// if running off 2 AA cells directly then the BOD voltage shoiuld be changed to 1.8v 
///  Burn E:FE    (1.8v BOD)   avrdude -P /dev/ttyACM0 -b 19200 -c avrisp -p m328p -U efuse:w:0xfe:m
// 26 feb 2021 draws about 65uA when sleeping (accel led removed but with voltage reg)
///PINS used
//LORA RA01 module SPI 11 12(MISO)  13      DIO0 2,   reset 9,  NSS 10,
// not connected 0(rx) 1(Tx)   3 4 5 6 7 8    A4 A5 A6 A7 cap sensors 17 14 15 16
// see https://web.archive.org/web/20151108120936/http://bengoncalves.net/2015/10/02/arduino-power-down-mode-with-accelerometer-compass-and-pressure-sensor/
//https://bengoncalves.wordpress.com/2015/10/02/arduino-power-down-mode-with-accelerometer-compass-and-pressure-sensor/

//future features-  if upside down dont transmit 

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
#include "tinyexpr.h"
#include <I2Cdev.h>
#include <MPU6050.h>
#include "OneWire.h"
#include "Wire.h"

//Function prototypes
int readVcc(void);
double GetAtmel328Temp(void);
byte batteryVoltageCompress (int batvoltage);
byte temperatureCompress (double temperature);
void UnusedPinsPullup();
float GetThermTemperature();
long GetThermADC();
bool testAccel();
void initAccel();
float calculateTilt();
void getAccSample();
float getTilt();
void get6050TempSample();


//Thermister constants and varables

#define THERMPOWERPIN A2 // which pin powers voltage divider (turned off unless measuring)        
#define THERMISTORPIN A3 // which analog pin to connect         
#define TEMPERATURENOMINAL 25   // temp. for nominal resistance (almost always 25 C)
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 10
#define ACOEFFICIENT 0.8483803702e-3
#define BCOEFFICIENT 2.572522990e-4
#define CCOEFFICIENT 1.703807621e-7
#define RESISTOR 10060  
long ADCtherm;

 //Accel constants and varables
 // definitions go here
MPU6050 accelgyro;
OneWire *oneWire;
//Ticker flasher;


#define TEMP_CELSIUS 0
#define TEMP_FAHRENHEIT 1
#define TEMP_KELVIN 2

#define MEDIANROUNDSMAX 29 //was 49
#define MEDIANROUNDSMIN 9 // was 29
#define MEDIANAVRG MEDIANROUNDSMIN
#define MEDIAN_MAX_SIZE MEDIANROUNDSMAX

RunningMedian samples = RunningMedian(MEDIANROUNDSMAX);
uint32_t DSreqTime = 0;
int16_t my_Offset[6];
int16_t ax, ay, az;
float atemp;

float Temperatur, Tilt; // , corrGravity;


//varables
const byte nodeID=2;//must be unique for each device
const int sleepDivSixteen =36; //sleep time divided by 16 (seconds)  36=10minutes, 54=15minutes   was 8
RH_RF95 rf95(10, 2); // Select, interupt. Singleton instance of the radio driver
static const RH_RF95::ModemConfig radiosetting = {
    BW_SETTING<<4 | CR_SETTING<<1 | ImplicitHeaderMode_SETTING,
    SF_SETTING<<4 | CRC_SETTING<<2,
    LowDataRateOptimize_SETTING<<3 | ACGAUTO_SETTING<<2};
struct payloadDataStruct{
  byte nodeID;
  byte rssi =0; //not used anymore
  byte voltage;
  byte temperature;//atmel328 temperature
  byte capsensor1Lowbyte;// therm ADC low
  byte capsensor1Highbyte;// therm ADC high
  byte capsensor2Lowbyte;//tilt low byte
  byte capsensor2Highbyte;//tilt high byte
  byte capsensor3Lowbyte;
  byte capsensor3Highbyte;
}txpayload;
byte tx_buf[sizeof(txpayload)] = {0};
byte RSSI =0;
long intTilt;
//float tilt;
//------------------------------------------------------------------------------
void setup()
{
clock_prescale_set(clock_div_2); // divides the Atmel clock by 2 to save power and allow to run at lower voltages

ACSR = B10000000;// Disable the analog comparator by setting the ACD bit (bit 7) of the ACSR register to one. //TH saved 100 micro A
//power_twi_disable(); // TWI (I2C)
delay(500);

//UnusedPinsPullup();//set unused pined to INPUTPULLUP to save power

Serial.begin(115200);//note if the main clock speed is slowed the baud will change
pinMode(THERMPOWERPIN, OUTPUT);
digitalWrite(THERMPOWERPIN, LOW);
pinMode(THERMISTORPIN,INPUT);

/*for (int i=1; i<1000;i++){
DPRINT("therm temp ");DPRINTln(GetThermTemperature());
delay(20);
LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
DPRINT("atmel temp ");DPRINTln(GetAtmel328Temp());
}*/

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

rf95.setTxPower(9, false);//2-20dBm
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
  DPRINT(" sleep");;DPRINT("  ");DPRINT(millis());delay(10);
  for (int i=0; i < sleepDivSixteen; i++){
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);//sleep atmega
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
  }
 
//pinMode (9, INPUT);// if the reset pin is held high on the RA-01 it will somtimes draw hundreds of micro amps
//pinMode(12, INPUT_PULLUP);// mISO
//digitalWrite(10, LOW);//Spi CHIP SELECT
  //digitalWrite(10, HIGH);//Spi CHIP SELECT
  delay(50);
  DPRINT("waking");DPRINT("  ");DPRINTln(millis());
  txpayload.temperature=temperatureCompress(GetAtmel328Temp());
  DPRINT(" mainloop freeMemory()=");
  DPRINTln(freeMemory());
  

  int battery= readVcc();
  DPRINT("batvoltage=");DPRINT(battery);
  txpayload.voltage=batteryVoltageCompress(battery);
  
  ADCtherm = GetThermADC();
    if ((ADCtherm >=0) & (ADCtherm <= 65535)){
    txpayload.capsensor1Lowbyte=(byte)(ADCtherm%256);
    txpayload.capsensor1Highbyte=(byte)(ADCtherm>>8);
    }
    else {txpayload.capsensor1Lowbyte=0;txpayload.capsensor1Highbyte=0;}
///Get tILT//////////////////////////////
  initAccel();
  float accTemp = accelgyro.getTemperature() / 340.00 + 36.53;
  //Tilt = getTilt();
  Tilt = (getTilt()+getTilt())/2;//th
  accelgyro.setSleepEnabled(true);
  //Wire.end();// should turn off pullups

  DPRINT("x: ");  DPRINT(ax);  DPRINT(" y: ");  DPRINT(ay);  DPRINT(" z: ");  DPRINTln(az);
  DPRINT(F("Tilt: "));  DPRINTln(Tilt);
  DPRINT("Tacc: ");  DPRINTln(accTemp);

  //intTilt = abs(round(Tilt*700));
  intTilt = Tilt*700;
  DPRINT("tilt = ");DPRINT(Tilt);
    DPRINT(" int tilt = ");DPRINT(intTilt);
  if (true) {  
  //if ((intTilt<(90*700)) & (intTilt>=0)) {
    txpayload.capsensor2Lowbyte=(byte)(intTilt%256);
    txpayload.capsensor2Highbyte=(byte)(intTilt>>8);
     }
    else {txpayload.capsensor2Lowbyte=0;txpayload.capsensor2Highbyte=0;
    DPRINTln(" int tilt out of bounds ");}
    
  
  memcpy(tx_buf, &txpayload, sizeof(txpayload) );
  byte zize=sizeof(txpayload);
  DPRINT(" sizeof data = ");DPRINT(sizeof(txpayload));
  DPRINT(" Acclow = ");DPRINT(txpayload.capsensor2Lowbyte);
  DPRINT(" Accih= ");DPRINT(txpayload.capsensor2Highbyte);DPRINT(" ");
//send packet
  
  rf95.send((uint8_t *)tx_buf, zize);
  delay(500);
  rf95.waitPacketSent(5000);//timeout after 5 seconds
  DPRINTln("packet sent");
  delay(20);
  
}


double GetAtmel328Temp(void)
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used// with the internal reference of 1.1V.
    // Channel 8 can not be selected with// the analogRead function yet.
  
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

//pinMode(A4, OUTPUT); digitalWrite(A4, LOW);
//pinMode(A5, OUTPUT); digitalWrite(A5, LOW);
pinMode(A6, OUTPUT); digitalWrite(A6, LOW);
pinMode(A7, OUTPUT); digitalWrite(A7, LOW);
//cap sensors were 17 14 15 16
//pinMode(17, OUTPUT); digitalWrite(17, LOW);
pinMode(14, OUTPUT); digitalWrite(14, LOW);
pinMode(15, OUTPUT); digitalWrite(15, LOW);
//pinMode(16, OUTPUT); digitalWrite(16, LOW);


}


float GetThermTemperature(){
RunningMedian ADCsamples = RunningMedian(20);
float lnR;
float ThermResistance;
float ThermTemperature;
//float resistor = RESISTOR;
float reading;
digitalWrite(THERMPOWERPIN, HIGH);
delay(1);

for (int i=0; i< NUMSAMPLES; i++){
  reading = analogRead(THERMISTORPIN);
  ADCsamples.add(reading);
  //DPRINT("adc reading ");DPRINTln(reading);
  delay(1);
}
digitalWrite(THERMPOWERPIN, LOW);

 ThermResistance = (1023/ ADCsamples.getMedian()) -1;
 ThermResistance = RESISTOR / ThermResistance;
 lnR = log(ThermResistance);
 ThermTemperature = 1 / (ACOEFFICIENT + BCOEFFICIENT * lnR + CCOEFFICIENT * lnR* lnR *lnR) - 273.15;
 DPRINT("ThermResistance ");DPRINT(ThermResistance);DPRINT("   ");
 DPRINT("last adc reading ");DPRINT(reading);DPRINT("   ");
 DPRINT("median temp ");DPRINT(ThermTemperature);DPRINT("   ");
 DPRINT("median lowest ");DPRINT(ADCsamples.getLowest());DPRINT("   ");
return ThermTemperature;

}

long GetThermADC(){
RunningMedian ADCsamples = RunningMedian(20);
int reading;
digitalWrite(THERMPOWERPIN, HIGH);
delay(1);

for (int i=0; i< NUMSAMPLES; i++){
  reading = analogRead(THERMISTORPIN);
  ADCsamples.add(reading);
  //DPRINT("adc reading ");DPRINTln(reading);
  delay(1);
}
digitalWrite(THERMPOWERPIN, LOW);
return ADCsamples.getMedian();

}


void initAccel()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(100000);
  //Wire.setClockStretchLimit(2 * 230);

  testAccel();
  // init the Accel
  accelgyro.initialize();
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_5);
  accelgyro.setTempSensorEnabled(true);
  accelgyro.setInterruptLatch(0); // pulse
  accelgyro.setInterruptMode(1);  // Active Low
  accelgyro.setInterruptDrive(1); // Open drain
  accelgyro.setRate(17);
  accelgyro.setIntDataReadyEnabled(true);
  //applyOffset();
}

float calculateTilt()
{
  float _ax = ax;
  float _ay = ay;
  float _az = az;
  return acos(_az / (sqrt(_ax * _ax + _ay * _ay + _az * _az))) * 180.0 / M_PI;
}

bool testAccel()
{
  DPRINTln("Starting Acc Test ");
  uint8_t res; //= Wire.status();
  //if (res != I2C_OK)
    //DPRINTln(String(F("I2C ERROR: ")) + res);

  bool con = accelgyro.testConnection();
  if (!con)
    DPRINTln("Acc Test Connection ERROR!");
DPRINTln("finishing Acc Test ");
  return  con == true;
}

void getAccSample()
{
  accelgyro.getAcceleration(&ax, &az, &ay);
  
}

void get6050TempSample(){
atemp =accelgyro.getTemperature();

}


float getTilt()
{
  uint32_t start = millis();
  uint8_t i = 0;

  for (; i < MEDIANROUNDSMAX; i++)
  {
    while (!accelgyro.getIntDataReadyStatus())
      delay(2);
    getAccSample();
    
  
    float _tilt = calculateTilt();
    samples.add(_tilt);

    //if (i >= MEDIANROUNDSMIN && isDS18B20ready())
     // break;
  }
  DPRINT("Samples:");
  DPRINT(++i);
  DPRINT(" min:");
  DPRINT(samples.getLowest());
  DPRINT(" max:");
  DPRINT(samples.getHighest());
  DPRINT(" time:");
  DPRINTln(millis() - start);
  DPRINT("6050temp ");DPRINTln(accelgyro.getTemperature()/340.00+36.53);
  return samples.getAverage(MEDIANAVRG);//original
  
  //return samples.getMedian();//th
}
