/*  University of Victoria
 *  ELEC 499 - AutoPot
 *  July 20, 2017
 *  Code compiled by Matthew Fitz
 *  
 *  Combined code for the following sensors:
 *  1) Conductivity                          - Pins A0, A1, A4
 *  2) Temperature Probe (for water)         - Pin 2
 *  3) Sonar (for water level)               - Pins 12, 13
 *  4) Liquid Level (for nutrient level)     - Pin 5
 *  5) Temperature Humidity (for air)        - Pin 7
 *  6) Light                                 - Pin A5
 *  
 *  References for each sensor:
 *  1) https://hackaday.io/project/7008-fly-wars-a-hackers-solution-to-world-hunger/log/24646-three-dollar-ec-ppm-meter-arduino
 *  2) https://www.dfrobot.com/wiki/index.php/Waterproof_DS18B20_Digital_Temperature_Sensor_(SKU:DFR0198)
 *  3) http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/
 *  4) https://www.dfrobot.com/wiki/index.php/Liquid_Level_Sensor-FS-IR02_SKU:_SEN0205
 *  5) https://www.dfrobot.com/wiki/index.php/DHT22_Temperature_and_humidity_module_SKU:SEN0137
 *  6) http://emant.com/316002.page
 */

///////////////////////////////////////////////////////////////////////////////////////////
// LIBRARIES
///////////////////////////////////////////////////////////////////////////////////////////

#include <OneWire.h> // Use for temperature
#include <DHT22.h> // Custom library for DHT22


 ///////////////////////////////////////////////////////////////////////////////////////////
// USER DEFINED GLOBAL VARIABLES
///////////////////////////////////////////////////////////////////////////////////////////

//##################################################################################
//-----------  Do not Replace R1 with a resistor lower than 300 ohms    ------------
//##################################################################################

// GENERAL
#define WAIT 5 // wait is set to 5 mins (300 * 1000ms)
#define pot_id 2
float Vin= 5;
int i;

// CONDUCTIVITY
int ECPin = A0; // Input to measure Vdrop
int ECGround = A1; // Set to ground
int ECPower = A4; // Set to 5V
int R1 = 560; // First resistor in voltage divider - 560 Ohms
int Ra = 25; //Resistance of powering Pins - 25 Ohms
//float PPMconversion=0.7; // Convert from EC to PPM
float temperatureCoef = 0.019; // temperature compensation. This changes depending on what chemical we are measuring
//float K = 1.15; // Cell constant. Found by running the calibration code
float K = 2.00; // For some reason this value changed during testing
float Vdrop = 0; // Voltage drop across water
float EC = 0; // EC for water
float EC25 = 0; // EC for water at 25 degC
//int PPM = 0; // PPM (parts per million) for water
float Rc = 0; // Resistor of water
float con_raw = 0; // Raw data from sensor
float con_buffer = 0; // Store sum of raw data to find average

// TEMPERATURE PROBE (DS18S20)
int Temp_probe_pin = 2; // Pins
OneWire ds(Temp_probe_pin);
float temperature=10;

// SONAR
#define trigPin 13
#define echoPin 12
long duration, distance;

// LIQUID LEVEL
int Liquid_level_pin = 5;
int Liquid_level = 0;

// TEMPERATURE HUMIDITY
#define DHT22_PIN 7
DHT22 myDHT22(DHT22_PIN);
float air_temp;
float humidity;

// LIGHT
int lightPin = A5;  // Pin for Photoresistor
float lit_buffer = 0;
float input;
float Vout;
float Lux;
float R2 = 10.0; // 10 Kohms

// SERIAL INPUT FOR LIGHTING AND LOOP DELAY
long delayCounter = 0;


///////////////////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // SERIAL
  Serial.begin(9600);
  // Serial.println("STARTING PROGRAM. (Ignore first set of data points)");
  // Serial.println("");
  
  // CONDUCTIVITY
  pinMode(ECPin,INPUT);
  pinMode(ECPower,OUTPUT); // Setting pin for sourcing current
  pinMode(ECGround,OUTPUT); // Setting pin for sinking current
  digitalWrite(ECGround,LOW); // We can leave the ground connected permanently
  R1=(R1+Ra); // Taking into acount Powering Pin Resistance

  // SONAR
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // LIQUID LEVEL
  pinMode(5,INPUT);
}


///////////////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP
///////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  
  
  // CONDUCTIVITY AND WATER PROBE
  getEC(); // Don't call this more than once every five seconds or you will polarise the water

  // SONAR
  getSonar();

  // LIQUID LEVEL
  getLiquidLevel(); // 1 means water above, 0 means water below

  // TEMPERATURE AND HUMIDITY
  getTempHum(); // Will output -100 for both values if there is an error
  
  // LIGHT
  getLight();
  
  // PRINT ALL SENSORS
  PrintReadings(); 
  
  // TURN ON PUMP
  //write code here for pump

  delay(WAIT*1000);
}


///////////////////////////////////////////////////////////////////////////////////////////
// USER DEFINED FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////

// TEMPERATURE PROBE
float getTempProbe() {
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      //Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      //Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float temperatureSum = tempRead / 16;
  
  return temperatureSum;
  
}


// CONDUCTIVITY
void getEC() {
// Reading temperature Of Solution
temperature = getTempProbe();
 
// Estimates Resistance of Liquid
digitalWrite(ECPower,HIGH);
con_raw= analogRead(ECPin);
con_raw= analogRead(ECPin); // This is not a mistake, First reading will be low beause if charged a capacitor
digitalWrite(ECPower,LOW);
 
// Converts to EC
Vdrop= (Vin*con_raw)/1024.0;
Rc=(Vdrop*R1)/(Vin-Vdrop);
Rc=Rc-Ra; // Accounting for Digital Pin Resitance
EC = 1000/(Rc*K);

// Compensating For Temperaure
EC25  =  EC/ (1+ temperatureCoef*(temperature-25.0));

// Convert to PPM
//PPM=(EC25)*(PPMconversion*1000);
}


// SONAR
void getSonar() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
}

// LIQUID LEVEL
void getLiquidLevel() {
  Liquid_level = digitalRead(Liquid_level_pin);
}


// TEMPERATURE AND HUMIDITY
void getTempHum() {
  DHT22_ERROR_t errorCode;

  errorCode = myDHT22.readData();
  switch(errorCode)
  {
    case DHT_ERROR_NONE:
      air_temp = myDHT22.getTemperatureC();
      humidity = myDHT22.getHumidity();
      break;
    case DHT_ERROR_CHECKSUM:
      air_temp = -100;
      humidity = -100;
      break;
    case DHT_BUS_HUNG:
      air_temp = -100;
      humidity = -100;
      break;
    case DHT_ERROR_NOT_PRESENT:
      air_temp = -100;
      humidity = -100;
      break;
    case DHT_ERROR_ACK_TOO_LONG:
      air_temp = -100;
      humidity = -100;
      break;
    case DHT_ERROR_SYNC_TIMEOUT:
      air_temp = -100;
      humidity = -100;
      break;
    case DHT_ERROR_DATA_TIMEOUT:
       air_temp = -100;
      humidity = -100;
      break;
    case DHT_ERROR_TOOQUICK:
      air_temp = -100;
      humidity = -100;
      break;
  }
}


// LIGHT
void getLight() {
  lit_buffer = 0;
  // Average the values
  for(i=0; i<50; ++i) {
    input = analogRead(lightPin);
    lit_buffer = lit_buffer + input;
  }
  input = lit_buffer/50.0;
  Vout = Vin*input/1024.0;
  Lux = (500*(Vin-Vout)) / (Vout*R2);
}


// PRINT ALL SENSORS
void PrintReadings() {
  Serial.print(pot_id); // print pot serial number/id
  Serial.print(",");
  // CONDUCTIVITY
  //Serial.print("CONDUCTIVITY: ");
  Serial.print(EC25, 6);
  //Serial.println(" MicroSiemens");
  //.print(PPM);
  Serial.print(",");
  
  // TEMPERATURE PROBE
  //Serial.print("WATER TEMPERATURE: ");
  Serial.print(temperature);
  Serial.print(",");

  // SONAR
  //Serial.print("WATER HEIGHT: ");
  Serial.print(distance);
  Serial.print(",");

  // LIQUID LEVEL
  //Serial.print("LIQUID LEVEL: ");
  Serial.print(Liquid_level,DEC);
  Serial.print(",");

  // TEMPERATURE AND HUMIDITY
  //Serial.print("AIR TEMPERATURE: ");
  Serial.print(air_temp);
  Serial.print(",");
  //Serial.print("HUMIDITY: ");
  Serial.print(humidity);
  Serial.print(",");
  
  // LIGHT
  //Serial.print("LIGHT: ");
  Serial.print(Lux);
  Serial.print("\n");
  //Serial.print("");
}

// UPDATE THE LIGHT IN THE SERIAL EVENT FUNCTION
void serialEvent(){
  //code to update light
}

