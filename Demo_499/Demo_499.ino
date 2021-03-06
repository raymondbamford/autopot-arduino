/*  University of Victoria
 *  ELEC 499 - AutoPot
 *  July 20, 2017
 *  Code compiled by Matthew Fitz
 *  
 *  Combined code for the following sensors:
 *  1) Conductivity                          - Pins A0, A1, A4
 *  2) Temperature Probe (for water)         - Pin 4
 *  3) Sonar (for water level)               - Pins 12, 13
 *  4) Liquid Level (for nutrient level)     - Pin 5 //// REMOVED
 *  5) Temperature Humidity (for air)        - Pin 7
 *  6) Photoresistor                         - Pin A5
 *  
 *  Also use the following peripherals
 *  1) Button 1 (Nutrient peristaltic Pump)  - Pin 2
 *  2) Button 2 (Reservoir Mister)           - Pin 3
 *  3) Peristaltic Pump                      - Pin A2
 *  4) Reservoir Mister                      - Pin 9
 *  5) Lamp                                  - Pin 10
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
#include <Servo.h> // Use for Reservoir Mister


 ///////////////////////////////////////////////////////////////////////////////////////////
// USER DEFINED GLOBAL VARIABLES
///////////////////////////////////////////////////////////////////////////////////////////

//##################################################################################
//-----------  Do not Replace R1 with a resistor lower than 300 ohms    ------------
//##################################################################################

// GENERAL
#define WAIT_MAIN 5000 // How long the main loop waits
#define WAIT_PUMP 1500 // How long the Peristaltic Pump waits
#define WAIT_MIST 1000 // How long the Reservoir Mister waits
//#define WAIT_LAMP 5000 // How long the Lamp waits
#define pot_id 2 // pot d
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
//float K = 2.00; // For some reason this value changed during testing
float K = 1.035; // For some reason this value changed during testing
float Vdrop = 0; // Voltage drop across water
float EC = 0; // EC for water
float EC25 = 0; // EC for water at 25 degC
//int PPM = 0; // PPM (parts per million) for water
float Rc = 0; // Resistor of water
float con_raw = 0; // Raw data from sensor
float con_buffer = 0; // Store sum of raw data to find average

// TEMPERATURE PROBE (DS18S20)
int Temp_probe_pin = 4; // Pins
OneWire ds(Temp_probe_pin);
float temperature=10;

// SONAR
#define trigPin 13
#define echoPin 12
long duration, distance;

// TEMPERATURE HUMIDITY
#define DHT22_PIN 7
DHT22 myDHT22(DHT22_PIN);
float air_temp;
float humidity;

// PHOTORESISTOR
int lightPin = A5;  // Pin for Photoresistor
float lit_buffer = 0;
float input;
float Vout;
float Lux;
float R2 = 10.0; // 10 Kohms

// SERIAL INPUT FOR PHOTORESISTORING AND LOOP DELAY
long delayCounter = 0;

// BUTTON 1
const int button1Pin = 2;    // the number of the pushbutton pin

// BUTTON 2
const int button2Pin = 3;

// PERISTLTIC PUMP
int base = A2;

// RESERVOIR MISTER
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position

// LAMP
int lamp = 10;

///////////////////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // SERIAL
  Serial.begin(9600);
  
  // CONDUCTIVITY
  pinMode(ECPin,INPUT);
  pinMode(ECPower,OUTPUT); // Setting pin for sourcing current
  pinMode(ECGround,OUTPUT); // Setting pin for sinking current
  digitalWrite(ECGround,LOW); // We can leave the ground connected permanently
  R1=(R1+Ra); // Taking into acount Powering Pin Resistance

  // SONAR
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // BUTTON 1
  pinMode(button1Pin, INPUT);

  // BUTTON 2
  pinMode(button2Pin, INPUT);

  // PERISTALTIC PUMP
  pinMode(base, OUTPUT);
  analogWrite(base, 0);

  // RESERVOIR MISTER
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  // Initializes moter by turning it on and off

  // LAMP
  pinMode(lamp, OUTPUT);
  digitalWrite(lamp, LOW);
}


///////////////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP
///////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  // CONDUCTIVITY AND WATER PROBE
  getEC(); // Don't call this more than once every five seconds or you will polarise the water

  // SONAR
  getSonar();

  // TEMPERATURE AND HUMIDITY
  getTempHum(); // Will output -100 for both values if there is an error
  
  // PHOTORESISTOR
  getLight();
  
  // PRINT ALL SENSORS
  PrintReadings(); 

  // Loop to poll buttons and lamp during poll
  for(int i = 0; i < (WAIT_MAIN)/100; i++) {

    //Serial.println("In button wait loop");
    // PERISTALTIC PUMP
    if(digitalRead(button1Pin) == LOW) {
      peristaltic();
      //Serial.println("button1");
    }

    // RESERVOIR MISTER
    if(digitalRead(button2Pin) == LOW) {
      mister();
      //Serial.println("button 2p");
    }
    
    // LAMP
    getLamp();
    
    delay(100);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////
// USER DEFINED FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////

// PERISTALTIC PUMP
void peristaltic() {
  analogWrite(base, 250);
  delay(WAIT_PUMP);
  analogWrite(base, 0);
}


// RESERVOIR MISTER
void mister() {
  for (pos = 0; pos <= 165; pos += 1) {
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
  delay(WAIT_MIST);

  for (pos = 165; pos >= 0; pos -= 1) {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}


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


// PHOTORESISTOR
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


// LAMP
void getLamp() {
  getLight();
  if(Lux < 20) {
    digitalWrite(lamp, HIGH);
    //delay(WAIT_LAMP);
    //digitalWrite(lamp, LOW);
  }
  else if(Lux > 40) {
    digitalWrite(lamp, LOW);
  }
}


// PRINT ALL SENSORS
void PrintReadings() {
  // POT ID
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

  // TEMPERATURE AND HUMIDITY
  //Serial.print("AIR TEMPERATURE: ");
  Serial.print(air_temp);
  Serial.print(",");
  //Serial.print("HUMIDITY: ");
  Serial.print(humidity);
  Serial.print(",");
  
  // PHOTORESISTOR
  //Serial.print("PHOTORESISTOR: ");
  Serial.print(Lux);
  Serial.print(";");
}
