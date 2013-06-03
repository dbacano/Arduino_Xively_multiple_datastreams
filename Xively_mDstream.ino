//------------------------------------------------------------------------------------
//Libraries
//------------------------------------------------------------------------------------

#include <SPI.h>
#include <Ethernet.h>
#include <OneWire.h>
#include <HttpClient.h>
#include <Cosm.h>
#include <avr/wdt.h>

//------------------------------------------------------------------------------------
//Define pin configuration on arduino ethernet and initializing variables
//------------------------------------------------------------------------------------

int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2
OneWire ds(DS18S20_Pin); // on digital pin 2

const int ptensao = A0;
const int pshunt1 = A1;
const int pshunt2 = A2;
const int pshunt3 = A3;
const int pshunt4 = A4;

float refresh_rate = 0.0;  //Dataloger Refresh Rate
long idn = 0;               //Use this to store the id # of our reading.

int temperature=0;
int tensao=0;
int shunt1=0;
int shunt2=0;
int shunt3=0;
int shunt4=0;
unsigned long tempo=0;

unsigned long lastConnectionTime = 0;                // last time we connected to Cosm
const unsigned long connectionInterval = 10000;      // delay between connecting to Cosm in milliseconds

//--------------------------------------------------------------------------------  
//Ethernet
//--------------------------------------------------------------------------------

// assign a MAC address for the ethernet controller.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
// fill in your address here:
byte mac[] = { 
  0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

// fill in an available IP address on your network here,
// for manual configuration:

// cosmKey

char cosmKey[] = "place key here";

// Define the strings for our datastream IDs
// Create the 8 feeds

char sensorId[] = "0";
char sensorId1[] = "1";
char sensorId2[] = "2";
char sensorId3[] = "3";
char sensorId4[] = "4";
char sensorId5[] = "5";
char sensorId6[] = "6";
char sensorId7[] = "7";

// Define the datastreams

CosmDatastream datastreams[] = {
  CosmDatastream(sensorId, strlen(sensorId), DATASTREAM_INT),  // define this ID in cosm
  CosmDatastream(sensorId1, strlen(sensorId1), DATASTREAM_INT), // define this ID in cosm
  CosmDatastream(sensorId2, strlen(sensorId2), DATASTREAM_INT), // define this ID in cosm
  CosmDatastream(sensorId3, strlen(sensorId3), DATASTREAM_INT), // define this ID in cosm
  CosmDatastream(sensorId4, strlen(sensorId4), DATASTREAM_INT), // define this ID in cosm
  CosmDatastream(sensorId5, strlen(sensorId5), DATASTREAM_INT), // define this ID in cosm
  CosmDatastream(sensorId6, strlen(sensorId6), DATASTREAM_INT), // define this ID in cosm
  CosmDatastream(sensorId7, strlen(sensorId7), DATASTREAM_INT), // define this ID in cosm
};

CosmFeed feed(104409, datastreams, 8 );  // Using 8 feeds with FeedID
EthernetClient client;
CosmClient cosmclient(client);

//------------------------------------------------------------------------------------
//Setup
//------------------------------------------------------------------------------------

void setup(){
  
  // Define pin configuration
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  
  Serial.begin(9600); // Baud rate
  
  watchdogSetup(); // watchdog start
  
  Serial.println("Watchdog is watching!");

  Serial.println("Initializing network");
  while (Ethernet.begin(mac) != 1) {
    Serial.println("Error getting IP address via DHCP, trying again...");
    delay(15000);
  };

  Serial.println("Network initialized");
  Serial.println();
};


//-------------------------------------------------------------------------   
//Start Loop
//-------------------------------------------------------------------------

void loop(){
  
//-------------------------------------------------------------------------   
// Ethernet
//-------------------------------------------------------------------------

    int cosmReturn = 0;
    
    wdt_reset(); // watchdog reset
   
    if (millis() - lastConnectionTime > connectionInterval) {
    
      //Get data
      temperature=(int) getTemp(); 
      tensao=(int) get_tensao();
      shunt1=(int) get_shunt1();
      shunt2=(int) get_shunt2();
      shunt3=(int) get_shunt3();
      shunt4=(int) get_shunt4();
      tempo=millis()/1000;
      
      idn++;//Increment ID number

      //Define precision of the data
      datastreams[0].setInt(idn);    
      datastreams[1].setInt(tensao);    
      datastreams[2].setInt(shunt1);    
      datastreams[3].setInt(shunt2);    
      datastreams[4].setInt(shunt3);    
      datastreams[5].setInt(shunt4);    
      datastreams[6].setInt(temperature);    
      datastreams[7].setInt(tempo);   

      cosmReturn = cosmclient.put(feed,cosmKey);   // Send feed to cosm
      
      Serial.print("COSM client returned : "); // Get retern code, similar to HTTP code
      Serial.println(cosmReturn);
  
      lastConnectionTime = millis();

  } 
}

//-------------------------------------------------------------------------
//Read temperature
//-------------------------------------------------------------------------

float getTemp(){  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.print("CRC is not valid!\n");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
  
}

//--------------------------------------------------------------------------------------
//tensao
//--------------------------------------------------------------------------------------

float get_tensao(){
  
  int tensao_0=analogRead(ptensao);
  delay(250);
  int tensao_1=analogRead(ptensao);
  delay(250);
  int tensao_2=analogRead(ptensao);
  delay(250);
  int tensao_3=analogRead(ptensao);
  delay(250);
  int tensao_4=analogRead(ptensao);
  
  float tensao_med=(tensao_0+tensao_1+tensao_2+tensao_3+tensao_4)/5;
  
  
  float tensao_sum=(tensao_med*31000/1023);

  return tensao_sum;
}

//--------------------------------------------------------------------------------------
//corrente 1
//--------------------------------------------------------------------------------------

float get_shunt1(){

  int shunt1_0=analogRead(pshunt1);
  delay(250);
  int shunt1_1=analogRead(pshunt1);
  delay(250);
  int shunt1_2=analogRead(pshunt1);
  delay(250);
  int shunt1_3=analogRead(pshunt1);
  delay(250);
  int shunt1_4=analogRead(pshunt1);
  
  float shunt1_med=(shunt1_0+shunt1_1+shunt1_2+shunt1_3+shunt1_4)/5;
  
  float shunts1=-(((5000*shunt1_med/1024)-2500)/0.066);
  
  return shunts1;
}

//--------------------------------------------------------------------------------------
//corrente 2
//--------------------------------------------------------------------------------------

float get_shunt2(){
  
  int shunt2_0=analogRead(pshunt2);
  delay(250);
  int shunt2_1=analogRead(pshunt2);
  delay(250);
  int shunt2_2=analogRead(pshunt2);
  delay(250);
  int shunt2_3=analogRead(pshunt2);
  delay(250);
  int shunt2_4=analogRead(pshunt2);
  
  float shunt2_med=(shunt2_0+shunt2_1+shunt2_2+shunt2_3+shunt2_4)/5;
  
  float shunts2=-(((5000*shunt2_med/1024)-2500)/0.066);
  
  return shunts2;
}

//--------------------------------------------------------------------------------------
//corrente 3
//--------------------------------------------------------------------------------------

float get_shunt3(){
  
  int shunt3_0=analogRead(pshunt3);
  delay(250);
  int shunt3_1=analogRead(pshunt3);
  delay(250);
  int shunt3_2=analogRead(pshunt3);
  delay(250);
  int shunt3_3=analogRead(pshunt3);
  delay(250);
  int shunt3_4=analogRead(pshunt3);
  
  float shunt3_med=(shunt3_0+shunt3_1+shunt3_2+shunt3_3+shunt3_4)/5;
  
  float shunts3=-(((5000*shunt3_med/1024)-2500)/0.066);
  
  return shunts3;
}

//--------------------------------------------------------------------------------------
//corrente 4
//--------------------------------------------------------------------------------------

float get_shunt4(){
  
  int shunt4_0=analogRead(pshunt4);
  delay(250);
  int shunt4_1=analogRead(pshunt4);
  delay(250);
  int shunt4_2=analogRead(pshunt4);
  delay(250);
  int shunt4_3=analogRead(pshunt4);
  delay(250);
  int shunt4_4=analogRead(pshunt4);
  
  float shunt4_med=(shunt4_0+shunt4_1+shunt4_2+shunt4_3+shunt4_4)/5;
  
  float shunts4=(((5000*shunt4_med/1024)-2500)/0.066);
  
  return shunts4;
}

//--------------------------------------------------------------------------------------
//Watchdog Timer
//--------------------------------------------------------------------------------------

void watchdogSetup(void)
{
cli();  // disable all interrupts
wdt_reset(); // reset the WDT timer
/*
WDTCSR configuration:
WDIE = 1: Interrupt Enable
WDE = 1 :Reset Enable
WDP3 = 0 :For 2000ms Time-out
WDP2 = 1 :For 2000ms Time-out
WDP1 = 1 :For 2000ms Time-out
WDP0 = 1 :For 2000ms Time-out
*/
// Enter Watchdog Configuration mode:
WDTCSR |= (1<<WDCE) | (1<<WDE);
// Set Watchdog settings:
WDTCSR = (1<<WDIE) | (1<<WDE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
sei();
}
