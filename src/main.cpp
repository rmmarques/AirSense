#include <Arduino.h>
#include <bsec.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_GFX.h>
#include <WiFi.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <DFRobot_ENS160.h>






#include "Adafruit_PM25AQI.h"
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();





#define I2C_COMMUNICATION  //I2C communication. Comment out this line of code if you want to use SPI communication.

DFRobot_ENS160_I2C ENS160(&Wire, /*I2CAddr*/ 0x53);





// Invoke library, pins defined in User_Setup.h
TFT_eSPI tft = TFT_eSPI(); 

const char* ssid = "I am gonna kill your fucking dog";
const char* password = "morangoscomacucar";

#define DEVICE "ESP32"
#define INFLUXDB_URL "https://eu-central-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "OvK1qsQXDixTlkudQhyGPr9STmRJH1tNfD-Ma3-T7CqPfEi1i4mqTLZfegJ9Q-ZCp0Ef7t4j_HQUS457ThNt9A=="
#define INFLUXDB_ORG "a932d6a20e239b6a"
#define INFLUXDB_BUCKET "humidityandtemperature"
  
// Time zone info
#define TZ_INFO "UTC1"///CHANGE TO CORRECT TIMEZONE//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  // Declare InfluxDB client instance with preconfigured InfluxCloud certificate
  InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
  
  // Declare Data point
  Point sensor("wifi_status");




#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

// Connections for PM2.5 Sensor
#define RXD2 34 // To sensor TXD
#define TXD2 33 // To sensor RXD

#define SEALEVELPRESSURE_HPA (1017)

Adafruit_BME680 bme; // I2C version

int readcounter = 0;


//function declaration/prorotypes
void printtodisplay();
void printtoserial();
void beep(int x);
void leds();

void setup() {
  Serial.begin(115200);










  // Set up UART connection
  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);





  // Init the TVOC and eCO2
  while( NO_ERR != ENS160.begin() ){
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");

  ENS160.setPWRMode(ENS160_STANDARD_MODE);

  ENS160.setTempAndHum(/*temperature=*/25.0, /*humidity=*/50.0);



  pinMode(12, OUTPUT);//r led
  pinMode(26, OUTPUT);//y led
  pinMode(32, OUTPUT);//g led
  pinMode(16, OUTPUT);//buzzer
  beep(1);//one beep on startup

  /////////////////////////////////////////////////////////////////////////////////////////WIFI and DB
    WiFi.begin(ssid, password);
    //Serial.println("Connecting");
    int i = 0;
    while(WiFi.status() != WL_CONNECTED && i <50){
        //Serial.print(".");
        i++;
        delay(100);
    }


  tft.init(); //display
  tft.setRotation(0); //display


    if(WiFi.status() == WL_CONNECTED){
      beep(2);//two beeps when connected to wifi
      Serial.println("Connected to WiFi");
      Serial.print("Local ESP32 IP: "); Serial.println(WiFi.localIP());
      timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");  
    }else{
      Serial.println("Can't connect to WIFI");
    }

  
    // Connect to InfluxDB
    if (client.validateConnection()) {
      beep(3);//three beeps when connected to db
      Serial.print("Connected to InfluxDB: "); Serial.println(client.getServerUrl());
    } else {
      Serial.print("InfluxDB "); Serial.println(client.getLastErrorMessage());
    }
    // Add tags to the data sent do db
    sensor.addTag("device", DEVICE);
    sensor.addTag("SSID", "home");
//////////////////////////////////////////////////////////////////////////////////////////////////////////


  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms 

}
/*
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};*/

//struct pms5003data data;

void loop() {

      //NEW DISPLAY THINGS//////////////////////////////////////////////////////////////////////////////////////////
  tft.fillScreen(TFT_BLACK);
  
  tft.setCursor(0, 0, 2);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);  tft.setTextSize(1);
  
  Serial.println();

  /*float valuepm10 = data.pm10_standard;
  float valuepm25 = data.pm25_standard;
  float valuepm100 = data.pm100_standard;*/


  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }


  printtodisplay();//prints sensor data to display
  printtoserial();//prints sensor data to serial
  leds();//traffic light leds

  //Serial.print("Gas = "); Serial.print(bme.gas_resistance / 1000.0); Serial.println(" KOhms");

  /*
  if (readPMSdata(&Serial1)) {
    // reading data was successful!
    //Serial.println();
    //Serial.println("---------------------------------------");
    //Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(valuepm10);
    Serial.print("\t\tPM 2.5: "); Serial.print(valuepm25);
    Serial.print("\t\tPM 10: "); Serial.println(valuepm100);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
    Serial.println("---------------------------------------");
  }*/

//////////////////////////////////////////////////////////////////////////////////////////////////////// WIFI and DB 

// Clear fields for reusing the point. Tags will remain the same as set above.
    sensor.clearFields();
  
    // Store measured value into point
    sensor.addField("reading", readcounter);
    sensor.addField("temperature", bme.temperature);
    sensor.addField("humidity", bme.humidity);
    sensor.addField("pressure", bme.pressure / 100);
    //sensor.addField("pm10", valuepm10);
    //sensor.addField("pm25", valuepm25);
    //sensor.addField("pm100", valuepm100);

    
    // Check WiFi and InfluxDB connection
    /*if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Wifi not connected");
      display.println("Wifi not connected");
    }else{
      Serial.println("Wifi connected");
      display.println("Wifi connected");
    }*/
  
    /*if (!client.writePoint(sensor)) {
      Serial.print("InfluxDB "); Serial.println(client.getLastErrorMessage());
    }else{
      Serial.println("InfluxDB connected");
      display.println("InfluxDB connected");
    }*/
    //client writepoint writes to database
    //tolineprotocol just text thing?

    //checks connection to db and wifi
    if (client.writePoint(sensor) && WiFi.status() == WL_CONNECTED){
          //prints what is being pushed to db
          Serial.print("Writing: "); Serial.println(sensor.toLineProtocol());
    }


    readcounter++; 
    delay(10000); 
}

void printtodisplay(){
  tft.print("Temperature: "); tft.print(bme.temperature); tft.println(" °C");
  tft.print("Humidity: "); tft.print(bme.humidity); tft.println(" %");
  tft.print("Pressure: "); tft.print(bme.pressure / 100.0); tft.println(" hPa");
  tft.print("TVOC: "); tft.print(ENS160.getTVOC()); tft.println(" ppb");
  tft.print("eCO2: "); tft.print(ENS160.getECO2()); tft.println(" ppm");
}

void printtoserial(){
  Serial.print("Temperature: "); Serial.print(bme.temperature); Serial.println(" °C");
  Serial.print("Humidity: "); Serial.print(bme.humidity); Serial.println(" %");
  Serial.print("Pressure: "); Serial.print(bme.pressure / 100.0); Serial.println(" hPa");
  Serial.print("TVOC: "); Serial.print(ENS160.getTVOC()); Serial.println(" ppb");
  Serial.print("eCO2: "); Serial.print(ENS160.getECO2()); Serial.println(" ppm");
}

void pushtodatabase(){

}

void beep(int x){
  if(x==1){
    tone(16, 500, 200);
  }
  if(x==2){
    tone(16, 500, 100);
    tone(16, 1000, 100);
  }
  if(x==3){
    tone(16, 500, 50);
    tone(16, 1000, 50);
    tone(16, 1500, 50);
  }
}

void leds(){
    if(bme.humidity >= 52){
      digitalWrite(12, HIGH); //red led
      digitalWrite(26, LOW);
      digitalWrite(32, LOW);
    }else if(bme.humidity < 52 && bme.humidity >= 48){
      digitalWrite(12, LOW);
      digitalWrite(26, HIGH); //yellow led
      digitalWrite(32, LOW);
    }else{
      digitalWrite(12, LOW);
      digitalWrite(26, LOW);
      digitalWrite(32, HIGH); //green led
    }
}
/*
boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}*/