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
#include <ESP32Encoder.h>





#include "Adafruit_PM25AQI.h"
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();





//#define I2C_COMMUNICATION
DFRobot_ENS160_I2C ENS160(&Wire, 0x53);





// Invoke library, pins defined in User_Setup.h
TFT_eSPI tft = TFT_eSPI(); 


const char* ssid = "I am gonna kill your fucking dog";
const char* password = "morangoscomacucar";

#define DEVICE "ESP32"
#define INFLUXDB_URL "https://eu-central-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "OvK1qsQXDixTlkudQhyGPr9STmRJH1tNfD-Ma3-T7CqPfEi1i4mqTLZfegJ9Q-ZCp0Ef7t4j_HQUS457ThNt9A=="
#define INFLUXDB_ORG "a932d6a20e239b6a"
#define INFLUXDB_BUCKET "humidityandtemperature"
// Time zone for database
#define TZ_INFO "UTC-1"
// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
// Declare Data point
Point sensor("wifi_status");



//BME680////////////////////////////////////////////////
#define SEALEVELPRESSURE_HPA (1017)
Adafruit_BME680 bme;
float hum_weighting = 0.25; // humidity has 25% weight for IAQ score
float gas_weighting = 0.75; // gas has 75% weight for IAQ score
int   humidity_score, gas_score, air_quality_score;
float gas_reference = 2500;
float hum_reference = 40;
int   getgasreference_count = 0;
int   gas_lower_limit = 10000;  // Bad air quality limit
int   gas_upper_limit = 300000; // Good air quality limit
int   iaq;
////////////////////////////////////////////////////////
//LEDS//////////////////////////////////////////////////
#define RED_PIN 13
#define YELLOW_PIN 12
#define GREEN_PIN 14
////////////////////////////////////////////////////////
//BUZZER////////////////////////////////////////////////
#define BUZZER_PIN 27
////////////////////////////////////////////////////////
//ENCODER///////////////////////////////////////////////
ESP32Encoder encoder;
String refresh;//need a separate frequency for encoder
unsigned long encodercurrentMillis;
unsigned long encoderpreviousMillis = 0UL; 
////////////////////////////////////////////////////////


// Connections for PM2.5 Sensor
#define RXD2 34 // To sensor TXD
#define TXD2 33 // To sensor RXD

    int particles03um;
    int particles05um;
    int particles10um;
    int particles25um;
    int particles50um;
    int particles100um;
    int pm10standard;
    int pm25standard;
    int pm100standard;


unsigned long previousMillis = 0UL; //to replace delays
unsigned long interval = 10000UL; //to replace delays
unsigned long currentMillis;


//function declaration/prorotypes
void printtodisplay();
void printtoserial();
void beep(int x);
void leds();
void pushtodatabase();
String CalculateIAQ(int score);
void GetGasReference();
int GetHumidityScore();
int GetGasScore();

boolean readPMSdata(Stream* stream);


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);//serial just for pms5003
  while (!Serial1);

  Wire.begin();
  bme.begin();



  //bme680 oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms 
  // Now run the sensor to normalise the readings, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
  // The sensor takes ~30-mins to fully stabilise
  GetGasReference();






  ESP32Encoder::useInternalWeakPullResistors=UP;

	// use pin 19 and 18 for the first encoder
	encoder.attachHalfQuad(26, 25);

	// set starting count value after attaching
	encoder.setCount(50);


  // Init the TVOC and eCO2
  //while( NO_ERR != ENS160.begin() ){
    //Serial.println("Communication with device failed, please check connection");
    //delay(3000);
  //}
  Serial.println("Begin ok!");

  ENS160.setPWRMode(ENS160_STANDARD_MODE);

  ENS160.setTempAndHum(/*temperature=*/25.0, /*humidity=*/50.0);



  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);//buzzer
  beep(1);//one beep on startup

  /////////////////////////////////////////////////////////////////////////////////////////WIFI and DB
    WiFi.begin(ssid, password);
    //Serial.println("Connecting");
    int i = 0;
    while(WiFi.status() != WL_CONNECTED && i <100){
        //Serial.print(".");
        i++;
        delay(100);
    }





    if(WiFi.status() == WL_CONNECTED){
      beep(2);//two beeps when connected to wifi
      Serial.println("Connected to WiFi");
      Serial.print("Local ESP32 IP: "); Serial.println(WiFi.localIP());
      timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");  
    }else{
      Serial.println("Coudn't connect to WIFI");
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

  tft.init(); //initialize tft display
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 1);
  tft.setRotation(0); //set display rotation



}

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data data;

void loop() {


  if(encoder.getCount()>100){
    encoder.setCount(100);
  }else if(encoder.getCount()<0){
    encoder.setCount(0);
  }


    if(encoder.getCount()>=100){
    interval = 300000UL;
    refresh = "300s   \n/                   ";
  }else if(encoder.getCount()<100 && encoder.getCount() >=90){
    interval = 200000UL;
    refresh = "200s   \n///                 ";
  }else if(encoder.getCount()<90 && encoder.getCount() >=80){
    interval = 150000UL;
    refresh = "150s   \n/////               ";
  }else if(encoder.getCount()<80 && encoder.getCount() >=70){
    interval = 100000UL;
    refresh = "100s   \n///////            ";
  }else if(encoder.getCount()<70 && encoder.getCount() >=60){
    interval = 80000UL;
    refresh = "80s    \n/////////           ";
  }else if(encoder.getCount()<60 && encoder.getCount() >=50){
    interval = 60000UL;
    refresh = "60s    \n///////////         ";
  }else if(encoder.getCount()<50 && encoder.getCount() >=40){
    interval = 50000UL;
    refresh = "50s    \n/////////////        ";
  }else if(encoder.getCount()<40 && encoder.getCount() >=30){
    interval = 40000UL;
    refresh = "40s    \n///////////////      ";
  }else if(encoder.getCount()<30 && encoder.getCount() >=20){
    interval = 30000UL;
    refresh = "30s    \n/////////////////    ";
  }else if(encoder.getCount()<20 && encoder.getCount() >=10){
    interval = 20000UL;
    refresh = "20s    \n///////////////////  ";
  }else if(encoder.getCount()<10 && encoder.getCount() >=0){
    interval = 1000UL;
    refresh = "10s    \n/////////////////////";
  }
  

  if (readPMSdata(&Serial1)){ //pms5003 is a little bitch and needs to read constantly like this or else it will throw a checksum error
    particles03um = data.particles_03um;
    particles05um = data.particles_05um;
    particles10um = data.particles_10um;
    particles25um = data.particles_25um;
    particles50um = data.particles_50um;
    particles100um = data.particles_100um;
    pm10standard = data.pm10_standard;
    pm25standard = data.pm25_standard;
    pm100standard = data.pm100_standard;
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////// WIFI and DB 

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
    encodercurrentMillis = millis();

    if(encodercurrentMillis - encoderpreviousMillis > 17L){
 	    encoderpreviousMillis = encodercurrentMillis;  
        tft.setCursor(0, 145, 1);
        tft.setTextColor(TFT_WHITE,TFT_BLACK);
        tft.print("Update Freq - ");
        tft.print(refresh);


    }
      

    currentMillis = millis();

    if(currentMillis - previousMillis > interval){
 	    previousMillis = currentMillis;  
      
      humidity_score = GetHumidityScore();
      gas_score      = GetGasScore();

      //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
      air_quality_score = humidity_score + gas_score;
      if ((getgasreference_count++) % 5 == 0) GetGasReference();
      Serial.println(CalculateIAQ(air_quality_score));

      printtodisplay();//prints sensor data to display
      printtoserial();//prints sensor data to serial
      leds();//traffic light leds
      pushtodatabase();//push sensor data to db

      }
    

  }

void printtodisplay(){
  tft.setCursor(0, 0, 1);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.print("Temperature: "); tft.print(bme.temperature); tft.println(" C");
  tft.print("Humidity: "); tft.print(bme.humidity); tft.println(" %");
  tft.print("Pressure: "); tft.print(bme.pressure / 100.0); tft.println(" hPa");
  //tft.print("TVOC: "); tft.print(ENS160.getTVOC()); tft.println(" ppb");
  //tft.print("eCO2: "); tft.print(ENS160.getECO2()); tft.println(" ppm");
  tft.println("PM 1.0 = " + String(pm10standard)                      );
  tft.println("PM 2.5 = " + String(pm25standard)                      );
  tft.println("PM 10.0 = " + String(pm100standard)               + "\n");
}

void printtoserial(){
  Serial.println();
  Serial.println("  Update Freq = " + String(interval/1000)               + "s\n");
  Serial.println("  Temperature = " + String(bme.temperature)             + "°C");
  Serial.println("     Humidity = " + String(bme.humidity)                + "%");
  Serial.println("     Pressure = " + String(bme.pressure / 100)          + " hPa");
  Serial.println("          Gas = " + String(gas_reference)               + " ohms\n");
  //Serial.println("         TVOC = " + String(ENS160.getTVOC())            + "ppb");
  //Serial.println("         eCO2 = " + String(ENS160.getECO2())            + "ppm\n");
  Serial.println("       PM 1.0 = " + String(pm10standard)                      );
  Serial.println("       PM 2.5 = " + String(pm25standard)                      );
  Serial.println("      PM 10.0 = " + String(pm100standard)               + "\n");
  Serial.println("---------------------------------------");
  Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(particles03um);
  Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(particles05um);
  Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(particles10um);
  Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(particles25um);
  Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(particles50um);
  Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(particles100um);

}

void pushtodatabase(){
  //Clear fields for reusing the point
  sensor.clearFields();
  // Store measured value into point
  sensor.addField("temperature", bme.temperature);
  sensor.addField("humidity", bme.humidity);
  sensor.addField("pressure", bme.pressure / 100);
  //sensor.addField("tvoc", ENS160.getTVOC());
  //sensor.addField("eco2", ENS160.getECO2());
  sensor.addField("pm10", pm10standard);
  sensor.addField("pm25", pm25standard);
  sensor.addField("pm100", pm100standard);
  sensor.addField("particles03", particles03um);
  sensor.addField("particles05", particles05um);
  sensor.addField("particles10", particles10um);
  sensor.addField("particles25", particles25um);
  sensor.addField("particles50", particles50um);
  sensor.addField("particles100", particles100um);
  sensor.addField("airqualityscore", (100-air_quality_score) * 5);

  if (client.writePoint(sensor) && WiFi.status() == WL_CONNECTED){
    //prints what is being pushed to db
    Serial.print("Writing: "); Serial.println(sensor.toLineProtocol());
  }
}

void beep(int x){
  if(x==1){
    tone(BUZZER_PIN, 500, 200);
  }
  if(x==2){
    tone(BUZZER_PIN, 500, 100);
    tone(BUZZER_PIN, 1000, 100);
  }
  if(x==3){
    tone(BUZZER_PIN, 500, 75);
    tone(BUZZER_PIN, 1000, 75);
    tone(BUZZER_PIN, 1500, 75);
  }
}

void leds(){
    if(bme.humidity >= 60){
      digitalWrite(RED_PIN, HIGH); //red led
      digitalWrite(YELLOW_PIN, LOW);
      digitalWrite(GREEN_PIN, LOW);
    }else if(bme.humidity < 60 && bme.humidity >= 50){
      digitalWrite(RED_PIN, LOW);
      digitalWrite(YELLOW_PIN, HIGH); //yellow led
      digitalWrite(GREEN_PIN, LOW);
    }else{
      digitalWrite(RED_PIN, LOW);
      digitalWrite(YELLOW_PIN, LOW);
      digitalWrite(GREEN_PIN, HIGH); //green led
    }
}

//fixes the problem with adafruit's pms library not working after 24h, no idea how it works
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
}

void GetGasReference() {
  // Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
  Serial.println("Getting a new gas reference value");
  int readings = 10;
  for (int i = 1; i <= readings; i++) { // read gas for 10 x 0.150mS = 1.5secs
    gas_reference += bme.readGas();
  }
  gas_reference = gas_reference / readings;
  Serial.println("Gas Reference = "+String(gas_reference,3));
}

String CalculateIAQ(int score) {
  String IAQ_text = "air quality is ";
  score = (100 - score) * 5;
  if      (score >= 301)                  IAQ_text += "Hazardous";
  else if (score >= 201 && score <= 300 ) IAQ_text += "Very Unhealthy";
  else if (score >= 176 && score <= 200 ) IAQ_text += "Unhealthy";
  else if (score >= 151 && score <= 175 ) IAQ_text += "Unhealthy for Sensitive Groups";
  else if (score >=  51 && score <= 150 ) IAQ_text += "Moderate";
  else if (score >=  00 && score <=  50 ) IAQ_text += "Good";
  Serial.print("IAQ Score = " + String(score) + ", ");
  return IAQ_text;
}

int GetHumidityScore() {  //Calculate humidity contribution to IAQ index
  float current_humidity = bme.readHumidity();
  if (current_humidity >= 38 && current_humidity <= 42) // Humidity +/-5% around optimum
    humidity_score = 0.25 * 100;
  else
  { // Humidity is sub-optimal
    if (current_humidity < 38)
      humidity_score = 0.25 / hum_reference * current_humidity * 100;
    else
    {
      humidity_score = ((-0.25 / (100 - hum_reference) * current_humidity) + 0.416666) * 100;
    }
  }
  return humidity_score;
}

int GetGasScore() {
  //Calculate gas contribution to IAQ index
  gas_score = (0.75 / (gas_upper_limit - gas_lower_limit) * gas_reference - (gas_lower_limit * (0.75 / (gas_upper_limit - gas_lower_limit)))) * 100.00;
  if (gas_score > 75) gas_score = 75;
  if (gas_score <  0) gas_score = 0;
  return gas_score;
}