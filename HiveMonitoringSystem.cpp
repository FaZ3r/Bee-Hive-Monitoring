//======================================================================================//
//                                                                                      //
//                 Hivemon V0.1.8 Firmware                             //
//                                                                                      //
//               //
//             Mod  by Octavian Ureche  Last Update: 02.09.2023                                                                        //
//======================================================================================//
// InfluxDB v2 server url, e.g. https://eu-central-1-1.aws.cloud2.influxdata.com (Use: InfluxDB UI -> Load Data -> Client Libraries)
  #define INFLUXDB_URL "https://eu-central-1-1.aws.cloud2.influxdata.com"

  //#define INFLUXDB_TOKEN "csuUk4To1FccrvKV_uz3gOtyZq9iTwB8-26Y4ROL8lBYtI8qWBuKQx6zWnciTD4YJ6idPoY8SaiXWiM5ZwKanA=="
 
  #define INFLUXDB_TOKEN "InfluxDBToken"
  #define INFLUXDB_ORG "ORG-NUMBER"
  #define INFLUXDB_BUCKET "ESP2"
// Set timezone string according to https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
// Examples:
//  Pacific Time:   "PST8PDT"
//  Eastern:        "EST5EDT"
//  Japanesse:      "JST-9"
//#define TZ_INFO "CET-1CEST,M3.5.0,M10.5.0/3"               // Central Europe:
#define TZ_INFO "EET-2EEST,M3.5.0/3,M10.5.0/4"
//#define TZ_INFO "UTC2"
#include "Arduino.h"
#include "WiFiMulti.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include "HX711.h"
#include "esp_adc_cal.h"
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

//=================== Definitions ==========================================


#define ssid "SSID"
#define pass "PASSWD"

#define DEVICE "ESP32"
#define LOCATION "LOCATION"
//#define SENSOR "sensor_data"


#define DHTPIN 					      27        // Digital pin connected to the DHT sensor
#define DHTTYPE 			        DHT22  	  // DHT 22 (AM2302)
#define DHT22_REFRESH_TIME		(10000u)	// Two seconds sensor refresh time

#define VOLT_PIN 				      36        //ADC pin used to read battery voltage
#define uS_TO_S_FACTOR			  1000000UL // Conversion factor from microseconds to seconds
#define TIME_TO_SLEEP 			  900     
 // Time ESP 32 will go to sleep (in seconds) 
//
float script_execution_start = millis();
float script_execution_end;
//=======================================================================================

HX711 scale;
DHT dht(DHTPIN, DHTTYPE);

// InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Data point
Point sensor("Sensor_Data");

WiFiMulti wifiMulti;
//=========================Declaring Variables and Constants ==================================

// Variables used in reading data pin and clock pin for the HX711 
uint8_t dataPin  = 19;				  //for ESP32
uint8_t clockPin = 18;                //for ESP32
// Variable used to pass scale readings
float HX711_weight = 0;
static void load_cell();

// Variables used in reading temperature and humidity (DHT22)
float dht22_temperature = 0;
float dht22_humidity = 0;
// DHT22 function declaration
static void DHT22_Read();

// Variable used in calculating the battery voltage
float calibration  = 0.9792; // Adjust for ultimate accuracy when input is measured using an accurate DVM, if reading too high then use e.g. 0.99, too low use 1.01
float vref = 1100;
// Variable used to pass scale readings
float batteryVoltage = 0;
static void Read_Battery_Voltage();

//=========================Deep Sleep Time ================================================

const uint64_t UpdateInterval = TIME_TO_SLEEP * uS_TO_S_FACTOR;  // Sleep time

//========================= Variables for wifi server setup =============================

//============================= Function definitions ===================================================//
//===================================================================================================//
  static void load_cell()
  {
    float w;
    scale.power_up(); 
    Serial.println("Scale power on.");                      // scale wake up
    delay(500);
    
    scale.begin(dataPin, clockPin);
    scale.set_scale(-22.93);   		  		// insert here scale obtained from SCALE_W_CALIBRATION.ino
    scale.set_offset(-206867);            		// insert here offset obtained from SCALE_W_CALIBRATION.ino
    //
    //  Serial.println(scale.get_scale());		//for checking
    //  Serial.println(scale.get_offset());		//for checking
    //  Serial.println("\nScale is calibrated");	//for checking

    //  Read scale value
    w = scale.get_units(10);					// weight value in grams    
    //  Serial.print("Weight [g]: ");
    //  Serial.println(scale.get_units(10));  
    // Check if value is valid or not
    if (isnan(w) )
      {
        Serial.println("Failed to read scale data");
      }
    else 
      {
        Serial.print("Weight: ");
        Serial.print(w);
		Serial.println(" g");
       //update the global variables
        HX711_weight = (float)(w);
      }
    Serial.println("\nScale power off.\n"); 
    scale.power_down();
  }
//===================================================================================================//
  static void DHT22_Read()
  {
    float t, h;
    dht.begin();
    delay(2000);
    // Read Temperature and Humidity
    t = dht.readTemperature();
    h = dht.readHumidity();
    // Check if values are valid or not
    if (isnan(t) || isnan(h))
      {
        Serial.println("Failed to read sensor data.");
      }
    else 
      {
        Serial.print("Air temperature: ");
        Serial.print(t);
		Serial.println("°C");
        Serial.print("Humidity: ");
        Serial.print(h);
        Serial.println(" %");
		//update the global variables
        dht22_temperature = (float)(t);
        dht22_humidity = (float)(h);
      }
  }
//===================================================================================================//
    static void Read_Battery_Voltage()
  {
    float bv;
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    vref = adc_chars.vref; // Obtain the device ADC reference voltage
    
    bv = (analogRead(VOLT_PIN) / 4095.0) * 4.2 * (1100 / vref) * calibration; // Read from ADC pin 36 voltage supplied by voltage divider
    //   Serial.println((float)analogRead(VOLT_PIN),2);
    //   Check if value is valid or not
      if (isnan(bv))
      {
        Serial.println("Failed to read battery voltage.");
      }
      else 
      {
        Serial.print("Battery Level: ");
        Serial.print(bv,3);
		Serial.println(" V");
        Serial.println();	
              //update the global variables
        batteryVoltage = (float)(bv);
      }
  }
//===================================================================================================//
  static void Wifi_connect()
  {
    // Connect WiFi
    Serial.print("Connecting to wifi");
	WiFi.mode(WIFI_STA);
	WiFi.disconnect();
    wifiMulti.addAP(ssid, pass);
  
    while (wifiMulti.run() != WL_CONNECTED) 
	{
      Serial.print(".");
      delay(100);
    }
    Serial.println();
    Serial.print("Local IP adress of Hivemon: ");
    Serial.println(WiFi.localIP());
	Serial.print("RRSI: ");
    Serial.println(WiFi.RSSI());
  }
//===================================================================================================//
  static void InfluxDB_Init()
  {
 // Add tags to the data point
	
	sensor.addTag("device", DEVICE);

 // Accurate time is necessary for certificate validation and writing in batches
    // We use the NTP servers in your area as provided by: https://www.pool.ntp.org/zone/
    // Syncing progress and the time will be printed to Serial.
    timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
 
    // Check server connection
    if (client.validateConnection()) {
      Serial.print("Connected to InfluxDB: ");
      Serial.println(client.getServerUrl());
    } 
	else 
	{
      Serial.print("InfluxDB connection failed: ");
      Serial.println(client.getLastErrorMessage());
    }
  }
//===================================================================================================//  
 static void InfluxDB_TaskMng()
 {
    sensor.clearFields();
    // Add readings as fields to point
    sensor.addField("temperature", float(dht22_temperature));
    sensor.addField("humidity", float(dht22_humidity));
    sensor.addField("batteryVoltage", float(batteryVoltage));
    sensor.addField("weight", float(HX711_weight));

    // Print what are we exactly writing
    Serial.print("Writing: ");
    Serial.println(client.pointToLineProtocol(sensor));
    
	// If no WiFi signal, try to recconect
	if (wifiMulti.run() != WL_CONNECTED)
	{
      Serial.print("WiFi connection lost");	
	}
	// Write point into buffer
	if (!client.writePoint(sensor))
	{
	  Serial.print("InfluxD write failed");
	}	  
//    client.writePoint(sensor);
 }  
//===================================================================================================//

//===================================================================================================//
//
//                           ESP32 Deep Sleep Mode
//
//===================================================================================================// 
  void Deep_Sleep_Now()
  {
    esp_sleep_enable_timer_wakeup(UpdateInterval);
    
    Serial.println("Hivemon going to sleep for .." + String(TIME_TO_SLEEP/60) + " minutes.");
    
    esp_deep_sleep_start();
  }

//========================= Setup Function ================================================
void setup() 
{
  Serial.begin(115200);
  delay(25);
  Serial.println("\nHivemon powered on.\n");
  Wifi_connect();
  InfluxDB_Init();
}
//===================================================================================================//

void loop() 
{
Serial.println("Start loop."); 
load_cell();
DHT22_Read();
Read_Battery_Voltage();

InfluxDB_TaskMng();

script_execution_end = millis();
float duration = script_execution_end - script_execution_start;
Serial.println("Script execution time = "+ String(duration/1000) + "secunde");

Deep_Sleep_Now();
}
//===================================================================================================//

//=============================End of the Program ===================================================//
