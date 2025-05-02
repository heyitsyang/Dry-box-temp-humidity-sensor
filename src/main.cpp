
/* The LilyGo T7 S3 MCU used for this project can be programmed
   using the USB-C port, but the monitor port must use the secondary
   serial pins on the board (TX, RX pins) via TTL-serial adapter
*/

/********************************
 *           ****               *
 * Dry box Temp/Humidity Sensor *
 *           ****               *
 ********************************/
#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <ESPmDNS.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266mDNS.h>
#endif
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
// add the below libraries from the Library Manager
//#include <TelnetStream.h>
#include <SensirionI2cSht4x.h>
#include <PubSubClient.h>
#include <ezTime.h>

// local definitions
#include "prototypes.h"
#include "credentials.h"    // <<<<<<<  COMMENT THIS LINE OUT & ENTER YOUR CREDENTIALS BELOW - this contains stuff for my WIFI network, not yours
#include "esp_adc_cal.h"    // from LilyGo TZ example code
#include "math.h"           // needed to calculate absolute humidity

// name the device
#define DEVICE_HOST_NAME "dry-box"

// TIME SETTINGS
#define MY_TIMEZONE "America/New_York"               // <<<<<<< use Olson format: https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
#define TIMEZONE_EEPROM_OFFSET 0                     // location-to-timezone info - saved in case eztime server is down

#define VERSION "Ver 0.2 build 2025.05.1"

// GPIO PIN DEFINITIONS
#define BATT_ADC_PIN 2
#define BUILT_IN_LED_PIN 17
#define I2C_SDA_PIN 47
#define I2C_SCL_PIN 46
#define BUTTON_PIN 12
#define LED_ON HIGH
#define LED_OFF LOW

// OPERATIONAL PARAMETERS & PREFERENCES
#define PREFER_FAHRENHEIT 1
#define DEEP_SLEEP_SECS  3600                        // deep sleep duration
#define uS_TO_S_FACTOR 1000000ULL                    // Conversion factor for micro seconds to seconds
#define FULL_BATT_VOLTAGE 4.1                        // Voltage of battery when fully charged

#undef SHT4X_NO_ERROR                                // make sure that we use the proper definition of SHT4X_NO_ERROR
#ifdef SHT4X_NO_ERROR
#endif
#define SHT4X_NO_ERROR 0

// MQTT
#define MQTT_MSG_BUFFER_SIZE 512                            // for MQTT message payload
#define MQTT_MAX_TOPIC_SIZE 1024                            // max topic string size(can be up to 65535)
#define MAX_MQTT_CONNECT_ATTTEMPTS 10


#define DRY_BOX_LWT_TOPIC "drybox/status/LWT"                  // MQTT Last Will & Testament
#define DRY_BOX_VERSION_TOPIC "drybox/version"                 // report software version at connect
#define DRY_BOX_WIFI_STRENGTH_TOPIC "drybox/wifi_dbm"
#define DRY_BOX_BATTERY_VOLTS_TOPIC "drybox/battery_volts"
#define DRY_BOX_BATTERY_PERCENT_TOPIC "drybox/battery_percent"
#define DRY_BOX_REPORT_TIME_STAMP_TOPIC "drybox/time_stamp"     // time sensor is read & reported
#define DRY_BOX_RELATIVE_HUMIDITY_TOPIC "drybox/relative_humidity"
#define DRY_BOX_ABSOLUTE_HUMIDITY_TOPIC "drybox/absolute_humidity"
#define DRY_BOX_TEMPERATURE_TOPIC "drybox/temperature"          // flow sensed when all valves are off & there should be none

#define DRY_BOX_RECV_COMMAND_TOPIC "drybox/cmd/#"

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Timezone myTZ;

esp_sleep_wakeup_cause_t wakeup_reason;

char mqttMsg[MQTT_MSG_BUFFER_SIZE];
char mqttTopic[MQTT_MAX_TOPIC_SIZE];
unsigned long mqttNow, lastReconnectAttempt = 0;

SensirionI2cSht4x sht_sensor;
static char sht_errorMessage[64];
static int16_t sht_error;
float shtTemperature = 0.0;
float shtHumidity = 0.0;

unsigned long waitTick;
int i;

/**********************
 *      SETUP
 **********************/
void setup()
{
  pinMode(BATT_ADC_PIN, INPUT);                // battery voltage input
  pinMode(BUILT_IN_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUILT_IN_LED_PIN, LED_ON);        // LED on
    waitTick = millis();
    while( (millis() - waitTick) < 200 );     // 200ms blink on 
  digitalWrite(BUILT_IN_LED_PIN, LED_OFF);       // LED off

  Serial.begin(115200);
  while (!Serial) 
    delay(10);
  Serial.printf("\n\n\nDry-box Temp/Hum Sensor %s\n", VERSION);
  
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  sht_sensor.begin(Wire, SHT40_I2C_ADDR_44);
  sht_sensor.softReset();
  delay(10);
  uint32_t serialNumber = 0;
  sht_error = sht_sensor.serialNumber(serialNumber);
  if (sht_error != SHT4X_NO_ERROR) {
      Serial.print(F("Error trying to execute serialNumber(): "));
      errorToString(sht_error, sht_errorMessage, sizeof sht_errorMessage);
      Serial.println(sht_errorMessage);
  }
  Serial.print("SHT sensor serialNumber: ");
  Serial.print(serialNumber);
  Serial.println();
  Serial.printf("Battery voltage: %f", readBatteryVoltage());

  initWiFi();
  waitForSync();  //sync the time
  setInterval(0);  //do not periodically sync NTP
  setup_OTA();
  myTZ.setLocation(F(MY_TIMEZONE)); 
  Serial.printf("Got local time: %s\n", myTZ.dateTime("[H:i:s.v]").c_str());

  wakeup_reason = esp_sleep_get_wakeup_cause();
  print_wakeup_reason(wakeup_reason);

  if (wakeup_reason == 0)   // first boot, so wake not due to sleep timer
  {
    digitalWrite(BUILT_IN_LED_PIN, LED_OFF);        // ensure is LED off
    Serial.println("You have 15 secs to start uploading new firmware");
    for(i=0; i<15; i++)
    {
      waitTick = millis();
      while( (millis() - waitTick) < 1000 )
      {
        ArduinoOTA.handle();                        // remember program never returns from OTA
      }
    }
    sendMQTT();
  }
  else
    sendMQTT();
  GoToSleep();
  Serial.println(F("SHOULD NEVER REACH THIS"));

}

/*********************** 
 *       LOOP
 ***********************/
void loop()
{ }  // never reaches here }




/*********************************
 *        SUBROUTINES
 *********************************/

/*
 * sendMQTT
 */
void sendMQTT()
{
  unsigned long startTick, pauseTick;


  connectMQTT();

  mqttClient.publish(DRY_BOX_VERSION_TOPIC, VERSION, true); // report firmware version
  Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), DRY_BOX_VERSION_TOPIC, VERSION);

  sprintf(mqttMsg, "%d", WiFi.RSSI());
  mqttClient.publish(DRY_BOX_WIFI_STRENGTH_TOPIC, mqttMsg, true);
  Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), DRY_BOX_WIFI_STRENGTH_TOPIC , "true");

  mqttClient.publish(DRY_BOX_REPORT_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str(), true);
  Serial.printf("%s MQTT SENT: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), DRY_BOX_REPORT_TIME_STAMP_TOPIC, myTZ.dateTime(RFC3339).c_str());

  sendBatteryStatus();
  sendTempHumStatus();
  
  return;
}

/*
* initWiFi
*/
void initWiFi() 
{
  unsigned long pauseTick;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  delay(1000);
  Serial.print("\nMAC address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("\n\nConnecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) 
  {
      digitalWrite(BUILT_IN_LED_PIN, LOW);        // LED off
      pauseTick = millis();
      while( (millis() - pauseTick) < 400 );
      digitalWrite(BUILT_IN_LED_PIN, HIGH);        // LED on
      pauseTick = millis();
      while( (millis() - pauseTick) < 400 );
  }
  Serial.println(F(""));
  Serial.print(F("WiFi connected to "));
  Serial.println(WIFI_SSID);
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());
  Serial.print(F("RRSI: "));
  Serial.println(WiFi.RSSI());
}

/*
 * setupOTA
 */
void setup_OTA()
{
  // Port defaults to 8266
  //  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(DEVICE_HOST_NAME);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else
      type = "filesystem";  // U_FS

    // NOTE: if updating FS this is the place to unmount FS using FS.end()

    Serial.println("OTA Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println(F("\nOTA End"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println(F("OTA Auth Failed"));
    else if (error == OTA_BEGIN_ERROR)
      Serial.println(F("OTA Begin Failed"));
    else if (error == OTA_CONNECT_ERROR)
      Serial.println(F("OTA Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println(F("OTA Receive Failed"));
    else if (error == OTA_END_ERROR)
      Serial.println(F("OTA End Failed"));
  });
  ArduinoOTA.begin();
}

/*
 * connectMQTT
 */
void connectMQTT()
{
  int connectAttemptCount = 0;
  mqttClient.setBufferSize(MQTT_MSG_BUFFER_SIZE);
  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(callback);
  while(connectAttemptCount < MAX_MQTT_CONNECT_ATTTEMPTS)
  {
    if (!mqttClient.connected())
    {
      mqttNow = millis();
      if (mqttNow - lastReconnectAttempt > 1000)
      {
        Serial.printf("[%s] Waiting for MQTT...\n", myTZ.dateTime(RFC3339).c_str());
        lastReconnectAttempt = mqttNow;
        connectAttemptCount++;
        // Attempt to reconnect
        if (reconnect())
        {
          mqttClient.publish(DRY_BOX_LWT_TOPIC, "Connected", true); // let broker know we're connected
          Serial.printf("\n%s MQTT SENT: %s/Connected\n", myTZ.dateTime("[H:i:s.v]").c_str(), DRY_BOX_LWT_TOPIC);
          lastReconnectAttempt = 0;
          return;
        }
      }
    }
  }
  Serial.println("ERROR-----Max MQTT connect attempts exceeded");
}

/*
 * MQTT callback
 */
void callback(char *topic, byte *payload, unsigned int length)
{
  // handle MQTT message arrival
  bool cmdValid = false;
  strncpy(mqttMsg, (char *)payload, length);
  mqttMsg[length] = (char)NULL; // terminate the string
  Serial.printf("\n%s MQTT RECVD: %s/%s \n", myTZ.dateTime("[H:i:s.v]").c_str(), topic, mqttMsg);
}

/*
 * MQTT reconnect
 */
boolean reconnect()
{

  // PubSubClient::connect(const char *id, const char *user, const char *pass, const char* willTopic, uint8_t willQos, boolean willRetain, const char* willMessage)
  if (mqttClient.connect(DEVICE_HOST_NAME, MQTT_USER_NAME, MQTT_PASSWORD, DRY_BOX_LWT_TOPIC, 2, true, "Disconnected"))
  {
    Serial.print(F("MQTT connected to "));
    Serial.println(F(MQTT_SERVER));
  }
  return mqttClient.connected();
}

/*
 * sendTempHumStatus
 */
void sendTempHumStatus()
{
  float shtTemperature, shtRelHumidity, shtAbsHumidity;
  
  sht_error = sht_sensor.measureHighPrecision(shtTemperature, shtRelHumidity);
  if (sht_error != SHT4X_NO_ERROR) {
      Serial.print("Error trying to execute measureHighPrecision(): ");
      errorToString(sht_error, sht_errorMessage, sizeof sht_errorMessage);
      Serial.println(sht_errorMessage);
      return;
  }

  shtAbsHumidity = calculate_absolute_humidity(shtTemperature, shtRelHumidity);

  if (PREFER_FAHRENHEIT)
      shtTemperature = (shtTemperature * 9/5) + 32 ;
  
  sprintf(mqttMsg, "%.2f", shtTemperature);
  mqttClient.publish(DRY_BOX_TEMPERATURE_TOPIC, mqttMsg, true);
  Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), DRY_BOX_TEMPERATURE_TOPIC, mqttMsg);
  sprintf(mqttMsg, "%.2f", shtRelHumidity);
  mqttClient.publish(DRY_BOX_RELATIVE_HUMIDITY_TOPIC, mqttMsg, true);
  Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), DRY_BOX_RELATIVE_HUMIDITY_TOPIC, mqttMsg);
  sprintf(mqttMsg, "%.2f", shtAbsHumidity);
  mqttClient.publish(DRY_BOX_ABSOLUTE_HUMIDITY_TOPIC, mqttMsg, true);
  Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), DRY_BOX_ABSOLUTE_HUMIDITY_TOPIC, mqttMsg);
}

/*
 * readADC_Cal
 */
uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

/*
 * readBatteryVoltage
 */
float readBatteryVoltage()
{
  float voltage = 0.0;
  uint32_t Vbatt = 0;

  voltage = ((float)((readADC_Cal(analogRead(BATT_ADC_PIN))) * 2))/1000;
  return(voltage);
}

/*
 * sendBatteryStatus
 */
void sendBatteryStatus()
{
  float volts;
  uint8_t percentage = 0;
  
  volts = readBatteryVoltage();;
  percentage = 2836.9625 * pow(volts, 4) - 43987.4889 * pow(volts, 3) + 255233.8134 * pow(volts, 2) - 656689.7123 * volts + 632041.7303;
  if (volts >= FULL_BATT_VOLTAGE) percentage = 100;
  if (volts <= 3.30) percentage = 0;  // orig 3.5
  
  sprintf(mqttMsg, "%.2f", volts);
  mqttClient.publish(DRY_BOX_BATTERY_VOLTS_TOPIC, mqttMsg, true);  
  Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), DRY_BOX_BATTERY_VOLTS_TOPIC, mqttMsg);
  sprintf(mqttMsg, "%d", percentage);
  mqttClient.publish(DRY_BOX_BATTERY_PERCENT_TOPIC, mqttMsg, true);  
  Serial.printf("%s MQTT SENT: %s/%s\n", myTZ.dateTime("[H:i:s.v]").c_str(), DRY_BOX_BATTERY_PERCENT_TOPIC, mqttMsg);
}

/*
 * GoToSleep
 */
void GoToSleep()
{
  // First we configure when to wakeup next
  digitalWrite(BUILT_IN_LED_PIN, LED_OFF);   // LED off
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_SECS * uS_TO_S_FACTOR);          // periodic wellness checkin                        // set new wakeup level to opposite of current
  Serial.printf("%s Going to sleep now for %ds \nZzzz Zzzz Zzzz Zzzz Zzzz Zzzz Zzzz Zzzz Zzzz Zzzz Zzzz Zzzz\n\n", 
                myTZ.dateTime("[H:i:s.v]").c_str(), DEEP_SLEEP_SECS);
  Serial.flush();
  esp_deep_sleep_start();
  Serial.println(F("This will never be printed"));
}


/*
 * print_wake_reason
 */
void print_wakeup_reason(esp_sleep_wakeup_cause_t wakeup_reason)
{
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : 
    {
      Serial.println(F("\nWakeup caused by external signal using RTC_IO"));
      break;
    }
    case ESP_SLEEP_WAKEUP_EXT1 :
    {
      Serial.println(F("\nWakeup caused by external signal using RTC_CNTL"));
      break;
    }
    case ESP_SLEEP_WAKEUP_TIMER :
    {
      Serial.println(F("\nWakeup caused by timer"));
      break;
    }
    case ESP_SLEEP_WAKEUP_TOUCHPAD :
    {
      Serial.println(F("\nWakeup caused by touchpad"));
      break;
    }
    case ESP_SLEEP_WAKEUP_ULP :
    {
      Serial.println(F("\nWakeup caused by ULP program"));
      break;
    }
    default :
    {
      Serial.printf("\nWakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
    }
  }
}


// Subroutine to calculate absolute humidity (g/m^3) from temperature (°C) and relative humidity (%) (generated by Grok AI)
float calculate_absolute_humidity(float temp_c, float rh_percent) {
  // Constants
  const float Rv = 461.5f; // Gas constant for water vapor (J/(kg·K))
  const float T0 = 273.15f; // Zero Celsius in Kelvin

  // Convert temperature to Kelvin
  float temp_k = temp_c + T0;

  // Calculate saturation vapor pressure (Pa) using Tetens' formula
  float es = 610.78f * expf((17.2694f * temp_c) / (temp_c + 237.3f));

  // Calculate actual vapor pressure (Pa)
  float e = (rh_percent / 100.0f) * es;

  // Calculate absolute humidity (g/m^3)
  float abs_humidity = (e * 1000.0f) / (Rv * temp_k);

  return abs_humidity;
}
