/*
 *  prototypes.h
 */
void initWiFi();
void setup_OTA();
void callback(char *topic, byte *payload, unsigned int length);
boolean reconnect();
void connectMQTT();
void sendMQTT();
void print_wakeup_reason(esp_sleep_wakeup_cause_t w_reason);
void GoToSleep();
void sendTempHumStatus();
float readBatteryVoltage();
void sendBatteryStatus();
uint32_t readADC_Cal(int ADC_Raw);
float calculate_absolute_humidity(float temp_c, float rh_percent);
