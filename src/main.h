void MQTT_callback(char* topic, byte* payload, unsigned int length);
void MQTT_Send(char const * topic, long value);
void MQTT_Send(char const * topic, int16_t value);
void MQTT_Send(char const * topic, float value);
void MQTT_Send(char const * topic, String value);
void UDBDebug(const char * message);
void UDBDebug(String message);
void LEDUpdate(int8_t red, int8_t green, int8_t blue);
