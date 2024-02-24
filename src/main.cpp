/*
Code based on example to receive multiple universes via Artnet and control a strip of ws2811 leds via 
Adafruit's NeoPixel library: https://github.com/adafruit/Adafruit_NeoPixel
This example may be copied under the terms of the MIT license, see the LICENSE file for details
*/

/*
 * Stürzt das Programm sporatisch ab bzw. wird ein Reboot durch den WDT ausgelöst kann das Einfügen von delay(0) oder yield() Funktionen (z.B. in while-Schleifen) die Situation verbessern. 
 */

#define UseMQTT 1

#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <WiFiUdp.h>
#include <ArtnetWifi.h>
#include "main.h"

#define UDPDEBUG 1
#ifdef UDPDEBUG
WiFiUDP udp;
const char * udpAddress = "192.168.0.63";
const int udpPort = 19814;
#endif

// #define FASTLED_INTERRUPT_RETRY_COUNT 1
#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>

#define numLedinRing 18
#define NUM_LEDS    40  // inner matrix
#define numLamps 2

#if defined(ARDUINO_ARCH_ESP32)
#define D0  33
#define D1  32
#define D2  27
#define D3  26
#endif

#define Relais1_PIN     D0
#define Relais2_PIN     D2
unsigned long lasttime; 
byte RelaisOn;

#define LED_PIN     D3 //D2
#define LED_PIN2    D1 //D4
#define COLOR_ORDER GRB
#define CHIPSET     WS2812

#define BRIGHTNESS  255
#define FRAMES_PER_SECOND 60

#define debug false

// Neopixel settings
const int numLeds = (numLedinRing+NUM_LEDS)*numLamps;
const int numberOfChannels = numLeds * 3; // Total number of channels you want to receive (1 led = 3 channels)


CRGB leds[numLeds];

// Artnet settings
ArtnetWifi artnet;
const int startUniverse = 1; // needs to fit to cabinet.xml <ArtNet>

// Check if we got all universes
const int maxUniverses = numberOfChannels / 512 + ((numberOfChannels % 512) ? 1 : 0);
bool universesReceived[maxUniverses];
bool sendFrame = 1;
int previousDataLength = 0;

#if defined(UseMQTT)
  // enable external Zigbee LED via MQTT -> Zigbee2Mqtt gateway
  const char* wifihostname = "pinball_ESP";  
  int8_t mqtterrorcounter=0;
  WiFiClient wifiClient;
  #include <PubSubClient.h>
  const char* mqtt_server = "192.168.0.46";
  // MQTT_User and MQTT_Pass defined via platform.ini, external file, not uploaded to github
  PubSubClient mqttclient(wifiClient);
  uint8_t LastRed=0, LastGreen=0, LastBlue=0;
  unsigned long lastSend=0; 
#endif

// connect to wifi – returns true if successful or false if not
void WIFI_Connect()
{
//  digitalWrite(BUILTIN_LED,1);
  WiFi.disconnect();
  Serial.println("Booting Sketch...");
  WiFi.mode(WIFI_STA);
  #if defined(ARDUINO_ARCH_ESP32)
  WiFi.setHostname("ArtnetPinball");
  #else
  WiFi.hostname("ArtnetPinball");
  #endif
  WiFi.begin(WIFI_SSID, WIFI_PASS);
    // Wait for connection
  for (int i = 0; i < 25; i++)
  {
    if ( WiFi.status() != WL_CONNECTED ) {
      delay ( 250 );
 //     digitalWrite(BUILTIN_LED,0);
      Serial.print ( "." );
      delay ( 250 );
 //     digitalWrite(BUILTIN_LED,1);
    }
  }
//  digitalWrite(BUILTIN_LED,0);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.print("Running Artnet on IP : ");
  Serial.println(WiFi.localIP());
  
  lasttime = 0;

}

void initTest()
{
  digitalWrite(Relais1_PIN, LOW);
        digitalWrite(Relais2_PIN, LOW);
        RelaisOn = true;
        delay ( 1000 );  // time to charge power
         lasttime=millis();
        
  Serial.println("begin test.");
  Serial.println(numLeds);
  for (int i = 0 ; i < numLeds ; i++) {
    leds[i]   = CRGB::Green;
    FastLED.show();
    delay(10);
    leds[i]   = CRGB::Black;
    //FastLED.show();
    //Serial.print ( "." );
  }

  Serial.println("end test.");
}



void loopxx() { 
  
    for(int a=0;a<numLeds;a++) {
      if((a%2)==0) {
        leds[a] = CRGB::White;
      }
      else
        leds[a] = CRGB::Green;
        
      FastLED.show();
      delay(50);
     
     
     leds[a] = CRGB::Black;
    }  
FastLED.show();

}


void onDmxFrame(uint16_t universe, uint16_t thelength, uint8_t sequence, uint8_t* data)
{
  artnet.printPacketHeader();
  
  if (!RelaisOn)
    {
        // relais on!
       if (debug) {
        Serial.println("Relais on");
       }
        digitalWrite(Relais1_PIN, LOW);
        digitalWrite(Relais2_PIN, LOW);
        RelaisOn = true;
//        digitalWrite(BUILTIN_LED,1);
        #if defined(UseMQTT)
          String message = "{\"color\":{\"r\":255,\"g\":102,\"b\":150}, \"state\":\"on\"}";
          MQTT_Send("zigbee2mqtt/LED/set", message);
        #endif

        delay ( 1000 );  // time to charge power
    }
  
  lasttime=millis();
  
  sendFrame = 1;

  if (universe == 0 ) {   
      /*for (int i = 0 ; i < 116 ; i++) { // all LED's  in Ring, just for testing
       leds[i].red   = byte(data[0]);
       leds[i].green = byte(data[1]);
       leds[i].blue  = byte(data[2]);
      } */
      #if defined(UseMQTT)
          LEDUpdate(byte(data[0]), byte(data[1]), byte(data[2]));
      #endif
      
      for (int i = 0 ; i < numLedinRing ; i++) {
       leds[i].red   = byte(data[0]);
       leds[i].green = byte(data[1]);
       leds[i].blue  = byte(data[2]);
      }
      yield();
      if (numLamps > 1) {
        int offset = numLedinRing+NUM_LEDS;
        for (int i = offset ; i < (numLedinRing+offset) ; i++) {
         leds[i].red   = byte(data[0]);
         leds[i].green = byte(data[1]);
         leds[i].blue  = byte(data[2]);
         yield();
        }
      }
      if (debug) {
        Serial.print("all: ");
        Serial.print(byte(data[0]), HEX);
        Serial.print(byte(data[1]), HEX);
        Serial.println(byte(data[2]), HEX);  
      }
      }
  else
  {
      // Store which universe has got in
      if ((universe - startUniverse) < maxUniverses)
        universesReceived[universe - startUniverse] = 1;
    
      for (int i = 0 ; i < maxUniverses ; i++)
      {
        if (universesReceived[i] == 0)
        {
          Serial.println("Broke");
          sendFrame = 0;
          break;
        }
      }
    if (debug) {
      Serial.println("got data");
      Serial.println(thelength);
    }

    // read universe and put into the right part of the display buffer
    // first LED is undercab color used for outside ring
      #if defined(UseMQTT)
        LEDUpdate(byte(data[0]), byte(data[1]), byte(data[2]));
      #endif

      for (int i = 0; i < numLedinRing; i++)
      {
        for (int j = 0; j < numLamps; j++) {
          
           int led = i + (j*(NUM_LEDS+numLedinRing));
           if (led < numLeds) {
              leds[led].red   = byte(data[0]);
              leds[led].green = byte(data[1]);
              leds[led].blue  = byte(data[2]);
          }
          yield();
       }               
      }

      // now LED's in the inner matrix, controlled by DOF
      // we need to adjust index. 
      // first ring, then matrix, then ring, then matrix and so on.
      
      for (int i = 1; i < thelength / 3; i++)
      {
        int ring = (i-1) / NUM_LEDS;
        int led = (i-1)  + ((ring+1) * numLedinRing);  // + (ring * NUM_LEDS)
 
        
          //int led = i + (universe - startUniverse) * (previousDataLength / 3);
        if ((led < numLeds) && ((i*3 +3) < (thelength))) {
           leds[led].red   = byte(data[i*3]);
           leds[led].green = byte(data[i*3 +1]);
           leds[led].blue  = byte(data[i*3 +2]);
           yield();
        }
        
      }
      //previousDataLength = length;   
  }  
  yield();
  if (sendFrame)
  {
    FastLED.show();
    // Reset universeReceived to 0
    memset(universesReceived, 0, maxUniverses);
  }
}

void setup()
{
  Serial.begin(115200);
   delay(10);

//  pinMode(BUILTIN_LED, OUTPUT);
  WIFI_Connect();
  pinMode(Relais1_PIN, OUTPUT);
  pinMode(Relais2_PIN, OUTPUT);
  digitalWrite(Relais1_PIN, HIGH);
  digitalWrite(Relais2_PIN, HIGH);  RelaisOn = false;
  lasttime = 0;
  artnet.begin();

  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, numLeds).setCorrection( TypicalSMD5050 );
  // the following line controls a 2nd ledstrip, if you want two of them...
  FastLED.addLeds<CHIPSET, LED_PIN2, COLOR_ORDER>(leds, numLeds).setCorrection( TypicalSMD5050 );
  FastLED.setBrightness( BRIGHTNESS );
  FastLED.clear();
  FastLED.show();

  initTest();

#if defined(UseMQTT)
  mqttclient.setServer(mqtt_server, 1883);
    mqttclient.setCallback(MQTT_callback);
    mqttclient.setBufferSize(1024);
   if (mqttclient.connect(wifihostname, MQTT_User, MQTT_Pass)) {
      //mqttclient.publish("outTopic","hello world");
      UDBDebug("MQTT connect successful"); 
   }  
    else
       UDBDebug("MQTT connect error");  
#endif       

  // this will be called for each packet received
  artnet.setArtDmxCallback(onDmxFrame);
}

void loop()
{
  
  if (WiFi.status() != WL_CONNECTED)
    {
      WIFI_Connect();
    }

#if defined(UseMQTT)
  if (!mqttclient.loop()) {
    if (mqttclient.connect(wifihostname, MQTT_User, MQTT_Pass)) {
      UDBDebug("MQTT reconnect successful"); 
      mqtterrorcounter=0;
   }  
    else
       UDBDebug("MQTT reconnect error");  
       if (mqtterrorcounter++ > 5)
        ESP.restart();
  };
#endif

  // we call the read function inside the loop
  artnet.read();

  if (RelaisOn)
    {
      if ((lasttime+600000)<millis())
      { // war 60 sekunden, jetzt 600 / 10 min
        // relais off!
      if (debug) {
        Serial.println("Relais off");
      }
        digitalWrite(Relais1_PIN, HIGH);
        digitalWrite(Relais2_PIN, HIGH);
        RelaisOn = false;
        lasttime=0;
         #if defined(UseMQTT)
          String message = "{\"state\":\"off\"}";
          MQTT_Send("zigbee2mqtt/LED/set", message);
        #endif       
 //       digitalWrite(BUILTIN_LED,0);
      }
    }
    
}


#if defined(UseMQTT)

void LEDUpdate(uint8_t red, uint8_t green, uint8_t blue) {
 // uint32_t ledlasttime=millis();

  //if ((lastSend+1000) < ledlasttime) {
   //     String message = "{\"color\":{\"r\":"+String(red)+",\"g\":"+String(green)+",\"b\":"+String(blue)+"}, \"state\":\"on\"}";
   //     MQTT_Send("Haus/LED/debug", message);  

   // lastSend = ledlasttime;
    if ((red != LastRed) || (blue != LastBlue) || (green != LastGreen)) {
      if ((red != 0) || (blue != 0) || (green !=0)) {
        LastRed=red;
        LastBlue=blue;
        LastGreen=green;

        String message = "{\"color\":{\"r\":"+String(red)+",\"g\":"+String(green)+",\"b\":"+String(blue)+"}, \"state\":\"on\"}";
        MQTT_Send("zigbee2mqtt/LED/set", message);   
      }
    }
  //}  
}


void UDBDebug(String message) {
#ifdef UDPDEBUG
  udp.beginPacket(udpAddress, udpPort);
  udp.write((const uint8_t* ) message.c_str(), (size_t) message.length());
  udp.endPacket();
#endif  
}

void UDBDebug(const char * message) {
#ifdef UDPDEBUG
  udp.beginPacket(udpAddress, udpPort);
  udp.write((const uint8_t*) message, strlen(message));
  udp.endPacket();
#endif  
}


void MQTT_Send(char const * topic, String value) {
    String strtopic = topic;
    if (!mqttclient.publish(topic, value.c_str(), true)) {
       UDBDebug("MQTT error");  
       if (!mqttclient.loop()) {
           if (mqttclient.connect(wifihostname, MQTT_User, MQTT_Pass)) {
              UDBDebug("MQTT reconnect successful"); 
              if (!mqttclient.publish(topic, value.c_str(), true)) {
                  UDBDebug("MQTT error");  
              }    
           }  
       else
          UDBDebug("MQTT reconnect error");  
       };
    };
}

void MQTT_Send(char const * topic, float value) {
    char buffer[10];
    snprintf(buffer, 10, "%f", value);
    MQTT_Send(topic, buffer);
}

void MQTT_Send(char const * topic, int16_t value) {
    char buffer[10];
    snprintf(buffer, 10, "%d", value);
    MQTT_Send(topic, buffer);
}

void MQTT_Send(char const * topic, long value) {
    char buffer[10];
    snprintf(buffer, 10, "%d", value);
    MQTT_Send(topic, buffer);
}

  // Callback function
void MQTT_callback(char* topic, byte* payload, unsigned int length) {

    String message = String(topic);
    int8_t joblength = message.length()+1;// 0 char
    payload[length] = '\0';
    String value = String((char *) payload);
}

#endif