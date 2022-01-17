
    #include "stream_communication.h"
    #include "Arduino.h"
    #include <Wire.h>
    #include <WiFiNINA.h>
    #include <WiFiUdp.h>          // UDP library
    #include <Adafruit_GFX.h>
    #include <Adafruit_SSD1306.h> 
    
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 32 // OLED display height, in pixels
        
    #define OLED_RESET    -1
    #define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

    
    const char ssid[]     = "";    // Network SSID (name)
    const char pass[]     = "";          // Network password (use for WPA, or use as key for WEP)
    int status = WL_IDLE_STATUS;
    
    WiFiSSLClient client;             // Instantiate the Wifi client

    // UDP Variables
    unsigned int localPort = 2390;        // local port to listen on
    const char* computerIP = " ";         // ENTER YOUR COMPUTER'S IP BETWEEN QUOTES
    WiFiUDP Udp;
    

    Data::Data(){
    }
    void Data::setup_communications(){
         display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
      // Connect to wifi
         status = WiFi.begin(ssid, pass);
         Udp.begin(localPort);       
         delay(500);
         Display(0,0,0,0);
    }

/* A lot of latency
    void Data::wifi_setup(){
         if (WiFi.status() != WL_CONNECTED   && (millis() - loop_time_wifi) > T_wifi ) {
            status = WiFi.begin(ssid, pass);
            Udp.begin(localPort);;
            loop_time_wifi = millis();
         }
    }
*/
    
    void Data::update_IOT(double RPM_iot, double pos_iot, int load_state_iot, int state_turbine_iot){
         if (status == WL_CONNECTED) {
             floatToBuff(RPM_iotBuff, float (RPM_iot));
             floatToBuff(pos_iotBuff, float (pos_iot));
             floatToBuff(load_state_iotBuff, float (load_state_iot));
             floatToBuff(state_turbine_iotBuff, float (state_turbine_iot));
               
             Udp.beginPacket(computerIP, RPMPort);
             Udp.write(RPM_iotBuff,4);
             Udp.endPacket();
    
             Udp.beginPacket(computerIP, PosPort);
             Udp.write(pos_iotBuff,4);
             Udp.endPacket();
    
             Udp.beginPacket(computerIP, LoadPort);
             Udp.write(load_state_iotBuff,4);
             Udp.endPacket();
    
             Udp.beginPacket(computerIP, StatePort);
             Udp.write(state_turbine_iotBuff,4);
             Udp.endPacket();
         }
    }

    void Data::floatToBuff(byte udpBuffer[4], float sensorVal){
         byte *sensorValByte = reinterpret_cast<byte*>(&sensorVal);
         memcpy(udpBuffer, sensorValByte, sizeof(sensorValByte));
    }


    void Data::Display ( double RPM_wind, double pos_wind, int load_state_wind, double power_wind ) {
         display.clearDisplay();
        
         display.setTextSize(1);             // Normal 1:1 pixel scale
         display.setTextColor(SSD1306_WHITE);        // Draw white text
         display.setCursor(0,0);             // Start at top-left corner
         display.print("State: "); display.print(load_state_wind);
         display.setCursor(60,0);             // Start at center
         display.print("P: "); display.print(power_wind);display.println("mW");
          
         display.setTextColor(SSD1306_WHITE); // Draw 'inverse' text
         display.print("Pitch angle: ");display.println(pos_wind);
          
         display.setTextSize(2);             // Draw 2X-scale text
         display.setTextColor(SSD1306_WHITE);
         display.print("RPM:"); display.println(RPM_wind);
        
         display.display();
      }


      void Data::serial_stream (double RPM_stream, double pos_stream, int load_state_stream, double power_stream){
          
               RPM_value.number   = float(RPM_stream);
               pos_value.number   = float(pos_stream);
               load_value.number  = float(load_state_stream);
               power_value.number = float(power_stream);
               
               Serial.write('S');
                 for(int i=0; i<4; i++){
                 Serial.write(RPM_value.bytes[i]);
               }
                 for(int i=0; i<4; i++){
                 Serial.write(pos_value.bytes[i]);
               }
                for(int i=0; i<4; i++){
                 Serial.write(load_value.bytes[i]);
               }
               for(int i=0; i<4; i++){
                 Serial.write(power_value.bytes[i]);
               }
               Serial.write('\n');
    }



    
