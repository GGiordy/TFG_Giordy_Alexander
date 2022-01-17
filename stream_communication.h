
    #ifndef STREAM_COMMUNICATION_H
    #define STREAM_COMMUNICATION_H
    
    #include "Arduino.h"

    class Data{
   
    public:
    

 // BIT SENDING PUBLIC VARIABLES
    typedef union{
    float number;
    uint8_t bytes[4];
    }value_t;
     
    value_t RPM_value;
    value_t load_value;
    value_t pos_value;
    value_t power_value;
    

 // FUNCTIONS
 
    Data();
    void update_IOT(double RPM_iot, double pos_iot, int load_state_iot, int state_turbine_iot);
    void floatToBuff(byte udpBuffer[4], float sensorVal);
    void wifi_setup();
    void setup_communications();
    void Display ( double RPM_wind, double pos_wind, int load_state_wind, double power_wind );
    void serial_stream (double RPM_stream, double pos_stream, int load_state_stream, double power_stream);

    
    private:
    
  // ARDUINO CLOUD_IOT PRIVATE VARIABLES
    const char* computerIP       = "";         // ENTER YOUR COMPUTER'S IP BETWEEN QUOTES
    const unsigned int RPMPort   = 65013; // Destination Ports
    const unsigned int PosPort   = 65014;
    const unsigned int LoadPort  = 65015;
    const unsigned int StatePort = 65016;
    
    const int T_wifi = 10000;
    long loop_time_wifi = 0;
    byte RPM_iotBuff[4];
    byte pos_iotBuff[4];
    byte load_state_iotBuff[4];
    byte state_turbine_iotBuff[4];
    };

    #endif 


    
    
