
    #ifndef SIGNAL_PROCESSING_H
    #define SIGNAL_PROCESSING_H
    
    #include "Arduino.h"
    #define SAMPLES 128
    
    
    class Signal{
    public:
    
   // FFT PUBLIC VARIABLES
      double freq                  = 0;               // Initial freq = 0Hz
      const double sample_rate     = 300;             // sample rate = 100x3
      const double sample_period   = 1e6/sample_rate; // sample time in microsecond
      long loop_time1              = 0;
      
   // ANALOG READ PUBLIC VARIABLES
      double vReal[SAMPLES];
      double vImag[SAMPLES];
      double maior                 = 0;    
      int scale                    = 1;
      double V_RMS                 = 0;
      double power                 = 0;
      
   // FUNCTIONS  
      Signal(uint8_t _analog_pin,uint8_t _opt1_pin,uint8_t _opt2_pin);
      void   read_signal(int loop_itValue);
      void   scale_value(double data);
      double frequency_meter();          
      double power_meter(double resistance_value);

      
    private:
    
      uint8_t analog_pin,opt1_pin,opt2_pin;
    
   // FFT PRIVATE VARIABLES
      const double high_pass_filter = 5;             //Hz     

      
   // ANALOG READ PRIVATE VARIABLES   
      const double VOffset         = 1.5;             // default value offset


    };

    #endif 
