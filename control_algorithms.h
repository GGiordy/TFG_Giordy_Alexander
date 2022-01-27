 
    #ifndef CONTROL_ALGORITHMS_H
    #define CONTROL_ALGORITHMS_H
    #include "Arduino.h"

 // CONTROL VARIABLES

    const double RPM_min               = 100;              // Minimal RPM = 100
    const double RPM_optim             = 500;              // Wanted RPM = 300
    const double RPM_range             = 0.2*RPM_optim;    // Max RPM range to pitch control
    const long T_control               = 500000;           // microseconds (min = 420 milliseconds)
    const long stop_delay              = 10000;            // Stop delay milliseconds
    inline boolean emergency_stop      = false;
    inline long loop_time_control      = 0;
    inline long loop_time_stop         = 0;

    
    class Pitch {
    public:
      
   // PITCH CONTROL PUBLIC VARIABLES
      double pos             = pos_min;
      double pos_local       = pos; 
      float  x,y,z;
      
       
   // FUNCTIONS 
      Pitch(uint8_t _pitch_pin);
      void   setup_servo();
      double pitch_initial();
      double pitch_IMU();
      double pitch_start();
      double pitch_control(double RPM_read, double prev_RPM_read);
      double pitch_stop();
      void   pitch_safe_write(double calculated_pos);
      double mapf(double x, double in_min, double in_max, double out_min, double out_max);
      
    private:
      
   // PITCH CONTROL PRIVATE VARIABLES 
      uint8_t pitch_pin;   
      const double alpha        = 0.95;
      const double Pitch_P_gain = 0.007;
      const double Pitch_D_gain = 0.03;
      const double Pitch_I_gain = 0.01;
      const double pos_max      = 90.0;
      const double pos_optim    = 80.0;   // Posición óptima del servo para el ángulo óptimo de trabajo 
      const double pos_min      = 45.0; 
      double local_pos          = pos;
      double ymin               = 0.25; 
      double ymax               = -0.25;

    };
    
    class Load {
    public:

   // LOAD CONTROL PUBLIC VARIABLES
      double  load_state        = 0;  
      double  R;                                  //measured resistances 

   
  
   // FUNCTIONS 
      Load(uint8_t _r1_pin, uint8_t _r2_pin,uint8_t _r3_pin,uint8_t _r4_pin);
      void   setup_relay();
      double relay_start(double RPM_read, double prev_RPM_read);
      double relay_switch(double RPM_read, double prev_RPM_read);
      double relay(int load_state_local);
      
    private:
      
   // LOAD CONTROL PRIVATE VARIABLES
      uint8_t r1_pin, r2_pin,r3_pin,r4_pin;
      double local_state=0;
      const double Load_P_gain  = 0.003;
      const double Load_D1_gain = 0.01;
      const double Load_D2_gain = 0.01;
    };
    

    #endif 
    
