
/*
    NANO33IOT_TFG_GIORDY_ALEXANDER_ANDRADE
*/
    #include "control_algorithms.h" 
    #include "signal_processing.h"
    #include "stream_communication.h"

    uint8_t analog_pin_select   = 14;
    uint8_t opt1_pin_select     = 3;            
    uint8_t opt2_pin_select     = 4; 
 
    uint8_t r1_pin_select       = 5;
    uint8_t r2_pin_select       = 6;
    uint8_t r3_pin_select       = 7;
    uint8_t r4_pin_select       = 8;            

    uint8_t pitch_pin_select   = 2;

    typedef struct{
    double RPM;
    double prev_RPM;
    double pos;
    int load_state;
    double R;
    double P;
    int state;
    }out_data_t;
    
    out_data_t out_data;


    Signal Signal(analog_pin_select, opt1_pin_select, opt2_pin_select);
    Load Load(r1_pin_select, r2_pin_select, r3_pin_select, r4_pin_select);      
    Pitch Pitch(pitch_pin_select);
    Data Data;

   
    void setup() {
         Serial.begin(115200);           // Initialize serial
         Pitch.setup_servo();
         Load.setup_relay();
         Data.setup_communications();
    }
  
  
    void loop() {

         while (out_data.RPM < RPM_min){
               initial_state(&out_data);
          }
         
         while (out_data.RPM > RPM_min && out_data.RPM < RPM_optim - RPM_range && out_data.state < 2){
               start_state(&out_data);
         }
           
         while (out_data.RPM > RPM_optim - RPM_range && out_data.RPM < RPM_optim + RPM_range ){
               pitch_control_state(&out_data);  // or marine_control_state();
         }
         
         while (((out_data.RPM < RPM_optim - RPM_range) ||(out_data.RPM > RPM_optim + RPM_range) )&& out_data.RPM > RPM_min){
               load_control_state(&out_data);
               while (emergency_stop == true){
                      emergency_stop_state(&out_data);
                }
         }           
    }


    void initial_state( out_data_t* out ){
         out->state      = 0; 
         for (int i=0; i<SAMPLES; i++){  
             Signal.read_signal(i);  
             Data.serial_stream( out->RPM, out->pos, out->load_state, out->P );
         }
         
         out->RPM        = Signal.frequency_meter()*10;  // RPM_TURBINE = 60* (F(Hz)/(Number_of_poles = 6))
         out->pos        = Pitch.pitch_initial();
         out->load_state = round (Load.relay_switch( out->RPM , out->prev_RPM));
         out->R          = Load.relay( out->load_state );
         out->P          = Signal.power_meter( out->R );
         out->prev_RPM   = out->RPM;                     //IMPORTANT!!
         control_sample_rate_regulation();


         Data.Display     ( out->RPM, out->pos, out->load_state, out->P );
         Data.update_IOT   ( out->RPM, out->pos, out->load_state, out->state);
    }

    void start_state( out_data_t* out ){
         out->state      = 1; 
         for (int i=0; i<SAMPLES; i++){  
             Signal.read_signal(i); 
             Data.serial_stream( out->RPM, out->pos, out->load_state, out->P );
         }
         
         out->RPM        = Signal.frequency_meter()*10;  // RPM_TURBINE = 60* (F(Hz)/(Number_of_poles = 6))
         out->pos        = Pitch.pitch_start();
         out->load_state = round (Load.relay_start( out->RPM , out->prev_RPM));
         out->R          = Load.relay( out->load_state );
         out->P          = Signal.power_meter( out->R );
         out->prev_RPM   = out->RPM;                     //IMPORTANT!!
         control_sample_rate_regulation();
         

         Data.Display     ( out->RPM, out->pos, out->load_state, out->P);
         Data.update_IOT   ( out->RPM, out->pos, out->load_state, out->state);
    }
     
    void pitch_control_state( out_data_t* out ){
         out->state      = 2; 
         for (int i=0; i<SAMPLES; i++){  
             Signal.read_signal(i);
             Data.serial_stream( out->RPM, out->pos, out->load_state, out->P );
         }
         
         out->RPM        = Signal.frequency_meter()*10;  // RPM_TURBINE = 60* (F(Hz)/(Number_of_poles = 6))
         out->pos        = Pitch.pitch_control( out->RPM, out->prev_RPM );
         out->R          = Load.relay( out->load_state );
         out->P          = Signal.power_meter( out->R );
         out->prev_RPM   = out->RPM;                     //IMPORTANT!!
         control_sample_rate_regulation();
         

         Data.Display     ( out->RPM, out->pos, out->load_state,out->P );
         Data.update_IOT   ( out->RPM, out->pos, out->load_state, out->state);
    }

        void load_control_state( out_data_t* out ){
         out->state      = 3; 
         for (int i=0; i<SAMPLES; i++){  
             Signal.read_signal(i);
             Data.serial_stream( out->RPM, out->pos, out->load_state, out->P );
         }
         
         out->RPM        = Signal.frequency_meter()*10;  // RPM_TURBINE = 60* (F(Hz)/(Number_of_poles = 6))
         out->load_state = round (Load.relay_switch( out->RPM , out->prev_RPM));
         out->R          = Load.relay( out->load_state );
         out->P          = Signal.power_meter( out->R );
         out->prev_RPM   = out->RPM;                     //IMPORTANT!!
         control_sample_rate_regulation();
         

         Data.Display     ( out->RPM, out->pos, out->load_state, out->P );
         Data.update_IOT   ( out->RPM, out->pos, out->load_state, out->state);
    }

    void emergency_stop_state( out_data_t* out ){
         out->state      = 4; 
         for (int i=0; i<SAMPLES; i++){  
             Signal.read_signal(i); 
             Data.serial_stream( out->RPM, out->pos, out->load_state, out->P );
         }
         
         out->RPM        = Signal.frequency_meter()*10;  // RPM_TURBINE = 60* (F(Hz)/(Number_of_poles = 6))
         out->pos        = Pitch.pitch_stop();
         out->load_state = round (Load.relay_switch( out->RPM , out->prev_RPM));
         out->R          = Load.relay( out->load_state );
         out->P          = Signal.power_meter( out->R );
         out->prev_RPM   = out->RPM;                     //IMPORTANT!!
         control_sample_rate_regulation();

         Data.Display     ( out->RPM, out->pos, out->load_state, out->P);
         Data.update_IOT   ( out->RPM, out->pos, out->load_state, out->state);
    }

    void marine_control_state( out_data_t* out ){
         out->state      = 5; 
         for (int i=0; i<SAMPLES; i++){  
             Signal.read_signal(i);
             out->pos = Pitch.pitch_IMU();
             Data.serial_stream( out->RPM, out->pos, out->load_state, out->P ); 
         }
         
         out->RPM        = Signal.frequency_meter()*10;  // RPM_TURBINE = 60* (F(Hz)/(Number_of_poles = 6))
         out->load_state = round (Load.relay_switch( out->RPM , out->prev_RPM));
         out->R          = Load.relay( out->load_state );
         out->P          = Signal.power_meter( out->R );
         out->prev_RPM   = out->RPM;                     //IMPORTANT!!
         control_sample_rate_regulation();
         

         Data.Display     ( out->RPM, out->pos, out->load_state, out->P );
         Data.update_IOT   ( out->RPM, out->pos, out->load_state, out->state);
    }


    void control_sample_rate_regulation(){
         while ((micros()- loop_time_control)<T_control);
         loop_time_control = micros();
      }

      

     
