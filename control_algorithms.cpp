
    #include "control_algorithms.h"
    #include "Arduino.h"
    #include <Arduino_LSM6DS3.h> 
    #include <Servo.h>



    Servo myservo;

///////////////////////////////   PITCH CONTROL  ////////////////////////////////////////////////
    Pitch::Pitch(uint8_t _pitch_pin){
         pitch_pin = _pitch_pin;
         myservo.attach(pitch_pin);
    }

///      Inicio del sistema con las palas perpendiculares a la dirección del viento    
    void  Pitch::setup_servo(){
          pos = pos_max;
          pitch_safe_write(pos_max);
          IMU.begin();
    }

///      Control del servomotor con el ángulo de inclinación del sistema entre ymin e ymax         
    double Pitch::pitch_IMU(){
         IMU.readAcceleration(x, y, z);
         if (y>0 && y<=ymin){
            pos = pos_optim - (pos_optim - pos_min)*(y/ymin);
            pitch_safe_write(pos);
          }
    
         if (y<0 && y>=ymax){
            pos = pos_optim + (pos_max - pos_optim)*(y/ymax);
            pitch_safe_write(pos);
          }
          return mapf(pos,pos_min,pos_max,pos_min,0);
    }

///      Para favorecer el arranque del sistema, se colocan las palas con el mayor ángulo de pitch = pos_min del servo
    double Pitch::pitch_initial(){
           pos = pos_min;
           pitch_safe_write(pos_min);
           return mapf(pos,pos_min,pos_max,pos_min,0);
    }
    
///      El ángulo de pitch se coloca hacia el ángulo óptimo de trabajo con una exponencial suavizada con factor alpha   
    double Pitch::pitch_start(){
           pos = alpha*pos +(1-alpha)*((pos_optim)); 
           pitch_safe_write(pos);
           return mapf(pos,pos_min,pos_max,pos_min,0);
    }
    
///      Si las RPM están dentro de la region de trabajo establecida, se aplica el control PID    
    double Pitch::pitch_control(double RPM_read, double prev_RPM_read){
           
           if (RPM_read >= prev_RPM_read){
              local_pos = pos +(RPM_optim - RPM_read)*Pitch_P_gain - ((RPM_read - prev_RPM_read)/(T_control*1e-6))*Pitch_D_gain -((RPM_read - prev_RPM_read)*(T_control*1e-6))*Pitch_I_gain;
              if (local_pos >= pos_min){
                  pos = local_pos;
                  pitch_safe_write(pos);
              }
           }
           if (RPM_read < prev_RPM_read){
              local_pos = pos +(RPM_optim - RPM_read)*Pitch_P_gain - ((RPM_read - prev_RPM_read)/(T_control*1e-6))*Pitch_D_gain -((RPM_read - prev_RPM_read)*(T_control*1e-6))*Pitch_I_gain;
              if (local_pos <= pos_max){
                  pos = local_pos;
                  pitch_safe_write(pos);
              }
           }
           return mapf(pos,pos_min,pos_max,pos_min,0);
    }
    
////      En caso de ser necesario, se aplica una para de emergencia durante (stop_delay) segundos
    double Pitch::pitch_stop(){
           if ((millis() - loop_time_stop) < stop_delay){
              pos = pos_max;
              pitch_safe_write(pos);
              return mapf(pos,pos_min,pos_max,pos_min,0);
              }
           else{
              loop_time_stop = 0;
              emergency_stop = false;
              return mapf(pos,pos_min,pos_max,pos_min,0);
           }
    }
    
////      Para no forzar el servomotor, se limitan los valores que puede alcanzar con el mecanismo
     void Pitch::pitch_safe_write(double calculated_pos){
          if(calculated_pos >= pos_min && calculated_pos <= pos_max){
             myservo.write(calculated_pos);
          }
     }

////      Sirve para extrapolar entre el rango de actuación del servo y el rango del ángulo de pitch
     double Pitch::mapf(double x, double in_min, double in_max, double out_min, double out_max){
     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
     }
     


///////////////////////////////   LOAD CONTROL  ////////////////////////////////////////////////

////      Se inicializa el control de pitch   
    Load::Load(uint8_t _r1_pin, uint8_t _r2_pin,uint8_t _r3_pin,uint8_t _r4_pin){
          r1_pin = _r1_pin;
          r2_pin = _r2_pin;
          r3_pin = _r3_pin;
          r4_pin = _r4_pin;                            
    }
    
////     Arranque del relé
    void Load::setup_relay(){
         pinMode(r1_pin,OUTPUT);
         pinMode(r2_pin,OUTPUT);
         pinMode(r3_pin,OUTPUT);
         pinMode(r4_pin,OUTPUT);
         relay(load_state = 0);
    }

////      Se establecen los estados del relé con sus respectivos valores de resistencia     
    double Load::relay(int load_state_local){
        switch (load_state_local){
          case 0:
          digitalWrite(r1_pin,HIGH);     //0
          digitalWrite(r2_pin,HIGH);     //0
          digitalWrite(r3_pin,HIGH);     //0
          digitalWrite(r4_pin,HIGH);     //0
          R = 559;
          break;
      
          case 1:
          digitalWrite(r1_pin,HIGH);     //0
          digitalWrite(r2_pin,HIGH);     //0
          digitalWrite(r3_pin,HIGH);     //0
          digitalWrite(r4_pin,LOW);      //1
          R = 76.3;
          break;
      
          case 2:
          digitalWrite(r1_pin,HIGH);     //0
          digitalWrite(r2_pin,HIGH);     //0
          digitalWrite(r3_pin,LOW);      //1
          digitalWrite(r4_pin,HIGH);     //0
          R = 24.4;
          break;
      
          case 3:
          digitalWrite(r1_pin,HIGH);     //0
          digitalWrite(r2_pin,HIGH);     //0
          digitalWrite(r3_pin,LOW);      //1
          digitalWrite(r4_pin,LOW);      //1
          R = 19.2;
          break;
      
          case 4:
          digitalWrite(r1_pin,HIGH);     //0
          digitalWrite(r2_pin,LOW);      //1
          digitalWrite(r3_pin,HIGH);     //0
          digitalWrite(r4_pin,HIGH);     //0
          R = 8.8;
          break;
          
          case 5:
          digitalWrite(r1_pin,HIGH);     //0
          digitalWrite(r2_pin,LOW);      //1
          digitalWrite(r3_pin,HIGH);     //0
          digitalWrite(r4_pin,LOW);      //1
          R = 8;
          break;
          
          case 6:
          digitalWrite(r1_pin,HIGH);     //0
          digitalWrite(r2_pin,LOW);      //1
          digitalWrite(r3_pin,LOW);      //1
          digitalWrite(r4_pin,HIGH);     //0
          R = 6.9;
          break;
          
          case 7:
          digitalWrite(r1_pin,HIGH);     //0
          digitalWrite(r2_pin,LOW);      //1
          digitalWrite(r3_pin,LOW);      //1
          digitalWrite(r4_pin,LOW);      //1
          R = 6.5;
          break;
          
          case 8:
          digitalWrite(r1_pin,LOW);      //1
          digitalWrite(r2_pin,HIGH);     //0
          digitalWrite(r3_pin,HIGH);     //0
          digitalWrite(r4_pin,HIGH);     //0
          R = 3.2;
          break;
          
          case 9:
          digitalWrite(r1_pin,LOW);      //1
          digitalWrite(r2_pin,HIGH);     //0
          digitalWrite(r3_pin,HIGH);     //0
          digitalWrite(r4_pin,LOW);      //1
          R = 3.1;
          break
          ;
          case 10:
          digitalWrite(r1_pin,LOW);      //1
          digitalWrite(r2_pin,HIGH);     //0
          digitalWrite(r3_pin,LOW);      //1
          digitalWrite(r4_pin,HIGH);     //0
          R = 3.1;
          break;
          
          case 11:
          digitalWrite(r1_pin,LOW);      //1
          digitalWrite(r2_pin,HIGH);     //0
          digitalWrite(r3_pin,LOW);      //1
          digitalWrite(r4_pin,LOW);      //1
          R = 3;
          break;
          
          case 12:
          digitalWrite(r1_pin,LOW);      //1
          digitalWrite(r2_pin,LOW);      //1
          digitalWrite(r3_pin,HIGH);     //0
          digitalWrite(r4_pin,HIGH);     //0
          R = 2.8;
          break;
          
          case 13:
          digitalWrite(r1_pin,LOW);      //1
          digitalWrite(r2_pin,LOW);      //1
          digitalWrite(r3_pin,HIGH);     //0
          digitalWrite(r4_pin,LOW);      //1
          R = 2.7;
          break;
          
          case 14:
          digitalWrite(r1_pin,LOW);      //1
          digitalWrite(r2_pin,LOW);      //1
          digitalWrite(r3_pin,LOW);      //1
          digitalWrite(r4_pin,HIGH);     //0
          R = 2.7;
          break;
          
          case 15:
          digitalWrite(r1_pin,LOW);      //1
          digitalWrite(r2_pin,LOW);      //1
          digitalWrite(r3_pin,LOW);      //1
          digitalWrite(r4_pin,LOW);      //1
          R = 2.7;
          break;   

          case 16:                       // Fake state
          loop_time_stop = millis();
          emergency_stop = true;         //Emergency stop started;
          
          R = 2.7;
          break;
          
          default:
              // 
          break;
        }
        return R;
      
    }

////      Control de carga PD      
    double Load::relay_switch(double RPM_read, double prev_RPM_read){
           if (RPM_read > RPM_optim + RPM_range){
              local_state = load_state -(RPM_optim - RPM_read)*Load_P_gain + ((RPM_read - prev_RPM_read)/(T_control*1e-6))*Load_D2_gain;
              if( local_state  <= 16){
                  load_state = local_state;
              } 
           }
           if (RPM_read < RPM_optim - RPM_range){
              local_state = load_state -(RPM_optim - RPM_read)*Load_P_gain + ((RPM_read - prev_RPM_read)/(T_control*1e-6))*Load_D2_gain;
              if ( local_state >= 0){
                   load_state = local_state;
              }
           }
           return load_state;
    }
    
////      Control de carga con acción derivativa al arranque del generador =! control de carga   
    double Load::relay_start(double RPM_read, double prev_RPM_read){
           if (RPM_read > prev_RPM_read){
              local_state = load_state +((RPM_read - prev_RPM_read)/(T_control*1e-6))*Load_D1_gain; //D control
              if( local_state  <= 16){
                  load_state = local_state;
              } 
           }
           if (RPM_read <= prev_RPM_read){
              local_state = load_state +((RPM_read - prev_RPM_read)/(T_control*1e-6))*Load_D1_gain; //D control
              if ( local_state >= 0){ 
                   load_state = local_state;
              }
           }
           return load_state;      
    }






    
