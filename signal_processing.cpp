
    #include "signal_processing.h"
    #include "Arduino.h"
    #include <arduinoFFT.h>
    arduinoFFT FFT = arduinoFFT();


///////////////////////////////   ANALOG READ FUNCTIONS  ////////////////////////////////////////////////  
    Signal::Signal(uint8_t _analog_pin,uint8_t _opt1_pin,uint8_t _opt2_pin){
            analog_pin = _analog_pin;
            opt1_pin = _opt1_pin;
            opt2_pin = _opt2_pin;
            pinMode(analog_pin,INPUT);
            pinMode(opt1_pin,OUTPUT);
            pinMode(opt2_pin,OUTPUT);
            digitalWrite(opt1_pin,LOW);       //scale = 1 (V <= 1,5V)
            digitalWrite(opt2_pin,HIGH);      //scale = 2 (1,5V <= V <= 3)
    } 

////      Lectura de la señal    
    void Signal::read_signal(int loop_itValue){
         vReal[loop_itValue] = ((analogRead(analog_pin))* (3.3 / 1024.0) - VOffset) / scale;
         vImag[loop_itValue] = 0;
         maior = max(vReal[loop_itValue],maior);
                     
      // regulate to approximately "sample_period" (max depends on BAUD and computer speed)
         while ((micros()-loop_time1)<sample_period);
         loop_time1 = micros();        
    }

////      Se controla la ganancia de entrada con los optoacopladores     
    void Signal::scale_value(double data){
         if (scale == 1 && data >= 1.5){
            digitalWrite(opt1_pin,LOW);
            digitalWrite(opt2_pin,HIGH);
            scale = 2;                    //Vread=Vin/2
         }
        if (scale == 2 && data < 0.5){
            digitalWrite(opt1_pin,HIGH);
            digitalWrite(opt2_pin,LOW);
            scale = 1;                    //Vread=Vin
        }
    }

////      Se obtiene la potencia generada      
    double Signal::power_meter(double resistance_value){
           V_RMS = maior / sqrt(2); 
           power = pow(V_RMS,2) * 1e3 / resistance_value;   // power in mW 
           maior = 0;
           return power;
    }


///////////////////////////////   FFT FUNCTIONS  ////////////////////////////////////////////////    
    double Signal::frequency_meter(){ 
           FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
           FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
           FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
           FFT.DCRemoval(vReal, SAMPLES);
           freq = FFT.MajorPeak(vReal, SAMPLES, sample_rate);
           // High pass filter
           if (freq < high_pass_filter){      
               freq = 0;
           }
           return freq;
    }
           

    


    
