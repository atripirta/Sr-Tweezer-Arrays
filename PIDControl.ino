// Before running this code QNimble needs to be setup on Arduino IDE, the details can be seen from their site.
// Depending on the Setpoint the Proportional and Integral parameters may need to be changed.
#define SETPOINT 0.35

//double adc2=0.0;
void setup(void) {  
  //configureADC(2,1,0,BIPOLAR_1250mV,getADC2);
  configureADC(1,1,0,BIPOLAR_1250mV,getADC1);
   // Have ADC take measurement every 1us, ±1.25V range
  
  
}
static double prev_adc = 0;
void getADC1(void) {
  static double integral = 0.0;
  double newadc = readADC1_from_ISR(); //read ADC voltage
  double prop = (newadc-SETPOINT)*1.05 ; //proportional
  integral += (newadc - SETPOINT) *0.08; // integral gain
  double newdac = prop + integral;
  if((abs(newdac)<0.9)){// this is the output for mixer
  writeDAC(1,-newdac);
 // writeDAC(3,newdac);
  }
  else{
  writeDAC(1,0.9); //Upper bound for RF power output to AOM
 // writeDAC(3,newdac);
  }// writeDAC(3,--); command is just to see what feedback it is giving to the amplifier, this line can be commented out.
  
  //invert for negative feedback  
  //O/P for Oscilloscope to see PD outputs
 // prev_adc = newadc; //store new adc value for differential calculation
}


void loop() { 
  
}
