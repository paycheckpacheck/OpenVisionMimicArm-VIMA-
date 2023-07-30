#include <math.h>
#include <Servo.h>

const int pin_Vout = 8; // output pin
const int pin_analog_in = A4; // analog input pin
const int sampleRate = 10; // sample rate in Hz
const int pin_Vout_in = A6;
const int frequency = 1; // frequency of sine wave in Hz
Servo servo1;

void setup() {
  // Set pinmode for output on pin8
  pinMode(pin_Vout, OUTPUT);
  Serial.begin(9600); // start serial communication
  Serial.flush();
  Serial.println("#time,corrected,V_in, Vout"); // header line
  servo1.attach(4);

}


double V_in_prior = analogRead(pin_analog_in);
int t= 0;

void loop() {
  //Vin from the PID control
  double V_in = analogRead(pin_analog_in); // read analog input
  //Read V_Out voltages
  double V_out_read = analogRead(pin_Vout_in);
  //Vout from arduino
  int V_out = (random(-100, 100)) ;

  //Corrected Vout 
  int V_out_corrected =  V_out + V_in_prior ;


  //Output Vout = sin
  int V_mapped = int(map(V_out, -400,0, 0, 255));
  analogWrite(pin_Vout, V_mapped); 

  Serial.print("Vout:"); Serial.print(V_out); Serial.print(", ");
  Serial.print("C_Vout:"); Serial.print(V_out_corrected); Serial.print(", ");
  Serial.print("Vin:"); Serial.print(V_in); Serial.print(", ");
  Serial.print("VoutRead:"); Serial.print(V_out_read); Serial.print(", ");


  Serial.println();
  int theta_map = int(map(V_in, 300, 900, 0, 90 ));

  servo1.write(theta_map);

  delay(500);

    

  // Vout from arduino

  V_in_prior = V_in;
  t++;


  ;
  }

