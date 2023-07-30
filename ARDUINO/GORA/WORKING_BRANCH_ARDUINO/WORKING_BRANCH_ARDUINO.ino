//#include "ServoEasing.hpp"
#include <Servo.h>

#include <AccelStepper.h>
#include <MultiStepper.h>


//number of characters int the serial strign
#define NUMBER_OF_CHARACTERS 27
// 120 -> zfill(4)  -> 0120       12 --> zfill(6) --> 000012
#define ZFILL 4
//Step size for stepping servos
#define STEP_SIZE 1
bool EMERGENCY_BOOL  = false;

float thetaList[6];









// EG X-Y position bed driven by 2 steppers
// Alas its not possible to build an array of these with different pins for each :-(
AccelStepper stepper1(AccelStepper::DRIVER, 11, 12);
AccelStepper stepper2(AccelStepper::DRIVER, 10, 9);

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;



//ServoEasing myServo;











//======================================================= 25 CHARACTER IMPLEMENTATION WITH A+B ENCODING

  //Reads the serial monitor and parses into thetaList with length 6, also parses the emergency bool variable which should be global


  //num_char is the total number of characters in the string including the stop and start characters 'A' and 'B'
  //ZFILL is defined as:  requested= 120 --> ZFILL(4) -->  0120: Bassically makes string length equal to zfill number
  

  //default EX: length = 27, zfill=4:    A0000000000000000000000000B == 0000 0000 0000 0000 0000 0000 0


  //IMPROVEMENT: Reliability

void parseSerialInput4GORA(float (&theta_list)[6] , bool EMERGENCY_BOOL, int num_char, int z_fill) {
  char buffer[num_char]; // static buffer to store incoming data
  static uint8_t buf_pos = 0; // static variable to track buffer position

  //While serial is available, USED TO ITERATE THROUGH STRING AS CHAR
  while (Serial.available() > 0) {


          // GET CHARACTER FROM SERIAL AND ANALYZE IT

    //Read a character from it
    char c = Serial.read();


    //IF Character is A, then its the start
    if (c == 'A') { // if found start character
      buf_pos = 0;    // reset buffer position
      continue;


      //If B is read, end the string and leave the while loop
      //ends the process of reading from serial for this main loop iter
    } else if (c == 'B') { // if found end character
      buffer[buf_pos] = '\0'; // null-terminate the string
      break;
    
    }else if(c == 'R'){
      //Call main loop
      break;
    }


    //STORE CHARACTER IN BUFFER IF ITS NOT A OR B
    if (buf_pos < num_char) { // ensure buffer overflow protection
      buffer[buf_pos++] = c; // store character in buffer
    }
  }

  // Split the input into six three-digit numbers
  for (int i = 0; i < ((num_char -1)/z_fill); i++) {
    String sub_str = String(buffer).substring(i*4, (i+1)*4);

    
    theta_list[i] = sub_str.toFloat();

  }

  // Access the 25th digit before the end character 'B', THE EMERGENCY BOOL
  if (buf_pos >= num_char) {
    String sub_str = String(buffer).substring(num_char-1, num_char);
    EMERGENCY_BOOL = sub_str.toInt();
  }


}

//======================================================SECOND IMPLEMENTATION//   parseSerialInput4GORA









void handshake(){
  

  
//=============================================================================HANDSHAKE PROCESS TO INSURE RELIABLE COMMS
//FLUSHES DATA
//READS SERIAL FROM COMPUTER
//WHILE COMPUTER ISNT RESPONDING WITH "SHAKING" i.e: WHILE "SHAKING" ISNT IN SERIAL
      //READ SERIAL TO UPDATE CONTENT TO CHECK FOR "SHAKING" ON NEXT LOOP
      //IF SERIAL IS AVAILABLE FOR WRITING
          //WRITE "SHAKING" TO COMPUTER

//DELAY AND FLUSH
//CALL MAINLOOP

  //==================================================   HANDSHAKE PROCESS WITH COMPUTER
  //FLUSH ALL DATA FOR RELIABILITY
  Serial.flush();

  //READ STRING FROM COMPUTER
  String content = Serial.readStringUntil('\n');
  Serial.println(content);



  //WHILE THE MSG FROM COMPUTER IS NOT SAYING SHAKING: WHILE THERE IS NO HANDSHAKE ESTABLISHED, LOOP
  while(content != "SHAKING"){

    //UPDATE MSG FROM COMPUTER FOR NEXT LOOP
    content = Serial.readStringUntil('\n');
    Serial.println(content);




    // IF Serial(COMPUTER -> MASTER) ARDUINO IS WRITABLE
    if (Serial.availableForWrite() ) {

      //REQUEST HANDSHAKE
      Serial.write("SHAKING");
      //Serial.println("SHAKING");

    }
    //200 MILLISECOND PHASE DELAY
    delay(200);
    Serial.flush();


    }//=================================================

    
    
}//======================================================================================= TERMINATE HANDSHAKE




/* 

EXAMPLE STRINGS ZFILL=4 NUM CHAR = 26

A015601310035002000300040B
A002101310036002000300040B
A001701190027002000300040B
               
A004400000027002000300040B
A006801800025002000300040B

*/













void setup() {
  // put your setup code here, to run once:

  //BEIGN SERIAL CONNECTION WITH COMPUTER
  Serial.begin(115200);
  //SET TIMEOUT INTERFACE
  Serial.setTimeout(10);
  
  //HANDSHAKE UNTIL RELIABLE CONNECTION IS ESTABLISHED
  handshake();  

  // Configure each stepper
  stepper1.setMaxSpeed(2000);
  stepper2.setMaxSpeed(2000);

  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);

    }




//IF EMERGENCY BOOL IS EVER TRUE, THE CODE WILL BREAK MAINLOOP AND ENTER VOID LOOP
void loop(){


  
  //IF NO ERRORS ARE DETECTED
  if(! EMERGENCY_BOOL){
    
  
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    READ DATA FROM SERIAL MONITOR
  parseSerialInput4GORA(thetaList, EMERGENCY_BOOL, NUMBER_OF_CHARACTERS, ZFILL);
  Serial.flush();


  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      WRITE TO MOTORS
  long pos[2];
  pos[0] = thetaList[1] * -8 * 21;
  pos[1] = thetaList[2] * -8 * 21;



  if(pos[0] != 0  && pos[1] != 0){
  steppers.moveTo(pos);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  }



  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     SEND READY MESSAGE
  Serial.print("READY");
  Serial.print(pos[0]);
  Serial.println(pos[1]);
  delay(100);
  Serial.flush();
  



}//else{break;}//    IF EMERGENCY, BREAK MAINLOOP
}

