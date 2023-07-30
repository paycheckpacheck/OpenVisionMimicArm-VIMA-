#include "CytronMotorDriver.h"
#include <SimpleEncoder.h>

#include <Servo.h>

//THIS SCRIPT IS ON THE

int BUTTON_PIN = 20;
Servo myServo;

float thetaList[6];
bool EMERGENCY_BOOL  = false;
bool ready= false;
int priorServoAngle = 0;











//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control MOTOR DRIVER CLASS
class Motor : public CytronMD, public SimpleEncoder {
private:
  float speed;

public:
  Motor(int pwm_dir_pin, int pin1, int pin2, float speed)
    : CytronMD(pwm_dir_pin, pin1, pin2), SimpleEncoder(BUTTON_PIN, PINA, PINB, 0, -1000, 1000) {
    this->speed = speed;
    //CytronMD::begin();
  }

  void driveAngle(float angle) {
    float rotations = angle / 360.0;
    driveRotations(rotations);
  }

  void driveRotations(float rotations) {
    float deltaTime = rotations * 1000.0;
    setSpeed(speed);
    unsigned long startTime = millis();
    while (millis() - startTime < deltaTime) {
      setSpeed(speed);
    }
    setSpeed(0);
  }
};
float speed = 200;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% V1



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MOTOR INITS
Motor differA(PWM_DIR, 13, 12, speed);
Motor differB(PWM_DIR, 11, 10, speed);
Motor elbow(PWM_DIR, 9, 8, speed);
Motor gripper(PWM_DIR, 7, 6, speed);
Motor shoulder(PWM_DIR, 5, 4, speed);
Motor base(PWM_DIR, 3, 2, speed);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 





//======================================================= 25 CHARACTER IMPLEMENTATION WITH A+B ENCODING

  //Reads the serial monitor and parses into list
  //EX: 0000000000000000000000000 == 0000 0000 0000 0000 0000 0000 0

  
void parseSerialInput4GERMY(float (&theta_list)[6]) {
  // Wait for the start marker 'A'
  while (Serial.read() != 'A') {}

  // Read 24 characters from the serial monitor
  String input = Serial.readStringUntil('B');
  input.trim(); // Remove any leading or trailing white space

  // Split the input into six four-digit numbers
  for (int i = 0; i < 6; i++) {
    String sub_str = input.substring(i*4, (i+1)*4);
    theta_list[i] = sub_str.toFloat();
  }
}

//======================================================FIRST IMPLEMENTATION//   parseSerialInput4GERMY





//======================================================= 25 CHARACTER IMPLEMENTATION WITH A+B ENCODING

  //Reads the serial monitor and parses into list
  //EX: 0000000000000000000000000 == 0000 0000 0000 0000 0000 0000 0


  //IMPROVEMENT: Reliability

void parseSerialInput4GERMY(float (&theta_list)[6], bool EMERGENCY_BOOL) {
  static char buffer[25]; // static buffer to store incoming data
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
    if (buf_pos < 25) { // ensure buffer overflow protection
      buffer[buf_pos++] = c; // store character in buffer
    }
  }

  // Split the input into six three-digit numbers
  for (int i = 0; i < 6; i++) {
    String sub_str = String(buffer).substring(i*4, (i+1)*4);
    theta_list[i] = sub_str.toFloat();
  }

  // Access the 25th digit before the end character 'B', THE EMERGENCY BOOL
  if (buf_pos >= 25) {
    String sub_str = String(buffer).substring(24, 25);
    EMERGENCY_BOOL = sub_str.toInt();
  }


}

//======================================================SECOND IMPLEMENTATION//   parseSerialInput4GERMY



/* 

A015601310035002000300040B
A002101310036002000300040B
A001701190027002000300040B
               
A004401220027002000300040B
A006801240025002000300040B

*/













void setup() {
  // put your setup code here, to run once:

  //BEIGN SERIAL CONNECTION WITH COMPUTER
  Serial.begin(9600);
  //SET TIMEOUT INTERFACE
  Serial.setTimeout(1);


  //ATTACH TEST SERVO
  myServo.attach(46);

  
  //==================================================   HANDSHAKE PROCESS
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
      Serial.println("SHAKING");

    }
    //200 MILLISECOND PHASE DELAY
    delay(200);

    }//=================================================

    Serial.flush()

    
    
    mainloop();
    }







void mainloop() {
  // put your main code here, to run repeatedly:

  while (1){

    


  


  // if emergency, stop program
  //if (EMERGENCY_BOOL){ready = false;}

  //if ready to start program
  if (ready){

  
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  READ DATA FROM SERIAL MONITOR
  parseSerialInput4GERMY(thetaList, EMERGENCY_BOOL);

  


  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   WRITE TO MOTORS
  base.driveAngle(thetaList[0]);   //base
  shoulder.driveAngle(thetaList[1]);  //shoulder
  elbow.driveAngle(thetaList[2]);   //elbow
  differA.driveAngle(thetaList[2]); //gamma
  differB.driveAngle(thetaList[2]); //alpha
  gripper.driveAngle(thetaList[5]); //phi


  /*if(abs(priorServoAngle - thetaList[2])  >=3){
  //Write to test servo
  myServo.write(thetaList[2]); //}
  priorServoAngle = thetaList[2];}*/


  //IF computer sends flush command, flush serial
  if(Serial.peek() == 'F'){
    Serial.flush();
    
  }





  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  CHECK ACCURACY OF MOVEMENT

  



  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  MAKE CORRECTION

  //delay(1000);



  //SEND READY MESSAGE
  Serial.println("READY");

  



}
}}


