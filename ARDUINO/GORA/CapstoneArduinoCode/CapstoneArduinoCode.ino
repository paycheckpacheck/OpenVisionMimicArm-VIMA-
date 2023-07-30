#include <AccelStepper.h>

// Variables for Communication
#define NUMBER_OF_CHARACTERS 27
#define ZFILL 4
bool emergency = false;

// Variables for Movement
float thetaList[6];                      // stores target angle
int targetPos[5];                        // Stores target position
int oldPos[5];                           // Stores last position
int reduction[] = { 21, 1, 1, 1, 1, 1 };  // Gear reduction of each joint
int maxVelocity = 100;                   // Steps per second
double moveTime = 0.05;                  // Time to complete each move in seconds

int counter = 0;


// Define Steppers
AccelStepper stepper[] = { (AccelStepper::DRIVER, 12, 11),  // an array of steppers (Step, Direction) pins
                           (AccelStepper::DRIVER, 10, 9),
                            };

void parseSerialInput4GORA(float (&theta_list)[6], bool EMERGENCY_BOOL, int num_char, int z_fill) {
  char buffer[num_char];       // static buffer to store incoming data
  static uint8_t buf_pos = 0;  // static variable to track buffer position

  //While serial is available, USED TO ITERATE THROUGH STRING AS CHAR
  while (Serial.available() > 0) {


    // GET CHARACTER FROM SERIAL AND ANALYZE IT

    //Read a character from it
    char c = Serial.read();


    //IF Character is A, then its the start
    if (c == 'A') {  // if found start character
      buf_pos = 0;   // reset buffer position
      continue;


      //If B is read, end the string and leave the while loop
      //ends the process of reading from serial for this main loop iter
    } else if (c == 'B') {     // if found end character
      buffer[buf_pos] = '\0';  // null-terminate the string
      break;

    } else if (c == 'R') {
      //Call main loop
      break;
    }


    //STORE CHARACTER IN BUFFER IF ITS NOT A OR B
    if (buf_pos < num_char) {  // ensure buffer overflow protection
      buffer[buf_pos++] = c;   // store character in buffer
    }
  }

  // Split the input into six three-digit numbers
  for (int i = 0; i < ((num_char - 1) / z_fill); i++) {
    String sub_str = String(buffer).substring(i * 4, (i + 1) * 4);


    theta_list[i] = sub_str.toFloat();
  }

  // Access the 25th digit before the end character 'B', THE EMERGENCY BOOL
  if (buf_pos >= num_char) {
    String sub_str = String(buffer).substring(num_char - 1, num_char);
    EMERGENCY_BOOL = sub_str.toInt();
  }
}

void handshake() {

  //==================================================   HANDSHAKE PROCESS WITH COMPUTER
  //FLUSH ALL DATA FOR RELIABILITY
  Serial.flush();

  //READ STRING FROM COMPUTER
  String content = Serial.readStringUntil('\n');
  Serial.println(content);



  //WHILE THE MSG FROM COMPUTER IS NOT SAYING SHAKING: WHILE THERE IS NO HANDSHAKE ESTABLISHED, LOOP
  while (content != "SHAKING") {

    //UPDATE MSG FROM COMPUTER FOR NEXT LOOP
    content = Serial.readStringUntil('\n');
    Serial.println(content);




    // IF Serial(COMPUTER -> MASTER) ARDUINO IS WRITABLE
    if (Serial.availableForWrite()) {

      //REQUEST HANDSHAKE
      Serial.write("SHAKING");
      //Serial.println("SHAKING");
    }
    //200 MILLISECOND PHASE DELAY
    delay(200);
    Serial.flush();
  }  //=================================================

  /*while (Serial.available()) {
    Serial.read();
  }*/
  return;
}


void setup() {
  // Start serial communication
  Serial.begin(115200);
  Serial.setTimeout(10);
  handshake();
  Serial.println("READY");

  // Initialize steppers
  for (int i = 0; i < 5; i++) { stepper[i].setMaxSpeed(maxVelocity); }
}

void loop() {
  if (Serial.available() > 20) {

    counter++;

    unsigned long startTime = millis();

    // Read data from serial
    parseSerialInput4GORA(thetaList, emergency, NUMBER_OF_CHARACTERS, ZFILL);

    Serial.write("Target Angle: ");
    Serial.println(thetaList[0]);

    // Calculate move
    for (int i = 0; i < 1; i++) {

      // Convert angles to steps
      targetPos[i] = round((thetaList[i] / 1.8) * reduction[i]);

      // Calculate move velocity and acceleration
      double velocity = 2 * (abs(double(targetPos[i]) - double(oldPos[i])) / moveTime);
      double acceleration = velocity / (moveTime / 2.0);

      // Set stepper parameters
      stepper[i].setMaxSpeed(velocity);
      stepper[i].setAcceleration(acceleration);
      stepper[i].moveTo(targetPos[i]);

      // Store Last position
      oldPos[i] = targetPos[i];
    }
    Serial.write("Steps: ");
    Serial.println(round(abs(double(targetPos[0]) - stepper[0].currentPosition())));
    Serial.write("Velocity: ");
    Serial.println(stepper[0].maxSpeed());
    Serial.write("Acceleration: ");
    Serial.println(stepper[0].acceleration());

    // Perform move
    while (stepper[0].isRunning() || stepper[1].isRunning() || stepper[2].isRunning() || stepper[3].isRunning() || stepper[4].isRunning()) {
      for (int i = 0; i < 5; i++) {
        stepper[i].run();
      }
    }
    for (int i = 0; i < 5; i++) {
      stepper[i].run();
    }

    while (Serial.available()) {
      Serial.read();
    }

    Serial.write("Loop Count: ");
    Serial.println(counter);
    Serial.write("Loop Time: ");
    Serial.println(millis() - startTime);
    Serial.println("");
  }
}
