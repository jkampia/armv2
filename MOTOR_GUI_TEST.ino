
#include <Wire.h>

const int STEPPIN1 =  8;
const int DIRPIN1 = 9;
const int ENAPIN1 = 10;

const int STEPPIN2 =  5;
const int DIRPIN2 = 6;
const int ENAPIN2 = 7;

double angle_offset;

double MOTOR1_STEPS_PER_DEG;
double MOTOR2_STEPS_PER_DEG;

int magnetStatus = 0;
int i = 0;

int endchar = 6969;

int J1_steps_list[32000];
int J2_steps_list[32000];
int J3_steps_list[32000];
int J4_steps_list[32000];
int J5_steps_list[32000];
int J6_steps_list[32000];

struct joint { // USED FOR PASSING JOINT INFORMATION INTO FUNCTIONS IN A MORE CONCISE MANNER

  double max_step_delay;
  double min_step_delay;
  double accel_rate;
  double desired_position;
  double current_position;
  
};

struct robot { // HOLDS ALL 6 ROBOT JOINTS

  joint J1;
  joint J2;
  joint J3;
  joint J4;
  joint J5;
  joint J6;
  
};

robot JO2; // DECLARE MAIN ROBOT 

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("Serial open");

  Wire.begin();

  Wire.setClock(800000L);

  pinMode(STEPPIN1, OUTPUT);
  pinMode(DIRPIN1, OUTPUT);
  pinMode(ENAPIN1, OUTPUT);

  pinMode(STEPPIN2, OUTPUT);
  pinMode(DIRPIN2, OUTPUT);
  pinMode(ENAPIN2, OUTPUT);

  Serial.println("Checking magnet presence...");

  checkMagnetPresence();
  ReadRawAngle();

  Serial.println("Encoder working!");

  angle_offset = ReadRawAngle();
  Serial.println("Angle offset: " + String(angle_offset));

  MOTOR1_STEPS_PER_DEG = 8.888888888888889; 360 / 3200;

  for (int i = 0; i < 1600; i++) {

    J1_steps_list[i] = i * 80;
    
  }

  J1_steps_list[1600] = endchar;

  //J1_steps_list[1600] = endchar;

  for (int i = 0; i < 3200; i++) { 

    J2_steps_list[i] = i * 40;
    
  }

  J2_steps_list[3200] = endchar;

  ///////////////// ROBOT SETUP ////////////////////

  JO2.J1.accel_rate = 50;
  JO2.J1.max_step_delay = 5000;
  JO2.J1.min_step_delay = 50;

  JO2.J2.accel_rate = 50;
  JO2.J2.max_step_delay = 5000;
  JO2.J2.min_step_delay = 50;

  JO2.J3.accel_rate = 50;
  JO2.J3.max_step_delay = 5000;
  JO2.J3.min_step_delay = 50;

  JO2.J4.accel_rate = 50;
  JO2.J4.max_step_delay = 5000;
  JO2.J4.min_step_delay = 50;

  JO2.J5.accel_rate = 50;
  JO2.J5.max_step_delay = 5000;
  JO2.J5.min_step_delay = 50;

  JO2.J6.accel_rate = 50;
  JO2.J6.max_step_delay = 5000;
  JO2.J6.min_step_delay = 50;
  

  Serial.println("Exiting setup.");
 
}

void loop() {

  digitalWrite(ENAPIN1, HIGH); // LOW = STEPPER DISABLED
  digitalWrite(ENAPIN2, HIGH); // LOW = STEPPER DISABLED

  //ReadRawAngle(); 
  
  if (Serial.available() > 0) {

    int type = Serial.read();

    if ((char)type == 'P') {

      int pos1 = Serial.parseInt();

      int pos2 = Serial.parseInt();

      int endchar = Serial.read();

      if ((char)endchar == 'E') {

        Serial.println("Motor 1 desired position: " + String(pos1));
        Serial.println("Motor 2 desired position: " + String(pos2));

         //JO2.J1.desired_position = pos;
         

         //generateStepArrays(JO2);

         //interruptStepLoop(J1_steps_list, J2_steps_list);
       
         //ReadRawAngle();
         
      }
      
    }

  }

  //float cur_angle = ReadRawAngle(); 
  //Serial.println(cur_angle); 
  
}

void interruptStepLoop(int *array1, int *array2) {

  int steps_log = 0;
  
  int i = 0;
  int j = 0;
  int k = 0;
  int a = 0;
  int b = 0;
  int c = 0; 

  bool trigger1 = false;
  bool trigger2 = false;
  bool trigger3 = false;
  bool trigger4 = false;
  bool trigger5 = false;
  bool trigger6 = false;

  float prev_time = 0;

  while (true) {

    if (array1[i] == endchar || array2[i] == endchar) {

      break;
      
    }

    if (array1[i] == steps_log) { // CONSISTENTLY APPLIES ~ 5 MICROSECONDS OF DELAY
      
      //Serial.println("Motor1: " + String(array1[i]));
      trigger1 = true;
      i++;
      
    }

    if (array2[j] == steps_log) {

      //Serial.println("Motor2: " + String(array2[j]));
      //instaStepMotor(STEPPIN2);
      trigger2 = true;
      j++;
      
    }

    mstep(trigger1, trigger2); 

    delayMicroseconds(1);

    //float cur_time = micros(); // TIME CHECK 
      //float time_elapsed = cur_time - prev_time; 
      //Serial.println("Elapsed microseconds: " + String(time_elapsed));
      //prev_time = micros();
    
    steps_log++;

    trigger1 = false; trigger2 = false; trigger3 = false; trigger4 = false; trigger5 = false; trigger6 = false; 

    //Serial.println(steps_log);
    
  }
  
}

void ScaleArrays(int *array1, int *array2) {

  int i, j;
  int longest_array;

  while(true) {

    if (array1[i] == endchar) {

      break;
    }
    else {

      i++;
      
    }
  }
  while(true) {

    if (array2[j] == endchar) {

      break;
      
    }
    
    else {

      j++;
      
    }
    
  }

  int J1_steps_list_length = i;
  int J2_steps_list_length = j;

  if (i > j) {

    longest_array = 1;
    
  }
}

void stepMotor(const int steppin, const int dirpin, int timedelay, int dir) {

  if (dir == 1) {
    digitalWrite(dirpin, HIGH);
  }
  else if (dir == 2) {
    digitalWrite(dirpin, LOW);
  }

  float truedelay = timedelay * 0.5;

  digitalWrite(steppin, HIGH);
  delayMicroseconds(truedelay);
  digitalWrite(steppin, LOW);
  delayMicroseconds(truedelay);
  
}

void mstep(bool m1, bool m2) {

  if (m1 == true) {
    digitalWrite(STEPPIN1, HIGH);
  }
  if (m2 == true) {
    digitalWrite(STEPPIN2, HIGH);
  }

  delayMicroseconds(1);

  digitalWrite(STEPPIN1, LOW); 
  digitalWrite(STEPPIN2, LOW);
  
}

void generateStepArrays(robot input) {

  double dist_degrees = input.J1.desired_position - ReadRawAngle();

  Serial.println("J1 degree distance: " + String(dist_degrees));

  double total_steps = abs(dist_degrees) * MOTOR1_STEPS_PER_DEG;

  Serial.println("J1 steps to take: " + String(total_steps));
  
  int steps_taken = 0;

  int motor_delay = input.J1.max_step_delay;

  bool plataeu = true; // MOTOR WILL REACH MAX SPEED

  int accel_intervals = (input.J1.max_step_delay - input.J2.min_step_delay) / input.J1.accel_rate;
  
  int decel_setpoint = total_steps - accel_intervals; 

  int motor_direction = 1;

  if (dist_degrees > 0) {

    motor_direction = 2;
    
  }
  else if (dist_degrees < 0) {

    motor_direction = 1;
    
  }

  if (accel_intervals > total_steps / 2) {

    plataeu = false; // MOTOR WILL NEVER REACH MAX SPEED
    
  }

  if (plataeu == true) {

    while (steps_taken <= total_steps) {

      if (steps_taken < accel_intervals) {

        motor_delay -= input.J1.accel_rate;
        
      }
      
      else if (steps_taken >= accel_intervals && steps_taken <= decel_setpoint) {

        motor_delay = input.J1.min_step_delay;
        
      }
      
      else if (steps_taken > decel_setpoint) {

        motor_delay += input.J1.accel_rate;
        
      }

      J1_steps_list[steps_taken + 1] = J1_steps_list[steps_taken] + motor_delay; // ADD CALCULATED NEXT MOTOR DELAY TO THE I + 1 ELEMENT OF THE STEP LIST

      steps_taken++;
      
    }
    
  }

  else if (plataeu == false) {

    accel_intervals = total_steps * 0.5;

    while (steps_taken <= total_steps) {

      if (steps_taken < accel_intervals) {

        motor_delay -= input.J1.accel_rate;
        
      }
      
      else if (steps_taken >= accel_intervals) {

        motor_delay += input.J1.accel_rate;
      
      }

      J1_steps_list[steps_taken + 1] = J1_steps_list[steps_taken] + motor_delay;

      steps_taken++;
      
    }
    
  }

  double final_error = correctJointPosition(input.J1.desired_position);

  Serial.println("Steps taken: " + String(steps_taken));

  Serial.println("Error: " + String(final_error));
  
}

double correctJointPosition(double pos) { // COMPARES CURRENT JOINT POSITION (READRAW ANGLE) AND DESIRED POSITION AND APPLIES SMALL CORRECTION

  double error = pos - ReadRawAngle();

  while (abs(error) > 0.2) {

    if (error > 0) {

      stepMotor(STEPPIN1, DIRPIN1, 5000, 2);
      
    }
    
    else {

      stepMotor(STEPPIN1, DIRPIN1, 5000, 1);
      
    }

    error = pos - ReadRawAngle();
    
  }

  return error;
  
}

float correctAngle(float angle) {

  // correctAngle modifies each output of ReadRawAngle to treat the initial joint angle (when the arduino is turned on)
  float correctedAngle;

  if (angle > angle_offset) {
     correctedAngle = angle - angle_offset;
  }
  else if (angle == angle_offset) {
    correctedAngle = 0;
  }
  else {
     correctedAngle = 360 + angle - angle_offset;
  }

  return correctedAngle;
  
}

/////////////////////// ENCODER STUFF //////////////////////////

float ReadRawAngle() // CALL TAKES AROUND 130 MICROSECONDS
{

    //----- read low-order bits 7:0
    Wire.beginTransmission(0x36); // connect to the sensor
    // read both low + high order bits (0x0C and 0x0D, autoincrementin address)
    // figure 21 - register map: Raw angle (11:8)
    // figure 21 - register map: Raw angle (7:0)
    Wire.write(0x0C);
    Wire.endTransmission();       // end transmission
    uint8_t ret = Wire.requestFrom(0x36, 2);    // request from the sensor
    //Serial.println("Wire.requestFrom(): " + String(ret));
    while (Wire.available() != 2) {} // wait until it becomes available
    int highbyte = Wire.read(); // Reading the data after the request
    int lowbyte = Wire.read();

    //Serial.println("Received from Wire.read(): " + String(highbyte) +  ", " + String(lowbyte));

    uint16_t rawAngle = (uint16_t)((highbyte & 0xffu)<< 8u) | (lowbyte & 0xffu);
    //Serial.println("Raw angle: " + String(rawAngle));
    float degAngle = rawAngle * 0.087890625; // 360/4096 = 0.087890625
    Serial.println("Current angle: " + String(degAngle, 4));
    degAngle = correctAngle(degAngle);
    return degAngle;
}

void checkMagnetPresence() {
  
  //Status register output: 0 0 MD ML MH 0 0 0
  //MH: Too strong magnet - 100111 - DEC: 39
  //ML: Too weak magnet - 10111 - DEC: 23
  //MD: OK magnet - 110111 - DEC: 55

  // ---- Check MD status bit (magnet detected)
  while ((magnetStatus & B00100000) != 32)                      // locks MCU until magnet correctly positioned
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36);                               //connect to the sensor
    Wire.write(0x0B);                                           //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission();                                     //end transmission
    Wire.requestFrom(0x36, 1);                                  //request from the sensor
    while (Wire.available() == 0);                              //wait until it becomes available
    magnetStatus = Wire.read();                                 //Reading the data after the request
    Serial.print("Magnet Status  ");
    Serial.println(magnetStatus, BIN);
    Serial.println(" ");
  }
  
  delay(1000);
  
}
