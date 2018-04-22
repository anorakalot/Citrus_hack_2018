//GET IMU WORKING CHECK!!!!!!!!!! 
//FINISH MOVEMENT STATE MACHINE PRETTY SURE ITS DONE

#include "Timer.h"
#include <NewPing.h>
#include "Time.h"
 
 
Timer t;

//GLOBAL FOR IMU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define OUTPUT_READABLE_YAWPITCHROLL
 
//Timer t;

float yawangle = 0;

MPU6050 mpu;

enum imuStates {INIT_1, SAMPLE} imu_States;
enum outStates {INIT_2, OUT} out_States;

#define pin 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}





// GLOBAL FOR SONIC READ
long duration_left;
long distance_left; // ACTUAL DISTANCE

long duration_middle;
long distance_middle; // ACTUAL DISTANCE

long duration_right;
long distance_right; // ACTUAL DISTANCE

//if doing new_ping
//int MAX_DISTANCE = 200
 
//NewPing sonar_left(trig_left, echo_left, MAX_DISTANCE);

//NewPing sonar_middle(trig_middle, echo_middle, MAX_DISTANCE);

//NewPing sonar_right(trig_right, echo_right, MAX_DISTANCE);
 
//MAKE SURE I FILL IN CORRECT PIN VALUES LATER
int trig_left = 22;
int echo_left = 23;

int trig_middle = 0;
int echo_middle = 0;

int trig_right = 0;
int echo_right = 0;


//GLOBAL FOR PHOTO_RES_STATES

int left_photo = 17;

int right_photo = 16;

int front_photo = 15;

int back_photo = 14;

int left_photo_value;
int right_photo_value;
int front_photo_value;
int back_photo_value;

//GLOBAL MOTOR

int motor_1_logic_1 = 23;
int motor_1_logic_2 = 22;

int motor_2_logic_1 = 20;
int motor_2_logic_2 = 21;

int turn_on_en_1 = 5;
int turn_on_en_2 = 6;

int left_speed = 150;
int right_speed = 150;

float curr_left;
float curr_right;
float curr_rever;

//TIME FOR HALT MOVEMENT GLOBAL VARIABLES 
//time_t time_1;
//time_t time_2;
unsigned long time_1;
unsigned long time_2;

//enum blinkStates {INIT_1, ON, OFF} blink_States;



//STATES FOR STATE MACHINES 
enum sonic_states { INIT_SONIC, ON_1, OFF_1} sonic_states;

enum photo_res_states { INIT_PHOTO, ON_2, OFF_2} photo_res_states;

enum motor_states { INIT_MOTOR , FORWARD, LEFT_TURN , RIGHT_TURN,REVERSE_TURN,HALT} motor_states;

 
void setup()
{
  Serial.begin(9600);
 // pinMode(pin, OUTPUT);
  //blink_Init();
  //t.every(1000, blink_Tick);


  //init functions for state machines
//  sonic_init();
//  photo_init();
  motor_init();


  //t.every(1000,sonic_tick);
  //t.every(1000,photo_tick); 
  t.every(400,movement_tick);


//TESTING
/*
      pinMode(turn_on_en_1,OUTPUT);
      pinMode(turn_on_en_2,OUTPUT);
        
      pinMode(motor_1_logic_1,OUTPUT);
        
      pinMode(motor_1_logic_2,OUTPUT);
        
      pinMode(motor_2_logic_1,OUTPUT);
        
      pinMode(motor_2_logic_2,OUTPUT);
        
      digitalWrite(turn_on_en_1,LOW);
      digitalWrite(turn_on_en_2,LOW);
*/





  //IMU
  
// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  
  pinMode(pin, OUTPUT);
  
  imu_Init();
  out_Init();
  //t.every(10, imu_Tick);
  //t.every(10, out_Tick);


}
 
 
void loop()
{
  t.update();


}


void photo_init() {
  photo_res_states = INIT_PHOTO;
}

void photo_tick() {
  //ACTIONS
  switch(photo_res_states){
    case INIT_PHOTO:
      pinMode(left_photo,INPUT);
      pinMode(right_photo,INPUT);
      pinMode(front_photo,INPUT);
      pinMode(back_photo,INPUT);
      break;
    case ON_2:
     
     left_photo_value = analogRead(left_photo);
     right_photo_value = analogRead(right_photo);
     front_photo_value = analogRead(front_photo);
     back_photo_value = analogRead(back_photo);

     //NEED TO MAP FRONT AND BACK 
     //ASSUMING FRONT IS RED AND BACK IS BLUE
     front_photo_value = map(front_photo_value, 730,1023,0,500);
     back_photo_value = map(back_photo_value,110,550,0,500);
     
     Serial.println(left_photo_value);
     Serial.println(right_photo_value);
     Serial.println(front_photo_value);
     Serial.println(back_photo_value);
     Serial.println();
     break;
    case OFF_2:
     break;
    default:
     break;
      
  }

  switch(photo_res_states) {
    case INIT_PHOTO:
      photo_res_states = ON_2;
      break;
    case ON_2:
      photo_res_states = OFF_2;
      
     break;
    case OFF_2:
      photo_res_states = ON_2;
     break;
    default:
      break;
    
  }
  
}


//SONIC SENSOR STATE MACHINE
void sonic_init() {
  sonic_states = INIT_SONIC;
  
}


void sonic_tick(){
  //actions
  switch(sonic_states) {
   case INIT_SONIC:
    pinMode(trig_left,OUTPUT);
    pinMode(echo_left,INPUT);

    pinMode(trig_middle,OUTPUT);
    pinMode(echo_middle,INPUT);
    
    pinMode(trig_right,OUTPUT);
    pinMode(echo_right,INPUT);
    
    break;
   
   case ON_1:
    //left sonic
    digitalWrite(trig_left,LOW);
    delay(10);
    digitalWrite(trig_left,HIGH);
    delay(10);
    digitalWrite(trig_left,LOW);
    duration_left = pulseIn(echo_left,HIGH);
    distance_left = (duration_left/2) / 29.1;
    Serial.println(distance_left);

    //middle sonic
    digitalWrite(trig_middle,LOW);
    delay(10);
    digitalWrite(trig_middle,HIGH);
    delay(10);
    digitalWrite(trig_middle,LOW);
    duration_middle = pulseIn(echo_middle,HIGH);
    distance_middle = (duration_middle/2) / 29.1;
    Serial.println(distance_middle);

    //right sonic
    digitalWrite(trig_right,LOW);
    delay(10);
    digitalWrite(trig_right,HIGH);
    delay(10);
    digitalWrite(trig_right,LOW);
    duration_right = pulseIn(echo_right,HIGH);
    distance_right = (duration_right/2) / 29.1;
    Serial.println(distance_right);
    
  
    //distance_left = sonar_left.ping_cm();
    //distance_middle = sonar_middle.ping_cm();
    //distance_right = sonar_right.ping_cm();
   

    
    break;
   case OFF_1:
    break;

   default:
    break;
  }


  //transitions
  switch(sonic_states) {
    case INIT_SONIC:
      sonic_states = ON_1;
      break;
    case ON_1:
      sonic_states = OFF_1;
      break;
    case OFF_1:
      sonic_states = ON_1;
      break;
    default:
      break;    
  }

  
}


//MOTOR STATE MACHINE
void motor_init() {
  motor_states = INIT_MOTOR;
  
}

void movement_tick() {
  //actions
  switch(motor_states){
    case INIT_MOTOR:
      pinMode(turn_on_en_1,OUTPUT);
      pinMode(turn_on_en_2,OUTPUT);
        
      pinMode(motor_1_logic_1,OUTPUT);
        
      pinMode(motor_1_logic_2,OUTPUT);
        
      pinMode(motor_2_logic_1,OUTPUT);
        
      pinMode(motor_2_logic_2,OUTPUT);
        
      digitalWrite(turn_on_en_1,LOW);
      digitalWrite(turn_on_en_2,LOW);
      break;
    case FORWARD:

    /*
      digitalWrite(turn_on_en_1,HIGH);
      digitalWrite(turn_on_en_2,HIGH);
      
      //analogWrite(motor_1_logic_2,left_speed);
      digitalWrite(motor_1_logic_2,HIGH);
      digitalWrite(motor_1_logic_1,LOW);
      //analogWrite(motor_2_logic_2,right_speed);
      //digitalWrite(motor_1_logic_2,HIGH);
      //digitalWrite(motor_2_logic_1,LOW);
     */
       digitalWrite(turn_on_en_1,HIGH);
       digitalWrite(turn_on_en_2,HIGH);
  
       analogWrite(motor_1_logic_2,left_speed);
       digitalWrite(motor_1_logic_1,LOW);
       analogWrite(motor_2_logic_2,right_speed);
       digitalWrite(motor_2_logic_1,LOW);

      
      break;
    //check logic for turns
    case LEFT_TURN:
     curr_left = yawangle; 
      while(curr_left - yawangle < 90){
        left_turn();
      }
      
      break;
  //check logic for turns 
   case RIGHT_TURN:
      curr_right = yawangle;
      while(yawangle - curr_right < 90){
        right_turn();
      }
      break;

//pretty sure this is right 
//cant be sure until testing
    case REVERSE_TURN:
      curr_rever = yawangle;
      //just does left turn until its reversed      
      while(curr_rever-yawangle < 180){
        left_turn();
      }
      
      break;
      
   


   //test this as well
   case HALT:
      time_1 = millis();

      time_2 = 1;

      while(time_2 - time_1 < 20000){
      time_2 = millis();
      digitalWrite(turn_on_en_1,LOW);
      digitalWrite(turn_on_en_2,LOW);

      digitalWrite(motor_1_logic_1,LOW);
      digitalWrite(motor_1_logic_2,LOW);
      digitalWrite(motor_2_logic_1,LOW);
      digitalWrite(motor_2_logic_2,LOW);
      }
      break;      

  } 
  


    //transitions 
    switch(motor_states){
     case INIT_MOTOR:
        motor_states = FORWARD;    
        break;
     case FORWARD:
    /*
        //IF ABOUT TO HIT A FRONT WALL
        //ASSUMING THAT ITS NOT SURROUNDED ON BOTH SIDES 
        if (distance_middle < 10){
          if (distance_left < 10){
            motor_states = RIGHT_TURN;
          }
          else if (distance_right< 10){
            motor_states = LEFT_TURN;
          }
          //if neither just do a right turn
          else{
            motor_states = RIGHT_TURN;
          }
        }

                    // GREEN                                           //RED
        else if (left_photo_value > 750 && right_photo_value > 750 && front_photo_value > 460 && back_photo_value > 570) {
          motor_states = HALT;
        }
        
        else if (left_photo_value > right_photo_value ){
          motor_states = LEFT_TURN;  
        }
        else if (right_photo_value > left_photo_value){
          motor_states = RIGHT_TURN;
        }
        else{
          motor_states = FORWARD;
        }
      */
      //use below to test motors and such
      motor_states = FORWARD;  
        break;

     case LEFT_TURN:
        motor_states = FORWARD;
        break;
     case RIGHT_TURN:
        motor_states = FORWARD;
        break;
     case HALT:
        motor_states = FORWARD;
        break;
     
     default:
        break;
    }
  
}

//GLOBAL TURN FUNCTIONS

void left_turn(){
  digitalWrite(turn_on_en_1,HIGH);
  digitalWrite(turn_on_en_2,HIGH);

  
  digitalWrite(motor_1_logic_1,100);
  digitalWrite(motor_1_logic_2,LOW);
  digitalWrite(motor_2_logic_1,LOW);
  digitalWrite(motor_2_logic_2,100);

}


void right_turn(){
  digitalWrite(turn_on_en_1,HIGH);
  digitalWrite(turn_on_en_2,HIGH);

  
  digitalWrite(motor_1_logic_1,LOW);
  analogWrite(motor_1_logic_2,100);
  analogWrite(motor_2_logic_1,100);
  digitalWrite(motor_2_logic_2,LOW);
}




//IMU

void imu_Init() {
  imu_States = INIT_1;
}

void imu_Tick()
{
  //static int count = 0;
  //Actions
  switch(imu_States) {
    case INIT_1:
      break;
     case SAMPLE:
     // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yawangle = ypr[0] * 180/M_PI;
        #endif
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(pin, blinkState);
    }
      break;
     default:
      break;
  }

  //Transitions
  switch(imu_States) {
    case INIT_1:
      imu_States = SAMPLE;
      break;
     case SAMPLE:
      imu_States = SAMPLE;
      break;
     default:
      break;
  }
}

void out_Init() {
  out_States = INIT_2;
}

void out_Tick() {
  switch(out_States) {
    case INIT_2:
      break;
    case OUT:
      Serial.println(yawangle);
      break;
    default:
      break;
  }

  switch(out_States) {
    case INIT_2:
      out_States = OUT;
      break;
    case OUT:
      out_States = OUT;
      break;
    default:
      break;
  }
}

