//GET IMU WORKING CHECK!!!!!!!!!! 

//FINISH MOVEMENT STATE MACHINE WHY DOESNT RUN WHEN ITS IN STATE MACHINE 

#include "Timer.h"
#include <NewPing.h>
#include "Time.h"
 
 
Timer t;





// GLOBAL FOR SONIC READ
//long duration_left;
long distance_left; // ACTUAL DISTANCE

//long duration_middle;
long distance_middle; // ACTUAL DISTANCE

//long duration_right;
long distance_right; // ACTUAL DISTANCE

int trig_left = 11;
int echo_left = 12;

int trig_middle = 9;
int echo_middle = 10;

int trig_right = 7;
int echo_right = 8;


//if doing new_ping
int MAX_DISTANCE = 200;
 
NewPing sonar_left(trig_left, echo_left, MAX_DISTANCE);

NewPing sonar_middle(trig_middle, echo_middle, MAX_DISTANCE);

NewPing sonar_right(trig_right, echo_right, MAX_DISTANCE);
 
//MAKE SURE I FILL IN CORRECT PIN VALUES LATER


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

int left_speed = 250;
int right_speed = 250;

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

enum motor_states { INIT_MOTOR , FORWARD, LEFT_TURN , RIGHT_TURN,REVERSE_TURN,HALT,REVERSE} motor_states;

 
void setup()
{
  Serial.begin(9600);
  // pinMode(pin, OUTPUT);
  //blink_Init();
  //t.every(1000, blink_Tick);


//USE 
  //init functions for state machines
  
  sonic_init();
  photo_init();
  motor_init();


  t.every(1000,sonic_tick);
  t.every(1000,photo_tick); 
  t.every(8,movement_tick);



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
     front_photo_value = map(front_photo_value, 454,1023,0,500);// 730
     back_photo_value = map(back_photo_value,77,600,0,500);

     Serial.print( "LEFT PHOTO ");
     Serial.println(left_photo_value);
     
     Serial.print( "RIGHT PHOTO ");
     Serial.println(right_photo_value);
     
     Serial.print( "FRONT PHOTO ");
     Serial.println(front_photo_value);
     Serial.print("BACK PHOTO ");
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
  /*
    //left sonic
    digitalWrite(trig_left,LOW);
    delay(10);
    digitalWrite(trig_left,HIGH);
    delay(10);
    digitalWrite(trig_left,LOW);
    duration_left = pulseIn(echo_left,HIGH);
    distance_left = (duration_left/2) / 29.1;
    Serial.print("LEFT DISTANCE ");
    Serial.println(distance_left);

    //middle sonic
    digitalWrite(trig_middle,LOW);
    delay(10);
    digitalWrite(trig_middle,HIGH);
    delay(10);
    digitalWrite(trig_middle,LOW);
    duration_middle = pulseIn(echo_middle,HIGH);
    distance_middle = (duration_middle/2) / 29.1;
    Serial.print("MIDDLE DISTANCE ");
    Serial.println(distance_middle);

    //right sonic
    digitalWrite(trig_right,LOW);
    delay(10);
    digitalWrite(trig_right,HIGH);
    delay(10);
    digitalWrite(trig_right,LOW);
    duration_right = pulseIn(echo_right,HIGH);
    distance_right = (duration_right/2) / 29.1;
    Serial.print("RIGHT DISTANCE ");
    Serial.println(distance_right);
    
  */
    distance_left = sonar_left.ping_cm();
    distance_middle = sonar_middle.ping_cm();
    distance_right = sonar_right.ping_cm();
   
    Serial.print("LEFT DISTANCE ");
    Serial.println(distance_left);

    Serial.print("MIDDLE DISTANCE ");
    Serial.println(distance_middle);
        
    Serial.print("RIGHT DISTANCE ");
    Serial.println(distance_right);
    

    
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

    
    Serial.println("INIT MOTOR state function");
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

     Serial.println("FORWARD state function");
     digitalWrite(turn_on_en_1,HIGH);
     digitalWrite(turn_on_en_2,HIGH);

     digitalWrite(motor_1_logic_1,LOW);
     analogWrite(motor_1_logic_2,left_speed);
     
     digitalWrite(motor_2_logic_1,LOW);
     analogWrite(motor_2_logic_2,right_speed);
     break;
   case LEFT_TURN:
      left_turn();
      break;

   case RIGHT_TURN:
     right_turn();
      break;

   case REVERSE_TURN:
      reverse_turn();
      
      break;
      
   


   case HALT:
      Serial.println("Halt state function ");
/*      time_1 = millis();

      time_2 = 1;

      while(time_2 - time_1 < 20000){
      time_2 = millis();
      */
      digitalWrite(turn_on_en_1,LOW);
      digitalWrite(turn_on_en_2,LOW);

      digitalWrite(motor_1_logic_1,LOW);
      digitalWrite(motor_1_logic_2,LOW);
      digitalWrite(motor_2_logic_1,LOW);
      digitalWrite(motor_2_logic_2,LOW);
      //}
      delay(5000);
      break;      


  case REVERSE:
    digitalWrite(turn_on_en_1,HIGH);
    digitalWrite(turn_on_en_2,HIGH);
    
    digitalWrite(motor_1_logic_1,HIGH);
    digitalWrite(motor_1_logic_2,LOW);
    
    digitalWrite(motor_2_logic_1,HIGH);
    digitalWrite(motor_2_logic_2,LOW);
  


  } 
  


    //transitions 
    switch(motor_states){
     case INIT_MOTOR:
        //motor_states = FORWARD;    
        motor_states = LEFT_TURN;
        break;
     case FORWARD:
     
        /* 
        //IF ABOUT TO HIT A FRONT WALL
        //ASSUMING THAT ITS NOT SURROUNDED ON BOTH SIDES 
        /*
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
        */
        
                    // GREEN                                           //RED
        if (left_photo_value > 700 && right_photo_value > 700 && front_photo_value > 450 && back_photo_value > 450) {
          motor_states = HALT;
        }
        
        else if ((left_photo_value- right_photo_value) >30 ){
          motor_states = LEFT_TURN;  
        }

        
        
        else if ((right_photo_value-left_photo_value) >30 ){
          motor_states = RIGHT_TURN;
        }

        else if ((back_photo_value - front_photo_value) > 20){
          motor_states = REVERSE;
        }
        else{
          motor_states = FORWARD;
        }
      
          
        break;
      
      
      //use below to test motors and such
      
      //motor_states = FORWARD;  
       
       //motor_states = HALT;
     
       // motor_states = LEFT_TURN;
      
     case LEFT_TURN:
        //motor_states = FORWARD;
        //motor_states = LEFT_TURN;
        
                    // GREEN                                           //RED
        if (left_photo_value > 700 && right_photo_value > 700 && front_photo_value > 450 && back_photo_value > 450) {
          motor_states = HALT;
        }
        
        else if ((left_photo_value- right_photo_value) >30 ){
          motor_states = LEFT_TURN;  
        }

        
        
        else if ((right_photo_value-left_photo_value) >30 ){
          motor_states = RIGHT_TURN;
        }

        else if ((back_photo_value - front_photo_value) > 20){
          motor_states = REVERSE;
        }
        else{
          motor_states = FORWARD;
        }
      
        break;
     case RIGHT_TURN:
        //motor_states = FORWARD;
       // motor_states = LEFT_TURN;

        
                    // GREEN                                           //RED
        if (left_photo_value > 700 && right_photo_value > 700 && front_photo_value > 450 && back_photo_value > 450) {
          motor_states = HALT;
        }
        
        else if ((left_photo_value- right_photo_value) >30 ){
          motor_states = LEFT_TURN;  
        }

        
        
        else if ((right_photo_value-left_photo_value) >30 ){
          motor_states = RIGHT_TURN;
        }

        else if ((back_photo_value - front_photo_value) > 20){
          motor_states = REVERSE;
        }
        else{
          motor_states = FORWARD;
        }
      
        break;
     case HALT:
        //motor_states = FORWARD;
        //motor_states = LEFT_TURN;
       
                    // GREEN                                           //RED
        if (left_photo_value > 700 && right_photo_value > 700 && front_photo_value > 450 && back_photo_value > 450) {
          motor_states = HALT;
        }
        
        else if ((left_photo_value- right_photo_value) >30 ){
          motor_states = LEFT_TURN;  
        }

        
        
        else if ((right_photo_value-left_photo_value) >30 ){
          motor_states = RIGHT_TURN;
        }

        else if ((back_photo_value - front_photo_value) > 20){
          motor_states = REVERSE;
        }
        else{
          motor_states = FORWARD;
        }
        
        
        break;
     
     default:
        break;
    }
  
}

//GLOBAL TURN FUNCTIONS

void left_turn(){
  digitalWrite(turn_on_en_1,HIGH);
  digitalWrite(turn_on_en_2,HIGH);

  
  digitalWrite(motor_1_logic_1,HIGH);
  digitalWrite(motor_1_logic_2,LOW);
  digitalWrite(motor_2_logic_1,LOW);
  digitalWrite(motor_2_logic_2,HIGH);
  delay(420);
}


void right_turn(){
  digitalWrite(turn_on_en_1,HIGH);
  digitalWrite(turn_on_en_2,HIGH);

  
  digitalWrite(motor_1_logic_1,LOW);
  analogWrite(motor_1_logic_2,HIGH);
  analogWrite(motor_2_logic_1,HIGH);
  digitalWrite(motor_2_logic_2,LOW);
  delay(420);
}

void reverse_turn(){
  
  digitalWrite(turn_on_en_1,HIGH);
  digitalWrite(turn_on_en_2,HIGH);

  
  digitalWrite(motor_1_logic_1,HIGH);
  digitalWrite(motor_1_logic_2,LOW);
  digitalWrite(motor_2_logic_1,LOW);
  digitalWrite(motor_2_logic_2,HIGH);
  delay(840);
}


