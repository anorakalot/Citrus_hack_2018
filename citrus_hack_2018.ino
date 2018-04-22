#include "Timer.h"
 
 
Timer t;


// GLOBAL FOR SONIC READ
long duration_left;
long distance_left; // ACTUAL DISTANCE

long duration_middle;
long distance_middle; // ACTUAL DISTANCE

long duration_right;
long distance_right; // ACTUAL DISTANCE



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



//enum blinkStates {INIT_1, ON, OFF} blink_States;




enum sonic_states { INIT_1, ON_1, OFF_1} sonic_states;

enum photo_res_states { INIT_2, ON_2, OFF_2} photo_res_states;

enum motor_states { INIT_MOTOR,FORWARD,LEFT_TURN} motor_states;

 
void setup()
{
  Serial.begin(9600);
 // pinMode(pin, OUTPUT);
  //blink_Init();
  //t.every(1000, blink_Tick);

  sonic_init();
  t.every(10,sonic_tick);
  t.every(10,photo_tick); 

}
 
 
void loop()
{
  t.update();
}


void photo_init() {
  photo_res_states = INIT_2;
}

void photo_tick() {
  //ACTIONS
  switch(photo_res_states){
    case INIT_2:
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
     break;
    case OFF_2:
     break;
    default:
      break;
      
  }

  switch(photo_res_states) {
    case INIT_2:
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

void sonic_init() {
  sonic_states = INIT_1;
  
}


void sonic_tick(){
  //actions
  switch(sonic_states) {
   case INIT_1:
    pinMode(trig_left,OUTPUT);
    pinMode(echo_left,INPUT);

    pinMode(trig_middle,OUTPUT);
    pinMode(echo_middle,INPUT);
    
    pinMode(trig_right,OUTPUT);
    pinMode(echo_right,INPUT);
    break;
   case ON_1:
    
    digitalWrite(trig_left,LOW);
    delay(10);
    digitalWrite(trig_left,HIGH);
    delay(10);
    digitalWrite(trig_left,LOW);
    duration_left = pulseIn(echo_left,HIGH);
    distance_left = (duration_left/2) / 29.1;
    Serial.println(distance_left);

    digitalWrite(trig_middle,LOW);
    delay(10);
    digitalWrite(trig_middle,HIGH);
    delay(10);
    digitalWrite(trig_middle,LOW);
    duration_middle = pulseIn(echo_middle,HIGH);
    distance_middle = (duration_middle/2) / 29.1;
    Serial.println(distance_middle);

    digitalWrite(trig_right,LOW);
    delay(10);
    digitalWrite(trig_right,HIGH);
    delay(10);
    digitalWrite(trig_right,LOW);
    duration_right = pulseIn(echo_right,HIGH);
    distance_right = (duration_right/2) / 29.1;
    Serial.println(distance_right);
  
    break;
   case OFF_1:
    break;

   default:
    break;
  }


  //transitions
  switch(sonic_states) {
    case INIT_1:
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
      digitalWrite(turn_on_en_1,HIGH);
      digitalWrite(turn_on_en_2,HIGH);
  
      analogWrite(motor_1_logic_2,left_speed);
      digitalWrite(motor_1_logic_1,LOW);
      analogWrite(motor_2_logic_2,right_speed);
      digitalWrite(motor_2_logic_1,LOW);
      break;
    case LEFT_TURN:
      break;
  }

  
}



/*
void blink_Init() {
  blink_States = INIT_1;
}

void blink_Tick()
{
  //Actions
  switch(blink_States) {
    case INIT_1:
      break;
    case ON:
      digitalWrite(pin, HIGH);
      break;
    case OFF:
      digitalWrite(pin, LOW);
      break;
    default:
      break;
  }
  
  //Transitions
  switch(blink_States) {
    case INIT_1:
      blink_States = ON;
      break;
     case ON:
      blink_States = OFF;
      break;
     case OFF:
      blink_States = ON;
      break;
     default:
      break;
  }
}
*/
