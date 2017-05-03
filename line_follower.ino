/* Line follower using 6 LED array
* Tested with Arduino Mega 2560
* Developed by Balaraman Lakshmanan
* Mail: Balaraman.L@gmail.com
* Open license. Feel free to use this.
*/


/*
* Hardware needed for this project
* Arduino Mega 2560
* 6 IR array line sensor 
* 2 motor drivers 
* 2 DC motors 
* 2 Wheels
* 1 base
* Jumber cables
* Double tapes & glue to stick
* Nuts & Bolts
*/

int motor_right_forward = 30;
int motor_right_reverse = 32;
int motor_left_forward = 34;
int motor_left_reverse = 36;
int motor_right_speed = 8; // PWM pin
int motor_left_speed = 9; // PWM pin

int motion_speed_control = 255; //Can adjust motor speed here
int turn_speed_control = 255; //Can adjust motor speed here

/*
* 6 IR array line sensor
* L1 L2 UP DOWN R2 R1
* |  |  |   |   |  |
* v  v  v   v   v  v
* 24 26 28 29  27  25
*/

const int left1_pin = 24;
const int left2_pin = 26;
const int right1_pin = 25;
const int right2_pin = 27;
const int up_pin = 28;
const int down_pin = 29;

int left1 = 0;
int right1 = 0;
int left2 = 0;
int right2 = 0;
int up = 0;
int down = 0;


void setup() 
{
  Serial.begin(9600); //baud rate
  setupMotors();
  setupLineFollower();
}

void setupLineFollower()
{
  pinMode(left1_pin, INPUT);
  pinMode(left2_pin, INPUT);
  pinMode(right1_pin, INPUT);
  pinMode(right2_pin, INPUT);
  pinMode(up_pin, INPUT);
  pinMode(down_pin, INPUT);
}

void setupMotors()
{
  pinMode(motor_left_forward, OUTPUT);
  pinMode(motor_right_forward, OUTPUT);
  pinMode(motor_left_reverse, OUTPUT);
  pinMode(motor_right_reverse, OUTPUT);
  pinMode(motor_left_speed, OUTPUT);
  pinMode(motor_right_speed, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  line_follower();

  /* Uncomment following lines to test using serial monitor */
  // test_arduino();
  // test_ir();
}

void stop_motors()
{
  Serial.println("Stopping motors");
  digitalWrite(motor_left_forward, LOW);
  digitalWrite(motor_left_reverse, LOW);
  digitalWrite(motor_right_forward, LOW);
  digitalWrite(motor_right_reverse, LOW);
  digitalWrite(motor_left_speed,LOW);
  digitalWrite(motor_right_speed,LOW);
}

void backward()
{
  Serial.println("Backward");
  digitalWrite(motor_left_speed,HIGH);
  digitalWrite(motor_right_speed,HIGH);
  digitalWrite(motor_left_reverse, HIGH);
  digitalWrite(motor_left_forward, LOW);
  digitalWrite(motor_right_reverse, HIGH);
  digitalWrite(motor_right_forward, LOW);
}

void forward()
{
  
  Serial.println("Forward");
  analogWrite(motor_right_speed, motion_speed_control);
  analogWrite(motor_left_speed, motion_speed_control);
  digitalWrite(motor_left_reverse, LOW);
  digitalWrite(motor_left_forward, HIGH);
  digitalWrite(motor_right_reverse, LOW);
  digitalWrite(motor_right_forward, HIGH);

}

void turn_left()
{
  Serial.println("left");
  analogWrite(motor_right_speed, turn_speed_control);
  analogWrite(motor_left_speed, turn_speed_control);
  digitalWrite(motor_left_reverse, LOW);
  digitalWrite(motor_left_forward, LOW);
  digitalWrite(motor_right_reverse, LOW);
  digitalWrite(motor_right_forward, HIGH);
}

void turn_right()
{
  Serial.println("right");
  analogWrite(motor_right_speed, turn_speed_control);
  analogWrite(motor_left_speed, turn_speed_control);
  digitalWrite(motor_left_reverse, LOW);
  digitalWrite(motor_left_forward, HIGH);
  digitalWrite(motor_right_reverse, LOW);
  digitalWrite(motor_right_forward, LOW);
}

void turn_left_hard()
{
  Serial.println("Hard left");
  analogWrite(motor_right_speed, turn_speed_control);
  analogWrite(motor_left_speed, turn_speed_control);
  digitalWrite(motor_left_reverse, HIGH);
  digitalWrite(motor_left_forward, LOW);
  digitalWrite(motor_right_reverse, LOW);
  digitalWrite(motor_right_forward, HIGH);
}

void turn_right_hard()
{
  Serial.println("Hard right");
  analogWrite(motor_right_speed, turn_speed_control);
  analogWrite(motor_left_speed, turn_speed_control);
  digitalWrite(motor_left_reverse, LOW);
  digitalWrite(motor_left_forward, HIGH);
  digitalWrite(motor_right_reverse, HIGH);
  digitalWrite(motor_right_forward, LOW);
}

void line_follower()
{

   // Follows black line and stops when placed out of line

   getValuesFromIRArray();

   if(isCenterIn())
   {
      forward();
   }
   else if(isRightSideIn())
   {
      turn_right_hard();
   }
   else if(isLeftSideIn())
   {
     turn_left_hard();
   }
   else if(stoppingCondition())
   {
     stop_motors();
   }
   
}

void getValuesFromIRArray()
{
   left1 = digitalRead(left1_pin); 
   left2 = digitalRead(left2_pin); 
   right1 = digitalRead(right1_pin); 
   right2 = digitalRead(right2_pin); 
   up = digitalRead(up_pin); 
   down = digitalRead(down_pin); 
}

bool isCenterIn()
{
  if((up > 0 || down > 0) && left1 == 0 && right1 == 0)
    return true;
  else
    return false;
}

bool isLeftSideIn()
{
  if((left1 == 1 || left2 == 1) && (right1 == 0 && right2 == 0))
      return true;

  else
      return false;
}

bool isRightSideIn()
{
  if((right1 == 1 || right2 == 1) && (left1 == 0 && left2 == 0))
      return true;

  else
      return false;
}

bool stoppingCondition()
{
  if((up == 0 && down == 0 && left2 == 0 && right2 == 0 && left1 == 0 && right1 == 0) || (up == 1 && down == 1 && left2 == 1 && right2 == 1 && left1 == 1 && right1 == 1))
    return true;
  else
    return false;
}

//test functions

void test_arduino(){
  Serial.println("Testing Arduino & Motor driver");
  forward();
  delay(1000);
  backward();
  delay(1000);
  turn_left();
  delay(1000);
  turn_right();
  delay(1000);
  stop_motors();
  delay(5000);
}

void test_ir(){

  int left1 = -1;
  int right1 = -1;
  int left2 = -1;
  int right2 = -1;
  int up = -1;
  int down = -1;

  Serial.println("Testing IR object sensor");
  
  left1 = digitalRead(left1_pin); 
  left2 = digitalRead(left2_pin); 
  right1 = digitalRead(right1_pin); 
  right2 = digitalRead(right2_pin); 
  up = digitalRead(up_pin); 
  down = digitalRead(down_pin);

  Serial.println("Left1::");
  Serial.println(left1);
  Serial.println("Right1::");
  Serial.println(right1);
  Serial.println("Left2::");
  Serial.println(left2);
  Serial.println("Right2::");
  Serial.println(right2);
  Serial.println("Up::");
  Serial.println(up);
  Serial.println("Down::");
  Serial.println(down);
  delay(1000);
}

