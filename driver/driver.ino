#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

const int DirKi = 2;
const int DirKa = 3;
// Put one pin empty. In our test, left motor's speed can't be controlled
// when pin 4 is used.
const int PwmKi = 5;
const int PwmKa = 6;
const int EmergencyButtonPin = 7;

// Prevent runaway at first start
bool hasStart = false;

// State of emergency button
bool emergencyState;

// Required dimensions of the vehicle for differential drive
const
  float wheel_rad = 0.038, wheel_sep = 0.380,
  track_length = 1.473,
  max_linear_speed = 1.0;

ros::NodeHandle node;

void emergencyCheck()
{
  int _btn = digitalRead(EmergencyButtonPin);
  if (_btn==HIGH)
    emergencyState = false;
  else if(_btn==LOW)
    emergencyState = true;
}

const int Kanan = 1;
const int Kiri = 0;
const int Maju = 2;
const int Mundur = 3;
void Jalan(unsigned char motor, unsigned char arah, unsigned char kec)
{
//  if (emergencyState==true)
//    return;

  if(motor == Kanan){
    if(arah == Mundur) digitalWrite(DirKa,HIGH);
    else digitalWrite(DirKa,LOW);
    analogWrite(PwmKa,kec);
  }
  else{
    if(arah == Mundur) digitalWrite(DirKi,HIGH);
    else digitalWrite(DirKi,LOW);
    analogWrite(PwmKi,kec);
  }
}

template<typename T>
T clamp(const T& v, const T& vmin, const T& vmax)
{
  if (v < vmin)
    return vmin;
  if (v > vmax)
    return vmax;
  else return v; 
}

void twistCallback(const geometry_msgs::Twist &msg)
{
  if (hasStart==false) {
    hasStart = true;
    return;
  }
  
  auto speed_angular = msg.angular.z;
  auto speed_linear  = msg.linear.x;
  auto w_r = speed_linear + ((speed_angular*track_length)/2.0);
  auto w_l = speed_linear - ((speed_angular*track_length)/2.0);

  // Limit speed, then convert to duty cycle (0-255)
  auto w_r_b = clamp((float)fabsf(w_r), 0.0f, max_linear_speed);
  auto w_l_b = clamp((float)fabsf(w_l), 0.0f, max_linear_speed);
  uint8_t w_rc = (uint8_t)(w_r_b * 255.0);
  uint8_t w_lc = (uint8_t)(w_l_b * 255.0);
  Jalan(Kanan, w_r>0 ? Maju : Mundur, w_rc);
  Jalan(Kiri, w_l>0 ? Maju : Mundur, w_lc);
}


void init_motor()
{
  pinMode(PwmKa, OUTPUT);
  pinMode(PwmKi, OUTPUT);
  pinMode(DirKa, OUTPUT);
  pinMode(DirKi, OUTPUT);
  delay(1000);
}

ros::Subscriber<geometry_msgs::Twist> twistSub("cmd_vel", twistCallback);

void setup() 
{
  pinMode(EmergencyButtonPin, INPUT);
  emergencyCheck();
  init_motor();
  Serial.begin(57600);
  node.subscribe(twistSub);
}

void loop() 
{
  emergencyCheck();
  node.spinOnce();
}
