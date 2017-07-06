/*
 This program enables the reading of DBUS 25 bits input 
 and output to DBUS 18 bit or 25 bit

  reading result = dBus.channels[0-7]

*/
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>
#include "DJI_DBUS.h"

//there are two versionsof DJI controller protocol
// one is the latestest firmware with 25 bits with 7 channel
//another is the old one with 6 channel of 18 bits
//please use America coding methods
/*
 * *
for each data 
mid value = bin 10000000000   (1024)   0x400
max value = bin 11010010100   (1684)   0x694
max value = bin 00101101100   (364)   0x16C
 */
void write18BitsDbusData();
boolean running = false;
const byte channel = 6;
//output format

static int shoot = 0;
static unsigned int lastChange = millis();
const unsigned int chgTime = 100;
int shooting(int);
uint32_t updatetime = 0;

//channel define
#define LEFT_UD 3
#define LEFT_LR 2
#define RIGHT_UD 1
#define RIGHT_LR 0
#define CHANNEL_L 5
#define CHANNEL_R 4
#define CHANNEL_UP 1
#define CHANNEL_MID 3
#define CHANNEL_DOWN 2
static uint16_t DBus_Output[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
static uint16_t ROS_Output[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
static uint16_t DBus_Final_Output[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
//the value of channel can only be 1-3 , the sequence is 01,11,10
//DBus_Output[LEFT_UD] is the value of left up and down

DJI_DBUS dBus(Serial1);
int led = 13; 
int reset = 8;
uint32_t currTime, displayTime = 0;
uint8_t i;

//ros
ros::NodeHandle nh;
void joy_cb( const sensor_msgs::Joy& joy);
ros::Subscriber<sensor_msgs::Joy> sub_joy("/vel_cmd", joy_cb);


void setup(){
  pinMode(led, OUTPUT);
  
//  Serial.println("DBUS Status");
  Serial2.begin(100000,SERIAL_8E1);
  dBus.begin();
  //ros
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub_joy);
  //Serial.begin(115200);
}

int joy_pub_count =0;
boolean LED_flag = false;
void loop(){
  
  if (dBus.Failsafe() == DBUS_SIGNAL_OK) LED_flag = true;
  else LED_flag =false;
  dBus.FeedLine();
  //digitalWrite(led, LOW);
  currTime = millis();

  if (dBus.toChannels == 1){
    dBus.UpdateChannels();
    dBus.toChannels = 0;
    
    for(i=0;i<channel;i++){
        DBus_Output[i] = dBus.channels[i];
    }
    
  }
  
  if(displayTime < currTime) {
      displayTime = currTime + 7;
      joy_pub_count++;
      write18BitsDbusData();
     
      if(joy_pub_count>10){
        joy_pub_count=0;
        nh.spinOnce();
      }
      //printDBUSStatus();
  }
    
  if(LED_flag)
    digitalWrite(led, HIGH);
  else 
    digitalWrite(led, LOW);
}

int shooting(int cmd){
  //0: not shooting, channel mid
  //1: motor running, channel up
  //2: shooting, channel down
  //3: not shooting, channel up
  //4: motor running, channel mid
  
  unsigned int curr = millis();
  if (cmd==1){
    if (shoot==3 && (curr - lastChange) > chgTime){
      shoot = 0; lastChange = curr; }
    else if (shoot==0 && (curr - lastChange) > chgTime){
      shoot = 1; lastChange = curr; }
    else if (shoot==1 && (curr - lastChange) > chgTime){
      shoot = 4; lastChange = curr; }
    else if (shoot==4 && (curr - lastChange) > chgTime){
      shoot = 2; lastChange = curr; }
  }
  
  else if (cmd==0){
    if (shoot==1 && (curr - lastChange) > chgTime){ 
      shoot = 4; lastChange = curr; }
    else if (shoot==2 && (curr - lastChange) > chgTime){
      shoot = 4; lastChange = curr; }
    else if (shoot==4 && (curr - lastChange) > chgTime){
      shoot = 3; lastChange = curr; }
    else if (shoot==3 && (curr - lastChange) > chgTime){
      shoot = 0; lastChange = curr; }
  }
  
  if (shoot==0) return CHANNEL_MID;
  else if (shoot==1) return (uint16_t)CHANNEL_UP;
  else if (shoot==2) return (uint16_t)CHANNEL_DOWN;
  else if (shoot==4) return (uint16_t)CHANNEL_MID;
  else if (shoot==5) return (uint16_t)CHANNEL_MID;
  else if (shoot==3) return (uint16_t)CHANNEL_UP;
}

void joy_cb( const sensor_msgs::Joy& joy){
  updatetime = millis();
  //left button up means auto
  //sequence : left  right_UD  right_LR
    ROS_Output[LEFT_LR] = (uint16_t)(joy.buttons[2]);
    ROS_Output[LEFT_UD] = (uint16_t)(joy.buttons[3]);
    ROS_Output[RIGHT_LR] = (uint16_t)(joy.buttons[0]);
    ROS_Output[RIGHT_UD] = (uint16_t)(joy.buttons[1]);
    if (DBus_Output[CHANNEL_R] == CHANNEL_UP) ROS_Output[CHANNEL_L] = shooting(joy.buttons[4]);
    else ROS_Output[CHANNEL_L] = CHANNEL_MID;
}
void write18BitsDbusData(){
  //05 04 20 00 01 D8 00 00 00 00 00 00 00 00 00 00 00 00 original 
  //00 04 20 00 01 58 00 00 00 00 00 00 00 00 00 00 00 00 adjusted    
  
  if(DBus_Output[CHANNEL_R] == CHANNEL_UP && currTime - updatetime < 500){
    //auto mode
    DBus_Final_Output[LEFT_UD] = ROS_Output[LEFT_UD];
    DBus_Final_Output[LEFT_LR] = ROS_Output[LEFT_LR];
    DBus_Final_Output[RIGHT_UD] = ROS_Output[RIGHT_UD];
    DBus_Final_Output[RIGHT_LR] = ROS_Output[RIGHT_LR];
    DBus_Final_Output[CHANNEL_L] = ROS_Output[CHANNEL_L];
    
  }else{
    if (DBus_Output[CHANNEL_R] == CHANNEL_DOWN) DBus_Final_Output[CHANNEL_R] = CHANNEL_DOWN;
    else DBus_Final_Output[CHANNEL_R] = CHANNEL_UP;
    DBus_Final_Output[LEFT_UD] = DBus_Output[LEFT_UD];
    DBus_Final_Output[LEFT_LR] = DBus_Output[LEFT_LR];
    DBus_Final_Output[RIGHT_UD] = DBus_Output[RIGHT_UD];
    DBus_Final_Output[RIGHT_LR] = DBus_Output[RIGHT_LR];
    if (shoot==0 && (currTime - lastChange) > chgTime) DBus_Final_Output[CHANNEL_L] = DBus_Output[CHANNEL_L];
    else DBus_Final_Output[CHANNEL_L] = shooting(0);
  }
  
  //add safety system
  if (DBus_Output[CHANNEL_L] == 0 && DBus_Output[CHANNEL_R]==0 && dBus.updatetime > 1000) {
    pinMode(reset, OUTPUT);
    digitalWrite(reset, LOW);
  }
  
  if(currTime - dBus.updatetime > 500){  
    DBus_Output[CHANNEL_L] = CHANNEL_MID;
    DBus_Final_Output[CHANNEL_L] = shooting(0);
    if (shoot==0) DBus_Final_Output[CHANNEL_R] = CHANNEL_MID;
    else DBus_Final_Output[CHANNEL_R] = CHANNEL_UP;
    DBus_Final_Output[LEFT_LR] = 1024;
    DBus_Final_Output[RIGHT_UD] = 1024;
    DBus_Final_Output[RIGHT_LR] = 1024;
    DBus_Final_Output[LEFT_UD] = 1024;
    LED_flag = false;
  }
  
  if(DBus_Final_Output[RIGHT_LR]>1524)
    DBus_Final_Output[RIGHT_LR]=1524;
  if(DBus_Final_Output[LEFT_LR]>1524)
    DBus_Final_Output[LEFT_LR]=1524;
  if(DBus_Final_Output[RIGHT_UD]>1524)
    DBus_Final_Output[RIGHT_UD]=1524;
  if(DBus_Final_Output[LEFT_UD]>1524)
    DBus_Final_Output[LEFT_UD]=1524;
  
  if(DBus_Final_Output[RIGHT_LR]<524)
    DBus_Final_Output[RIGHT_LR]=524;
  if(DBus_Final_Output[LEFT_LR]<524)
    DBus_Final_Output[LEFT_LR]=524;
  if(DBus_Final_Output[RIGHT_UD]<524)
    DBus_Final_Output[RIGHT_UD]=524;
  if(DBus_Final_Output[LEFT_UD]<524)
    DBus_Final_Output[LEFT_UD]=524;
  long sum = (((long)DBus_Final_Output[LEFT_LR]-1024)*((long)DBus_Final_Output[LEFT_LR]-1024)+
  ((long)DBus_Final_Output[RIGHT_LR]-1024)*((long)DBus_Final_Output[RIGHT_LR]-1024)+
  ((long)DBus_Final_Output[RIGHT_UD]-1024)*((long)DBus_Final_Output[RIGHT_UD]-1024)
  );
  if(sum>(long)480*480*2){
    //Serial.println(sum);
      if(DBus_Final_Output[RIGHT_LR]>1224)
        DBus_Final_Output[RIGHT_LR]=1224;
      if(DBus_Final_Output[LEFT_LR]>1224)
        DBus_Final_Output[LEFT_LR]=1224;
      if(DBus_Final_Output[RIGHT_UD]>1224)
        DBus_Final_Output[RIGHT_UD]=1224;
      
      if(DBus_Final_Output[RIGHT_LR]<824)
        DBus_Final_Output[RIGHT_LR]=824;
      if(DBus_Final_Output[LEFT_LR]<824)
        DBus_Final_Output[LEFT_LR]=824;
      if(DBus_Final_Output[RIGHT_UD]<824)
        DBus_Final_Output[RIGHT_UD]=824;
  }
  
  
  
  

    
  Serial2.write((uint8_t) ( ((DBus_Final_Output[0]&0x00FF)>>0) ) );//data1 0-7
  Serial2.write((uint8_t) ( ((DBus_Final_Output[0]&0x0700)>>8) | ((DBus_Final_Output[1]&0x001F)<<3) ) );//data1 8-10 data2 0-4 
  Serial2.write((uint8_t) ( ((DBus_Final_Output[1]&0x07E0)>>5) | ((DBus_Final_Output[2]&0x0003)<<6) ) );// data2 5-10 data3 0-1
  Serial2.write((uint8_t) ( ((DBus_Final_Output[2]&0x03FC)>>2) ) );// data3 2-9
  Serial2.write((uint8_t) ( ((DBus_Final_Output[2]&0x0400)>>10) | ((DBus_Final_Output[3]&0x007F)<<1) ) );// data3 10 data 4 0-6
  Serial2.write((uint8_t) ( ((DBus_Final_Output[3]&0x0780)>>7) | ((DBus_Final_Output[4]&0x0003)<<4) | ((DBus_Final_Output[5]&0x0003)<<6) ) );// data3 10 data 4 0-6
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
  Serial2.write(0x00);
}
void printDBUSStatus()
{
  Serial.print("Thr ");
  Serial.print(DBus_Output[2]);
  Serial.print(" Ail ");
  Serial.print(DBus_Output[0]);
  Serial.print(" Ele ");
  Serial.print(DBus_Output[1]);
  Serial.print(" Rud ");
  Serial.print(DBus_Output[3]);
  Serial.print(" Channel1 ");
  Serial.print(DBus_Output[5]);
  Serial.print(" Channel2  ");
  Serial.print(DBus_Output[4]);
  Serial.print(" Stat ");
  if (dBus.Failsafe() == DBUS_SIGNAL_FAILSAFE) {
    Serial.print("FailSafe");
  } else if (dBus.Failsafe() == DBUS_SIGNAL_LOST) {
    Serial.print("Signal Lost");
  } else if (dBus.Failsafe() == DBUS_SIGNAL_OK) {
    Serial.print("OK");
  }
  Serial.println(".");
  
}
