/*
 This program enables the reading of DBUS 25 bits input 
 and output to DBUS 18 bit or 25 bit

  reading result = dBus.channels[0-7]

*/
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
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

//shooting
static boolean shoot = false;
static int step = 0;
static uint16_t ch_shoot = CHANNEL_MID;
static unsigned int lastChange = millis();
const unsigned int chgTime = 100;
void startShoot(void);
void stopShoot(void);

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
static int32_t ROS_Upload[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
static float ROS_Localization_Upload[6]  = {0, 0, 0, 0, 0, 0};
//the value of channel can only be 1-3 , the sequence is 01,11,10
//DBus_Output[LEFT_UD] is the value of left up and down

DJI_DBUS dBus(Serial1);
int led = 13; 
uint32_t currTime, displayTime = 0;
uint8_t i;

//ros
ros::NodeHandle nh;
void joy_cb( const sensor_msgs::Joy& joy);
void publish_joy(void);
void readLocalizationSystem(void);
void publish_localization(void);
sensor_msgs::Joy joy_msg;
geometry_msgs::Twist localization_msg;
ros::Subscriber<sensor_msgs::Joy> sub_joy("/vel_cmd", joy_cb);
ros::Publisher pub_joy( "/joy_msg", &joy_msg);
ros::Publisher pub_localization( "/localization", &localization_msg);
char frameid[] = "/joy_msg";


void setup(){
  pinMode(led, OUTPUT);
  
//  Serial.println("DBUS Status");
  Serial2.begin(100000,SERIAL_8E1);
  dBus.begin();
  Serial3.begin(115200);
  //ros
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub_joy);
  nh.advertise(pub_localization);
  nh.subscribe(sub_joy);
  joy_msg.header.frame_id =  frameid;
  joy_msg.buttons_length=channel;
  //joy_msg.axes_length=6;
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
  
  readLocalizationSystem();
  if(displayTime < currTime) {
      displayTime = currTime + 7;
      joy_pub_count++;
      write18BitsDbusData();
     
      if(joy_pub_count>10){
        joy_pub_count=0;
        publish_joy();
        publish_localization();
        nh.spinOnce();
      }
      //printDBUSStatus();
  }
    
  if(LED_flag)
    digitalWrite(led, HIGH);
  else 
    digitalWrite(led, LOW);
}

void startShoot(void){
  unsigned int curr = millis();
  if (step==2 || (curr - lastChange) <= chgTime) return;
  else if (step==0 && (curr - lastChange) > chgTime){
    ch_shoot = CHANNEL_MID; step = 1; lastChange = curr; return; }
  else if (step==1 && (curr - lastChange) > chgTime){
    ch_shoot = CHANNEL_DOWN; step = 2; lastChange = curr; return; }
}

void stopShoot(void){
  unsigned int curr = millis();
  if (step==23 || (curr - lastChange) <= chgTime) return;
  else if (step==0 && (curr - lastChange) > chgTime){ 
    step = 1; ch_shoot = CHANNEL_MID; lastChange = curr; return; }
  else if (step==1 && (curr - lastChange) > chgTime){
    step = 2; ch_shoot = CHANNEL_UP; lastChange = curr; return; }
  else if (step==2 &&(curr - lastChange) > chgTime){
    step = 3; ch_shoot = CHANNEL_MID; lastChange = curr; return; }
}
  
  if (shoot==0) return CHANNEL_MID;
  else if (shoot==1) return (uint16_t)CHANNEL_UP;
  else if (shoot==2) return (uint16_t)CHANNEL_DOWN;
  else if (shoot==4) return (uint16_t)CHANNEL_MID;
  else if (shoot==5) return (uint16_t)CHANNEL_MID;
  else if (shoot==3) return (uint16_t)CHANNEL_UP;
}

void joy_cb( const sensor_msgs::Joy& joy){
  //left button up means auto
  //sequence : left  right_UD  right_LR
    ROS_Output[LEFT_LR] = (uint16_t)(joy.buttons[2]);
    ROS_Output[LEFT_UD] = (uint16_t)(joy.buttons[3]);
    ROS_Output[RIGHT_UD] = (uint16_t)(joy.buttons[0]);
    ROS_Output[RIGHT_LR] = (uint16_t)(joy.buttons[1]);
    if (joy.buttons[4]) startShoot();
    else stopShoot();
}
void publish_joy(void){
  //upload_data_display();
  joy_msg.header.stamp = nh.now();
  joy_msg.buttons = ROS_Upload;
  //joy_msg.axes = ROS_Localization_Upload;
  pub_joy.publish(&joy_msg);
  
}
void publish_localization(void){
  localization_msg.linear.x = ROS_Localization_Upload[0];
  localization_msg.linear.y = ROS_Localization_Upload[1];
  localization_msg.linear.z = ROS_Localization_Upload[2];
  localization_msg.angular.x = ROS_Localization_Upload[3];
  localization_msg.angular.y = ROS_Localization_Upload[4];
  localization_msg.angular.z = ROS_Localization_Upload[5];
  pub_localization.publish(&localization_msg);
  
}
float pos_x=0;
float pos_y=0;
float zangle=0;
float xangle=0;
float yangle=0;
float w_z=0;
void readLocalizationSystem(void){
  static union
  {
   unsigned char data[24];
   float ActVal[6];
  }posture;
  static unsigned char count=0;
  static unsigned char i=0;

  while(Serial3.available()>0)   
  {
     unsigned char ch = Serial3.read();
     switch(count)
     {
       case 0:
         if(ch==0x0d)
           count++;
         else
           count=0;
         break;
         
       case 1:
         if(ch==0x0a)
         {
           i=0;
           count++;
         }
         else if(ch==0x0d);
         else
           count=0;
         break;
         
       case 2:
         posture.data[i]=ch;
           i++;
           if(i>=24)
         {
           i=0;
           count++;
         }
         break;
         
       case 3:
         if(ch==0x0a)
           count++;
         else
           count=0;
         break;
         
       case 4:
         if(ch==0x0d)
         {
                zangle=posture.ActVal[0];
                    xangle=posture.ActVal[1];
                    yangle=posture.ActVal[2];
                    pos_x =posture.ActVal[3];
                    pos_y =posture.ActVal[4];
                    w_z   =posture.ActVal[5];
                                  ROS_Localization_Upload[0] = pos_x/1000.0;
                                  ROS_Localization_Upload[1] = pos_y/1000.0;
                                  ROS_Localization_Upload[2] = w_z;
                                  ROS_Localization_Upload[3] = xangle;
                                  ROS_Localization_Upload[4] = yangle;
                                  ROS_Localization_Upload[5] = zangle;
            //upload_data_display();
                                  }
               count=0;
                                //data updated
                                 return;
         break;
       
       default:
         count=0;
         break;    
     } 
   }
  return;
}
void write18BitsDbusData(){
  //05 04 20 00 01 D8 00 00 00 00 00 00 00 00 00 00 00 00 original 
  //00 04 20 00 01 58 00 00 00 00 00 00 00 00 00 00 00 00 adjusted    
  
  DBus_Final_Output[CHANNEL_R] = DBus_Output[CHANNEL_R];
  if(DBus_Output[CHANNEL_L] == CHANNEL_UP){
    //auto mode
    DBus_Final_Output[LEFT_UD] = ROS_Output[LEFT_UD];
    DBus_Final_Output[LEFT_LR] = ROS_Output[LEFT_LR];
    DBus_Final_Output[RIGHT_UD] = ROS_Output[RIGHT_UD];
    DBus_Final_Output[RIGHT_LR] = ROS_Output[RIGHT_LR];
    DBus_Final_Output[CHANNEL_L] = ch_shoot;
    
  }else{
    stopShoot();
    DBus_Final_Output[CHANNEL_L] = ch_shoot;
    DBus_Final_Output[LEFT_UD] = DBus_Output[LEFT_UD];
    DBus_Final_Output[LEFT_LR] = DBus_Output[LEFT_LR];
    DBus_Final_Output[RIGHT_UD] = DBus_Output[RIGHT_UD];
    DBus_Final_Output[RIGHT_LR] = DBus_Output[RIGHT_LR];
  }
  
  //add safety system
  if(DBus_Output[CHANNEL_L] == 0||DBus_Output[CHANNEL_R]==0){  
    stopShoot();
    DBus_Final_Output[CHANNEL_L] = ch_shoot;
    DBus_Final_Output[CHANNEL_R] = CHANNEL_MID;
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
  
  
  for(i = 0;i<channel;i++){
    ROS_Upload[i] = DBus_Final_Output[i];
  }
  //upload_data_display();
  

  
  

    
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
void upload_data_display()
{
  for(i =0;i<6;i++){
    Serial.print(ROS_Localization_Upload[i]);
    Serial.print("\t");
  }
  Serial.println(".");
  
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
