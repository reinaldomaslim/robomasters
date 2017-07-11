/*
 This program enables the reading of DBUS 25 bits input 
 and output to DBUS 18 bit or 25 bit

  reading result = dBus.channels[0-7]

*/

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

//there are two versions of DJI controller protocol
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
const byte channel = 6;
//output format

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

static uint16_t ROS_Output[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
int32_t ROS_Upload[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
float ROS_Localization_Upload[6]  = {0, 0, 0, 0, 0, 0};
//the value of channel can only be 1-3 , the sequence is 01,11,10
//DBus_Output[LEFT_U] is the value of left up and down





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
ros::Subscriber<sensor_msgs::Joy> sub_joy("/cmd_vel", joy_cb);
ros::Publisher pub_joy( "/joy_msg", &joy_msg);
ros::Publisher pub_localization( "/localization", &localization_msg);
char frameid[] = "/joy_msg";


void read_data(void);
void publish_data(void);

void setup(){
  pinMode(led, OUTPUT);
  //Serial.begin(115200);
  Serial2.begin(115200);
  //Serial.println("DBUS Status");
  Serial3.begin(115200);

  
  //ros
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub_joy);
  nh.advertise(pub_localization);
  nh.subscribe(sub_joy);
  joy_msg.header.frame_id = frameid;
  joy_msg.buttons_length=channel;
  //joy_msg.axes_length=6;

  
}


void loop(){
  
  currTime = millis();

  
  read_data();
  readLocalizationSystem();
  if(displayTime < currTime) {
      displayTime = currTime + 20;
      publish_data();
      nh.spinOnce();
      //printDBUSStatus();
      //upload_data_display();
  }

  
}

void joy_cb( const sensor_msgs::Joy& joy){
  //left button up means auto
  //sequence : left  right_UD  right_LR
  static union
  {
    uint16_t joyData[5]  = {1024, 1024, 1024, 1024, 0};
    byte toByte[10];
  }ROS_Output;
  ROS_Output.joyData[0] = (uint16_t)(joy.buttons[0]);
  ROS_Output.joyData[1] = (uint16_t)(joy.buttons[1]);
  ROS_Output.joyData[2] = (uint16_t)(joy.buttons[2]);
  ROS_Output.joyData[3] = (uint16_t)(joy.buttons[3]);
  ROS_Output.joyData[4] = (uint16_t)(joy.buttons[4]);
  Serial2.write(0xFA);
  Serial2.write(0xBC);
  for(i=0;i<10;i++){
    Serial2.write(ROS_Output.toByte[i]);
  }
  Serial2.write(0xDE);
  
    
}
void publish_data(void){
  publish_joy();
  publish_localization();
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

void publish_joy(void){
  //upload_data_display();
  joy_msg.header.stamp = nh.now();
  joy_msg.buttons = ROS_Upload;
  //joy_msg.axes = ROS_Localization_Upload;
  pub_joy.publish(&joy_msg);
  
}

void read_data(void){
  static int read_data_count =0;
  static int count=0;
  static unsigned char ch;
  static union
  {
    uint16_t joyData[channel]  = {1024, 1024, 1024, 1024, CHANNEL_MID, CHANNEL_MID};
    byte toByte[12];
  }joy_read;
  
  while(Serial2.available()>0){
     ch = Serial2.read();
     //Serial.write(ch);
     switch(read_data_count)
     {
       case 0:
         if(ch==0xFA)
           read_data_count++;
         else
           read_data_count=0;
         break;
         
       case 1:
         if(ch==0xBC)
         {
           count=0;
           read_data_count++;
         }else
           read_data_count=0;
         break;
       case 2:
        if(count < 12){
          joy_read.toByte[count]=(byte)ch;
        }
        count++;
        if(count>=12){
           count=0;
           read_data_count++;
        }
       break;
         
       case 3:
         if(ch==0xDE){
           read_data_count=0;
           for(i=0;i<6;i++){
            ROS_Upload[i] = joy_read.joyData[i];
           }
           //printDBUSStatus();
         }else
           read_data_count=0;
         break;
       default:
         read_data_count=0;
         break;    
     } 
   }
}

void upload_data_display()
{
  Serial.print("x ");
  Serial.print(ROS_Localization_Upload[0]);
  Serial.print("\ty ");
  Serial.print(ROS_Localization_Upload[1]);
  Serial.print("\tw ");
  Serial.print(ROS_Localization_Upload[5]);
  Serial.println(".");
  
}

void printDBUSStatus()
{

  Serial.print("Thr ");
  Serial.print(ROS_Upload[2]);
  Serial.print(" Ail ");
  Serial.print(ROS_Upload[0]);
  Serial.print(" Ele ");
  Serial.print(ROS_Upload[1]);
  Serial.print(" Rud ");
  Serial.print(ROS_Upload[3]);
  Serial.print(" Channel1 ");
  Serial.print(ROS_Upload[5]);
  Serial.print(" Channel2  ");
  Serial.print(ROS_Upload[4]);
  Serial.print(" Stat ");
  Serial.println(".");
  
}
