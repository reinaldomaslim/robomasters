/*
  DJI DBUS data decoder library
  (c) S. Driussi 20141215
  Not for commercial use

  Work started from mikeshub Futaba library
  https://github.com/mikeshub/FUTABA_SBUS
  
  Refer to naza_dbus_decoder_wiring.jpg diagram for proper connection
*/
#include "DJI_DBUS.h"

void DJI_DBUS::begin(){
	uint8_t loc_sbusData[25] = {
	  0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
	int16_t loc_channels[18]  = {
	  		1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
	int16_t loc_servos[18]    = {
  			1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
  	_serial.begin(BAUDRATE);

	memcpy(sbusData,loc_sbusData,18);//25
	memcpy(channels,loc_channels,18);
	memcpy(servos,loc_servos,18);
	failsafe_status = DBUS_SIGNAL_OK;
	sbus_passthrough = 1;
	toChannels = 0;
	bufferIndex=0;
	feedState = 0;
}

int16_t DJI_DBUS::Channel(uint8_t ch) {
  // Read channel data
  if ((ch>0)&&(ch<=16)){
    return channels[ch-1];
  }
  else{
    return 1023;
  }
}
uint8_t DJI_DBUS::DigiChannel(uint8_t ch) {
  // Read digital channel data
  if ((ch>0) && (ch<=2)) {
    return channels[15+ch];
  }
  else{
    return 0;
  }
}
void DJI_DBUS::Servo(uint8_t ch, int16_t position) {
  // Set servo position
  if ((ch>0)&&(ch<=16)) {
    if (position>2048) {
      position=2048;
    }
    servos[ch-1] = position;
  }
}
void DJI_DBUS::DigiServo(uint8_t ch, uint8_t position) {
  // Set digital servo position
  if ((ch>0) && (ch<=2)) {
    if (position>1) {
      position=1;
    }
    servos[15+ch] = position;
  }
}
uint8_t DJI_DBUS::Failsafe(void) {
  return failsafe_status;
}

void DJI_DBUS::PassthroughSet(int mode) {
  // Set passtrough mode, if true, received channel data is send to servos
  sbus_passthrough = mode;
}

int DJI_DBUS::PassthroughRet(void) {
  // Return current passthrough mode
  return sbus_passthrough;
}
void DJI_DBUS::UpdateServos(void) {
  // Send data to servos
  // Passtrough mode = false >> send own servo data
  // Passtrough mode = true >> send received channel data
  uint8_t i;
  if (sbus_passthrough==0) {
    // clear received channel data
    for (i=1; i<24; i++) {
      sbusData[i] = 0;
    }

    // reset counters
    ch = 0;
    bit_in_servo = 0;
    byte_in_sbus = 1;
    bit_in_sbus = 0;

    // store servo data
    for (i=0; i<176; i++) {
      if (servos[ch] & (1<<bit_in_servo)) {
        sbusData[byte_in_sbus] |= (1<<bit_in_sbus);
      }
      bit_in_sbus++;
      bit_in_servo++;

      if (bit_in_sbus == 8) {
        bit_in_sbus =0;
        byte_in_sbus++;
      }
      if (bit_in_servo == 11) {
        bit_in_servo =0;
        ch++;
      }
    }

    // DigiChannel 1
    if (channels[16] == 1) {
      sbusData[23] |= (1<<0);
    }
    // DigiChannel 2
    if (channels[17] == 1) {
      sbusData[23] |= (1<<1);
    }

    // Failsafe
    if (failsafe_status == DBUS_SIGNAL_LOST) {
      sbusData[23] |= (1<<2);
    }

    if (failsafe_status == DBUS_SIGNAL_FAILSAFE) {
      sbusData[23] |= (1<<2);
      sbusData[23] |= (1<<3);
    }
  }
  // send data out
  //serialPort.write(sbusData,25);
  //for (i=0;i<25;i++) {
  //  _serial.write(sbusData[i]);
  //}
}
void DJI_DBUS::UpdateChannels(void) {
  uint8_t i;

  channels[0]  = ((sbusData[0]|sbusData[1]<< 8) & 0x07FF);
  channels[1]  = ((sbusData[1]>>3|sbusData[2]<<5) & 0x07FF);
  channels[2]  = ((sbusData[2]>>6|sbusData[3]<<2|sbusData[4]<<10) & 0x07FF);
  channels[3]  = ((sbusData[4]>>1|sbusData[5]<<7) & 0x07FF);
  channels[4]  = ((sbusData[5]>>4) & 0x03);
  channels[5]  = sbusData[5]>>6;



  // Failsafe
  failsafe_status = DBUS_SIGNAL_OK;
  if (sbusData[16] & (1<<2)) {//23
    failsafe_status = DBUS_SIGNAL_FAILSAFE;
  }
  if (sbusData[16] & (1<<3)) {//23
    failsafe_status = DBUS_SIGNAL_LOST;
  }

}
void DJI_DBUS::FeedLine(void){
  if (_serial.available() > 17){
    bufferIndex = 0;
    while(_serial.available() > 0){
      inData = _serial.read();
      inBuffer[bufferIndex++] = inData;
      if (bufferIndex < 18 ){
        continue;
      }
      else if (bufferIndex == 18){
          if (inBuffer[6]==0x00 && inBuffer[17] == 0x00){
            memcpy(sbusData,inBuffer,18);
            toChannels = 1;
            break;
          }
      }else{
        bufferIndex=0;
      }
        
    }
  }
}

