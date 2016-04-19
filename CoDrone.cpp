/*
  SmartDroneControl.cpp - SmartDroneControl library
  Copyright (C) 2014 RoboLink.  All rights reserved.
  LastUpdate : 2016-02-17
*/

#include "CoDrone.h"
#include "Arduino.h"
#include <EEPROM.h>
/***************************************************************************/

static const unsigned short crc16tab[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

unsigned short CoDroneClass::CRC16_Make(unsigned char *buf, int len) //CRC16-CCITT Format
{
  unsigned short crc = 0 ;
  for (int counter = 0; counter < len; counter++)
  {
    crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ * (char *)buf++) & 0x00FF];
  }
  return crc;
}

boolean CoDroneClass::CRC16_Check(unsigned char data[], int len, unsigned char crc[])
{
  boolean crcCheck = false;
  
  unsigned short receiveCRC = ((crc[1] << 8) | (crc[0]  & 0xff));
  unsigned short  makeCRC = CRC16_Make(data, len + 2);
  
  if (receiveCRC == makeCRC )	   	   	crcCheck = true;
  else					  								  	crcCheck = false;
  return crcCheck;
}

/***************************************************************************/

void CoDroneClass::begin(void)
{
	SendInterval = 50; // millis seconds				
	analogOffset = 10;	// analog sensor offset

	roll = 0;
	pitch = 0;
	yaw = 0;
	throttle = 0;

  StartLED();
		
	if (EEPROM.read(eep_AddressCheck) == 1)
	{  	
		for (int i = 0; i <= 5; i++)
		{
			devAddressConnected[i] = EEPROM.read(eep_AddressFirst+i);
		}		
	}
	
	pinMode(8, INPUT_PULLUP);
	pinMode(9, INPUT_PULLUP);
	pinMode(10, INPUT_PULLUP);
	
	pinMode(11, INPUT);
	pinMode(12, OUTPUT);
	pinMode(13, OUTPUT);
	pinMode(14, INPUT);
	pinMode(15, INPUT);
	pinMode(16, OUTPUT);
	pinMode(17, OUTPUT);
	pinMode(18, INPUT);
	
	digitalWrite(11, LOW);
	digitalWrite(12, LOW);
	digitalWrite(13, LOW);
	digitalWrite(14, LOW);
	digitalWrite(15, LOW);
	digitalWrite(16, LOW);
	digitalWrite(17, LOW);
	digitalWrite(18, LOW);
	
  delay(500);
  Send_LinkModeBroadcast(LinkModeActive);
}

/***************************************************************************/
//////////////////////Command////////////////////////////////////////////
/***************************************************************************/
////////////////////link board////////////////////////

void CoDroneClass::Send_LinkModeBroadcast(byte mode)
{
  Send_Command(cType_LinkModeBroadcast, mode);
}


void CoDroneClass::Send_LinkState()
{
  Send_Command(cType_Request, dType_LinkState);
}
void CoDroneClass::LinkReset()
{
  Send_Command(cType_LinkSystemReset, 0);
}
void CoDroneClass::Send_Discover(byte action)
{	
	if(action == DiscoverStop)	  	Send_Command(cType_LinkDiscoverStop, 0);		//DiscoverStop
	else if(action == DiscoverStart)	
	{
		Send_Command(cType_LinkDiscoverStart, 0);  	//DiscoverStart  
		discoverFlag = 1;
	}
}
void CoDroneClass::Send_Connect(byte index) //0, 1, 2
{
	connectFlag = 1;
		
	if(index == 0)
	{				
		for (int i = 0; i <= 5; i++)		devAddressBuf[i] = devAddress0[i];
	}	
	else if (index == 1)
	{
		for (int i = 0; i <= 5; i++)		devAddressBuf[i] = devAddress1[i];
	}
	else if (index == 2)
	{
		for (int i = 0; i <= 5; i++)		devAddressBuf[i] = devAddress2[i];
	}	
	
  Send_Command(cType_LinkConnect, index);
}
void CoDroneClass::Send_Disconnect()
{
  Send_Command(cType_LinkDisconnect, 0);
}
void CoDroneClass::Send_RSSI_Polling(byte action)
{
	if(action == 0)	  		Send_Command(cType_LinkRssiPollingStop, 0);				//RssiPollingStop
	else if(action == 1)	Send_Command(cType_LinkRssiPollingStart, 0x02);  	//RssiPollingStart	(0x02 * 100 = 200ms)
}

//////////////////ModeDrone/////////////////////////////



void CoDroneClass::DroneModeChange(byte event)
{
	  Send_Command(cType_ModeDrone, event);
	  delay(100);
}


void CoDroneClass::Send_DroneMode(byte event)
{
	  Send_Command(cType_ModeDrone, event);
}
////////////////////FlightEvent////////////////////////

void CoDroneClass::Send_Coordinate(byte mode)
{
	if(mode == cSet_Absolute)	  				Send_Command(cType_Coordinate, cSet_Absolute);
	else if(mode == cSet_Relative) 			Send_Command(cType_Coordinate, cSet_Relative);
}
void CoDroneClass::Send_Trim(byte event)
{
	  Send_Command(cType_Trim, event);
}

void CoDroneClass::Send_ClearGyroBiasAndTrim()
{
	  Send_Command(cType_ClearGyroBiasAndTrim, 0);
}


void CoDroneClass::FlightEvent(byte event)
{
	  Send_Command(cType_FlightEvent, event);
}

void CoDroneClass::DriveEvent(byte event)
{
	  Send_Command(cType_DriveEvent, event);
}

void CoDroneClass::Send_ResetHeading()
{
	  Send_Command(cType_ResetHeading, 0);
}


/***************************************************************************/
///////////////////////////////////////////////////////////////////////////

void CoDroneClass::Control()
{
  byte _packet[10];
  byte _crc[2];
  
  byte _cType = dType_Control;
  byte _len = 4;
  
  //header
  _packet[0] = _cType;
  _packet[1] = _len;

  //data
  _packet[2] = roll;
  _packet[3] = pitch;
  _packet[4] = yaw;
  _packet[5] = throttle;

  unsigned short crcCal = CRC16_Make(_packet, _len+2);
  _crc[0] = (crcCal >> 8) & 0xff;
  _crc[1] = crcCal & 0xff;
  
  Send_Processing(_packet,_len,_crc);  
  
  roll = 0;
	pitch = 0;
	yaw = 0;
	throttle = 0;  
  
}


////////////////////////////////////////////////////////

void CoDroneClass::Send_Command(int sendCommand, int sendOption)
{	
  byte _packet[9];
  byte _crc[2];
  
  byte _cType = dType_Command;
  byte _len   = 0x02;  
  
  //header
  _packet[0] = _cType;
  _packet[1] = _len;

 //data
  _packet[2] = sendCommand;
  _packet[3] = sendOption;
  
 unsigned short crcCal = CRC16_Make(_packet, _len+2);
  _crc[0] = (crcCal >> 8) & 0xff;
  _crc[1] = crcCal & 0xff;
  
  Send_Processing(_packet,_len,_crc);       
}
/////////////////////////////////////////////////////

void CoDroneClass::LedColor(byte sendMode, byte sendColor, byte sendInterval)
{	

  byte _packet[9];
  byte _crc[2];
  
  byte _cType = dType_LedMode;
  byte _len   = 0x03;  
  
  //header
  _packet[0] = _cType;
  _packet[1] = _len;

 //data
  _packet[2] = sendMode;
  _packet[3] = sendColor;
  _packet[4] = sendInterval;
  
 unsigned short crcCal = CRC16_Make(_packet, _len+2);
  _crc[0] = (crcCal >> 8) & 0xff;
  _crc[1] = crcCal & 0xff;
  
  Send_Processing(_packet,_len,_crc);     
}



void CoDroneClass::LedColor(byte sendMode, byte r, byte g, byte b, byte sendInterval)
{	
  byte _packet[9];
  byte _crc[2];
  
  byte _cType = dType_LedModeColor;
  byte _len   = 0x05;  
  
  //header
  _packet[0] = _cType;
  _packet[1] = _len;

 //data
  _packet[2] = sendMode;
  _packet[3] = r;
  _packet[4] = g;  
  _packet[5] = b;  
  _packet[6] = sendInterval;
  
 unsigned short crcCal = CRC16_Make(_packet, _len+2);
  _crc[0] = (crcCal >> 8) & 0xff;
  _crc[1] = crcCal & 0xff;
  
  Send_Processing(_packet,_len,_crc);     
}

void CoDroneClass::LedColor(byte sendMode, byte sendColor[], byte sendInterval)
{	
  byte _packet[9];
  byte _crc[2];
  
  byte _cType = dType_LedModeColor;
  byte _len   = 0x05;  
  
  //header
  _packet[0] = _cType;
  _packet[1] = _len;

 //data
  _packet[2] = sendMode;
  _packet[3] = sendColor[0];
  _packet[4] = sendColor[1];  
  _packet[5] = sendColor[2];  
  _packet[6] = sendInterval;
  
 unsigned short crcCal = CRC16_Make(_packet, _len+2);
  _crc[0] = (crcCal >> 8) & 0xff;
  _crc[1] = crcCal & 0xff;
  
  Send_Processing(_packet,_len,_crc);     
}




void CoDroneClass::LedEvent(byte sendMode, byte sendColor, byte sendInterval, byte sendRepeat)
{	
  byte _packet[9];
  byte _crc[2];
  
  byte _cType = dType_LedEvent;
  byte _len   = 0x04;  
  
  //header
  _packet[0] = _cType;
  _packet[1] = _len;

 //data
  _packet[2] = sendMode;
  _packet[3] = sendColor;
  _packet[4] = sendInterval;
  _packet[5] = sendRepeat;
  
 unsigned short crcCal = CRC16_Make(_packet, _len+2);
  _crc[0] = (crcCal >> 8) & 0xff;
  _crc[1] = crcCal & 0xff;
  
  Send_Processing(_packet,_len,_crc);     
}



void CoDroneClass::LedEvent(byte sendMode, byte sendColor[], byte sendInterval, byte sendRepeat)
{	
  byte _packet[9];
  byte _crc[2];
  
  byte _cType = dType_LedEventColor;
  byte _len   = 0x06;  
  
  //header
  _packet[0] = _cType;
  _packet[1] = _len;

 //data
  _packet[2] = sendMode;
  _packet[3] = sendColor[0];
  _packet[4] = sendColor[1];  
  _packet[5] = sendColor[2];  
  _packet[6] = sendInterval;
  _packet[7] = sendRepeat;

  
 unsigned short crcCal = CRC16_Make(_packet, _len+2);
  _crc[0] = (crcCal >> 8) & 0xff;
  _crc[1] = crcCal & 0xff;
  
  Send_Processing(_packet,_len,_crc);     
}


void CoDroneClass::LedEvent(byte sendMode, byte r, byte g, byte b, byte sendInterval, byte sendRepeat)
{	
  byte _packet[9];
  byte _crc[2];
  
  byte _cType = dType_LedEventColor;
  byte _len   = 0x06;  
  
  //header
  _packet[0] = _cType;
  _packet[1] = _len;

 //data
  _packet[2] = sendMode;
  _packet[3] = r;
  _packet[4] = g;  
  _packet[5] = b; 
  _packet[6] = sendInterval;
  _packet[7] = sendRepeat;

  
 unsigned short crcCal = CRC16_Make(_packet, _len+2);
  _crc[0] = (crcCal >> 8) & 0xff;
  _crc[1] = crcCal & 0xff;
  
  Send_Processing(_packet,_len,_crc);     
}


/////////////////////////////////////////////////////////////


void CoDroneClass::Send_DroneState()
{
	Send_Command(cType_Request, Req_State);    
}

void CoDroneClass::Send_DroneAttitude()
{
	Send_Command(cType_Request, Req_Attitude);    
}




void CoDroneClass::Send_Ping()
{
   
  byte _packet[10];
  byte _crc[2];
  
  byte _cType = dType_Ping;
  byte _len = 0;
  
  //header
  _packet[0] = _cType;
  _packet[1] = _len;

  unsigned short crcCal = CRC16_Make(_packet, _len+2);
  _crc[0] = (crcCal >> 8) & 0xff;
  _crc[1] = crcCal & 0xff;
  
  Send_Processing(_packet,_len,_crc);  
    
}
/////////////////////////////////////////////////////////////




void CoDroneClass::AutoConnect(byte mode)
{	
	// Connected check
	LinkStateCheck();		
  if (receiveLinkState  == linkMode_Connected)
  {
    pairing = true;
    ConnectLED();    
  }
  // AutoConnect start
	else     
	{				
	  if (mode == NeardbyDrone)	
	  {
	  	Send_Discover(DiscoverStart);  
	  	PreviousMillis = millis();

			DDRC = 0xff;
			
	  	while(!pairing)
	  	{  		
	  		// Receive();  		 
	  		 //if(!pairing)
	  		// {
		  		 if((discoverFlag == 3) && (connectFlag == 0)) //Address find
		  		 {	  		 	
		  		 	DDRC = 0b01100110;
		  		 	PORTC = 0x00;
		  		 	  		 		  		 	
		  		 	delay(50);
		  		 	discoverFlag = 0;
		  		 	Send_ConnectNearbyDrone();  	  		 				//  Connect Start
		  		 }
		  		 
		  		 else if (discoverFlag == 4)	// Address not find : re-try
		  		 {
		  		 	delay(50);
		  		 	Send_Discover(DiscoverStart);
		  		 	PreviousMillis = millis();
		  		 }
		  		 else
		  		 {	  	
			  		if (TimeCheck(400))		//time out & LED
		    		{
		      		if (displayLED++ == 4) 
		      		{
		      			displayLED = 0;	 
		      			delay(50);     
		      			Send_Discover(DiscoverStart);
		      		}
		      		PORTC = (0b1<<displayLED) | (0b10000000>>displayLED);	   
		      		PreviousMillis = millis();   		     
						}
		  		}	  		 
		  //	}
		  Receive();  
	  }
	  	delay(50);  	 	  	
	  }
	  
	  else if(mode == ConnectedDrone)   
	  {
	  	Send_Discover(DiscoverStart);  
	  	PreviousMillis = millis();

			DDRC = 0xff;	  	
			
	  	while(!pairing)
	  	{  		
	  		
  		 if ((discoverFlag == 3) && (connectFlag == 0))	//Address find
  		 {  		 	
  		 	DDRC = 0b01100110;
  		 	PORTC = 0x00;
  		 	  		 		  	
  		 	delay(50);
  		 	discoverFlag = 0;
  		 	Send_ConnectConnectedDrone();  	 	//  Connect Start 		 	
  		 }
  		 else if (discoverFlag == 4)	// Address not find : re-try
  		 {
  		 	Send_Discover(DiscoverStart);
  		  PreviousMillis = millis();
  		 }
  		 else
  		 {	  	
	  		if (TimeCheck(400))  //time out & LED
    		{
      		if (displayLED++ == 4) 
      		{
      			displayLED = 0;	 
      			delay(50);     
      			Send_Discover(DiscoverStart);
      		}
      		PORTC = (0b1<<displayLED) | (0b10000000>>displayLED);	   
      		PreviousMillis = millis();   		     
				}
  		}
	  		 Receive();  		
	  	}
	  	delay(50);
	  } 
	}
}

void CoDroneClass::AutoConnect(byte mode, byte address[])
{		
	// Connected check
	LinkStateCheck();		
  if (receiveLinkState  == linkMode_Connected)
  {
    pairing = true;
    ConnectLED();
  }
    
  // AutoConnect start
	else     
	{		 	
	  if (mode == AddressInputDrone)		
	  {
	  	Send_Discover(DiscoverStart);  
	  	PreviousMillis = millis();
	  	
			DDRC = 0xff;	 	

	  	while(!pairing)
	  	{  		
	  		 
	  		   		 
	  		 if((discoverFlag == 3) && (connectFlag == 0))	//Address find
	  		 {  		 	
	 				DDRC = 0b01100110;
	  		 	PORTC = 0x00;
	  		 	delay(50);
	  		 	discoverFlag = 0;
	  		 	Send_ConnectAddressInputDrone(address);  	  		//  Connect Start 			 	
	  		 }  		
	  		 else if (discoverFlag == 4)	// Address not find : re-try
	  		 {
	  		 	Send_Discover(DiscoverStart);
	  		 	PreviousMillis = millis();
	  		 }
	  		 else
	  		 {	  	
		  		if (TimeCheck(400))	//time out & LED
	    		{
	      		if (displayLED++ == 4) 
	      		{
	      			displayLED = 0;	 
	      			delay(50);     
	      			Send_Discover(DiscoverStart);
	      		}
	      		PORTC = (0b1<<displayLED) | (0b10000000>>displayLED);	   
	      		PreviousMillis = millis();   		     
					}
	  		}
	  		 Receive();  			
	  	}
	  	delay(50);  	
	  }
	}
}


void CoDroneClass::Send_ConnectAddressInputDrone(byte address[])
{
		if (devCount > 0)
  {
		//ConnectedDrone same address check
		byte AddrCheck0 = 0;
		byte AddrCheck1 = 0;
		byte AddrCheck2 = 0;
		
		for (int i = 0; i <= 5; i++)
		{
			if (address[i] == devAddress0[i])  AddrCheck0++;
			if (address[i] == devAddress1[i])  AddrCheck1++;
			if (address[i] == devAddress2[i])  AddrCheck2++;
		}		
	  if(AddrCheck0 == 6)	Send_Connect(0);   
	  else if(AddrCheck1 == 6)	Send_Connect(1);   
	  else if(AddrCheck2 == 6)	Send_Connect(2);    
	}	
}

void CoDroneClass::Send_ConnectConnectedDrone()
{	
	if (devCount > 0)
  {
		//ConnectedDrone same address check
		byte AddrCheck0 = 0;
		byte AddrCheck1 = 0;
		byte AddrCheck2 = 0;
		
		for (int i = 0; i <= 5; i++)
		{
			if (devAddressConnected[i] == devAddress0[i])  AddrCheck0++;
			if (devAddressConnected[i] == devAddress1[i])  AddrCheck1++;
			if (devAddressConnected[i] == devAddress2[i])  AddrCheck2++;
		}		
	  if(AddrCheck0 == 6)	Send_Connect(0);   
	  else if(AddrCheck1 == 6)	Send_Connect(1);   
	  else if(AddrCheck2 == 6)	Send_Connect(2);    
	}	
}

void CoDroneClass::Send_ConnectNearbyDrone()
{
  if (devCount > 0)
  {
    if (devRSSi0 > devRSSi1)
    {
      if (devRSSi0 > devRSSi2)	Send_Connect(0);     
      else			Send_Connect(2);
    }
    else
    {
      if (devRSSi1 > devRSSi2)	 Send_Connect(1);
      else	 		Send_Connect(2);
    }
  }
}







void CoDroneClass::Control(int interval)
{
    if (TimeCheck(interval))  //delay
    {
      Control();
      PreviousMillis = millis();
    }
}





void CoDroneClass::Send_Processing(byte _data[], byte _length, byte _crc[])
{		
  byte _packet[20];
  
  //START CODE  
  _packet[0] = START1;
  _packet[1] = START2;

  //HEADER & DATA
  for(int i = 0; i < _length + 3 ; i++)
  {
   _packet[i+2] = _data[i];	   
  }
  //CRC
  
  _packet[_length + 4] =_crc[1];
  _packet[_length + 5] =_crc[0]; 
    
 //	Serial1.write(_packet, _length + 6);
 	Serial.write(_packet, _length + 6);
}









/***************************************************************************/
void CoDroneClass::Receive()
{	
	if (Serial.available() > 0)
  {
    int input = Serial.read();
    
	//	Serial.write(input);	

    cmdBuff[cmdIndex++] = (char)input;

    if (cmdIndex >= MAX_PACKET_LENGTH)
    {
      checkHeader = 0;
      cmdIndex = 0;
    }
    else
    {
      if (cmdIndex == 1)
      {
        if (cmdBuff[0] == START1)	checkHeader = 1;
        else
        {
          checkHeader = 0;
          cmdIndex = 0;
        }
      }
      else if (cmdIndex == 2)
      {
        if (checkHeader == 1)
        {
          if (cmdBuff[1] == START2)	checkHeader = 2;
          else
          {
            checkHeader = 0;
            cmdIndex = 0;
          }
        }
      }      
      else if (checkHeader == 2)
      {
        if (cmdIndex == 3)
        {
          receiveDtype =  cmdBuff[2];
          dataBuff[cmdIndex - 3] = cmdBuff[cmdIndex - 1];
        }
        else if (receiveDtype != dType_StringMessage) //not message string
        {
          if (cmdIndex == 4)
          {
            receiveLength = cmdBuff[3];
            dataBuff[cmdIndex - 3] = cmdBuff[cmdIndex - 1];
          }
          else if (cmdIndex > 4)
          {
            if (receiveLength + 5 > cmdIndex)	 	dataBuff[cmdIndex - 3] = cmdBuff[cmdIndex - 1];     
                   
            else if (receiveLength + 6 > cmdIndex)	crcBuff[0]  = cmdBuff[cmdIndex - 1];
            
            else if (receiveLength + 6 <= cmdIndex)
            {
              crcBuff[1]  = cmdBuff[cmdIndex - 1];

              if (CRC16_Check(dataBuff, receiveLength, crcBuff))  receiveComplete = 1;
              else  receiveComplete = -1;

						//	Serial.println(receiveComplete);
              ///////////////////////////////////////////////////////////
              if (receiveComplete == 1)
              {                       	
              	if (receiveDtype == dType_LinkState)		
                {
                	receiveLinkState = dataBuff[2];
                	receiveLikMode = dataBuff[3];
                }
                	
                else if (receiveDtype == dType_LinkEvent)		
                {
                	receiveEventState = dataBuff[2];
                }
                          
                /***********************************************/     
               
               
                else if (receiveDtype == dType_State)		//dron state
                {
	                droneState[0] = dataBuff[2];
	                droneState[1] = dataBuff[3];
	                droneState[2] = dataBuff[4];
	                droneState[3] = dataBuff[5];
	                droneState[4] = dataBuff[6];
	                droneState[5] = dataBuff[7];	
	                droneState[6] = dataBuff[8];	             
              	}
                else if (receiveDtype == dType_Attitude)		//dron Attitude
                { 
                	droneAttitude[0] = dataBuff[2];
	                droneAttitude[1] = dataBuff[3];
	                droneAttitude[2] = dataBuff[4];
	                droneAttitude[3] = dataBuff[5];
	                droneAttitude[4] = dataBuff[6];
	                droneAttitude[5] = dataBuff[7];	  
                }         
                /***********************************************/                  
                                
                else if (receiveDtype == dType_LinkDiscoveredDevice)
                {
                  byte devIndex = dataBuff[2];

                  if (devIndex == 0)
                  {
                    for (int i = 3; i <= 8; i++)
                    {
                      devAddress0[i - 3] = dataBuff[i];
                    }
                    devRSSi0 = dataBuff[9];
                    devFind[0] = 1;                    
                    PORTC |= 0b00000010;
                  }
                  else if (devIndex == 1)
                  {
                    for (int i = 3; i <= 8; i++)
                    {
                      devAddress1[i - 3] = dataBuff[i];
                    }
                    devRSSi1 = dataBuff[9];
                    devFind[1] = 1;
                    PORTC |= 0b00000110;
                  }
                  else if (devIndex == 2)
                  {
                    for (int i = 3; i <= 8; i++)
                    {
                      devAddress2[i - 3] = dataBuff[i];
                    }
                    devRSSi2 = dataBuff[9];
                    devFind[2] = 1;
                    PORTC |= 0b00100110;
                  }

                  devCount = devFind[0] +  devFind[1] +  devFind[2];
                //Serial.println(devCount);
                //  DisplayAddress(devCount); //Address display
                }
              }
              ///////////////////////////////////////////////////////////
              //    Serial.println();
              checkHeader = 0;
              cmdIndex = 0;
            }
          }
        }
        else
        {
          checkHeader = 0;
          cmdIndex = 0;
        }
      }
    }
  }
  ReceiveEventCheck();
}
/***************************************************************************/

void CoDroneClass::PrintDroneAddress()
{
	for(char i = 0; i <= 4; i++)
	{
		Serial.print("0x");
		Serial.print(devAddressConnected[i],HEX);
		Serial.print(", ");
	}
	Serial.print("0x");
	Serial.print(devAddressConnected[5],HEX);
}

/**********************************************************/
void CoDroneClass::DisplayAddress(byte count)
{

  if (count == 1)    Serial.print("index 0 : ");
  else if (count == 2)  Serial.print("index 1 : ");
  else if (count == 3)     Serial.print("index 2 : ");

  for (int i = 0; i <= 5; i++)
  {
    if (count == 1)
    {
      Serial.print(devAddress0[i], HEX); Serial.print("\t");
    }
    else if (count == 2)
    {
      Serial.print(devAddress1[i], HEX); Serial.print("\t");
    }

    else if (count == 3)
    {
      Serial.print(devAddress2[i], HEX); Serial.print("\t");
    }
  }
  /*
  if (count == 0)     Serial.println(DevRSSi0 - 256);
  else if (count == 1)   Serial.println(DevRSSi1 - 256);
  else if (count == 2)   Serial.println(DevRSSi2 - 256);
  */
}
/**********************************************************/


void CoDroneClass::LED(int command)
{
  if (command == ON)
  {
   // digitalWrite(12, HIGH);	
 //   digitalWrite(13, HIGH);    
 //   digitalWrite(16, HIGH);
   // digitalWrite(17, HIGH);
  }
  else if (command == OFF)
  {
    //  digitalWrite(12, LOW);
   //   digitalWrite(13, LOW);      
   //   digitalWrite(16, LOW);
   //   digitalWrite(17, LOW);
  }
}

void CoDroneClass::Blink(int time, int count)
{
    for (int i = 0; i < count; i++)
    {
   //   digitalWrite(12, HIGH);
   //   digitalWrite(13, HIGH);
   //   digitalWrite(16, HIGH);
  //    digitalWrite(17, HIGH);    
        
      delay(time);      

   //   digitalWrite(12, LOW);
   //   digitalWrite(13, LOW);      
   //   digitalWrite(16, LOW);
   //   digitalWrite(17, LOW);
      
      delay(time);
    }
}



void CoDroneClass::StartLED()
{		
  int led_sign = 0;
  DDRC =  0b11111111;
  while (led_sign < 8)
  {
    PORTC = ((1 << (led_sign++)) - 1);
    delay(50);
  }
  while (led_sign > -2)
  {
    PORTC = ((1 << (led_sign--)) - 1);
    delay(50);
  }
}


void CoDroneClass::ConnectLED()
{	
	DDRC = 0b00100100;
	PORTC = 0b00100100;	
}


/**********************************************************/

int CoDroneClass::LowBatteryCheck(byte value)
{
	
	int bat = -1;
	timeOutRetry = 0;
	CoDrone.Send_DroneState();
	
	PreviousMillis = millis();
		  	
	while(1)
	{		
		CoDrone.Receive();
				
	  if (CoDrone.droneState[0] != 0 )
	  {	  		  
	 		battery = droneState[5];
	 		bat = droneState[5];
		  if(bat < value)
		  {
					BeepWarning(5);
			}
			
			droneState[0] = 0;
		  droneState[1] = 0;
		  droneState[2] = 0;
		  droneState[3] = 0;
		  droneState[4] = 0;
		  droneState[5] = 0;
		  droneState[6] = 0;
		  
		  break;
	  }
	  
		else if (TimeCheck(1000))	//time out
		{			
			timeOutRetry ++;
			if(timeOutRetry <3)
			{
				bat = -1;
				droneState[0] = 0;
			  droneState[1] = 0;
			  droneState[2] = 0;
			  droneState[3] = 0;
			  droneState[4] = 0;
			  droneState[5] = 0;
			  droneState[6] = 0;
			  break;	 
			}
			else
			{
				CoDrone.Send_DroneState();
				delay(50);	
				PreviousMillis = millis();				
			}   
		}		  
	}
	delay(50);
	return bat;
}

/**********************************************************/

void CoDroneClass::LinkStateCheck()
{
	receiveLinkState = 0;
	
	Send_LinkState();
		
  delay(50);
 
  while (receiveLinkState <= 0) 	 Receive();
  
}

/**********************************************************/


void CoDroneClass::ReceiveEventCheck()
{
  if ((receiveComplete > 0) && (receiveEventState > 0))
  {  	  	
 // 	Serial.println(receiveEventState);

		if (receiveEventState == linkEvent_None)
    {
    	/*

    	 Serial.print(linkEvent_None);    	 
       Serial.println(" : linkEvent_None");
      */
    }    
		else if (receiveEventState == linkEvent_SystemReset)
    {
    	/*
    	 Serial.print(linkEvent_SystemReset);
       Serial.println(" : linkEvent_SystemReset");
      */
    }
    
		else if (receiveEventState == linkEvent_Initialized)
    {
    	/*
    	 Serial.print(linkEvent_Initialized);
       Serial.println(" : linkEvent_Initialized");
      */
    }
    
		else if (receiveEventState == linkEvent_Scanning)
    {
    	if(discoverFlag == 1) discoverFlag = 2;
    	/*
       Serial.print(linkEvent_Scanning);
       Serial.println(" : linkEvent_Scanning");
      */
    }
		else if (receiveEventState == linkEvent_ScanStop)
    {
    	if(discoverFlag == 2)
    	{
    		if(devCount > 0)
    		{
    		 discoverFlag = 3;
    		}
    		else
    		{
    			discoverFlag = 4;
    		}    		 
    	}
    	
    	/*
    	 Serial.print(linkEvent_ScanStop);
       Serial.println(" : linkEvent_ScanStop");
      */
    }
    
    
		else if (receiveEventState == linkEvent_FoundDroneService)
    {
    	/*
    	 Serial.print(linkEvent_FoundDroneService);
       Serial.println(" : linkEvent_FoundDroneService");
      */
    }
        
    
    
		else if (receiveEventState == linkEvent_Connecting)
    {
    	/*
    	 Serial.print(linkEvent_Connecting);
       Serial.println(" : linkEvent_Connecting");
      */
    }
		else if (receiveEventState == linkEvent_Connected)
    {
    	/*
    	 Serial.print(linkEvent_Connected);
       Serial.println(" : linkEvent_Connected");
      */
    }
    
		else if (receiveEventState == linkEvent_ConnectionFaild)
    {
    	/*
    	 Serial.print(linkEvent_ConnectionFaild);
       Serial.println(" : linkEvent_ConnectionFaild");
      */
    }
		else if (receiveEventState == linkEvent_ConnectionFaildNoDevices)
    {
    	/*
    	 Serial.print(linkEvent_ConnectionFaildNoDevices);
       Serial.println(" : linkEvent_ConnectionFaildNoDevices");
      */  
    }
		else if (receiveEventState == linkEvent_ConnectionFaildNotReady)
    {
    	/*
    	 Serial.print(linkEvent_ConnectionFaildNotReady);
       Serial.println(" : linkEvent_ConnectionFaildNotReady");
      */
    }
    
    
    else if (receiveEventState == linkEvent_PairingStart)
    {
    	/*
    	 Serial.print(linkEvent_PairingStart);
       Serial.println(" : linkEvent_PairingStart");
      */
    }
    
    else if (receiveEventState == linkEvent_PairingSuccess)
    {
    	/*
    	 Serial.print(linkEvent_PairingSuccess);
       Serial.println(" : linkEvent_PairingSuccess");
      */
    }
    else if (receiveEventState == linkEvent_PairingFaild)
    {
    	/*
    	 Serial.print(linkEvent_PairingFaild);
       Serial.println(" : linkEvent_PairingFaild");
      */
    }
    
    
    else if (receiveEventState == linkEvent_BondingSuccess)
    {
    	/*
    	 Serial.print(linkEvent_BondingSuccess);
       Serial.println(" : linkEvent_BondingSuccess");
      */
    }
    
    
    else if (receiveEventState == linkEvent_LookupAttribute)
    {
    	/*
    	 Serial.print(linkEvent_LookupAttribute);
       Serial.println(" : linkEvent_LookupAttribute");
      */
    }
    
    
    else if (receiveEventState == linkEvent_RssiPollingStart)
    {
    	/*
    	 Serial.print(linkEvent_RssiPollingStart);
       Serial.println(" : linkEvent_RssiPollingStart");
      */
    }    
    else if (receiveEventState == linkEvent_RssiPollingStop)
    {
    	/*
    	 Serial.print(linkEvent_RssiPollingStop);
       Serial.println(" : linkEvent_RssiPollingStop");
      */
    }
    
    
    else if (receiveEventState == linkEvent_DiscoverService)
    {
    	/*
    	 Serial.print(linkEvent_DiscoverService);
       Serial.println(" : linkEvent_DiscoverService");
      */
    }
    else if (receiveEventState == linkEvent_DiscoverCharacteristic)
    {
    	/*
    	 Serial.print(linkEvent_DiscoverCharacteristic);
       Serial.println(" : linkEvent_DiscoverCharacteristic");
      */
    }
    else if (receiveEventState == linkEvent_DiscoverCharacteristicDroneData)
    {
    	/*
    	 Serial.print(linkEvent_DiscoverCharacteristicDroneData);
       Serial.println(" : linkEvent_DiscoverCharacteristicDroneData");
      */      
    }    
    else if (receiveEventState == linkEvent_DiscoverCharacteristicDroneConfig)
    {
    	/*
    	 Serial.print(linkEvent_DiscoverCharacteristicDroneConfig);
       Serial.println(" : linkEvent_DiscoverCharacteristicDroneConfig");
      */
    }
    else if (receiveEventState == linkEvent_DiscoverCharacteristicUnknown)
    {
    	/*
    	 Serial.print(linkEvent_DiscoverCharacteristicUnknown);
       Serial.println(" : linkEvent_DiscoverCharacteristicUnknown");
      */
    }
    else if (receiveEventState == linkEvent_DiscoverCCCD)
    {
    	/*
    	 Serial.print(linkEvent_DiscoverCCCD);
       Serial.println(" : linkEvent_DiscoverCCCD");
      */
    }
          
    else if (receiveEventState == linkEvent_ReadyToControl)
    {
      if(connectFlag == 1)
      {
				connectFlag = 0;         
										
				EEPROM.write(eep_AddressCheck, 0x01);						
				for (int i = 0; i <= 5; i++)
				{
				//	devAddressConnected[i] = devAddressBuf[i];
			    EEPROM.write(eep_AddressFirst + i, devAddressBuf[i]); //servo1 standard position					
			  }
			  
			  /*
	    	 Serial.print(linkEvent_ReadyToControl);
	       Serial.println(" : linkEvent_ReadyToControl");
	      */ 
		  }
		  ConnectLED();
		  pairing = true;    
		  delay(500);
    }
    
    
    else if (receiveEventState == linkEvent_Disconnecting)
    {
    	/*
    	 Serial.print(linkEvent_Disconnecting);
       Serial.println(" : linkEvent_Disconnecting");
      */
    }
    else if (receiveEventState == linkEvent_Disconnected)
    {
    	/*
    	 Serial.print(linkEvent_Disconnected);
       Serial.println(" : linkEvent_Disconnected");
      */
    }
    
    
    else if (receiveEventState == linkEvent_GapLinkParamUpdate)
    {
    	/*
    	 Serial.print(linkEvent_GapLinkParamUpdate);
       Serial.println(" : linkEvent_GapLinkParamUpdate");
      */
    }
 
    else if (receiveEventState == linkEvent_RspReadError)
    {
    	/*
    	 Serial.print(linkEvent_RspReadError);
       Serial.println(" : linkEvent_RspReadError");
      */
    }
    
    else if (receiveEventState == linkEvent_RspReadSuccess)
    {
    	/*
    	 Serial.print(linkEvent_RspReadSuccess);
       Serial.println(" : linkEvent_RspReadSuccess");
      */
    }
    
    else if (receiveEventState == linkEvent_RspWriteError)
    {
    	/*
    	 Serial.print(linkEvent_RspWriteError);
       Serial.println(" : linkEvent_RspWriteError");
      */
    }
    else if (receiveEventState == linkEvent_RspWriteSuccess)
    {
    	/*
    	 Serial.print(linkEvent_RspWriteSuccess);
       Serial.println(" : linkEvent_RspWriteSuccess");
      */
    }
         
    else if (receiveEventState == linkEvent_SetNotify)
    {
    	/*
    	 Serial.print(linkEvent_SetNotify);
       Serial.println(" : linkEvent_SetNotify");
      */
    }
    
    else if (receiveEventState == linkEvent_Write)
    {
    	/*
    	 Serial.print(linkEvent_Write);
       Serial.println(" : linkEvent_Write");
      */
    }
    
    receiveComplete = -1;
    receiveDtype = -1;
    receiveLength = -1;
    receiveEventState = -1;
    receiveLinkState = -1;
  }
}


void CoDroneClass::ButtonPreesHoldWait(int button)
{
  do {
    delay(10);
  }    while (digitalRead(button));
}


void CoDroneClass::ButtonPreesHoldWait(int button1, int button2)
{
  do {
    delay(10);
  }    while (digitalRead(button1) && digitalRead(button2));
}


boolean CoDroneClass::TimeCheck(word interval) //milliseconds
{
  boolean time = false;
  unsigned long currentMillis = millis();
  if (currentMillis - PreviousMillis > interval)
  {
    PreviousMillis = currentMillis;
    time = true;
  }
  return time;
}


boolean CoDroneClass::TimeOutConnetionCheck(word interval) //milliseconds
{
  boolean time = false;
  unsigned long currentMillis = millis();
  if (currentMillis - timeOutConnectionPreviousMillis > interval)
  {
    timeOutConnectionPreviousMillis = currentMillis;
    time = true;
  }
  return time;
}


/*
void CoDroneClass::PrintSensor()
{

	int analogValue3 = analogRead(A3);  //throttle
	int analogValue4 = analogRead(A4);  //yaw
	int analogValue5 = analogRead(A5);  //pitch
	int analogValue6 = analogRead(A6);  //roll
	
	THROTTLE  = AnalogScaleChange(analogValue3);
	YAW = AnalogScaleChange(analogValue4);
	PITCH = AnalogScaleChange(analogValue5);
	ROLL = AnalogScaleChange(analogValue6);
	Serial.print(throttle);
	Serial.print("\t");
	Serial.print(yaw);
	Serial.print("\t");
	Serial.print(pitch);
	Serial.print("\t");
	Serial.println(roll);
}

/*********************************************************/
/*
void CoDroneClass::ReadSensor(void)
{
  bt1 = digitalRead(11);
  bt2 = digitalRead(12);
  //  bt3 = digitalRead(13);
  bt4 = digitalRead(14);
  //  bt6 = digitalRead(16);
  bt7 = digitalRead(17);
  bt8 = digitalRead(18);
      
  analogValue0 = analogRead(A0);
  analogValue1 = analogRead(A1);
  analogValue2 = analogRead(A2);
  analogValue3 = analogRead(A3);  //throttle
  analogValue4 = analogRead(A4);  //yaw
  analogValue5 = analogRead(A5);  //pitch
  analogValue6 = analogRead(A6);  //roll
  analogValue7 = analogRead(A7);
}
*/

int CoDroneClass::AnalogScaleChange(int analogValue)
{	
    int ScaleChange = map(analogValue, 0, 1023, -100, 100);
    if ((ScaleChange > -1 * analogOffset) && (ScaleChange < analogOffset)) ScaleChange = 0;        
    return ScaleChange;
}

/*********************************************************/

void CoDroneClass::BeepWarning(int count) 
{
	for (int i = 0; i < count; i++)
	{
		CoDrone.Buzz(2000, 4);
		delay(400);
	}
}

/*********************************************************/
void CoDroneClass::Buzz(long frequency, int tempo) 
{
 	DDRD |= 0b10000000;
 		
	int length = 1000/tempo;  								 //TEMPO
	long delayValue = 1000000/frequency/2; 		 // calculate the delay value between transitions
	long numCycles = frequency * length/ 1000; // calculate the number of cycles for proper timing
	
	for (long i=0; i < numCycles; i++)
	{ 		
		PORTD |= 0b10000000;		
		while(!TimeCheckBuzz(delayValue));		
		PORTD &=~0b10000000;					
		while(!TimeCheckBuzz(delayValue));
	}
}

boolean CoDroneClass::TimeCheckBuzz(word interval) //micros seconds
{
  boolean time = false; 
  unsigned long currentTimes = micros();
  if(currentTimes - PreviousBuzz >= interval) 
  {
    PreviousBuzz = currentTimes;
    time = true;
  }
  return time;
}



/*********************************************************/

CoDroneClass CoDrone;	