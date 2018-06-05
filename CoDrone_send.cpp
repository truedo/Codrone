#include "CoDrone.h"
#include "Arduino.h"
#include <EEPROM.h>

//-------------------------------------------------------------------------------------------------------//
//------------------------------------------ Command ----------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//

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

void CoDroneClass::Send_Processing(byte _data[], byte _length, byte _crc[])
{		
	byte _packet[30];
	
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
		
	DRONE_SERIAL.write(_packet, _length + 6);
		
	#if defined(DEBUG_MODE_ENABLE)
	DEBUG_SERIAL.print("> SEND : ");
	
	for(int i = 0; i < _length+6 ; i++)
	{
		DEBUG_SERIAL.print(_packet[i],HEX);	  	
		DEBUG_SERIAL.print(" ");	     
	}
	DEBUG_SERIAL.println("");	
	#endif 	
}

void CoDroneClass::Send_Check(byte _data[], byte _length, byte _crc[])
{
	if(sendCheckFlag == 1)
	{
		timeOutSendPreviousMillis = millis();
		
		while(sendCheckFlag != 3)
		{
			while(!TimeOutSendCheck(SEND_CHECK_TIME))
			{
				Receive();
				if(sendCheckFlag == 3) break;
			}
			if(sendCheckFlag == 3) break;
			if(sendCheckCount == 3) break;			//check count 
			Send_Processing(_data,_length,_crc);
			sendCheckCount++;
		}
		sendCheckFlag = 0;
		sendCheckCount = 0;
	}	
}
//-------------------------------------------------------------------------------------------------------//


//-------------------------------------------------------------------------------------------------------//
//--------------------------------------------- Send ----------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//


void CoDroneClass::Send_Coordinate(byte mode)
{
	if(mode == cSet_Absolute)				Send_Command(cType_Coordinate, cSet_Absolute);
	else if(mode == cSet_Relative)	Send_Command(cType_Coordinate, cSet_Relative);
}
void CoDroneClass::Send_ClearGyroBiasAndTrim()
{
	sendCheckFlag = 1;
	Send_Command(cType_ClearGyroBiasAndTrim, 0);
}
void CoDroneClass::Send_ResetHeading()
{
	Send_Command(cType_ResetHeading, 0);
}
void CoDroneClass::Send_Disconnect()
{
	Send_Command(cType_LinkDisconnect, 0);
}

//-------------------------------------------------------------------------------------------------------//



//-------------------------------------------------------------------------------------------------------//
//----------------------------------------- LED Event ---------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//

void CoDroneClass::LedColorProcess(byte _dType, byte sendMode, byte r, byte g, byte b, byte sendInterval)
{	
	byte _packet[9];
	byte _crc[2];	
	byte _len  = 0;  	
	byte _index = 0;
	
	if(_dType == dType_LedMode)	
	{	
		_len  = 0x03;  
		g = sendInterval;		
	}
	else if(_dType == dType_LedModeColor)		_len  = 0x05; 
	
	//header
	_packet[_index++] = _dType;
	_packet[_index++] = _len;		
	//data
	_packet[_index++] = sendMode;
	_packet[_index++] = r;
	_packet[_index++] = g;  
	_packet[_index++] = b;  
	_packet[_index++] = sendInterval;	
		
	unsigned short crcCal = CRC16_Make(_packet, _len+2);
	_crc[0] = (crcCal >> 8) & 0xff;
	_crc[1] = crcCal & 0xff;	
	Send_Processing(_packet,_len,_crc);    
}
//-------------------------------------- LedColor -------------------------------------------------------//
void CoDroneClass::LedColor(byte sendMode, byte sendColor, byte sendInterval)
{		
	LedColorProcess(dType_LedMode, sendMode, sendColor, 0, 0,sendInterval);
}
void CoDroneClass::LedColor(byte sendMode, byte r, byte g, byte b, byte sendInterval)
{		
	LedColorProcess(dType_LedModeColor, sendMode, r, g, b,sendInterval);
}
void CoDroneClass::LedColor(byte sendMode, byte sendColor[], byte sendInterval)
{
	LedColorProcess(dType_LedModeColor, sendMode,  sendColor[0], sendColor[1], sendColor[2], sendInterval);
}

//----------------------------------- LedColorDefault ---------------------------------------------------//

void CoDroneClass::LedColorDefault(byte sendMode, byte r, byte g, byte b, byte sendInterval)
{	
	LedColorProcess(dType_LedDefaultColor, sendMode, r, g, b,sendInterval);	
}

void CoDroneClass::LedColorDefault(byte sendMode, byte sendColor[], byte sendInterval)
{	
	LedColorProcess(dType_LedDefaultColor, sendMode, sendColor[0], sendColor[1], sendColor[2], sendInterval);	
}

void CoDroneClass::LedColorDefault(byte sendMode, byte sendColor[], byte sendInterval, byte sendMode2, byte sendColor2[], byte sendInterval2)
{	
	byte _packet[12];
	byte _crc[2];

	//header
	_packet[0] = dType_LedDefaultColor2;
	_packet[1] = 10;

	//data
	_packet[2] = sendMode;
	_packet[3] = sendColor[0];
	_packet[4] = sendColor[1];  
	_packet[5] = sendColor[2];  
	_packet[6] = sendInterval;

	_packet[7] = sendMode2;
	_packet[8] = sendColor2[0];
	_packet[9] = sendColor2[1];  
	_packet[10] = sendColor2[2];  
	_packet[11] = sendInterval2;
	
	unsigned short crcCal = CRC16_Make(_packet, _packet[1]+2);
	_crc[0] = (crcCal >> 8) & 0xff;
	_crc[1] = crcCal & 0xff;
	
	Send_Processing(_packet,_packet[1],_crc);     
}

//-------------------------------------------------------------------------------------------------------//
