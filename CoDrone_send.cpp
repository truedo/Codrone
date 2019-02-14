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
	sendingData = true;		//data sending start
	
	byte _packet[30];
	
	//START CODE  
	_packet[0] = START1;
	_packet[1] = START2;

	//HEADER & DATA
	for(int i = 0; i < _length + 3 ; i++)	 _packet[i+2] = _data[i];	   
	
	//CRC  
	_packet[_length + 4] =_crc[1];
	_packet[_length + 5] =_crc[0]; 
	
	
	if(sendDataControl == true)	DRONE_SERIAL.write(_packet, _length + 6);			//data send control
		
	#if defined(DEBUG_MODE_ENABLE)
	DEBUG_SERIAL.print("> SEND : ");	
	for(int i = 0; i < _length+6 ; i++)
	{
		DEBUG_SERIAL.print(_packet[i],HEX);	  	
		DEBUG_SERIAL.print(" ");	     
	}
	DEBUG_SERIAL.println("");	
	#endif 	
	
	
	sendingData = false;		//data sending end
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

void CoDroneClass::calibrate()
{
	byte _packet[9];
	byte _crc[2];

  	//header
	_packet[0] = dType_Command;
	_packet[1] = 0x02;

 	//data
	_packet[2] = 0x53;
	_packet[3] = 0;

	unsigned short crcCal = CRC16_Make(_packet, _packet[1]+2);
	_crc[0] = (crcCal >> 8) & 0xff;
	_crc[1] = crcCal & 0xff;


	Send_Processing(_packet,_packet[1],_crc);
	Send_Check(_packet,_packet[1],_crc);
	delay(50);
	CoDrone.Buzz(523,2);
	CoDrone.Buzz(659,2);
	CoDrone.Buzz(783,2);
	CoDrone.Buzz(1055,2);
}


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




//-------------------------------------------------------------------------------------------------------//
//----------------------------------------- Drone Event -------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//
void CoDroneClass::DroneModeChange(byte event)
{
	sendCheckFlag = 1;
	Send_Command(cType_ModeDrone, event);
	delay(300);
}
void CoDroneClass::FlightEvent(byte event)
{
	sendCheckFlag = 1;
	Send_Command(cType_FlightEvent, event);
}
void CoDroneClass::DriveEvent(byte event)
{
	Send_Command(cType_DriveEvent, event);
}
//-------------------------------------------------------------------------------------------------------//

//-------------------------------------------------------------------------------------------------------//
//-------------------------------------------- Trim -----------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//
void CoDroneClass::Set_Trim(byte event)
{
	sendCheckFlag = 1;
	Send_Command(cType_Trim, event);
}

void CoDroneClass::Set_TrimReset()
{
	Set_TrimFlight(0, 0, 0, 0);
}

void CoDroneClass::Set_TrimFlight(int _roll, int _pitch, int _yaw, int _throttle)
{
	sendCheckFlag = 1;

	byte _packet[10];
	byte _crc[2];

	byte _cType = dType_TrimFlight;
	byte _len   = 8;

	//header
	_packet[0] = _cType;
	_packet[1] = _len;

	byte L_roll 		= _roll & 0xff;
	byte H_roll 		= (_roll >> 8) & 0xff;

	byte L_pitch 		= _pitch & 0xff;
	byte H_pitch 		= (_pitch >> 8) & 0xff;

	byte L_yaw	 		= _yaw & 0xff;
	byte H_yaw 			= (_yaw >> 8) & 0xff;

	byte L_throttle = _throttle & 0xff;
	byte H_throttle = (_throttle >> 8) & 0xff;

	//data
	_packet[2] = L_roll;
	_packet[3] = H_roll;

	_packet[4] = L_pitch;
	_packet[5] = H_pitch;

	_packet[6] = L_yaw;
	_packet[7] = H_yaw;

	_packet[8] = L_throttle;
	_packet[9] = H_throttle;

	unsigned short crcCal = CRC16_Make(_packet, _len + 2);
	_crc[0] = (crcCal >> 8) & 0xff;
	_crc[1] = crcCal & 0xff;

	Send_Processing(_packet,_len,_crc);
}


//-------------------------------------------------------------------------------------------------------//
//------------------------------------------- Control ---------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//
void CoDroneClass::Control(int interval)
{
	if (TimeCheck(interval))  //delay
	{
		Control();
		PreviousMillis = millis();
	}
}

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
	
	#if	!defined(__AVR_ATmega328PB__)
	
		roll = 0;
		pitch = 0;
		yaw = 0;
		throttle = 0;
	
	#endif
}
//-------------------------------------------------------------------------------------------------------//

//-------------------------------------------------------------------------------------------------------//
//-------------------------------------------- Move -----------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//
void CoDroneClass::GoToHeight(int _range)
{
	int _dir = 0;
	//---------------------------------------------------------------//
	int value = getHeight();
	if (value < _range)			_dir = 1;	// up
	else if(value > _range)	_dir = -1;	// down
	//---------------------------------------------------------------//
	while(1)
	{
		value = getHeight();
		if (receiveRangeSuccess)
		{
			if((_dir > 0) && (value < _range))
			{
				THROTTLE = 70;
				CoDrone.Control();
			}	
			else if((_dir < 0) && (value > _range))
			{
				THROTTLE = -20;
				CoDrone.Control();
			}
			
			else
			{
				THROTTLE = 0;
				CoDrone.Control();
				break;
			}
		}
	}
}

void CoDroneClass::TurnDegree(int _angle)
{
	int _bias = 3;
	int _dir = 0;
	int _pow = 20;

	if (_angle > 0)	_dir =  1;
	else          	_dir = -1;

	gyrodata angle;
	angle = CoDrone.getGyroAngles();
	int yawPast = angle.yaw;

	_angle = _dir * (abs(_angle) - _bias) + yawPast;

	while (1)
	{
		angle = CoDrone.getGyroAngles();
		int _nowAngle = angle.yaw;

		if (abs(yawPast - _nowAngle) > 180)	_angle -= _dir * 360;

		yawPast = _nowAngle;
		YAW = _dir * _pow;

		if ((_dir > 0) && (_angle > _nowAngle)) 			CoDrone.Control();	// right turn
		else if ((_dir < 0) && (_angle < _nowAngle)) 	CoDrone.Control();	// left turn
		else
		{
			YAW = 0;
			CoDrone.Control();
			break;
		}
	}
}
//-------------------------------------------------------------------------------------------------------//



//-------------------------------------------------------------------------------------------------------//
//------------------------------------------ Battle -----------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//
void CoDroneClass::BattleBegin(byte teamSelect)
{
	team = teamSelect;
	byte _color = 0;
	
	switch (team) 
	{
		case TEAM_RED:
		  weapon = RED_MISSILE;
		  _color = Red;
		  break;	      
		case TEAM_BLUE:
			weapon = BLUE_MISSILE;
			_color = Blue;
		  break;	      
		case TEAM_GREEN:
			weapon = GREEN_MISSILE;
			_color = Green;
		  break;	      
		case TEAM_YELLOW:
			weapon = YELLOW_MISSILE;
			_color = Yellow;
		  break;	      
		case FREE_PLAY:
			weapon = FREE_MISSILE;
			_color = White;
		  break;
	}	

	CoDrone.LedColor(ArmHold, _color, 7);
	delay(300);
	CoDrone.LedColor(EyeDimming, _color, 7);
	delay (300);
}


void CoDroneClass::BattleReceive()
{
	Receive();
	if(irMessageReceive > 0)
	{
		if(team == TEAM_RED && irMessageReceive == BLUE_MISSILE || irMessageReceive == GREEN_MISSILE || irMessageReceive == YELLOW_MISSILE || irMessageReceive == FREE_MISSILE)	BattleDamageProcess();
		else if(team == TEAM_BLUE && irMessageReceive == RED_MISSILE || irMessageReceive == GREEN_MISSILE || irMessageReceive == YELLOW_MISSILE || irMessageReceive == FREE_MISSILE)	BattleDamageProcess();
		else if(team == TEAM_GREEN && irMessageReceive == BLUE_MISSILE || irMessageReceive == RED_MISSILE || irMessageReceive == YELLOW_MISSILE || irMessageReceive == FREE_MISSILE)	BattleDamageProcess();
		else if(team == TEAM_YELLOW &&  irMessageReceive == BLUE_MISSILE || irMessageReceive == GREEN_MISSILE || irMessageReceive == RED_MISSILE || irMessageReceive == FREE_MISSILE)	BattleDamageProcess();
		else if(team == FREE_PLAY && irMessageReceive == RED_MISSILE || irMessageReceive == BLUE_MISSILE || irMessageReceive == GREEN_MISSILE || irMessageReceive == YELLOW_MISSILE || irMessageReceive == FREE_MISSILE)	BattleDamageProcess();
		irMessageReceive = 0;
	}
}

void CoDroneClass::BattleDamageProcess()
{
	if(displayMode == 1)
	{
		energy--;
		if(energy > 0)
		{
			DDRC = 0xff;
			PORTC = (0xff >> (MAX_ENERGY - energy)) << ((MAX_ENERGY - energy) / 2);
			CoDrone.Buzz(4000, 8);
		}
		else
		{
			delay(300);
			CoDrone.LedColor(ArmNone, Black, 7);
			DDRC = 0xff;
			PORTC = 0x00;

			for(int i = 0; i >= 3; i++)
			{
				CoDrone.Buzz(3000, 4);
				delay(100);
				CoDrone.Buzz(2000, 4);
				delay(100);
			}
		}
		delay(100);
		DDRC = 0b01100110;
		PORTC = 0b00100100;
	}
}

void CoDroneClass::BattleShooting()
{
	sendCheckFlag = 1;

	byte _packet[12];
	byte _crc[2];

	byte _cType = dType_LedEventCommandIr;
	byte _len   = 10;

	//header
	_packet[0] = _cType;
	_packet[1] = _len;

	//data
	//led event base
	_packet[2] = ArmDimming;
	_packet[3] = Magenta;
	_packet[4] = 7;
	_packet[5] = 2;

	//command base
	_packet[6] = cType_FlightEvent;
	_packet[7] = fEvent_Shot;

	//irData u32 - 4byte
	unsigned long data = weapon;
	_packet[8] = data & 0xff;
	_packet[9] = (data >> 8) & 0xff;
	_packet[10] 	=	(data >> 16) & 0xff;
	_packet[11] 	= (data >> 24) & 0xff;

	unsigned short crcCal = CRC16_Make(_packet, _len+2);
	_crc[0] = (crcCal >> 8) & 0xff;
	_crc[1] = crcCal & 0xff;

	Send_Processing(_packet,_len,_crc);
	Send_Check(_packet,_len,_crc);
}


//-------------------------------------------------------------------------------------------------------//

