#include "CoDrone.h"
#include "Arduino.h"
#include <EEPROM.h>

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
	roll = 0;
	pitch = 0;
	yaw = 0;
	throttle = 0;
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
		if (receiveRangeSuccess == true)
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

