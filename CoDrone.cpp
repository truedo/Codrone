/*
	CoDrone.cpp	-	CoDrone	library
	Copyright	(C)	2014 RoboLink.	All	rights reserved.
	LastUpdate : 2018-05-31
*/

#include "CoDrone.h"
#include "Arduino.h"
#include <EEPROM.h>

CoDroneClass CoDrone;

//-------------------------------------------------------------------------------------------------------//
//-------------------------------------------- Begin ----------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//

void CoDroneClass::begin(long	baud)
{
	DRONE_SERIAL.begin(baud);							// 드론과	통신 개시	(115200bps)
	#if	defined(FIND_HWSERIAL1)
		DEBUG_SERIAL.begin(baud);						// Serial	Debug	Begin	(115200bps)
		displayMode	=	0;										// LED Display 0 = BOARD LED 0FF,	1	=	BOARD	LED	ON
	#endif
	SendInterval = 50;										// millis	seconds
	analogOffset = 10;										// analog	sensor offset
	LED_Display(LED_DISPLAY_START, 0);		// LED_Start();

	if (EEPROM.read(EEP_AddressCheck))		// Connected Drone Address Read
	{
		for	(int i = 0;	i	<= 5;	i++)	devAddressConnected[i] = EEPROM.read(EEP_AddressFirst+i);
		isConnected = true;
	}

	#if	!defined(__AVR_ATmega328PB__)
		Send_LinkModeBroadcast(LinkModeActive);			//Link Active	Mode
		delay(100);
	#endif
}
//-------------------------------------------------------------------------------------------------------//


//-------------------------------------------------------------------------------------------------------//
//---------------------------------------- Discover Drone -----------------------------------------------//
//-------------------------------------------------------------------------------------------------------//

void CoDroneClass::Send_Discover(byte	action)
{
	if(action	== DiscoverStop)				Send_Command(cType_LinkDiscoverStop, 0);
	else if(action ==	DiscoverStart)
	{
		Send_Command(cType_LinkDiscoverStart,	0);
		discoverFlag = 1;
	}
}
//-------------------------------------------------------------------------------------------------------//


//-------------------------------------------------------------------------------------------------------//
//----------------------------------------- AutoConnect ------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//

void CoDroneClass::AutoConnect()								// NearbyDrone or ConnectedDrone
{
	pinMode(10, INPUT_PULLUP);    								//DipSw3
  if (!digitalRead(10))	 isConnected = false;		//DipSw3 down ↓

  if(isConnected)	AutoConnect(ConnectedDrone);
  else AutoConnect(NearbyDrone);
}

void CoDroneClass::AutoConnect(byte	mode)
{
	connectMode = mode;
	ConnectionProcess();
}

void CoDroneClass::AutoConnect(byte	mode,	byte address[])
{
	connectMode = mode;
	for	(int i = 0;	i	<= 5;	i++)	devAddressConnected[i] = address[i];
	ConnectionProcess();
}
//-------------------------------------------------------------------------------------------------------//


//-------------------------------------------------------------------------------------------------------//
//----------------------------------------- ConnectionProcess ------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//

void CoDroneClass::ConnectionProcess()
{
	byte displayLED = 0;
	//-----------------------------------------------------------//
	if (getState() != 0)		pairing	=	true;
	//-----------------------------------------------------------//
	else																			// AutoConnect start
	{
		devNow = -1;
		Send_Discover(DiscoverStart);
		PreviousMillis = millis();
		LED_Display(LED_DISPLAY_DDRC,	0xff);

		while(!pairing)
		{
			if((discoverFlag ==	3) &&	(connectFlag ==	0))	// Address find
			{
				LED_Display(LED_DISPLAY_STANDARD,	0);
				PreviousMillis = millis();

		//		discoverFlag = 0;

				if(devNow > -1)
				{
					connectFlag = 1;
					Send_Command(cType_LinkConnect,	devNow);
				}
			}


			else if((!(connectFlag == 1)) && (TimeCheck(400)))	// time	out	&	LED
			{
				if (displayLED++ ==	4)
				{
					displayLED = 0;
					
					RSSI_High = -255;
					devNow = -1;
					Send_Discover(DiscoverStart);
				}
				LED_Display(LED_DISPLAY_MOVE_RADER,	displayLED);
				PreviousMillis = millis();
			}


		  while (DRONE_SERIAL.available() > 0) 	Receive();
		}
		
		delay(1000);
	}
				
	DDRC = 	0b11111111;		
	PORTC = 0b11100111;
		
	delay(1000);
	
	DDRC = 	0b11111111;		
	PORTC = 0b01100110;
	
	delay(1000);
		
	LED_Connect();
	
  while (DRONE_SERIAL.available() > 0)     Receive();
  delay(50);
  roll = 0;	pitch = 0;	yaw = 0;	throttle = 0;
  Control();
  delay(50);
  while (DRONE_SERIAL.available() > 0)     Receive();
  delay(50);
}

//-------------------------------------------------------------------------------------------------------//


//-------------------------------------------------------------------------------------------------------//
//------------------------------------------ Receive ----------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//

void CoDroneClass::Receive()
{
	if (DRONE_SERIAL.available() > 0)
	{
		int	input	=	DRONE_SERIAL.read();

		#if	defined(DEBUG_MODE_ENABLE)
		//	DEBUG_SERIAL.print(input,HEX);	DEBUG_SERIAL.print(" ");
		#endif
		cmdBuff[cmdIndex++]	=	(char)input;

		if (cmdIndex >=	MAX_PACKET_LENGTH)
		{
			checkHeader	=	0;
			cmdIndex = 0;
		}
		else
		{
			if (cmdIndex ==	1)
			{
				if (cmdBuff[0] ==	START1)	checkHeader	=	1;
				else
				{
					checkHeader	=	0;
					cmdIndex = 0;
				}
			}
			else if	((cmdIndex	== 2)	&& (checkHeader	== 1))
			{
				if (cmdBuff[1] ==	START2)	checkHeader	=	2;
				else
				{
					checkHeader	=	0;
					cmdIndex = 0;
				}
			}
			else if	(checkHeader ==	2)
			{
				if (cmdIndex ==	3)	receiveDtype = cmdBuff[2];		// Receive Dtype

				else if	(receiveDtype	!= dType_StringMessage)			// not message	string
				{
					if (cmdIndex ==	4)	receiveLength	=	cmdBuff[3];	// Receive Length

					else if	(cmdIndex	>	4)
					{
						if (receiveLength	+	6	<= cmdIndex)
						{
							byte crcBuff[2];
							crcBuff[0] =	cmdBuff[cmdIndex - 2];
							crcBuff[1] =	cmdBuff[cmdIndex - 1];

							byte dataBuff[30];
							for(int	i	=	0; i < receiveLength + 5 ; i++)	dataBuff[i - 3]	=	cmdBuff[i	-	1];

							byte receiveComplete = 0;
							if (CRC16_Check(dataBuff,	receiveLength, crcBuff))	receiveComplete	=	1;

							byte receiveCompleteData[28];

							if (receiveComplete	== 1)
							{
								if	((receiveDtype ==	dType_Pressure)			||	(receiveDtype ==	 dType_Battery)		||	(receiveDtype	==	dType_Motor)				||	(receiveDtype	== dType_ImuRawAndAngle)||
										(receiveDtype ==	dType_CountFlight)	||	(receiveDtype	== dType_Range)				||	(receiveDtype ==	dType_TrimAll)			||	(receiveDtype	== dType_CountDrive)		||
										(receiveDtype ==	dType_TrimFlight)		||	(receiveDtype ==	 dType_ImageFlow)	||	(receiveDtype ==	 dType_Temperature)	||	(receiveDtype	== dType_State)					||
										(receiveDtype ==	dType_Attitude)			||	(receiveDtype ==	dType_GyroBias)		||	(receiveDtype	== dType_IrMessage)			||	(receiveDtype	== dType_TrimDrive)			||
										(receiveDtype	==	dType_Button)				||	(receiveDtype ==	dType_Ack)				||	(receiveDtype ==	dType_LinkDiscoveredDevice)	||
										(receiveDtype ==	dType_LinkState)		||	(receiveDtype ==	dType_LinkEvent)	||	(receiveDtype	== dType_LinkEventAddress)	||	(receiveDtype ==	dType_LinkRssi))
										{
											for(byte i = 0; i <= 27; i++)	receiveCompleteData[i] = dataBuff[i + 2];
										}
							}
							checkHeader	=	0;
							cmdIndex = 0;
							if(receiveComplete)	ReceiveEventCheck(receiveCompleteData);
						}
					}
				}
				else
				{
					checkHeader	=	0;
					cmdIndex = 0;
				}
			}
		}
	}
}
//-------------------------------------------------------------------------------------------------------//


//-------------------------------------------------------------------------------------------------------//
//---------------------------------- ReceiveEventCheck --------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//

void CoDroneClass::ReceiveEventCheck(byte	_completeData[])
{
	//-------------------------------------------------------------------------------------------//
	//-------------------------------------------------------------------------------------------//
	if (receiveDtype	== dType_LinkDiscoveredDevice)//Discovered Device
	{
		
		byte devAddressNow[6];
		int devIndex	=	_completeData[0];

		for	(int i = 1;	i	<= 6;	i++)	devAddressNow[i-1] = _completeData[i];
		RSSI_Now =_completeData[27];

		//-------------------------------------------------------------------------------------//

		if(connectMode == NearbyDrone)
		{
			if(RSSI_High < RSSI_Now)
			{
				RSSI_High = RSSI_Now;
				devNow = devIndex;
				for	(int i = 0;	i	<= 5;	i++)	devAddressBuf[i] = devAddressNow[i];
			}
		}

		else if((connectMode == ConnectedDrone) || (connectMode == AddressInputDrone))
		{
			byte _addrCk = 0;
			byte _rvAddrCk = 0;

			for	(int i = 0;	i	<= 5;	i++)		// same	address	check
			{
				if (devAddressConnected[i] ==	devAddressNow[i])	 	_addrCk++;
				if (devAddressConnected[i] ==	devAddressNow[5-i])	_rvAddrCk++;
			}

			if((_addrCk == 6)	|| (_rvAddrCk	== 6))
			{
				devNow = devIndex;
				for	(int i = 0;	i	<= 5;	i++)	devAddressBuf[i] = devAddressNow[i];
			}
		}

		//-------------------------------------------------------------------------------------//

	}
	//-------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------//
	//---------------------------------------------------------------------------------------------//
	else if (receiveDtype ==	dType_State)
	{

		droneState[0]	=	_completeData[0];
		droneState[1]	=	_completeData[1];
		droneState[2]	=	_completeData[2];
		droneState[3]	=	_completeData[3];
		droneState[4]	=	_completeData[4];
		droneState[5]	=	_completeData[5];
		droneState[6]	=	_completeData[6];

		receiveStateSuccess = 1;

		#if	defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	Request	Drone	State");
			DEBUG_SERIAL.print("ModeDrone	\t");

			if(droneState[0] ==	dMode_None)									DEBUG_SERIAL.println("None");
			else if(droneState[0]	== dMode_Flight)					DEBUG_SERIAL.println("Flight");
			else if(droneState[0]	== dMode_FlightNoGuard)		DEBUG_SERIAL.println("FlightNoGuard");
			else if(droneState[0]	== dMode_FlightFPV)				DEBUG_SERIAL.println("FlightFPV");
			else if(droneState[0]	== dMode_Drive)						DEBUG_SERIAL.println("Drive");
			else if(droneState[0]	== dMode_DriveFPV)				DEBUG_SERIAL.println("DriveFPV");
			else if(droneState[0]	== dMode_Test)						DEBUG_SERIAL.println("Test");

			DEBUG_SERIAL.print("ModeVehicle	\t");

			if(droneState[1] ==	vMode_None)									DEBUG_SERIAL.println("None");
			else if(droneState[1]	== vMode_Boot)						DEBUG_SERIAL.println("Boot");
			else if(droneState[1]	== vMode_Wait)						DEBUG_SERIAL.println("Wait");
			else if(droneState[1]	== vMode_Ready)						DEBUG_SERIAL.println("Ready");
			else if(droneState[1]	== vMode_Running)					DEBUG_SERIAL.println("Running");
			else if(droneState[1]	== vMode_Update)					DEBUG_SERIAL.println("Update");
			else if(droneState[1]	== vMode_UpdateComplete)	DEBUG_SERIAL.println("UpdateComplete");
			else if(droneState[1]	== vMode_Error)						DEBUG_SERIAL.println("Error");

			DEBUG_SERIAL.print("ModeFlight \t");

			if(droneState[2] ==	fMode_None)									DEBUG_SERIAL.println("None");
			else if(droneState[2]	== fMode_Ready)						DEBUG_SERIAL.println("Ready");
			else if(droneState[2]	== fMode_TakeOff)					DEBUG_SERIAL.println("TakeOff");
			else if(droneState[2]	== fMode_Flight)					DEBUG_SERIAL.println("Flight");
			else if(droneState[2]	== fMode_Flip)						DEBUG_SERIAL.println("Flip");
			else if(droneState[2]	== fMode_Stop)						DEBUG_SERIAL.println("Stop");
			else if(droneState[2]	== fMode_Landing)					DEBUG_SERIAL.println("Landing");
			else if(droneState[2]	== fMode_Reverse)					DEBUG_SERIAL.println("Reverse");
			else if(droneState[2]	== fMode_Accident)				DEBUG_SERIAL.println("Accident");
			else if(droneState[2]	== fMode_Error)						DEBUG_SERIAL.println("Error");

			DEBUG_SERIAL.print("ModeDrive	\t");

			if(droneState[3] ==	dvMode_None)								DEBUG_SERIAL.println("None");
			else if(droneState[3]	== dvMode_Ready)					DEBUG_SERIAL.println("Ready");
			else if(droneState[3]	== dvMode_Start)					DEBUG_SERIAL.println("Start");
			else if(droneState[3]	== dvMode_Drive)					DEBUG_SERIAL.println("Drive");
			else if(droneState[3]	== dvMode_Stop)						DEBUG_SERIAL.println("Stop");
			else if(droneState[3]	== dvMode_Accident)				DEBUG_SERIAL.println("Accident");
			else if(droneState[3]	== dvMode_Error)					DEBUG_SERIAL.println("Error");

			DEBUG_SERIAL.print("SensorOrientation	\t");

			if(droneState[4] ==	senOri_None)								DEBUG_SERIAL.println("None");
			else if(droneState[4]	== senOri_Normal)					DEBUG_SERIAL.println("Normal");
			else if(droneState[4]	== senOri_ReverseStart)		DEBUG_SERIAL.println("ReverseStart");
			else if(droneState[4]	== senOri_Reverse)				DEBUG_SERIAL.println("Reverse");

			DEBUG_SERIAL.print("Coordinate \t");

			if(droneState[5] ==	cSet_None)									DEBUG_SERIAL.println("None");
			else if(droneState[5]	== cSet_Absolute)					DEBUG_SERIAL.println("Absolute");
			else if(droneState[5]	== cSet_Relative)					DEBUG_SERIAL.println("Relative");

			DEBUG_SERIAL.print("Battery	\t");
			DEBUG_SERIAL.println(droneState[6]);

		#endif
	}
	 //---------------------------------------------------------------------------------------------//

	else if	(receiveDtype	== dType_LinkRssi)
	{
		rssi = _completeData[0];
		rssi = rssi	-	256;

		#if	defined(DEBUG_MODE_ENABLE)
			DEBUG_SERIAL.print("RSSI : ");
			DEBUG_SERIAL.println(rssi);
		#endif
	}

	else if	(receiveDtype == dType_Ack)
	{
		#if	defined(DEBUG_MODE_ENABLE)

			unsigned long _systemTime	=	((_completeData[3] <<	24)	|	(_completeData[2]	<< 16) | (_completeData[1] <<	8) | (_completeData[0]	&	0xff));
			byte _dType = _completeData[4];

			DEBUG_SERIAL.print("- dataType ");
			DEBUG_SERIAL.print(_dType,HEX);
			DEBUG_SERIAL.print("\tsystemTime ");
			DEBUG_SERIAL.println(_systemTime);
		#endif
	}

	else if	(receiveDtype	== dType_IrMessage)
	{
			byte irMassageDirection	=	_completeData[0];

			unsigned long	_irMessge[4];

			_irMessge[0] = _completeData[1];
			_irMessge[1] = _completeData[2];
			_irMessge[2] = _completeData[3];
			_irMessge[3] = _completeData[4];

			irMessageReceive	=	((_irMessge[3] <<	24)	|	(_irMessge[2]	<< 16) | (_irMessge[1] <<	8) | (_irMessge[0]	&	0xff));

			#if	defined(DEBUG_MODE_ENABLE)

				DEBUG_SERIAL.println("");
				DEBUG_SERIAL.println("-	IrMassage");
				DEBUG_SERIAL.print("[	");
				DEBUG_SERIAL.print(_completeData[0],HEX);
				DEBUG_SERIAL.print(",	");
				DEBUG_SERIAL.print(_completeData[1],HEX);
				DEBUG_SERIAL.print(",	");
				DEBUG_SERIAL.print(_completeData[2],HEX);
				DEBUG_SERIAL.print(",	");
				DEBUG_SERIAL.print(_completeData[3],HEX);
				DEBUG_SERIAL.print(",	");
				DEBUG_SERIAL.print(_completeData[4],HEX);
				DEBUG_SERIAL.println(" ]");

				DEBUG_SERIAL.print("IrMassageDirection\t");
				DEBUG_SERIAL.print(irMassageDirection);

				if(irMassageDirection	== 1)				DEBUG_SERIAL.println(" (Left)");
				else if	(irMassageDirection	== 2)	DEBUG_SERIAL.println(" (Front)");
				else if	(irMassageDirection	== 3)	DEBUG_SERIAL.println(" (Right)");
				else if	(irMassageDirection	== 4)	DEBUG_SERIAL.println(" (Rear)");
				else DEBUG_SERIAL.println("None");

				DEBUG_SERIAL.print("irMessageReceive\t");
				DEBUG_SERIAL.println(irMessageReceive);

			#endif
	 }
	//---------------------------------------------------------------------------------------------//
	else if	(receiveDtype	== dType_Attitude)
	{
		attitudeRoll	=	((_completeData[1] <<	8) | (_completeData[0]	&	0xff));
		attitudePitch	=	((_completeData[3] <<	8) | (_completeData[2]	&	0xff));
		attitudeYaw		=	((_completeData[5] <<	8) | (_completeData[4]	&	0xff));

		receiveAttitudeSuccess = 1;

		#if	defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	Attitude");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.println(" ]");

			DEBUG_SERIAL.print("ROLL\t");
			DEBUG_SERIAL.println(attitudeRoll);

			DEBUG_SERIAL.print("PITCH\t");
			DEBUG_SERIAL.println(attitudePitch);

			DEBUG_SERIAL.print("YAW\t");
			DEBUG_SERIAL.println(attitudeYaw);

		#endif
	}
//---------------------------------------------------------------------------------------------//
	else if	(receiveDtype	== dType_GyroBias)
	{
		int GyroBias_Roll		= ((_completeData[1] << 8) | (_completeData[0]  & 0xff));
		int GyroBias_Pitch	= ((_completeData[3] << 8) | (_completeData[2]  & 0xff));
		int GyroBias_Yaw		= ((_completeData[5] << 8) | (_completeData[4]  & 0xff));

		receiveGyroSuccess = 1;

		#if	defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	GyroBias");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.println(" ]");

			DEBUG_SERIAL.print("GyroBias ROLL\t");
			DEBUG_SERIAL.println(GyroBias_Roll);

			DEBUG_SERIAL.print("GyroBias PITCH\t");
			DEBUG_SERIAL.println(GyroBias_Pitch);

			DEBUG_SERIAL.print("GyroBias YAW\t");
			DEBUG_SERIAL.println(GyroBias_Yaw);

		#endif
	}
//---------------------------------------------------------------------------------------------//
	else if	(receiveDtype	== dType_TrimAll)
	{
		TrimAll_Roll			=	((_completeData[1] <<	8) | (_completeData[0]	&	0xff));
		TrimAll_Pitch			=	((_completeData[3] <<	8) | (_completeData[2]	&	0xff));
		TrimAll_Yaw				=	((_completeData[5] <<	8) | (_completeData[4]	&	0xff));
		TrimAll_Throttle	=	((_completeData[7] <<	8) | (_completeData[6]	&	0xff));
		TrimAll_Wheel			=	((_completeData[9] <<	8) | (_completeData[8]	&	0xff));

		#if defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	TrimAll");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[6],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[7],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[8],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[9],HEX);
			DEBUG_SERIAL.println(" ]");

			DEBUG_SERIAL.print("Trim ROLL\t");
			DEBUG_SERIAL.println(TrimAll_Roll);

			DEBUG_SERIAL.print("Trim PITCH\t");
			DEBUG_SERIAL.println(TrimAll_Pitch);

			DEBUG_SERIAL.print("Trim YAW\t");
			DEBUG_SERIAL.println(TrimAll_Yaw);

			DEBUG_SERIAL.print("Trim Throttle\t");
			DEBUG_SERIAL.println(TrimAll_Throttle);

			DEBUG_SERIAL.print("Trim Wheel\t");
			DEBUG_SERIAL.println(TrimAll_Wheel);

		 #endif
	}
//---------------------------------------------------------------------------------------------//
	else if	(receiveDtype	== dType_TrimFlight)		//
	{
		TrimAll_Roll		=	((_completeData[1] <<	8) | (_completeData[0]	&	0xff));
		TrimAll_Pitch		=	((_completeData[3] <<	8) | (_completeData[2]	&	0xff));
		TrimAll_Yaw			=	((_completeData[5] <<	8) | (_completeData[4]	&	0xff));
		TrimAll_Throttle	=	((_completeData[7] <<	8) | (_completeData[6]	&	0xff));

		receiveTrimSuccess = 1;

		#if	defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	TrimFlight");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[6],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[7],HEX);
			DEBUG_SERIAL.println(" ]");

			DEBUG_SERIAL.print("Trim ROLL\t");
			DEBUG_SERIAL.println(TrimAll_Roll);

			DEBUG_SERIAL.print("Trim PITCH\t");
			DEBUG_SERIAL.println(TrimAll_Pitch);

			DEBUG_SERIAL.print("Trim YAW\t");
			DEBUG_SERIAL.println(TrimAll_Yaw);

			DEBUG_SERIAL.print("Trim Throttle\t");
			DEBUG_SERIAL.println(TrimAll_Throttle);

		#endif
	}
//---------------------------------------------------------------------------------------------//
	else if	(receiveDtype	== dType_TrimDrive)
	{
		 #if defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	TrimDrive");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.println(" ]");

			TrimAll_Wheel			=	((_completeData[1] <<	8) | (_completeData[0]	&	0xff));

			DEBUG_SERIAL.print("Trim Wheel\t");
			DEBUG_SERIAL.println(TrimAll_Wheel);

		#endif
	}
	//---------------------------------------------------------------------------------------------//
	else if(receiveDtype ==	dType_ImuRawAndAngle)
	{
		ImuAccX	= (_completeData[3] << 8) | (_completeData[2]);// x and y are switched
		ImuAccY	= (_completeData[1] << 8) | (_completeData[0]);// x and y are switched
		ImuAccZ	= -(_completeData[5] << 8) | (_completeData[4]);// y needs to be flipped to have gravity be negative

		ImuGyroRoll		= (_completeData[7] << 8) | (_completeData[6]);;
		ImuGyroPitch	= (_completeData[9] << 8) | (_completeData[8]);
		ImuGyroYaw		= (_completeData[11] << 8) | (_completeData[10]);

		ImuAngleRoll	= (_completeData[13] << 8) | (_completeData[12]);
		ImuAnglePitch	= (_completeData[15] << 8) | (_completeData[14]);
		ImuAngleYaw		= (_completeData[17] << 8) | (_completeData[16]);

		receiveAccelSuccess = 1;

		#if defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	ImuRawAndAngle");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[6],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[7],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[8],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[9],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[10],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[11],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[12],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[13],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[14],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[15],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[16],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[17],HEX);
			DEBUG_SERIAL.println(" ]");

			DEBUG_SERIAL.print("AccX\t");
			DEBUG_SERIAL.println(ImuAccX);

			DEBUG_SERIAL.print("AccY\t");
			DEBUG_SERIAL.println(ImuAccY);

			DEBUG_SERIAL.print("AccZ\t");
			DEBUG_SERIAL.println(ImuAccZ);

			DEBUG_SERIAL.print("GyroRoll\t");
			DEBUG_SERIAL.println(ImuGyroRoll);

			DEBUG_SERIAL.print("GyroPitch\t");
			DEBUG_SERIAL.println(ImuGyroPitch);

			DEBUG_SERIAL.print("GyroYaw	\t");
			DEBUG_SERIAL.println(ImuGyroYaw);

			DEBUG_SERIAL.print("AngleRoll\t");
			DEBUG_SERIAL.println(ImuAngleRoll);

			DEBUG_SERIAL.print("AnglePitch\t");
			DEBUG_SERIAL.println(ImuAnglePitch);

			DEBUG_SERIAL.print("AngleYaw\t");
			DEBUG_SERIAL.println(ImuAngleYaw);

		#endif
	}

//---------------------------------------------------------------------------------------------//

	else if(receiveDtype ==	dType_Pressure)
	{
		long d1			= ((_completeData[3] << 32)  |(_completeData[2] << 16)  |(_completeData[1] << 8)  | (_completeData[0]  & 0xff));
		long d2			= ((_completeData[7] << 32)  |(_completeData[6] << 16)  |(_completeData[5] << 8)  | (_completeData[4]  & 0xff));
		temperature	= ((_completeData[11] << 32) |(_completeData[10] << 16) |(_completeData[9] << 8)  | (_completeData[8]  & 0xff));
		pressure		= ((_completeData[15] << 32) |(_completeData[14] << 16) |(_completeData[13] << 8) | (_completeData[12]  & 0xff));

		receivePressureSuccess = 1;

		#if	defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	Pressure");
			DEBUG_SERIAL.print("[	");

			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[6],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[7],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[8],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[9],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[10],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[11],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[12],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[13],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[14],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[15],HEX);
			DEBUG_SERIAL.println(" ]");

			DEBUG_SERIAL.print("d1\t");
			DEBUG_SERIAL.println(d1);
			DEBUG_SERIAL.print("d2\t");
			DEBUG_SERIAL.println(d2);
			DEBUG_SERIAL.print("temperature\t");
			DEBUG_SERIAL.println(temperature);
			DEBUG_SERIAL.print("pressure\t");
			DEBUG_SERIAL.println(pressure);

		#endif
	}
	//---------------------------------------------------------------------------------------------//
	else if	(receiveDtype	==	dType_ImageFlow)
	{
		fVelocitySumX	= ((_completeData[3] << 32)  |(_completeData[2] << 16)  |(_completeData[1] << 8)  | (_completeData[0]  & 0xff));
		fVelocitySumY	= ((_completeData[7] << 32)  |(_completeData[6] << 16)  |(_completeData[5] << 8)  | (_completeData[4]  & 0xff));

		receiveOptSuccess = 1;

		#if	defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	ImageFlow");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[6],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[7],HEX);
			DEBUG_SERIAL.println(" ]");

			DEBUG_SERIAL.print("fVelocitySumX\t");
			DEBUG_SERIAL.println(fVelocitySumX);
			DEBUG_SERIAL.print("fVelocitySumY\t");
			DEBUG_SERIAL.println(fVelocitySumY);

		#endif
	}
	//---------------------------------------------------------------------------------------------//
	else if	(receiveDtype	== dType_Button)
	{
		#if	defined(DEBUG_MODE_ENABLE)
			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	Button");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.println(" ]");
		#endif
	}
	//---------------------------------------------------------------------------------------------//
	else if	(receiveDtype	== dType_Battery)
	{
		int	batteryV30											=	((_completeData[1] <<	8) | (_completeData[0]	&	0xff));
		int	batteryV33											=	((_completeData[3] <<	8) | (_completeData[2]	&	0xff));
		int	batteryGradient								=	((_completeData[5] <<	8) | (_completeData[4]	&	0xff));
		int	batteryyIntercept							=	((_completeData[7] <<	8) | (_completeData[6]	&	0xff));
		int	flagBatteryCalibration					=	_completeData[8];
		int	batteryRaw											=	((_completeData[12]	<< 32) |(_completeData[11] <<	16)	|(_completeData[10]	<< 8)	|	(_completeData[9]	 & 0xff));
		batteryPercent											=	_completeData[13];
		batteryVoltage											=	((_completeData[15]	<< 8)	|	(_completeData[14]	&	0xff));

		receiveBatterySuccess = 1;

		#if	defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	Battery");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[6],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[7],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[8],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[9],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[10],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[11],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[12],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[13],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[14],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[15],HEX);
			DEBUG_SERIAL.println(" ]");

			DEBUG_SERIAL.print("v30\t");
			DEBUG_SERIAL.println(batteryV30);
			DEBUG_SERIAL.print("v33\t");
			DEBUG_SERIAL.println(batteryV33);
			DEBUG_SERIAL.print("gradient\t");
			DEBUG_SERIAL.println(batteryGradient);
			DEBUG_SERIAL.print("yIntercept\t");
			DEBUG_SERIAL.println(batteryyIntercept);
			DEBUG_SERIAL.print("flagBatteryCalibration\t");
			DEBUG_SERIAL.println(flagBatteryCalibration);
			DEBUG_SERIAL.print("batteryRaw\t");
			DEBUG_SERIAL.println(batteryRaw);
			DEBUG_SERIAL.print("batteryPercent\t");
			DEBUG_SERIAL.println(batteryPercent);
			DEBUG_SERIAL.print("voltage\t");
			DEBUG_SERIAL.println(batteryVoltage);

		#endif
	}

	else if	(receiveDtype	==	dType_Range)
	{
		sensorRange[0] = ((_completeData[1]	<< 8)	|	(_completeData[0]	 & 0xff));	//left
		sensorRange[1] = ((_completeData[3]	<< 8)	|	(_completeData[2]	 & 0xff));	//front
		sensorRange[2] = ((_completeData[5]	<< 8)	|	(_completeData[4]	 & 0xff));	//right
		sensorRange[3] = ((_completeData[7]	<< 8)	|	(_completeData[6]	 & 0xff));	//rear
		sensorRange[4] = ((_completeData[9]	<< 8)	|	(_completeData[8]	 & 0xff));	//top
		sensorRange[5] = ((_completeData[11] <<	8)| (_completeData[10]	 & 0xff));	//bottom

		receiveRangeSuccess	=	1;

		#if	defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	Range");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[6],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[7],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[8],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[9],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[10],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[11],HEX);
			DEBUG_SERIAL.println(" ]");

			int	left	=	sensorRange[0];
			int	front	=	sensorRange[1];
			int	right	=	sensorRange[2];
			int	real	=	sensorRange[3];
			int	top		=	sensorRange[4];
			int	bottom	=	sensorRange[5];

			DEBUG_SERIAL.print("left\t");
			DEBUG_SERIAL.println(left);
			DEBUG_SERIAL.print("front\t");
			DEBUG_SERIAL.println(front);
			DEBUG_SERIAL.print("right\t");
			DEBUG_SERIAL.println(right);
			DEBUG_SERIAL.print("real\t");
			DEBUG_SERIAL.println(real);
			DEBUG_SERIAL.print("top\t");
			DEBUG_SERIAL.println(top);
			DEBUG_SERIAL.print("bottom\t");
			DEBUG_SERIAL.println(bottom);
		#endif
	}
	else if	(receiveDtype	==	dType_CountDrive)
	{
		#if	defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	CountDrive");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[6],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[7],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[8],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[9],HEX);
			DEBUG_SERIAL.println(" ]");

			//	 timeFlight
			int	countAccident	=	((_completeData[9] <<	8) | (_completeData[8]	&	0xff));
			DEBUG_SERIAL.print("Count	Accident\t");
			DEBUG_SERIAL.println(countAccident);

		#endif
	}

	else if	(receiveDtype	== dType_CountFlight)
	{
		#if	defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	CountFlight");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[6],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[7],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[8],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[9],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[10],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[11],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[12],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[13],HEX);
			DEBUG_SERIAL.println(" ]");

			//	 timeFlight
			int	countTakeOff	=	((_completeData[9] <<	8) | (_completeData[8]	&	0xff));
			int	countLanding	=	((_completeData[11]	<< 8)	|	(_completeData[10]	&	0xff));
			int	countAccident	=	((_completeData[13]	<< 8)	|	(_completeData[12]	&	0xff));

			DEBUG_SERIAL.print("Count	TakeOff\t");
			DEBUG_SERIAL.println(countTakeOff);

			DEBUG_SERIAL.print("Count	Landing\t");
			DEBUG_SERIAL.println(countLanding);

			DEBUG_SERIAL.print("Count	Accident\t");
			DEBUG_SERIAL.println(countAccident);

		#endif
	}
//---------------------------------------------------------------------------------------------//
	else if	(receiveDtype	==	dType_Motor)
	{
		#if	defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	Motor");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[6],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[7],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[8],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[9],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[10],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[11],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[12],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[13],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[14],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[15],HEX);
			DEBUG_SERIAL.println(" ]");

			int	m1_forward	=	((_completeData[1] <<	8) | (_completeData[0]	&	0xff));
			int	m1_reverse	=	((_completeData[3] <<	8) | (_completeData[2]	&	0xff));
			int	m2_forward	=	((_completeData[5] <<	8) | (_completeData[4]	&	0xff));
			int	m2_reverse	=	((_completeData[7] <<	8) | (_completeData[6]	&	0xff));
			int	m3_forward	=	((_completeData[9] <<	8) | (_completeData[8]	&	0xff));
			int	m3_reverse	=	((_completeData[11]	<< 8)	|	(_completeData[10]	&	0xff));
			int	m4_forward	=	((_completeData[13]	<< 8)	|	(_completeData[12]	&	0xff));
			int	m4_reverse	=	((_completeData[15]	<< 8)	|	(_completeData[14]	&	0xff));

			DEBUG_SERIAL.println("[1]	[2]");
			DEBUG_SERIAL.println("	[	]	 ");
			DEBUG_SERIAL.println("[4]	[3]");

			DEBUG_SERIAL.print("m1_forward\t");
			DEBUG_SERIAL.println(m1_forward);

			DEBUG_SERIAL.print("m1_reverse\t");
			DEBUG_SERIAL.println(m1_reverse);

			DEBUG_SERIAL.print("m2_forward\t");
			DEBUG_SERIAL.println(m2_forward);

			DEBUG_SERIAL.print("m2_reverse\t");
			DEBUG_SERIAL.println(m2_reverse);

			DEBUG_SERIAL.print("m3_forward\t");
			DEBUG_SERIAL.println(m3_forward);

			DEBUG_SERIAL.print("m3_reverse\t");
			DEBUG_SERIAL.println(m3_reverse);

			DEBUG_SERIAL.print("m4_forward\t");
			DEBUG_SERIAL.println(m4_forward);

			DEBUG_SERIAL.print("m4_reverse\t");
			DEBUG_SERIAL.println(m4_reverse);

		#endif
	}
//---------------------------------------------------------------------------------------------//
	else if	(receiveDtype	== dType_Temperature)
	{
		#if	defined(DEBUG_MODE_ENABLE)

			DEBUG_SERIAL.println("");
			DEBUG_SERIAL.println("-	Temperature");
			DEBUG_SERIAL.print("[	");
			DEBUG_SERIAL.print(_completeData[0],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[1],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[2],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[3],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[4],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[5],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[6],HEX);
			DEBUG_SERIAL.print(",	");
			DEBUG_SERIAL.print(_completeData[7],HEX);
			DEBUG_SERIAL.println(" ]");

			long imu_temp				=	((_completeData[3] <<	32)	|(_completeData[2] <<	16)	|(_completeData[1] <<	8) | (_completeData[0]	&	0xff));
			long pressure_temp	=	((_completeData[7]	<< 32) |(_completeData[6] <<	16)	|(_completeData[5]	<< 8)	|	(_completeData[4]	&	0xff));

			DEBUG_SERIAL.print("Temperature	IMU\t");
			DEBUG_SERIAL.println(imu_temp);

			DEBUG_SERIAL.print("Temperature	Pressure\t");
			DEBUG_SERIAL.println(pressure_temp);

		#endif
	}

 //---------------------------------------------------------------------------------------------//

	else if	(receiveDtype	== dType_LinkState)
	{
		byte	receiveLinkState	=	_completeData[0];
	//	receiveLikMode		=	_completeData[1];

		#if	defined(DEBUG_MODE_ENABLE)
			DEBUG_SERIAL.print(receiveLinkState);
		#endif

		if(receiveLinkState	== linkMode_None)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkMode - None");
			#endif
		}
		else if(receiveLinkState ==	linkMode_Boot)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkMode - Boot");
			#endif
		}
		else if(receiveLinkState ==	linkMode_Ready)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkMode - Ready");
			#endif
		}
		else if(receiveLinkState ==	linkMode_Connecting)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkMode - Connecting");
			#endif
		}
		else if(receiveLinkState ==	linkMode_Connected)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkMode - Connected");
			#endif
		}
		else if(receiveLinkState ==	linkMode_Disconnecting)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkMode - Disconnecting");
			#endif
		}
		else if(receiveLinkState ==	linkMode_ReadyToReset)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkMode - ReadyToReset");
			#endif
		}
		linkState	=	receiveLinkState;
	}
//---------------------------------------------------------------------------------------------//

	else if	((receiveDtype ==	dType_LinkEvent) ||	(receiveDtype	== dType_LinkEventAddress))
	{
		byte	receiveEventState	=	_completeData[0];

		#if	defined(DEBUG_MODE_ENABLE)
			if(receiveDtype	== dType_LinkEvent)	DEBUG_SERIAL.print(receiveEventState);
			else if	(receiveDtype	== dType_LinkEventAddress) DEBUG_SERIAL.print(receiveEventState);
		#endif

//-----------------------------------------------------------------------------------------------//

		if (receiveEventState	== linkEvent_ScanStop)
		{
			if((discoverFlag ==	1) ||	(discoverFlag	== 2))	discoverFlag = 3;

			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - ScanStop");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_Connected)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - Connected");
			#endif

			if(connectFlag ==	1)
			{
				connectFlag	=	0;
				EEPROM.write(EEP_AddressCheck, 0x01);
				for	(int i = 0;	i	<= 5;	i++)		EEPROM.write(EEP_AddressFirst	+	i, devAddressBuf[i]);
			}
			pairing	=	true;
	//		delay(2500);
		}

		else if	(receiveEventState ==	linkEvent_ReadyToControl)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - ReadyToControl");
			#endif

			if(connectFlag ==	1)
			{
				connectFlag = 0;
				EEPROM.write(EEP_AddressCheck, 0x01);
				for	(int i = 0;	i	<= 5;	i++)		EEPROM.write(EEP_AddressFirst	+	i, devAddressBuf[i]);
			}
			pairing	=	true;
	//		delay(500);
		}

		else if	(receiveEventState ==	linkEvent_RspWriteSuccess)
		{
			if(sendCheckFlag ==	1)	sendCheckFlag	=	2;
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - RspWriteSuccess");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_Write)
		{
			if(sendCheckFlag ==	2)	sendCheckFlag	=	3;
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - Write");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_SystemReset)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - SystemReset");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_Initialized)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - Initialized");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_Scanning)
		{
			if(discoverFlag	== 1)	discoverFlag = 2;
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - Scanning");
			#endif
		}

//----------------------------------------------------------------------//

		else if	(receiveEventState ==	linkEvent_None)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - None");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_FoundDroneService)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - FoundDroneService");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_Connecting)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - Connecting");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_ConnectionFaild)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - ConnectionFaild");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_ConnectionFaildNoDevices)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - ConnectionFaildNoDevices");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_ConnectionFaildNotReady)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - ConnectionFaildNotReady");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_PairingStart)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - PairingStart");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_PairingSuccess)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - PairingSuccess");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_PairingFaild)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - PairingFaild");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_BondingSuccess)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - BondingSuccess");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_LookupAttribute)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - LookupAttribute");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_RssiPollingStart)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - RssiPollingStart");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_RssiPollingStop)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - RssiPollingStop");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_DiscoverService)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - DiscoverService");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_DiscoverCharacteristic)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - DiscoverCharacteristic");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_DiscoverCharacteristicDroneData)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - DiscoverCharacteristicDroneData");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_DiscoverCharacteristicDroneConfig)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - DiscoverCharacteristicDroneConfig");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_DiscoverCharacteristicUnknown)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - DiscoverCharacteristicUnknown");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_DiscoverCCCD)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - DiscoverCCCD");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_Disconnecting)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - Disconnecting");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_Disconnected)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - Disconnected");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_GapLinkParamUpdate)
		{
		#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - GapLinkParamUpdate");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_RspReadError)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - RspReadError");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_RspReadSuccess)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - RspReadSuccess");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_RspWriteError)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - RspWriteError");
			#endif
		}

		else if	(receiveEventState ==	linkEvent_SetNotify)
		{
			#if	defined(DEBUG_MODE_ENABLE)
				DEBUG_SERIAL.println(" : linkEvent - SetNotify");
			#endif
		}
	}
}
//-------------------------------------------------------------------------------------------------------//0