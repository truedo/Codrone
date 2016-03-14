/*
  SmartDroneControl.h - SmartDroneControl library
  Copyright (C) 2014 RoboLink.  All rights reserved.
  LastUpdate : 2015-08-07
*/

#ifndef CoDrone_h
#define CoDrone_h
#include "Arduino.h"
#include <avr/interrupt.h>

typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

//////////////////////////////////////////////////////////////////////////////////

//START CODE
#define START1          	0x0A
#define START2          	0x55

//////////////////////////////////////////////////////////////////////////////////

#define OFF				0x00
#define ON					0x01
	
#define DOWN        0x00
#define UP          0x01

#define MAX_PACKET_LENGTH 	100

/////////////////////////////////////////////
/***********************************************************************/

#define ROLL					CoDrone.roll
#define PITCH					CoDrone.pitch
#define YAW						CoDrone.yaw
#define THROTTLE			CoDrone.throttle
#define EVENT					CoDrone.event
#define STATE					CoDrone.state
#define SEND_INTERVAL	CoDrone.SendInterval
#define ANALOG_OFFSET	CoDrone.analogOffset

/***********************************************************************/

#define DiscoverStop  			cType_DiscoverStop
#define DiscoverStart  			cType_DiscoverStart

#define	PAIRING							CoDrone.pairing

#define	NeardbyDrone    		1
#define	ConnectedDrone  		2
#define AddressInputDrone 	3

//eeprom address
#define	eep_AddressCheck   	10
#define	eep_AddressFirst  	11
#define	eep_AddressEnd  		15


//////////////////////////////////////////////////////////////////////////


#define Flight 					dMode_Flight
#define FlightNoGuard		dMode_FlightNoGuard,
#define FlightFPV				dMode_FlightFPV
#define Drive 				 	dMode_Drive
#define DriveFPV				dMode_DriveFPV
#define Test						dMode_Test

#define Absolute 				cSet_Absolute
#define Relative		 		cSet_Relative

#define TakeOff 				fEvent_TakeOff
#define FlipFront				fEvent_FlipFront
#define FlipRear				fEvent_FlipRear
#define FlipLeft				fEvent_flipLeft
#define FlipRight				fEvent_FlipRight
#define Stop						fEvent_Stop
#define Landing					fEvent_Landing
#define TurnOver				fEvent_TurnOver
#define Shot						fEvent_Shot
#define UnderAttack			fEvent_UnderAttack
#define Square					fEvent_Square
#define CircleLeft			fEvent_CircleLeft
#define CircleRight			fEvent_CircleRight
#define Rotate180				fEvent_Rotate180


#define RollIncrease			trim_RollIncrease
#define RollDecrease			trim_RollDecrease
#define PitchIncrease			trim_PitchIncrease
#define PitchDecrease			trim_PitchDecrease
#define YawIncrease				trim_YawIncrease
#define YawDecrease				trim_YawDecrease
#define ThrottleIncrease	trim_ThrottleIncrease
#define ThrottleDecrease	trim_ThrottleDecrease



/***********************************************************************/

enum ModeLink
{
	/*
	link_None = 0, ///< ����
	link_Boot, ///< ����
	link_Initialized, ///< ��ġ �ʱ�ȭ �Ϸ� (�̺�Ʈ ���� �� �ٷ� Disconnected�� ��ȯ)
	link_Discovering , ///< ��ġ �˻�
	link_DiscoveryStop, ///< ��ġ �˻� �ߴ� (�̺�Ʈ ���� �� �ٷ� Disconnected�� ��ȯ)
	link_Connecting, ///< ��ġ ���� ��
	link_ConnectionFaild, ///< ���� ���� (�̺�Ʈ ���� �� �ٷ� Disconnected�� ��ȯ)
	link_Connected, ///< ��ġ ���� �Ϸ�
	link_LookupAttribute, ///< ��ġ ���� �� �Ӽ� �˻�
	link_Ready, ///< ��ġ �۵� ���
	link_Disconnecting, ///< ��ġ ���� ���� ��
	link_Disconnected, ///< ��ġ ���� ���� �Ϸ�
	link_EndOfType
	*/
	
	linkMode_None = 0,	 	 ///< ����
	linkMode_Boot,	 	 	 ///< ���� 	 	
	linkMode_Ready,	 		 ///< ���(���� ��)
	linkMode_Connecting,	 	 ///< ��ġ ���� ��
	linkMode_Connected,	 	 ///< ��ġ ���� �Ϸ�
	linkMode_Disconnecting,	 	 ///< ��ġ ���� ���� ��
	linkMode_ReadyToReset,	 	 ///< ���� ���(1�� �ڿ� ��ġ ����)	
	linkMode_EndOfType

};



enum EventLink
	{
		linkEvent_None = 0,		///< ����

		linkEvent_SystemReset,		///< �ý��� ����

		linkEvent_Initialized,		///< ��ġ �ʱ�ȭ �Ϸ�

		linkEvent_Scanning,		///< ��ġ �˻� ����
		linkEvent_ScanStop,		///< ��ġ �˻� �ߴ�

		linkEvent_FoundDroneService,	///< ��� ���� �˻� �Ϸ�

		linkEvent_Connecting,		///< ��ġ ���� ����		
		linkEvent_Connected,		///< ��ġ ����

		linkEvent_ConnectionFaild,		///< ���� ����
		linkEvent_ConnectionFaildNoDevices,	///< ���� ���� - ��ġ�� ����
		linkEvent_ConnectionFaildNotReady,	///< ���� ���� - ��� ���°� �ƴ�

		linkEvent_PairingStart,		///< �� ����
		linkEvent_PairingSuccess,		///< �� ����
		linkEvent_PairingFaild,		///< �� ����

		linkEvent_BondingSuccess,		///< Bonding ����

		linkEvent_LookupAttribute,		///< ��ġ ���� �� �Ӽ� �˻�(GATT Event ����)

		linkEvent_RssiPollingStart,		///< RSSI Ǯ�� ����
		linkEvent_RssiPollingStop,		///< RSSI Ǯ�� ����

		linkEvent_DiscoverService,			///< ���� �˻�
		linkEvent_DiscoverCharacteristic,		///< �Ӽ� �˻�
		linkEvent_DiscoverCharacteristicDroneData,	///< �Ӽ� �˻�
		linkEvent_DiscoverCharacteristicDroneConfig,	///< �Ӽ� �˻�
		linkEvent_DiscoverCharacteristicUnknown,	///< �Ӽ� �˻�
		linkEvent_DiscoverCCCD,			///< CCCD �˻�

		linkEvent_ReadyToControl,		///< ���� �غ� �Ϸ�

		linkEvent_Disconnecting,		///< ��ġ ���� ���� ����
		linkEvent_Disconnected,		///< ��ġ ���� ���� �Ϸ�

		linkEvent_GapLinkParamUpdate,	///< GAP_LINK_PARAM_UPDATE_EVENT

		linkEvent_RspReadError,		///< RSP �б� ����
		linkEvent_RspReadSuccess,		///< RSP �б� ����

		linkEvent_RspWriteError,		///< RSP ���� ����
		linkEvent_RspWriteSuccess,		///< RSP ���� ����

		linkEvent_SetNotify,		///< Notify Ȱ��ȭ

		linkEvent_Write,			///< ������ ���� �̺�Ʈ

		EndOfType
	};







/***********************************************************************/

enum DataType
{
	dType_None = 0, ///< ����
	// �ý��� ����
	dType_Ping, ///< ��� Ȯ��(reserve)
	dType_Ack, ///< ������ ���ſ� ���� ����
	dType_Error, ///< ����(reserve, ��Ʈ �÷��״� ���Ŀ� ����)
	dType_Request, ///< ������ Ÿ���� ������ ��û
	dType_DeviceName, ///< ��ġ�� �̸� ����
	// ����, ���
	dType_Control = 0x10, ///< ����
	dType_Command, ///< ���
	dType_Command2, ///< ���� ���(2���� ������ ���ÿ� ����)
	DType_Command3, ///< ���� ���(3���� ������ ���ÿ� ����)
	// LED
	dType_LedMode = 0x20, ///< LED ��� ����
	dType_LedMode2, ///< LED ��� 2�� ����
	dType_LedModeCommand, ///< LED ���, Ŀ�ǵ�
	dType_LedModeCommandIr, ///< LED ���, Ŀ�ǵ�, IR ������ �۽�
	dType_LedModeColor, ///< LED ��� 3�� ���� ����
	dType_LedModeColor2, ///< LED ��� 3�� ���� ���� 2��
	dType_LedEvent, ///< LED �̺�Ʈ
	dType_LedEvent2, ///< LED �̺�Ʈ 2��,
	dType_LedEventCommand, ///< LED �̺�Ʈ, Ŀ�ǵ�
	dType_LedEventCommandIr, ///< LED �̺�Ʈ, Ŀ�ǵ�, IR ������ �۽�
	dType_LedEventColor, ///< LED �̺�Ʈ 3�� ���� ����
	dType_LedEventColor2, ///< LED �̺�Ʈ 3�� ���� ���� 2��
	// ����
	dType_Address = 0x30, ///< IEEE address
	dType_State, ///< ����� ����(���� ���, ��������, ���͸���)
	dType_Attitude, ///< ����� �ڼ�(Vector)
	dType_Trim, ///< Ʈ��(Vector)
	dType_GyroBias, ///< ���̷� ���̾ ��(Vector)
	dType_AdjustControl, ///< ����� ����(����+����)
	dType_AdjustFlightControl, ///< ���� ����� ����
	dType_AdjustDriveControl, ///< ���� ����� ����
	// ������ �ۼ���
	dType_IrMessage = 0x40, ///< IR ������ �ۼ���
	// �߿��� ������Ʈ
	dType_UpdateInformationMain = 0x50, ///< ���� �߿��� ����
	dType_UpdateInformationSub, ///< ���� �߿��� ����
	dType_UpdateRequestMain, ///< ���� �߿��� ������Ʈ ��û
	dType_UpdateRequestSub, ///< ���� �߿��� ������Ʈ ��û
	dType_UpdateMain, ///< ���� �߿��� ������Ʈ ������
	dType_UpdateSub, ///< ���� �߿��� ������Ʈ ������
	
	//Ȯ��
	/*
	dType_LinkState = 0xC0, ///< ��ũ ����� ����
	dType_LinkRssi, ///< ����� ��ġ�� RSSI��
	dType_StringMessage, ///< ���ڿ� �޼���
	dType_DiscoveredDevice, ///< �˻��� ��ġ
	*/
	dType_LinkState = 0xC0,		///< ��ũ ����� ����
	dType_LinkEvent,		///< ��ũ ����� �̺�Ʈ
	dType_LinkEventAddress,		///< ��ũ ����� �̺�Ʈ + �ּ�
	dType_LinkRssi,			///< ��ũ�� ����� ��ġ�� RSSI��
	dType_LinkDiscoveredDevice,	///< �˻��� ��ġ
	dType_LinkPasscode,          	///< ������ ��� ��ġ�� ��ȣ ����

	dType_StringMessage = 0xD0, 	///< ���ڿ� �޼���
	dType_EndOfType
};



/***********************************************************************/

enum CommandType
{
	cType_None = 0, 		///< �̺�Ʈ ����
	// ����
	cType_ModeDrone = 0x10, 	///< ��� ���� ��� ��ȯ
	// ����
	cType_Coordinate = 0x20, 	///< ���� ���� ����
	cType_Trim, ///< Ʈ�� ����
	cType_FlightEvent, ///< ���� �̺�Ʈ ����
	cType_DriveEvent, ///< ���� �̺�Ʈ ����
	cType_Stop, ///< ����
	cType_ResetHeading = 0x50, ///< ������ ����(�ۼַ�Ʈ ��� �� �� ���� heading�� 0���� ����)
	cType_ClearGyroBiasAndTrim, ///< ���̷� ���̾�� Ʈ�� ���� �ʱ�ȭ
	// ���
	cType_PairingActivate = 0x80, ///< �� Ȱ��ȭ
	cType_PairingDeactivate, ///< �� ��Ȱ��ȭ
	cType_TerminateConnection, ///< ���� ����
	// ��û
	cType_Request = 0x90, ///< ������ Ÿ���� ������ ��û
	
	//Ȯ��
	//cType_Tester = 0xE0 ,
	cType_SystemReset  = 0xE0, ///< �ý��� �����
	cType_DiscoverStart, ///< ��ġ �˻� ����
	cType_DiscoverStop, ///< ��ġ �˻� �ߴ�
	cType_Connect, ///< ����
	cType_Disconnect, ///< ���� ����
	cType_RssiPollingStart, ///< ����� ��ġ�� RSSI �� ���� ����
	cType_RssiPollingStop, ///< ����� ��ġ�� RSSI �� ���� �ߴ�	
	cType_EndOfType
};

/***********************************************************************/
enum ModeDrone
{
	dMode_None = 0, ///< ����
	dMode_Flight = 0x10, ///< ���� ���(���� ����)
	dMode_FlightNoGuard, ///< ���� ���(���� ����)
	dMode_FlightFPV, ///< ���� ���(FPV)
	dMode_Drive = 0x20, ///< ���� ���
	dMode_DriveFPV, ///< ���� ���(FPV)
	dMode_Test = 0x30, ///< �׽�Ʈ ���
	dMode_EndOfType
};

/***********************************************************************/

enum Coordinate
{
	cSet_None = 0, ///< ����
	cSet_Absolute, ///< ���� ��ǥ��
	cSet_Relative, ///< ��� ��ǥ��
	cSet_EndOfType
};

/***********************************************************************/

enum Trim
{
	trim_None = 0, ///< ����
	trim_RollIncrease, ///< Roll ����
	trim_RollDecrease, ///< Roll ����
	trim_PitchIncrease, ///< Pitch ����
	trim_PitchDecrease, ///< Pitch ����
	trim_YawIncrease, ///< Yaw ����
	trim_YawDecrease, ///< Yaw ����
	trim_ThrottleIncrease, ///< Throttle ����
	trim_ThrottleDecrease, ///< Throttle ����
	trim_EndOfType
};

/***********************************************************************/

enum FlightEvent
{
	fEvent_None = 0, ///< ����
	fEvent_TakeOff, ///< �̷�
	fEvent_FlipFront, ///< ȸ��
	fEvent_FlipRear, ///< ȸ��
	fEvent_flipLeft, ///< ȸ��
	fEvent_FlipRight, ///< ȸ��
	fEvent_Stop, ///< ����
	fEvent_Landing, ///< ����
	fEvent_TurnOver, ///< ������
	fEvent_Shot, ///< �̻����� �� �� ������
	fEvent_UnderAttack, ///< �̻����� ���� �� ������
	fEvent_Square, ///< ������ ����
	fEvent_CircleLeft, ///< �������� ȸ��
	fEvent_CircleRight, ///< ���������� ȸ��
	fEvent_Rotate180,
	fEvent_EndOfType
};

/***********************************************************************/
enum Request
{
// ����
	Req_Address = 0x30, ///< IEEE address
	Req_State, ///< ����� ����(���� ���, ��������, ���͸���)
	Req_Attitude, ///< ����� �ڼ�(Vector)
	Req_Trim, ///< Ʈ��(Vector)
	Req_GyroBias, ///< ���̷� ���̾ ��(Vector)
	Req_AdjustControl, ///< ����� ����(����+����)
	Req_AdjustFlightControl, ///< ���� ����� ����
	Req_AdjustDriveControl, ///< ���� ����� ����
	// �߿��� ������Ʈ
	Req_UpdateInformationMain = 0x50, ///< ���� �߿��� ����
	Req_UpdateInformationSub, ///< ���� �߿��� ����
};


/***********************************************************************/
enum ModeLight
{
  Light_None,
  WaitingForConnect, ///< ���� ��� ����
  Connected,
  
  EyeNone = 0x10,
  EyeHold, ///< ������ ������ ��� ��
  EyeMix, ///< ���������� LED �� ����
  EyeFlicker, ///< ������
  EyeFlickerDouble, ///< ������(�� �� �����̰� ������ �ð���ŭ ����)
  EyeDimming, ///< ��� �����Ͽ� õõ�� ������
  
  ArmNone = 0x40,
  ArmHold, ///< ������ ������ ��� ��
  ArmMix, ///< ���������� LED �� ����
  ArmFlicker, ///< ������
  ArmFlickerDouble, ///< ������(�� �� �����̰� ������ �ð���ŭ ����)
  ArmDimming, ///< ��� �����Ͽ� õõ�� ������
  ArmFlow, ///< �տ��� �ڷ� �帧
  ArmFlowReverse, ///< �ڿ��� ������ �帧
  EndOfLedMode
};

enum Colors
{
	AliceBlue, AntiqueWhite, Aqua,
	Aquamarine, Azure, Beige,
	Bisque, Black, BlanchedAlmond,
	Blue, BlueViolet, Brown,
	BurlyWood, CadetBlue, Chartreuse,
	Chocolate, Coral, CornflowerBlue,
	Cornsilk, Crimson, Cyan,
	DarkBlue, DarkCyan, DarkGoldenRod,
	DarkGray, DarkGreen, DarkKhaki,
	DarkMagenta, DarkOliveGreen, DarkOrange,
	DarkOrchid, DarkRed, DarkSalmon,
	DarkSeaGreen, DarkSlateBlue, DarkSlateGray,
	DarkTurquoise, DarkViolet, DeepPink,
	DeepSkyBlue, DimGray, DodgerBlue,
	FireBrick, FloralWhite, ForestGreen,
	Fuchsia, Gainsboro, GhostWhite,
	Gold, GoldenRod, Gray,
	Green, GreenYellow, HoneyDew,
	HotPink, IndianRed, Indigo,
	Ivory, Khaki, Lavender,
	LavenderBlush, LawnGreen, LemonChiffon,
	LightBlue, LightCoral, LightCyan,
	LightGoldenRodYellow, LightGray, LightGreen,
	LightPink, LightSalmon, LightSeaGreen,
	LightSkyBlue, LightSlateGray, LightSteelBlue,
	LightYellow, Lime, LimeGreen,
	Linen, Magenta, Maroon,
	MediumAquaMarine, MediumBlue, MediumOrchid,
	MediumPurple, MediumSeaGreen, MediumSlateBlue,
	MediumSpringGreen, MediumTurquoise, MediumVioletRed,
	MidnightBlue, MintCream, MistyRose,
	Moccasin, NavajoWhite, Navy,
	OldLace, Olive, OliveDrab,
	Orange, OrangeRed, Orchid,
	PaleGoldenRod, PaleGreen, PaleTurquoise,
	PaleVioletRed, PapayaWhip, PeachPuff,
	Peru, Pink, Plum,
	PowderBlue, Purple, RebeccaPurple,
	Red, RosyBrown, RoyalBlue,
	SaddleBrown, Salmon, SandyBrown,
	SeaGreen, SeaShell, Sienna,
	Silver, SkyBlue, SlateBlue,
	SlateGray, Snow, SpringGreen,
	SteelBlue, Tan, Teal,
	Thistle, Tomato, Turquoise,
	Violet, Wheat, White,
	WhiteSmoke, Yellow, YellowGreen,
	EndOfColor
};




/***********************************************************************/
/***********************************************************************/

struct CommandBase
{
	u8 commandType; ///< ��� Ÿ��
	u8 option; ///< ��ɿ� ���� �ɼ�(System.h�� ������ ���� ���)
};


/***********************************************************************/
/***********************************************************************/
/***********************************************************************/

class CoDroneClass
{
public:

	void begin(void);
	
	unsigned short CRC16_Make(unsigned char *buf, int len); //CRC16-CCITT Format
	boolean CRC16_Check(unsigned char data[], int len, unsigned char crc[]);
	
	void Control();
	void Control(int interval);
	
	void Send_Processing(byte _data[], byte _length, byte _crc[]);
	
	
	
	
	
	void Send_LinkState();
	void Send_Discover(byte action);
	void Send_Connect(byte index);
	void Send_Disconnect();		
	void Send_RSSI_Polling(byte action);
	
		void Send_Ping();
	
	void Send_DroneMode(byte event);
	void Send_Coordinate(byte mode);
	void Send_Trim(byte event);
	
	void FlightEvent(byte event);
	void Send_ResetHeading();	
	void Send_Command(int sendCommand, int sendOption);
	
		/////////////////////////////////
		

	void AutoConnect(byte mode);
  void AutoConnect(byte mode, byte address[]);	
	void Send_ConnectAddressInputDrone(byte address[]);
	void Send_ConnectConnectedDrone();
	void Send_ConnectNearbyDrone();
	
	
	void LedColor(byte sendMode, byte sendColor, byte sendInterval);
	void LedColor(byte sendMode, byte r, byte g, byte b, byte sendInterval);
	void LedColor(byte sendMode, byte sendColor[], byte sendInterval);
	
	void LedEvent(byte sendMode, byte sendColor, byte sendInterval, byte sendRepeat);
	void LedEvent(byte sendMode, byte sendColor[], byte sendInterval, byte sendRepeat);
	void LedEvent(byte sendMode, byte r, byte g, byte b, byte sendInterval, byte sendRepeat);
					
	void LinkReset();
	
	
			/////////////////////////////////
	
	
	void PrintDroneAddress();

	void LinkStateCheck();
	void ReceiveEventCheck();
	void StartLED();
	void ConnectLED();
	/////////////////////////////////

	byte cmdBuff[MAX_PACKET_LENGTH];
	byte dataBuff[MAX_PACKET_LENGTH];
	byte crcBuff[2];
	int cmdIndex;
	byte checkHeader;
	int receiveDtype;
	int receiveOption;
	int receiveLength;
	int receiveEventState;
	int receiveLinkState;
	int receiveComplete;
	int receiveCRC;


int discoverFlag;
int connectFlag;



byte displayLED = 0;


/////////////////////////////////
	
	void DisplayAddress(byte count);
	

	
//*****************************************/

	byte devCount = 0;
	byte devFind[3];
	
	int devRSSi0 = -1;
	int devRSSi1 = -1;
	int devRSSi2 = -1;
	
	byte devAddress0[6];
	byte devAddress1[6];
	byte devAddress2[6];
	
	byte devAddressBuf[6];
	byte devAddressConnected[6];
	
	boolean pairing = 0;
//*****************************************/
	
	/////////////////////////////////


	//void IntervalSend(int interval, int8_t _throttle, int8_t _yaw, int8_t _roll, int8_t _pitch);
	void ButtonPreesHoldWait(int button);
	void ButtonPreesHoldWait(int button1, int button2);
	
	/////////////////////////////////


	//////////////////////////////////
	void ReadSensor(void);
	void PrintSensor(void);
	int AnalogScaleChange(int analogValue);			
	//////////////////////////////////
	void LED(int command);
	void Blink(int time, int count);	
	boolean TimeCheck(word interval); //milliseconds
	boolean TimeOutConnetionCheck(word interval); //milliseconds	
	//////////////////////////////////
	int roll;
	int pitch;
	int yaw;
	int throttle;
	int event;		
	int type;
	int SendInterval; //millis seconds	
	int state;	
	int analogOffset;
	
	//////////////////////////////////
	void Receive(void);
	//byte cmdBuff[MAX_CMD_LENGTH];
	//int cmdIndex;
	//boolean checkHeader;


private:
	byte packet[9];
	long PreviousMillis;
	long timeOutConnectionPreviousMillis;
};

extern CoDroneClass CoDrone;

#endif 