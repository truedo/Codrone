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
#define START1    	0x0A
#define START2   		0x55

//////////////////////////////////////////////////////////////////////////////////

#define OFF					0x00
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
	linkMode_None = 0,	 	 ///< 없음
	linkMode_Boot,	 	 	 ///< 부팅 	 	
	linkMode_Ready,	 		 ///< 대기(연결 전)
	linkMode_Connecting,	 	 ///< 장치 연결 중
	linkMode_Connected,	 	 ///< 장치 연결 완료
	linkMode_Disconnecting,	 	 ///< 장치 연결 해제 중
	linkMode_ReadyToReset,	 	 ///< 리셋 대기(1초 뒤에 장치 리셋)	
	linkMode_EndOfType

};


enum EventLink
	{
		linkEvent_None = 0,		///< 없음

		linkEvent_SystemReset,		///< 시스템 리셋

		linkEvent_Initialized,		///< 장치 초기화 완료

		linkEvent_Scanning,		///< 장치 검색 시작
		linkEvent_ScanStop,		///< 장치 검색 중단

		linkEvent_FoundDroneService,	///< 드론 서비스 검색 완료

		linkEvent_Connecting,		///< 장치 연결 시작		
		linkEvent_Connected,		///< 장치 연결

		linkEvent_ConnectionFaild,		///< 연결 실패
		linkEvent_ConnectionFaildNoDevices,	///< 연결 실패 - 장치가 없음
		linkEvent_ConnectionFaildNotReady,	///< 연결 실패 - 대기 상태가 아님

		linkEvent_PairingStart,		///< 페어링 시작
		linkEvent_PairingSuccess,		///< 페어링 성공
		linkEvent_PairingFaild,		///< 페어링 실패

		linkEvent_BondingSuccess,		///< Bonding 성공

		linkEvent_LookupAttribute,		///< 장치 서비스 및 속성 검색(GATT Event 실행)

		linkEvent_RssiPollingStart,		///< RSSI 풀링 시작
		linkEvent_RssiPollingStop,		///< RSSI 풀링 중지

		linkEvent_DiscoverService,			///< 서비스 검색
		linkEvent_DiscoverCharacteristic,		///< 속성 검색
		linkEvent_DiscoverCharacteristicDroneData,	///< 속성 검색
		linkEvent_DiscoverCharacteristicDroneConfig,	///< 속성 검색
		linkEvent_DiscoverCharacteristicUnknown,	///< 속성 검색
		linkEvent_DiscoverCCCD,			///< CCCD 검색

		linkEvent_ReadyToControl,		///< 제어 준비 완료

		linkEvent_Disconnecting,		///< 장치 연결 해제 시작
		linkEvent_Disconnected,		///< 장치 연결 해제 완료

		linkEvent_GapLinkParamUpdate,	///< GAP_LINK_PARAM_UPDATE_EVENT

		linkEvent_RspReadError,		///< RSP 읽기 오류
		linkEvent_RspReadSuccess,		///< RSP 읽기 성공

		linkEvent_RspWriteError,		///< RSP 쓰기 오류
		linkEvent_RspWriteSuccess,		///< RSP 쓰기 성공

		linkEvent_SetNotify,		///< Notify 활성화

		linkEvent_Write,			///< 데이터 쓰기 이벤트

		EndOfType
	};



/***********************************************************************/

enum DataType
{
	dType_None = 0, ///< 없음
	// 시스템 정보
	dType_Ping, ///< 통신 확인(reserve)
	dType_Ack, ///< 데이터 수신에 대한 응답
	dType_Error, ///< 오류(reserve, 비트 플래그는 추후에 지정)
	dType_Request, ///< 지정한 타입의 데이터 요청
	dType_DeviceName, ///< 장치의 이름 변경
	// 조종, 명령
	dType_Control = 0x10, ///< 조종
	dType_Command, ///< 명령
	dType_Command2, ///< 다중 명령(2가지 설정을 동시에 변경)
	DType_Command3, ///< 다중 명령(3가지 설정을 동시에 변경)
	// LED
	dType_LedMode = 0x20, ///< LED 모드 지정
	dType_LedMode2, ///< LED 모드 2개 지정
	dType_LedModeCommand, ///< LED 모드, 커맨드
	dType_LedModeCommandIr, ///< LED 모드, 커맨드, IR 데이터 송신
	dType_LedModeColor, ///< LED 모드 3색 직접 지정
	dType_LedModeColor2, ///< LED 모드 3색 직접 지정 2개
	dType_LedEvent, ///< LED 이벤트
	dType_LedEvent2, ///< LED 이벤트 2개,
	dType_LedEventCommand, ///< LED 이벤트, 커맨드
	dType_LedEventCommandIr, ///< LED 이벤트, 커맨드, IR 데이터 송신
	dType_LedEventColor, ///< LED 이벤트 3색 직접 지정
	dType_LedEventColor2, ///< LED 이벤트 3색 직접 지정 2개
	// 상태
	dType_Address = 0x30, ///< IEEE address
	dType_State, ///< 드론의 상태(비행 모드, 방위기준, 배터리량)
	dType_Attitude, ///< 드론의 자세(Vector)
	dType_Trim, ///< 트림(Vector)
	dType_GyroBias, ///< 자이로 바이어스 값(Vector)
	dType_AdjustControl, ///< 제어기 조정(비행+주행)
	dType_AdjustFlightControl, ///< 비행 제어기 조정
	dType_AdjustDriveControl, ///< 주행 제어기 조정
	// 데이터 송수신
	dType_IrMessage = 0x40, ///< IR 데이터 송수신
	// 펌웨어 업데이트
	dType_UpdateInformationMain = 0x50, ///< 메인 펌웨어 정보
	dType_UpdateInformationSub, ///< 서브 펌웨어 정보
	dType_UpdateRequestMain, ///< 메인 펌웨어 업데이트 요청
	dType_UpdateRequestSub, ///< 서브 펌웨어 업데이트 요청
	dType_UpdateMain, ///< 메인 펌웨어 업데이트 데이터
	dType_UpdateSub, ///< 서브 펌웨어 업데이트 데이터
	
	//확장
	/*
	dType_LinkState = 0xC0, ///< 링크 모듈의 상태
	dType_LinkRssi, ///< 연결된 장치의 RSSI값
	dType_StringMessage, ///< 문자열 메세지
	dType_DiscoveredDevice, ///< 검색된 장치
	*/
	dType_LinkState = 0xC0,		///< 링크 모듈의 상태
	dType_LinkEvent,		///< 링크 모듈의 이벤트
	dType_LinkEventAddress,		///< 링크 모듈의 이벤트 + 주소
	dType_LinkRssi,			///< 링크와 연결된 장치의 RSSI값
	dType_LinkDiscoveredDevice,	///< 검색된 장치
	dType_LinkPasscode,          	///< 연결할 대상 장치의 암호 지정

	dType_StringMessage = 0xD0, 	///< 문자열 메세지
	dType_EndOfType
};


/***********************************************************************/

enum CommandType
{
	cType_None = 0, 		///< 이벤트 없음
	// 설정
	cType_ModeDrone = 0x10, 	///< 드론 동작 모드 전환
	// 제어
	cType_Coordinate = 0x20, 	///< 방위 기준 변경
	cType_Trim, ///< 트림 변경
	cType_FlightEvent, ///< 비행 이벤트 실행
	cType_DriveEvent, ///< 주행 이벤트 실행
	cType_Stop, ///< 정지
	cType_ResetHeading = 0x50, ///< 방향을 리셋(앱솔루트 모드 일 때 현재 heading을 0도로 변경)
	cType_ClearGyroBiasAndTrim, ///< 자이로 바이어스와 트림 설정 초기화
	// 통신
	cType_PairingActivate = 0x80, ///< 페어링 활성화
	cType_PairingDeactivate, ///< 페어링 비활성화
	cType_TerminateConnection, ///< 연결 종료
	// 요청
	cType_Request = 0x90, ///< 지정한 타입의 데이터 요청
	
	//확장
	//cType_Tester = 0xE0 ,
	cType_SystemReset  = 0xE0, ///< 시스템 재시작
	cType_DiscoverStart, ///< 장치 검색 시작
	cType_DiscoverStop, ///< 장치 검색 중단
	cType_Connect, ///< 연결
	cType_Disconnect, ///< 연결 끊기
	cType_RssiPollingStart, ///< 연결된 장치의 RSSI 값 수집 시작
	cType_RssiPollingStop, ///< 연결된 장치의 RSSI 값 수집 중단	
	cType_EndOfType
};

/***********************************************************************/
enum ModeDrone
{
	dMode_None = 0, ///< 없음
	dMode_Flight = 0x10, ///< 비행 모드(가드 포함)
	dMode_FlightNoGuard, ///< 비행 모드(가드 없음)
	dMode_FlightFPV, ///< 비행 모드(FPV)
	dMode_Drive = 0x20, ///< 주행 모드
	dMode_DriveFPV, ///< 주행 모드(FPV)
	dMode_Test = 0x30, ///< 테스트 모드
	dMode_EndOfType
};

/***********************************************************************/

enum Coordinate
{
	cSet_None = 0, ///< 없음
	cSet_Absolute, ///< 고정 좌표계
	cSet_Relative, ///< 상대 좌표계
	cSet_EndOfType
};

/***********************************************************************/

enum Trim
{
	trim_None = 0, ///< 없음
	trim_RollIncrease, ///< Roll 증가
	trim_RollDecrease, ///< Roll 감소
	trim_PitchIncrease, ///< Pitch 증가
	trim_PitchDecrease, ///< Pitch 감소
	trim_YawIncrease, ///< Yaw 증가
	trim_YawDecrease, ///< Yaw 감소
	trim_ThrottleIncrease, ///< Throttle 증가
	trim_ThrottleDecrease, ///< Throttle 감소
	trim_EndOfType
};

/***********************************************************************/

enum FlightEvent
{
	fEvent_None = 0, ///< 없음
	fEvent_TakeOff, ///< 이륙
	fEvent_FlipFront, ///< 회전
	fEvent_FlipRear, ///< 회전
	fEvent_flipLeft, ///< 회전
	fEvent_FlipRight, ///< 회전
	fEvent_Stop, ///< 정지
	fEvent_Landing, ///< 착륙
	fEvent_TurnOver, ///< 뒤집기
	fEvent_Shot, ///< 미사일을 쏠 때 움직임
	fEvent_UnderAttack, ///< 미사일을 맞을 때 움직임
	fEvent_Square, ///< 정방향 돌기
	fEvent_CircleLeft, ///< 왼쪽으로 회전
	fEvent_CircleRight, ///< 오른쪽으로 회전
	fEvent_Rotate180,
	fEvent_EndOfType
};

/***********************************************************************/
enum Request
{
// 상태
	Req_Address = 0x30, ///< IEEE address
	Req_State, ///< 드론의 상태(비행 모드, 방위기준, 배터리량)
	Req_Attitude, ///< 드론의 자세(Vector)
	Req_Trim, ///< 트림(Vector)
	Req_GyroBias, ///< 자이로 바이어스 값(Vector)
	Req_AdjustControl, ///< 제어기 조정(비행+주행)
	Req_AdjustFlightControl, ///< 비행 제어기 조정
	Req_AdjustDriveControl, ///< 주행 제어기 조정
	// 펌웨어 업데이트
	Req_UpdateInformationMain = 0x50, ///< 메인 펌웨어 정보
	Req_UpdateInformationSub, ///< 서브 펌웨어 정보
};


/***********************************************************************/
enum ModeLight
{
  Light_None,
  WaitingForConnect, ///< 연결 대기 상태
  Connected,
  
  EyeNone = 0x10,
  EyeHold, ///< 지정한 색상을 계속 켬
  EyeMix, ///< 순차적으로 LED 색 변경
  EyeFlicker, ///< 깜빡임
  EyeFlickerDouble, ///< 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)
  EyeDimming, ///< 밝기 제어하여 천천히 깜빡임
  
  ArmNone = 0x40,
  ArmHold, ///< 지정한 색상을 계속 켬
  ArmMix, ///< 순차적으로 LED 색 변경
  ArmFlicker, ///< 깜빡임
  ArmFlickerDouble, ///< 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)
  ArmDimming, ///< 밝기 제어하여 천천히 깜빡임
  ArmFlow, ///< 앞에서 뒤로 흐름
  ArmFlowReverse, ///< 뒤에서 앞으로 흐름
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
	void Receive(void);

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
		
	void DisplayAddress(byte count);
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
	
	void ButtonPreesHoldWait(int button);
	void ButtonPreesHoldWait(int button1, int button2);	
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
	

	
private:
	byte packet[9];
	long PreviousMillis;
	long timeOutConnectionPreviousMillis;
};

extern CoDroneClass CoDrone;

#endif 