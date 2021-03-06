/*
	CoDrone.h - CoDrone library
	Copyright (C) 2014 RoboLink.  All rights reserved.
	LastUpdate : 2018-04-27
*/

#ifndef CoDrone_h
#define CoDrone_h
#include "Arduino.h"
#include <avr/interrupt.h>

/***********************************************************************/
////////////////////////Serial Select////////////////////////////////////
/***********************************************************************/
#if defined(UBRRH) || defined(UBRR0H)
#define FIND_HWSERIAL0
#endif

#if defined(__AVR_ATmega128__)
#define DEBUG_MODE_ENABLE
#endif

//#if defined(__AVR_ATmega128__)
#if defined(UBRR1H)
#define FIND_HWSERIAL1
#endif

#if defined (FIND_HWSERIAL1)	//Serial Other Setting	- two serial
#define DRONE_SERIAL	Serial1		//drone serial
#define DEBUG_SERIAL	Serial		//debug serial1

#else							//Serial Smart Setting	- one serial
#define DRONE_SERIAL		Serial		//drone serial
#define DEBUG_SERIAL		Serial1		//debug serial1

#endif


/***********************************************************************/
////////////////////////////HEADER///////////////////////////////////////
/***********************************************************************/
//START CODE
#define START1			0x0A
#define START2			0x55

/***********************************************************************/

//serial buffer
#if defined (FIND_HWSERIAL1)	// Atmega128
#define MAX_PACKET_LENGTH			200

#else													// Smart Setting
#define MAX_PACKET_LENGTH			40
#endif

/***********************************************************************/

//#define	SEND_CHECK_TIME			3
#define	SEND_CHECK_TIME				50

/***********************************************************************/

#define ROLL							CoDrone.roll
#define PITCH							CoDrone.pitch
#define YAW								CoDrone.yaw
#define THROTTLE					CoDrone.throttle

#define STATE							CoDrone.state
#define SEND_INTERVAL			CoDrone.SendInterval
#define ANALOG_OFFSET			CoDrone.analogOffset
#define BATTERY						CoDrone.battery
#define RSSI							CoDrone.rssi

#define AttitudeROLL			CoDrone.attitudeRoll
#define AttitudePITCH			CoDrone.attitudePitch
#define AttitudeYAW				CoDrone.attitudeYaw

/***********************************************************************/

#define DiscoverStop			cType_LinkDiscoverStop
#define DiscoverStart			cType_LinkDiscoverStart

//#define PollingStop				cType_LinkRssiPollingStop
//#define PollingStart			cType_LinkRssiPollingStart

#define	PAIRING						CoDrone.pairing

#define LinkModeMute 			LinkBroadcast_Mute
#define LinkModeActive		LinkBroadcast_Active
#define LinkModePassive 	LinkBroadcast_Passive

#define	NearbyDrone				1
#define	ConnectedDrone		2
#define AddressInputDrone	3

//eeprom address
#define	EEP_AddressCheck	10
#define	EEP_AddressFirst	11
#define	EEP_AddressEnd		15

//------------------------------------------------------------------------------------//
#define FREE_PLAY					0
#define TEAM_RED					1
#define TEAM_BLUE					2
#define TEAM_GREEN				3
#define TEAM_YELLOW				4
#define MAX_ENERGY				8

/**********************	IR DATA ****************************************/

#define FREE_MISSILE			0xaa01
#define RED_MISSILE				0xbb01
#define BLUE_MISSILE			0xcc01
#define GREEN_MISSILE			0xdd01
#define YELLOW_MISSILE		0xee01

/***********************************************************************/

#define Flight 						dMode_Flight
#define FlightNoGuard			dMode_FlightNoGuard
#define FlightFPV					dMode_FlightFPV
#define Drive 						dMode_Drive
#define DriveFPV					dMode_DriveFPV

#define Absolute 					cSet_Absolute
#define Relative					cSet_Relative

#define TakeOff 					fEvent_TakeOff
#define FlipFront					fEvent_FlipFront
#define FlipRear					fEvent_FlipRear
#define FlipLeft					fEvent_flipLeft
#define FlipRight					fEvent_FlipRight
#define Stop							fEvent_Stop
#define Landing						fEvent_Landing
#define TurnOver					fEvent_TurnOver
#define Shot							fEvent_Shot
#define UnderAttack				fEvent_UnderAttack
#define Square						fEvent_Square
#define CircleLeft				fEvent_CircleLeft
#define CircleRight				fEvent_CircleRight
#define Rotate180					fEvent_Rotate180

#define RollIncrease			trim_RollIncrease
#define RollDecrease			trim_RollDecrease
#define PitchIncrease			trim_PitchIncrease
#define PitchDecrease			trim_PitchDecrease
#define YawIncrease				trim_YawIncrease
#define YawDecrease				trim_YawDecrease
#define ThrottleIncrease	trim_ThrottleIncrease
#define ThrottleDecrease	trim_ThrottleDecrease

//-------------------------------------------------------------------------------------//

#define	Address					Req_Address					///< IEEE address
#define	Attitude				Req_Attitude

typedef struct gyrodata
{
	int roll;
	int pitch;
	int yaw;
}gyrodata;

typedef struct acceldata
{
	int x;
	int y;
	int z;
}acceldata;

typedef struct optdata
{
	int x;
	int y;
}optdata;


typedef struct trimdata
{
	int roll;
	int pitch;
	int yaw;
	int throttle;
}trimdata;



/***********************************************************************/

#define LED_DISPLAY_START				0
#define LED_DISPLAY_PORTC				1
#define LED_DISPLAY_DDRC				2
#define LED_DISPLAY_STANDARD		3
#define LED_DISPLAY_MOVE_SLIDE	4
#define LED_DISPLAY_CONNECT			5
#define LED_DISPLAY_BLINK				6
#define LED_DISPLAY_MOVE_RADER	7

/***********************************************************************/
/////////////////////////LINK MODULE/////////////////////////////////////
/***********************************************************************/
enum ModeLink
{
	linkMode_None = 0,			///< 없음
	linkMode_Boot,					///< 부팅
	linkMode_Ready,					///< 대기(연결 전)
	linkMode_Connecting,		///< 장치 연결 중
	linkMode_Connected,			///< 장치 연결 완료
	linkMode_Disconnecting,	///< 장치 연결 해제 중
	linkMode_ReadyToReset,	///< 리셋 대기(1초 뒤에 장치 리셋)
	linkMode_EndOfType
};

enum ModeLinkBroadcast
{
	LinkBroadcast_None = 0,		///< 없음
	LinkBroadcast_Mute,				///< LINK 모듈 데이터 송신 중단 . 아두이노 펌웨어 다운로드
	LinkBroadcast_Active,			///< 페트론 연결 모드 . 모드 전환 메세지 전송
	LinkBroadcast_Passive,		///< 페트론 연결 모드 . 모드 전환 메세지 전송하지 않음
	LinkBroadcast_EndOfType
};

enum EventLink
{
	linkEvent_None = 0,														///< 없음
	linkEvent_SystemReset,												///< 시스템 리셋
	linkEvent_Initialized,												///< 장치 초기화 완료
	linkEvent_Scanning,														///< 장치 검색 시작
	linkEvent_ScanStop,														///< 장치 검색 중단
	linkEvent_FoundDroneService,									///< 드론 서비스 검색 완료
	linkEvent_Connecting,													///< 장치 연결 시작
	linkEvent_Connected,													///< 장치 연결
	linkEvent_ConnectionFaild,										///< 연결 실패
	linkEvent_ConnectionFaildNoDevices,						///< 연결 실패 - 장치가 없음
	linkEvent_ConnectionFaildNotReady,						///< 연결 실패 - 대기 상태가 아님
	linkEvent_PairingStart,												///< 페어링 시작
	linkEvent_PairingSuccess,											///< 페어링 성공
	linkEvent_PairingFaild,												///< 페어링 실패
	linkEvent_BondingSuccess,											///< Bonding 성공
	linkEvent_LookupAttribute,										///< 장치 서비스 및 속성 검색(GATT Event 실행)
	linkEvent_RssiPollingStart,										///< RSSI 풀링 시작
	linkEvent_RssiPollingStop,										///< RSSI 풀링 중지
	linkEvent_DiscoverService,										///< 서비스 검색
	linkEvent_DiscoverCharacteristic,							///< 속성 검색
	linkEvent_DiscoverCharacteristicDroneData,		///< 속성 검색
	linkEvent_DiscoverCharacteristicDroneConfig,	///< 속성 검색
	linkEvent_DiscoverCharacteristicUnknown,			///< 속성 검색
	linkEvent_DiscoverCCCD,												///< CCCD 검색
	linkEvent_ReadyToControl,											///< 제어 준비 완료
	linkEvent_Disconnecting,											///< 장치 연결 해제 시작
	linkEvent_Disconnected,												///< 장치 연결 해제 완료
	linkEvent_GapLinkParamUpdate,									///< GAP_LINK_PARAM_UPDATE_EVENT
	linkEvent_RspReadError,												///< RSP 읽기 오류
	linkEvent_RspReadSuccess,											///< RSP 읽기 성공
	linkEvent_RspWriteError,											///< RSP 쓰기 오류
	linkEvent_RspWriteSuccess,										///< RSP 쓰기 성공
	linkEvent_SetNotify,													///< Notify 활성화
	linkEvent_Write,															///< 데이터 쓰기 이벤트
	EndOfType
};


/***********************************************************************/
//////////////////////////////DRONE/////////////////////////////////////
/***********************************************************************/
enum DataType
{
	dType_None = 0,									///< 없음
	// 시스템 정보
	dType_Ping,											///< 통신 확인(reserve)
	dType_Ack,											///< 데이터 수신에 대한 응답
	dType_Error,										///< 오류(reserve, 비트 플래그는 추후에 지정)
	dType_Request,									///< 지정한 타입의 데이터 요청
	dType_DeviceName,								///< 장치의 이름 변경
	// 조종, 명령
	dType_Control = 0x10,						///< 조종
	dType_Command,									///< 명령
	dType_Command2,									///< 다중 명령(2가지 설정을 동시에 변경)
	DType_Command3,									///< 다중 명령(3가지 설정을 동시에 변경)
	// LED
	dType_LedMode = 0x20,						///< LED 모드 지정
	dType_LedMode2,									///< LED 모드 2개 지정
	dType_LedModeCommand,						///< LED 모드, 커맨드
	dType_LedModeCommandIr,					///< LED 모드, 커맨드, IR 데이터 송신
	dType_LedModeColor,							///< LED 모드 3색 직접 지정
	dType_LedModeColor2,						///< LED 모드 3색 직접 지정 2개
	dType_LedEvent,									///< LED 이벤트
	dType_LedEvent2,								///< LED 이벤트 2개,
	dType_LedEventCommand,					///< LED 이벤트, 커맨드
	dType_LedEventCommandIr,				///< LED 이벤트, 커맨드, IR 데이터 송신
	dType_LedEventColor,						///< LED 이벤트 3색 직접 지정
	dType_LedEventColor2,						///< LED 이벤트 3색 직접 지정 2개
	dType_LedDefaultColor,					///< LED 초기 모드 3색 직접 지정
	dType_LedDefaultColor2,					///< LED 초기 모드 3색 직접 지정 2개
	// 상태
	dType_Address = 0x30,						///< IEEE address
	dType_State,										///< 드론의 상태(비행 모드, 방위기준, 배터리량)
	dType_Attitude,									///< 드론의 자세(Vector)
	dType_GyroBias,									///< 자이로 바이어스 값(Vector)
	dType_TrimAll,									///< 전체 트림 (비행+주행)�
	dType_TrimFlight,								///< 비행 트림
	dType_TrimDrive,								///< 주행 트림
	dType_CountFlight,							///< 비행 관련 카운트
	dType_CountDrive,								///< 주행 관련 카운트
	// 데이터 송수신
	dType_IrMessage = 0x40,					///< IR 데이터 송수신
	// 센서
	dType_ImuRawAndAngle = 0x50,		///< IMU Raw + Angle
	dType_Pressure,									///< 압력 센서 데이터
	dType_ImageFlow,								///< ImageFlow
	dType_Button,										///< 버튼 입력
	dType_Battery,									///< 배터리
	dType_Motor,										///< 모터 제어 및 현재 제어 값 확인
	dType_Temperature,							///< 온도
	dType_Range,										///< 거리 센서
	// 링크 보드
	dType_LinkState = 0xE0,					///< 링크 모듈의 상태
	dType_LinkEvent,								///< 링크 모듈의 이벤트
	dType_LinkEventAddress,					///< 링크 모듈의 이벤트 + 주소
	dType_LinkRssi,									///< 링크와 연결된 장치의 RSSI값
	dType_LinkDiscoveredDevice,			///< 검색된 장치
	dType_LinkPasscode,							///< 연결할 대상 장치의 암호 지정
	dType_StringMessage = 0xD0,			///< 문자열 메세지
	dType_EndOfType
};

/***********************************************************************/
enum CommandType
{
	cType_None = 0,									///< 이벤트 없음
	// 설정
	cType_ModeDrone = 0x10,					///< 드론 동작 모드 전환
	// 제어
	cType_Coordinate = 0x20,				///< 방위 기준 변경
	cType_Trim,											///< 트림 변경
	cType_FlightEvent,							///< 비행 이벤트 실행
	cType_DriveEvent,								///< 주행 이벤트 실행
	cType_Stop,											///< 정지
	cType_ResetHeading = 0x50,			///< 방향을 리셋(앱솔루트 모드 일 때 현재 heading을 0도로 변경)
	cType_ClearGyroBiasAndTrim,			///< 자이로 바이어스와 트림 설정 초기화
	cType_ClearTrim,								///< 트림 초기화
	// 통신
	cType_PairingActivate = 0x80,		///< 페어링 활성화
	cType_PairingDeactivate,				///< 페어링 비활성화
	cType_TerminateConnection,			///< 연결 종료
	// 요청
	cType_Request = 0x90,						///< 지정한 타입의 데이터 요청
	// 링크 보드
	cType_LinkModeBroadcast = 0xE0,	///< LINK 송수신 모드 전환
	cType_LinkSystemReset,					///< 시스템 재시작
	cType_LinkDiscoverStart,				///< 장치 검색 시작
	cType_LinkDiscoverStop,					///< 장치 검색 중단
	cType_LinkConnect,							///< 연결
	cType_LinkDisconnect,						///< 연결 해제
	cType_LinkRssiPollingStart,			///< RSSI 수집 시작
	cType_LinkRssiPollingStop,			///< RSSI 수집 중단

	cType_EndOfType
};

/***********************************************************************/
enum ModeDrone
{
	dMode_None = 0,					///< 없음
	dMode_Flight = 0x10,		///< 비행 모드(가드 포함)
	dMode_FlightNoGuard,		///< 비행 모드(가드 없음)
	dMode_FlightFPV,				///< 비행 모드(FPV)
	dMode_Drive = 0x20,			///< 주행 모드
	dMode_DriveFPV,					///< 주행 모드(FPV)
	dMode_Test = 0x30,			///< 테스트 모드
	dMode_EndOfType
};

/***********************************************************************/
enum ModeVehicle
{
	vMode_None = 0,
	vMode_Boot,							///< 부팅
	vMode_Wait,							///< 연결 대기 상태
	vMode_Ready,						///< 대기 상태
	vMode_Running,					///< 메인 코드 동작
	vMode_Update,						///< 펌웨어 업데이트
	vMode_UpdateComplete,		///< 펌웨어 업데이트 완료
	vMode_Error,						///< 오류
	vMode_EndOfType
};

/***********************************************************************/
enum ModeFlight
{
	fMode_None = 0,
	fMode_Ready,						///< 비행 준비
	fMode_TakeOff,					///< 이륙 (Flight로 자동전환)
	fMode_Flight,						///< 비행
	fMode_Flip,							///< 회전
	fMode_Stop,							///< 강제 정지
	fMode_Landing,					///< 착륙
	fMode_Reverse,					///< 뒤집기
	fMode_Accident,					///< 사고 (Ready로 자동전환)
	fMode_Error,						///< 오류
	fMode_EndOfType
};

/***********************************************************************/
enum ModeDrive
{
	dvMode_None = 0,
	dvMode_Ready,						///< 준비
	dvMode_Start,						///< 출발
	dvMode_Drive,						///< 주행
	dvMode_Stop,						///< 강제 정지
	dvMode_Accident,				///< 사고 (Ready로 자동전환)
	dvMode_Error,						///< 오류
	dvMode_EndOfType
};

/***********************************************************************/
enum SensorOrientation
{
	senOri_None = 0,
	senOri_Normal,					///< 정상
	senOri_ReverseStart,		///< 뒤집히기 시작
	senOri_Reverse,					///< 뒤집힘
	senOri_EndOfType
};

/***********************************************************************/
enum Coordinate
{
	cSet_None = 0,					///< 없음
	cSet_Absolute,					///< 고정 좌표계
	cSet_Relative,					///< 상대 좌표계
	cSet_EndOfType
};

/***********************************************************************/

enum Trim
{
	trim_None = 0,					///< 없음
	trim_RollIncrease,			///< Roll 증가
	trim_RollDecrease,			///< Roll 감소
	trim_PitchIncrease,			///< Pitch 증가
	trim_PitchDecrease,			///< Pitch 감소
	trim_YawIncrease,				///< Yaw 증가
	trim_YawDecrease,				///< Yaw 감소
	trim_ThrottleIncrease,	///< Throttle 증가
	trim_ThrottleDecrease,	///< Throttle 감소
	trim_EndOfType
};

/***********************************************************************/

enum FlightEvent
{
	fEvent_None = 0,				///< 없음
	fEvent_TakeOff,					///< 이륙
	fEvent_FlipFront,				///< 회전
	fEvent_FlipRear,				///< 회전
	fEvent_flipLeft,				///< 회전
	fEvent_FlipRight,				///< 회전
	fEvent_Stop,						///< 정지
	fEvent_Landing,					///< 착륙
	fEvent_TurnOver,				///< 뒤집기
	fEvent_Shot,						///< 미사일을 쏠 때 움직임
	fEvent_UnderAttack,			///< 미사일을 맞을 때 움직임
	fEvent_Square,					///< 정방향 돌기
	fEvent_CircleLeft,			///< 왼쪽으로 회전
	fEvent_CircleRight,			///< 오른쪽으로 회전
	fEvent_Rotate180,				///< 180도 회전
	fEvent_EndOfType
};

enum DriveEvent
{
	dEvent_None = 0,
	dEvent_Ready,						///< 준비
	dEvent_Start,						///< 출발
	dEvent_Drive,						///< 주행
	dEvent_Stop,						///< 강제 정지
	dEvent_Accident,				///< 사고 (Ready로 자동전환)
	dEvent_Error,						///< 오류
	dEvent_EndOfType
};

/***********************************************************************/
enum Request
{
	// 상태
	Req_Address = 0x30,					///< IEEE address
	Req_State,									///< 드론의 상태(비행 모드, 방위기준, 배터리량)
	Req_Attitude,								///< 드론의 자세(Vector)
	Req_GyroBias,								///< 자이로 바이어스 값(Vector)
	Req_TrimAll,								///< 전체 트림
	Req_TrimFlight,							///< 비행 트림
	Req_TrimDrive,							///< 주행 트림
	Req_CountFlight,						///< 비행 관련 카운트
	Req_CountDrive,							///< 주행 관련 카운트
	// 센서
	Req_ImuRawAndAngle = 0x50,	///< IMU Raw + Angle
	Req_Pressure,								///< 압력 센서 데이터
	Req_ImageFlow,							///< ImageFlow
	Req_Button,									///< 버튼 입력
	Req_Battery,								///< 배터리
	Req_Motor,									///< 모터 제어 및 현재 제어 값 확인
	Req_Temperature,						///< 온도
	Req_Range,									///< 거리 센서
	Req_EndOfType
};

/***********************************************************************/
enum ModeLight
{
	Light_None,
	WaitingForConnect, 		///< 연결 대기 상태
	Connected,
	EyeNone = 0x10,
	EyeHold,							///< 지정한 색상을 계속 켬
	EyeMix,								///< 순차적으로 LED 색 변경
	EyeFlicker,						///< 깜빡임
	EyeFlickerDouble,			///< 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)
	EyeDimming,						///< 밝기 제어하여 천천히 깜빡임
	ArmNone = 0x40,
	ArmHold,							///< 지정한 색상을 계속 켬
	ArmMix,								///< 순차적으로 LED 색 변경
	ArmFlicker,						///< 깜빡임
	ArmFlickerDouble,			///< 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)
	ArmDimming,						///< 밝기 제어하여 천천히 깜빡임
	ArmFlow,							///< 앞에서 뒤로 흐름
	ArmFlowReverse,				///< 뒤에서 앞으로 흐름
	EndOfLedMode
};

/***********************************************************************/
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

class CoDroneClass
{
public:

	//------------------------------------------------------------------------------------//
	void begin(long baud);
	void Receive(void);
	void Control();
	void Control(int interval);
	void Send_Command(int sendCommand, int sendOption);
	void Send_Processing(byte _data[], byte _length, byte _crc[]);
	//------------------------------------------------------------------------------------//

	void LinkReset();
	void Send_LinkModeBroadcast(byte mode);
	void Send_LinkState();

	//------------------------------------------------------------------------------------//

	void ConnectionProcess();
	void AutoConnect();
	void AutoConnect(byte mode);
	void AutoConnect(byte mode, byte address[]);	
	void Send_ConnectDrone(byte mode, byte address[]);
	void Send_Disconnect();
	void Send_Discover(byte action);
	void Send_Check(byte _data[], byte _length, byte _crc[]);

//------------------------------------------------------------------------------------//

	void calibrate();
	
	void Send_ResetHeading();
	void Send_Coordinate(byte mode);
	void Send_ClearGyroBiasAndTrim();	
	void DroneModeChange(byte event);
	void FlightEvent(byte event);
	void DriveEvent(byte event);

//------------------------------------------------------------------------------------//

	void BattleShooting();
	void BattleReceive();
	void BattleBegin(byte teamSelect);
	void BattleDamageProcess();

//------------------------------------------------------------------------------------//

	void Set_Trim(byte event);
	void Set_TrimReset();
	void Set_TrimFlight(int _roll, int _pitch, int _yaw, int _throttle);

//------------------------------------------------------------------------------------//

	void LedColorProcess(byte _dType, byte sendMode, byte r, byte g, byte b, byte sendInterval);
	void LedColor(byte sendMode, byte sendColor, byte sendInterval);
	void LedColor(byte sendMode, byte r, byte g, byte b, byte sendInterval);
	void LedColor(byte sendMode, byte sendColor[], byte sendInterval);
	void LedColorDefault(byte sendMode, byte r, byte g, byte b, byte sendInterval);
	void LedColorDefault(byte sendMode, byte sendColor[], byte sendInterval);
	void LedColorDefault(byte sendMode, byte sendColor[], byte sendInterval, byte sendMode2, byte sendColor2[], byte sendInterval2);

//------------------------------------------------------------------------------------//

	void ReceiveEventCheck(byte _completeData[]);
	void DisplayRSSI();
	int LowBatteryCheck(byte value);

//------------------------------------------------------------------------------------//

	void LED_Display(byte mode, int value);
	void LED_Connect();
	void LED_Blink(int time, int count);

//------------------------------------------------------------------------------------//

	unsigned short CRC16_Make(unsigned char *buf, int len); //CRC16-CCITT Format
	boolean CRC16_Check(unsigned char data[], int len, unsigned char crc[]);

//------------------------------------------------------------------------------------//

	void PrintDroneAddress();

//------------------------------------------------------------------------------------//

	void ButtonPressHoldWait(int button);
	void ButtonPressHoldWait(int button1, int button2);
	int AnalogScaleChange(int analogValue);

//------------------------------------------------------------------------------------//

	boolean TimeCheck(word interval); 						//milliseconds
	boolean TimeOutSendCheck(word interval); 			//milliseconds
	boolean TimeCheckBuzz(word interval); 				//microseconds

//------------------------------------------------------------------------------------//

	void Buzz(long frequency, int tempo);

//------------------------------------------------------------------------------------//

	void GoToHeight(int _range);
	void TurnDegree(int _angle);


//------------------------------------------------------------------------------------//
	void ReceiveGetData(byte _reqType);
	int getBatteryPercentage();
	int getBatteryVoltage();
	int getHeight();
	int getState();
	int getDroneTemp();
	int getPressure();

	optdata getOptFlowPosition();
	acceldata getAccelerometer();
	gyrodata getAngularSpeed();
	gyrodata getGyroAngles();
	trimdata getTrim();

//------------------------------------------------------------------------------------//

	byte receiveAttitudeSuccess = 0;
	byte receiveRangeSuccess = 0;
	byte receiveGyroSuccess = 0;
	byte receiveAccelSuccess = 0;
	byte receivePressureSuccess = 0;
	byte receiveTrimSuccess = 0;
	byte receiveStateSuccess = 0;
	byte receiveBatterySuccess = 0;
	byte receiveOptSuccess = 0;

//------------------------------------------------------------------------------------//

	int roll = 0;
	int pitch = 0;
	int yaw = 0;
	int throttle = 0;

	int attitudeRoll	= 0;
	int attitudePitch	= 0;
	int attitudeYaw	= 0;

	int batteryPercent	= 0;
	int batteryVoltage	= 0;

	long fVelocitySumX 	= 0;
	long fVelocitySumY	= 0;

	long temperature	= 0;
	long pressure		= 0;

	int ImuAccX	= 0;
	int ImuAccY	= 0;
	int ImuAccZ	= 0;

	int ImuGyroRoll		= 0;
	int ImuGyroPitch	= 0;
	int ImuGyroYaw		= 0;

	int ImuAngleRoll	= 0;
	int ImuAnglePitch	= 0;
	int ImuAngleYaw		= 0;

	int TrimAll_Roll;
	int TrimAll_Pitch;
	int TrimAll_Yaw;
	int TrimAll_Throttle;
	int TrimAll_Wheel;

//------------------------------------------------------------------------------------//

	byte cmdBuff[MAX_PACKET_LENGTH];

	byte checkHeader;
	byte cmdIndex;
	byte receiveDtype;
	byte receiveLength;

//------------------------------------------------------------------------------------//
	
	int SendInterval; //millis seconds
	int analogOffset;

	byte displayMode = 1;	//smar inventor : default 1

	boolean pairing = false;
	boolean	isConnected = false;
	
	byte timeOutRetry = 0;
	byte sendCheckCount = 0;
	byte sendCheckFlag = 0;
	byte energy = 8;
	byte team = FREE_PLAY;
	
	unsigned long weapon = FREE_MISSILE;

//------------------------------------------------------------------------------------//
	//for CodeLoader
	boolean sendingData = false;	
	boolean sendDataControl = true;
	
//------------------------------------------------------------------------------------//
	byte linkState = 0;
	int rssi = 0;
	byte battery = 0;
	unsigned long	irMessageReceive;
	byte droneState[7];
	int sensorRange[6];
	long PreviousMillis;

private:
	long PreviousBuzz;
	long timeOutSendPreviousMillis;
	
	byte discoverFlag = 0;
	byte connectFlag = 0;
	int connectMode = 0;
	
	int devNow = -1;		
	int RSSI_High = -255;
	int RSSI_Now = 0;
	
	byte devAddressBuf[6];
	byte devAddressConnected[6];
		
	//---------------------------------------------//


};

extern CoDroneClass CoDrone;

#endif