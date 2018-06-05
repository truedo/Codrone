#include "CoDrone.h"
#include "Arduino.h"
#include <EEPROM.h>

//-------------------------------------------------------------------------------------------------------//
//------------------------------------------- GetData ---------------------------------------------------//
//-------------------------------------------------------------------------------------------------------//
void CoDroneClass::ReceiveGetData(byte _reqType)
{
	byte *_reqCheckType;

	sendCheckFlag = 1;
	Send_Command(cType_Request, _reqType);
	
//---------------------------------------------------------------------------------//
	
	if		 (_reqType == Req_Attitude) 			_reqCheckType = &receiveAttitudeSuccess;
	else if(_reqType == Req_Battery) 				_reqCheckType = &receiveBatterySuccess;
	else if(_reqType == Req_Range) 					_reqCheckType = &receiveRangeSuccess;
	else if(_reqType == Req_State) 					_reqCheckType = &receiveStateSuccess;
	else if(_reqType == Req_ImageFlow) 			_reqCheckType = &receiveOptSuccess;
	else if(_reqType == Req_Pressure) 			_reqCheckType = &receivePressureSuccess;
	else if(_reqType == Req_TrimFlight) 		_reqCheckType = &receiveTrimSuccess;
	else if(_reqType == Req_ImuRawAndAngle) _reqCheckType = &receiveAccelSuccess;

//--------------------------------------------------------------------------------//
	*_reqCheckType = 0;

	long oldTime = millis();
	while(!*_reqCheckType)
	{
		Receive();
		if (oldTime + 500 < millis()) break; //time out check
	}
}
//--------------------------------------------------------------------------------//

int CoDroneClass::getPressure()
{
	byte _reqType = Req_Pressure;
	ReceiveGetData(_reqType);
	return pressure;
}

int CoDroneClass::getDroneTemp()
{
	byte _reqType = Req_Pressure;
	ReceiveGetData(_reqType);
	return temperature;
}

int CoDroneClass::getState()
{
	byte _reqType = Req_State;
	ReceiveGetData(_reqType);
	return droneState[2];
}

int CoDroneClass::getHeight()
{
	byte _reqType = Req_Range;
	ReceiveGetData(_reqType);
	return sensorRange[5];
}

int CoDroneClass::getBatteryPercentage()
{
	byte _reqType = Req_Battery;
	ReceiveGetData(_reqType);
	return batteryPercent;
}

int CoDroneClass::getBatteryVoltage()
{
	byte _reqType = Req_Battery;
	ReceiveGetData(_reqType);
	return batteryVoltage;
}

acceldata CoDroneClass::getAccelerometer()
{
	byte _reqType = Req_ImuRawAndAngle;
	ReceiveGetData(_reqType);

	acceldata result;
	result.x = ImuAccX;
	result.y = ImuAccY;
	result.z = ImuAccZ;
	return result;
}

trimdata CoDroneClass::getTrim()
{
	byte _reqType = Req_TrimFlight;
	ReceiveGetData(_reqType);

	trimdata result;
	result.roll = TrimAll_Roll;
	result.pitch = TrimAll_Pitch;
	result.yaw = TrimAll_Yaw;
	result.throttle = TrimAll_Throttle;

	return result;
}

optdata CoDroneClass::getOptFlowPosition()
{
	byte _reqType = Req_ImageFlow;
	ReceiveGetData(_reqType);

	optdata result;
	result.x = fVelocitySumX;
	result.y = fVelocitySumY;
	return result;
}

gyrodata CoDroneClass::getAngularSpeed()
{
	byte _reqType = Req_ImuRawAndAngle;
	ReceiveGetData(_reqType);

	gyrodata result;
	result.roll = ImuGyroRoll;
	result.pitch = ImuGyroPitch;
	result.yaw = ImuGyroYaw;
	return result;
}

gyrodata CoDroneClass::getGyroAngles()
{
	byte _reqType = Req_Attitude;
	ReceiveGetData(_reqType);

	gyrodata result;
	result.roll = attitudeRoll;
	result.pitch = attitudePitch;
	result.yaw = attitudeYaw;
	return result;
}
//-------------------------------------------------------------------------------------------------------//

