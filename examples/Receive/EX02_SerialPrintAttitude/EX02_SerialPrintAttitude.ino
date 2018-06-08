/**********************************************************************************************
  - Attitude To SerialMonitor
  드론의 현재 자세 정보 (ROLL, PITCH, YAW) 값을 시리얼 모니터 창에 표시합니다.

  드론을 켠 상태에서 PC와 보드를 USB로 연결합니다.
  아두이노 창의 시리얼 모니터를 열고 통신 속도를 115200bps로 설정합니다.
************************************************************************************************/

#include <CoDrone.h>

void setup()
{
  CoDrone.begin(115200);                        // 드론과의 통신 개시 (115200bps)
  CoDrone.AutoConnect(NearbyDrone);             // 가장 가까운 위치의 드론과 연결
  delay(500);
}

void loop()
{
  AttitudeToSerialMonitor();
}

void AttitudeToSerialMonitor()
{
  CoDrone.Send_LinkModeBroadcast(LinkBroadcast_Active);  //link module mode change => Active
  delay(100);

  gyrodata angle;
  angle = CoDrone.getGyroAngles();                      //save request data

  CoDrone.Send_LinkModeBroadcast(LinkModeMute);       //link module mode change => Mute
  delay(100);
  //---------------------------------------------------------------------------------------------//
  Serial.println("");
  Serial.println("--------- Now attitude -----------");
  Serial.print("ROLL\t");
  Serial.println(angle.roll);
  Serial.print("PITCH\t");
  Serial.println(angle.pitch);
  Serial.print("YAW\t");
  Serial.println(angle.yaw);
  delay(100);

  //---------------------------------------------------------------------------------------------//
}




