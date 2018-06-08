/*************************************************************************
  Range Sensor - 고도 값 표시
  연결된 드론의 고도 값을 Serial Monitor로 표시합니다.
  고도값은 mm 값입니다.

  드론을 켠 상태에서 PC와 보드를 USB로 연결합니다.
  아두이노 창의 시리얼 모니터를 열고 통신 속도를 115200bps로 설정합니다.
***************************************************************************/

#include <CoDrone.h> // 코드론을 사용하기 위한 헤더파일 

void setup()
{
  CoDrone.begin(115200);               // 드론과의 통신 개시 (115200bps)
  CoDrone.AutoConnect(NearbyDrone);    // 가장 가까운 위치의 드론과 연결
  delay(500);
}
void loop()
{
  RangeSensorToSerialMonitor();
}

void RangeSensorToSerialMonitor()
{
  //---------------------------------------------------------------------------------------------//
  CoDrone.Send_LinkModeBroadcast(LinkBroadcast_Active);  //link module mode change => Active
  delay(100);

  int height = CoDrone.getHeight();                      //save request data

  CoDrone.Send_LinkModeBroadcast(LinkModeMute);          //link module mode change => Mute
  delay(100);

  Serial.println("");
  Serial.println("--------------- Sensor ---------------");
  Serial.print("range \t\t");
  Serial.print(CoDrone.sensorRange[5]);
  Serial.println(" mm");
  Serial.println("-------------------------------------- ");

  delay(100);
  //---------------------------------------------------------------------------------------------//
}




