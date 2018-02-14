/*****************************************************************
  Range Sensor - 고도 값 표시
  연결된 드론의 고도 값을 Serial Monitor로 표시합니다.
  고도값은 mm 값입니다.
*******************************************************************/

#include <CoDrone.h> // 코드론을 사용하기 위한 헤더파일 

void setup()
{
  CoDrone.begin(115200);                // 드론 플러그의 통신 개시 (115200bps)
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
  CoDrone.Request_Range();

  long oldTime = millis();
  while (CoDrone.receiveRangeSuccess == false)            //receiveRangeSuccess check
  {
    CoDrone.Receive();
    if (oldTime + 1000 < millis()) break;                 //time out check
  }

  //---------------------------------------------------------------------------------------------//
  if (CoDrone.receiveRangeSuccess ==  true)
  {
    CoDrone.Send_LinkModeBroadcast(LinkModeMute);       //link module mode change => Mute
    delay(300);

    Serial.println("");
    Serial.println("--------------- Sensor ---------------");
    Serial.print("range \t\t");
    Serial.print(CoDrone.sensorRange[5]);
    Serial.println(" mm");
    Serial.println("-------------------------------------- ");
    delay(500);
  }
  //---------------------------------------------------------------------------------------------//
}



