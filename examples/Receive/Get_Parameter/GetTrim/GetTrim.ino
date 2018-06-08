/*****************************************************************
  getState - 드론의 미세 조정 값을 표시합니다.
  roll, pitch, yaw, throttle의 미세 조정값을 표시합니다. 

  드론을 켠 상태에서 PC와 보드를 USB로 연결합니다.
  아두이노 창의 시리얼 모니터를 열고 통신 속도를 115200bps로 설정합니다.
*******************************************************************/

#include <CoDrone.h> // 코드론을 사용하기 위한 헤더파일 

void setup()
{
  CoDrone.begin(115200);                // 드론 플러그의 통신 개시 (115200bps)
  CoDrone.AutoConnect(NearbyDrone);     // 가장 가까운 위치의 드론과 연결
  CoDrone.DroneModeChange(Flight);      // 드론을 플라이트 모드로 설정합니다. (비행형)

}

void loop()
{
  CoDrone.Send_LinkModeBroadcast(LinkBroadcast_Active);  //link module mode change => Active
  delay(100);

  trimdata trim;
  trim = CoDrone.getTrim();                               //save request data

  CoDrone.Send_LinkModeBroadcast(LinkModeMute);       //link module mode change => Mute
  delay(100);

  Serial.println("");
  Serial.println("--------- Now -----------");
  Serial.print("trim roll : \t");
  Serial.println(trim.roll);
  Serial.print("trim pitch : \t");
  Serial.println(trim.pitch);
  Serial.print("trim yaw : \t");
  Serial.println(trim.yaw);
  Serial.print("trim throttle : \t");
  Serial.println(trim.throttle);

  delay(100);
}
