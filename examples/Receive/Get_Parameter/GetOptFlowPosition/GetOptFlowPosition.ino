/*****************************************************************
  getOptFlowPosition - 옵티컬 플로우 센서 값을 표시합니다. 
  옵티컬 플로우 센서의 x와 y 좌표를 표시합니다.

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

  optdata opt;
  opt = CoDrone.getOptFlowPosition();                     //save request data

  CoDrone.Send_LinkModeBroadcast(LinkModeMute);       //link module mode change => Mute
  delay(100);

  Serial.println("");
  Serial.println("--------- Now -----------");
  Serial.print("opt x : \t");
  Serial.println(opt.x);
  Serial.print("opt y : \t");
  Serial.println(opt.y);
  delay(100);
}
