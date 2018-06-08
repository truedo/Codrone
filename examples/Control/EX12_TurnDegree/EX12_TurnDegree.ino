/*****************************************************************
  - TurnDegree
  드론이 입력한 각도로 회전합니다.
  오른쪽 회전 (양수)  :  1 ~  360
  왼쪽 회전  (음수)   : -1 ~ -360
*******************************************************************/

#include <CoDrone.h> // 코드론을 사용하기 위한 헤더파일 

void setup()
{
  CoDrone.begin(115200);                // 드론 플러그의 통신 개시 (115200bps)
  CoDrone.AutoConnect(NearbyDrone);     // 가장 가까운 위치의 드론과 연결
  CoDrone.DroneModeChange(Flight);      // 드론을 플라이트 모드로 설정합니다. (비행형)

  CoDrone.FlightEvent(TakeOff);
  delay(3000);

  CoDrone.TurnDegree(90);
  delay(1000);

  CoDrone.FlightEvent(Landing);        // 서서히 착륙
}

void loop()
{
}
