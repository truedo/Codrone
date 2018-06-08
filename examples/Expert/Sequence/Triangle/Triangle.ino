/*****************************************************************
  - Triangle 
  드론이 삼각형 모양으로 비행합니다.
  유저의 드론 상태에 따라 pitch값 조절이 필요합니다.
*******************************************************************/

#include <CoDrone.h> // 코드론을 사용하기 위한 헤더파일 

void setup()
{
  CoDrone.begin(115200);              // 드론 플러그의 통신 개시 (115200bps)

  CoDrone.AutoConnect(NearbyDrone);   // 가장 가까운 위치의 드론과 연결

  CoDrone.DroneModeChange(Flight);       // 드론을 플라이트 모드로 설정합니다. (비행형)

  delay(300);                              // 대기 시간

  if (PAIRING == true)                   //연결(페어링)이 성공한 경우에만 실행
  {
    CoDrone.FlightEvent(TakeOff);       // 이륙
    delay(3000);

    PITCH = 50;
    CoDrone.Control();
    delay(1000);                        // 대기 시간
 
    CoDrone.TurnDegree(-120);
    
    PITCH = 50;
    CoDrone.Control();
    delay(1000);                        // 대기 시간
    
    CoDrone.TurnDegree(-120);
    
    PITCH = 50;
    CoDrone.Control();
    delay(1000);                        // 대기 시간

    CoDrone.FlightEvent(Landing);        // 서서히 착륙
  }
}

void loop()
{
}
