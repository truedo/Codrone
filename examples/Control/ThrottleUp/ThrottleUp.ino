/*****************************************************************
  - Throttle Up
  쓰로틀 값을 입력해서 드론을 상하강 시킬 수 있습니다.
  상승(양수)  :  1 ~  100
  하강(음수)  : -1 ~ -100
*******************************************************************/
#include <CoDrone.h> // 코드론을 사용하기 위한 헤더파일 

void setup()
{
  Serial.begin(115200);                 // 드론과 통신 개시(115200bps)

  CoDrone.begin();                      // 드론 플러그의 기능 개시

  CoDrone.AutoConnect(NeardbyDrone);    // 가장 가까운 드론과 연결

  if (PAIRING == true)                   // 연결(페어링)이 성공한 경우에만 실행
  {
    THROTTLE  = 100;                     // THROTTLE 값 입력
    CoDrone.Control();                  // 조종값 전송

    delay(2000);                         // 대기 시간 입력

    CoDrone.FlightEvent(Stop);          // 정지
  }
}

void loop()
{
}


