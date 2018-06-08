/*****************************************************************
  - Connect 0
  AutoConnect()  
  - NearbyDrone, ConnectedDrone 방식이 혼합된 연결 방식입니다.
  
  * 스위치 설정은 스마트 보드의 전원을 껏다가 켜거나 리셋버튼을 눌러야만 적용이 됩니다. 
  * 
  |--------| 
  | □□□ | Switch 3 : Down ↓
  | ■■■ | 
  | 1 2 3  | NearbyDrone or ConnectedDrone
  |--------| 
  스위치 3번이 내려간 상태
 - 이전에 연결한 드론이 있을 때는, 이전에 연결한 드론을 연결하며, 
 그렇지 않은 경우에는 주변의 가장 가까운 드론을 검색합니다.

  |--------| 
  | □□■ | Switch 3 : UP  ↑
  | ■■□ | 
  | 1 2 3  | NearbyDrone
  |--------| 
 스위치 3번이 올라간 상태
 - 다른 드론을 연결해야하거나, 실수로 타인의 드론을 연결한 경우에 사용합니다.
 (이 상태에서는 무조건 주변의 가장 가까운 드론을 검색하게 됩니다.)  
 검색이 되었으면 딥스위치 3번을 내립니다.  
************************************************************************/

#include <CoDrone.h> // 코드론을 사용하기 위한 헤더파일 

void setup()
{
  CoDrone.begin(115200);              // 드론과 통신 개시 (115200bps)
  
  CoDrone.AutoConnect();              // 딥스위치 3번의 상태에 따라 자동으로 드론 연결 (예제 참조) Connect -> EX05_AutoConnect_4-SimpleConnect 

  CoDrone.DroneModeChange(Flight);    // 드론을 플라이트 모드로 설정합니다. (비행형)

  // 2초간 드론을 띄운후 착륙하기
  if (PAIRING == true)                 // 연결(페어링)이 성공한 경우에만 실행
  {
    CoDrone.FlightEvent(TakeOff);     // 이륙
    delay(2000);                       // 대기 시간
    CoDrone.FlightEvent(Landing);     // 서서히 착륙
  }
}

void loop()
{

}


