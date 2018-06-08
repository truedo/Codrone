/*****************************************************************
  Trim - Trim all
-드론이 한쪽으로 치우치거나 출력의 제어가 안되는 경우에는 트림 기능을 사용하여 보정합니다.

한번에 여러 값을 입력할 경우에는 TrimFlight 기능을 사용하여 동시에 입력하도록 합니다.

모든 값은 -500 ~ 500으로 설정하길 바랍니다. 초기 기준 값은 0입니다.

*******************************************************************/
#include <CoDrone.h> // 코드론을 사용하기 위한 헤더파일 

// Value Range -500 ~ 500

int roll      = 0;
int pitch     = 0;
int yaw       = 0;
int throttle  = 0;

void setup()
{  
  CoDrone.begin(115200);                // 드론 플러그의 통신 개시 (115200bps)
  CoDrone.AutoConnect(NearbyDrone);     // 가장 가까운 위치의 드론과 연결
 
  CoDrone.Set_TrimFlight(roll, pitch, yaw, throttle);    
}

void loop()
{
}


