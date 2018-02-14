/*****************************************************************
  - LED Default Color
  드론의 LED를 제어합니다.(LED 설정 저장)
  - 전원이 껏다 켜진후에도 LED가 현재 상태를 유지합니다.

  눈과 날개의 두곳의 LED를 동시에 설정할 수 있습니다.
  따라서 2개의 LED 설정값을 입력합니다.
  LedColorDefault(Mode,color[],Time, Mode2,color2[],Time2); 모드1, 색상 배열1, 시간1, 모드2, 색상 배열2, 시간2의 형식으로 입력합니다.

  모드는 아래의 모드 테이블 참조
  색상 배열은 R,G,B 값을 입력 하며, 각각 0~255 까지 입력 가능합니다.
  시간은 0~255 까지 입력 가능합니다. (Mode에 따라 다르게 적용)
  
*******************************************************************/
#include <CoDrone.h> // 코드론을 사용하기 위한 헤더파일 

byte mode1 = ArmHold;
byte color1[] = {0, 0, 255};              //color1 색상 배열 (R,G,B)
byte modeTime1 = 255;                     // 모드 시간 변수

byte mode2 = EyeHold;
byte color2[] = {255, 255, 0};            //color2 색상 배열 (R,G,B)
byte modeTime2 = 255;                     // 모드 시간 변수

void setup()
{
  CoDrone.begin(115200);                // 드론 플러그의 통신 개시 (115200bps)
  CoDrone.AutoConnect(NearbyDrone);     // 가장 가까운 위치의 드론과 연결
  CoDrone.LedColorDefault(mode1, color1, modeTime2, mode2, color2, modeTime2);   // color에 입력된 색, mode와 modeTime에 따라 동작합니다.
}

void loop()
{
  
}

/*********************************************************************************************************************************
    mode - TABLE
  EyeNone
  EyeHold,          ///< 지정한 색상을 계속 켬
  EyeMix,           ///< 순차적으로 LED 색 변경
  EyeFlicker,       ///< 깜빡임
  EyeFlickerDouble, ///< 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)
  EyeDimming,       ///< 밝기 제어하여 천천히 깜빡임

  ArmNone
  ArmHold,           ///< 지정한 색상을 계속 켬
  ArmMix,            ///< 순차적으로 LED 색 변경
  ArmFlicker,        ///< 깜빡임
  ArmFlickerDouble,  ///< 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)
  ArmDimming,        ///< 밝기 제어하여 천천히 깜빡임
  ArmFlow,           ///< 앞에서 뒤로 흐름
  ArmFlowReverse,    ///< 뒤에서 앞으로 흐름
  *********************************************************************************************************************************/
