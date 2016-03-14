/*****************************************************************
  - LED Color 1
  드론의 LED를 제어합니다.
  LedColor(Mode,Color,Time); 모드, 색상, 시간의 형식으로 입력합니다.
  * 모드는 아래의 모드 테이블 참조
  * 색상은 아래의 색상 테이블 참조
  * 시간은 0~255 까지 입력 가능합니다. (Mode에 따라 다르게 적용)
*******************************************************************/
#include <CoDrone.h> // 코드론을 사용하기 위한 헤더파일 

byte modeTime = 7;                        // 모드 시간 변수
int delayTime = 1000;                     // 대기 시간 변수

void setup()
{
  CoDrone.begin();                      // 드론 플러그의 기능 개시
  Serial.begin(115200);                 // 드론과 통신 개시(115200bps)

  CoDrone.AutoConnect(NeardbyDrone);   // 가장 가까운 위치의 드론과 연결
}

void loop()
{
  CoDrone.LedColor(ArmDimming, Yellow, modeTime);   // 노랑색으로 밝기 제어하여 천천히 깜빡이며  modeTime 따라 동작합니다.
  delay(delayTime);                                  // 대기 시간 입력

  CoDrone.LedColor(ArmDimming, Cyan, modeTime);     // 하늘색으로 밝기 제어하여 천천히 깜빡이며 modeTime 따라 동작합니다.
  delay(delayTime);                                  // 대기 시간 입력
}

/*********************************************************************************************************************************
  * mode - TABLE
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
  
/***********************************************************************************************************************************
  * color - TABLE
  AliceBlue, AntiqueWhite, Aqua,  Aquamarine, Azure, Beige,  Bisque, Black, BlanchedAlmond,  Blue, BlueViolet, Brown,
  BurlyWood, CadetBlue, Chartreuse,  Chocolate, Coral, CornflowerBlue,  Cornsilk, Crimson, Cyan, DarkBlue, DarkCyan,
  DarkGoldenRod,  DarkGray, DarkGreen, DarkKhaki,  DarkMagenta, DarkOliveGreen, DarkOrange, DarkOrchid, DarkRed,
  DarkSalmon,  DarkSeaGreen, DarkSlateBlue, DarkSlateGray,  DarkTurquoise, DarkViolet, DeepPink,  DeepSkyBlue, DimGray,
  DodgerBlue, FireBrick, FloralWhite, ForestGreen,  Fuchsia, Gainsboro, GhostWhite, Gold, GoldenRod, Gray, Green,
  GreenYellow, HoneyDew,  HotPink, IndianRed, Indigo, Ivory, Khaki, Lavender,  LavenderBlush, LawnGreen, LemonChiffon,
  LightBlue, LightCoral, LightCyan, LightGoldenRodYellow, LightGray, LightGreen,  LightPink, LightSalmon, LightSeaGreen,
  LightSkyBlue, LightSlateGray, LightSteelBlue,  LightYellow, Lime, LimeGreen, Linen, Magenta, Maroon, MediumAquaMarine,
  MediumBlue, MediumOrchid, MediumPurple, MediumSeaGreen, MediumSlateBlue, MediumSpringGreen, MediumTurquoise, MediumVioletRed,
  MidnightBlue, MintCream, MistyRose, Moccasin, NavajoWhite, Navy, OldLace, Olive, OliveDrab, Orange, OrangeRed, Orchid,
  PaleGoldenRod, PaleGreen, PaleTurquoise, PaleVioletRed, PapayaWhip, PeachPuff, Peru, Pink, Plum, PowderBlue, Purple, RebeccaPurple,
  Red, RosyBrown, RoyalBlue, SaddleBrown, Salmon, SandyBrown, SeaGreen, SeaShell, Sienna, Silver, SkyBlue, SlateBlue, SlateGray,
  Snow, SpringGreen, SteelBlue, Tan, Teal, Thistle, Tomato, Turquoise, Violet, Wheat, White, WhiteSmoke, Yellow, YellowGreen,
  ***********************************************************************************************************************************/
