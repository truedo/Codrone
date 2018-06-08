/************************************************************************
  Range Sensor - 고도 값 표시
  연결된 드론의 고도 값을 LED 불빛으로 표시합니다.
  고도값은 mm 값입니다.
************************************************************************/

#include <CoDrone.h> // 코드론을 사용하기 위한 헤더파일 

int scale = 200; // LED 불빛 한개의 기준 고도 200mm
int firstLEDpin = 11; // 스마트 보드의 LED 핀 시작 번호

void setup()
{
  CoDrone.begin(115200);                // 드론 플러그의 통신 개시 (115200bps)
  CoDrone.AutoConnect(NearbyDrone);     // 가장 가까운 위치의 드론과 연결

  delay(500);

  for (int thisPin = 11; thisPin <= 18; thisPin++)   pinMode(thisPin, OUTPUT);
  for (int thisPin = 11; thisPin <= 18; thisPin++)   digitalWrite(thisPin, LOW);
}

void loop()
{
  int height = CoDrone.getHeight();                      //save request data

  int _sensor = map(height, 0, 2000, 0, 7);
  for (int thisPin = 11; thisPin <= 18; thisPin++)  digitalWrite(thisPin, LOW);
  for (int i = firstLEDpin; i <= _sensor + firstLEDpin; i++)  digitalWrite(i , HIGH);
}


