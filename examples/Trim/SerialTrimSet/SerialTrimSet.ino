/*****************************************************************
  Trim - Serial Trim Set

  -드론이 한쪽으로 치우치거나 출력의 제어가 안되는 경우에는 트림 기능을 사용하여 보정합니다.
  드론의 트림 값을 시리얼 통신을 통해서 설정합니다.

  드론을 켠 상태에서 PC와 보드를 USB로 연결합니다.
  아두이노 창의 시리얼 모니터를 열고 115200bps, Newline 으로 설정합니다.
  화면에 나오는 명령어를 입력하면 해당명령이 실행됩니다.

  # 개별 명령 : 개별 명령은 명령어만 입력하면 작동합니다.
  help  : 명령어를 화면에 다시 출력합니다.
  now   : 현재 드론의 트림을 확인합니다.
  reset : 드론의 전체 트림 값을 기본으로 설정합니다.(0으로 설정)

  # 설정 명령 : 설정 명령은 명령어와 설정할 수치 값을 같이 입력해야 합니다.
   예를 들어 roll을 20으로 변경하는 경우,
  :"roll 20" 과 같이 roll 명령 후에 한칸을 띄운 후 수치를 입력합니다.

    ex) throttle -70
        pitch 40
        yaw -20

  모든 값은 -100 ~ 100으로 설정하며, 초기 기준 값은 0입니다.

  roll [value] : roll 값을 설정합니다.
  yaw  [value] : yaw 값을 설정합니다.
  pitch [value]: pitch 값을 성정합니다.
  throttle [value] : throttle 값을 설정합니다.

*******************************************************************/

#include <CoDrone.h> // 코드론을 사용하기 위한 헤더파일 

#define MAX_CMD_LENGTH  15
char cmdBuff[MAX_CMD_LENGTH];
int cmdIndex = 0;
char* cmd;
char* arg1;

#define _Roll       1
#define _Yaw        2
#define _Throttle   3
#define _Pitch      4

void setup()
{
  CoDrone.begin(115200);                // 드론 플러그의 통신 개시 (115200bps)
  CoDrone.AutoConnect(NearbyDrone);    // 가장 가까운 위치의 드론과 연결

  PrintTrimAll();                        // 현재 트림값 표시 및 변수에 저장

  delay(100);
  CoDrone.Send_LinkModeBroadcast(LinkModeMute);
  delay(100);

  PrintMenu();
  printPrompt();
}

void loop()
{
  if (Serial.available() > 0)
  {
    int input = Serial.read();
    Serial.write(input); // echo
    if (input == '\r')
    {
      input = '\n';
      Serial.write(input); // echo
    }
    cmdBuff[cmdIndex++] = (char)input;
    if (cmdIndex >= MAX_CMD_LENGTH)
    {
      cmdIndex = 0;
      printCmdError();
    }
    else
    {
      if (cmdBuff[cmdIndex - 1] == '\n')
      {
        cmdBuff[cmdIndex - 1] = 0; // end of string
        cmd = strtok(cmdBuff, " "); // spilt command
        arg1 = strtok(0, " "); // spilt argument1

        if (strcmp(cmd, "help") == 0)         PrintMenu();
        else if (strcmp(cmd, "reset") == 0)      cmdTrimReset();
        else if (strcmp(cmd, "now") == 0)
        {
          delay(100);
          CoDrone.Send_LinkModeBroadcast(LinkBroadcast_Active);
          PrintTrimAll();
        }

        else if ((strcmp(cmd, "roll") == 0) || (strcmp(cmd, "pitch") == 0) || (strcmp(cmd, "yaw") == 0) ||  (strcmp(cmd, "throttle") == 0))
        {
          int _Trim = atoi(arg1);
          delay(150);
          CoDrone.Send_LinkModeBroadcast(LinkBroadcast_Active);
          delay(150);

          if (strcmp(cmd, "roll") == 0)             CoDrone.TrimAll_Roll = _Trim;
          else if (strcmp(cmd, "pitch") == 0)       CoDrone.TrimAll_Pitch = _Trim;
          else if (strcmp(cmd, "yaw") == 0)         CoDrone.TrimAll_Yaw = _Trim;
          else if (strcmp(cmd, "throttle") == 0)    CoDrone.TrimAll_Throttle = _Trim;
          CoDrone.Set_TrimFlight(CoDrone.TrimAll_Roll, CoDrone.TrimAll_Pitch, CoDrone.TrimAll_Yaw, CoDrone.TrimAll_Throttle);

          delay(650);
          PrintTrimAll();
        }

        else
        {
          if (cmdIndex - 1 == 0)  printPrompt();
          else                    printCmdError();
        }
        cmdIndex = 0;
      }
    }
  }
}

void cmdTrimReset()
{
  delay(150);
  CoDrone.Send_LinkModeBroadcast(LinkBroadcast_Active);
  delay(150);
  CoDrone.Set_TrimFlight(0, 0, 0, 0);
  delay(650);
  PrintTrimAll();
}

void PrintMenu()
{
  while (Serial.available() > 0)      byte Temp = Serial.read();

  Serial.println();
  Serial.println("--------------- Trim Set Mode --------------- ");
  Serial.println("- Serial Monitor Mode => Newline, 115200 baud");
  Serial.println("  Value Range -500 ~ 500");
  Serial.println();
  Serial.println("help");
  Serial.println("now");
  Serial.println("reset");
  Serial.println("roll[value]\t EX : roll 20");
  Serial.println("pitch[value]\t EX : pitch -20");
  Serial.println("yaw[value]\t EX : yaw 20");
  Serial.println("throttle[value]\t EX : throttle -20");
  Serial.println("--------------------------------------------- ");

  printPrompt();
}

void PrintTrimAll()
{
  delay(500);

  trimdata trim;
  trim = CoDrone.getTrim();

  delay(100);
  CoDrone.Send_LinkModeBroadcast(LinkModeMute);       //link module mode change => Mute
  delay(100);

  Serial.println("");
  Serial.println("");
  Serial.println("--------------- Now Trim ---------------");

  Serial.print("roll \t\t");
  Serial.println(trim.roll);

  Serial.print("pitch \t\t");
  Serial.println(trim.pitch);

  Serial.print("yaw \t\t");
  Serial.println(trim.yaw);

  Serial.print("throttle \t");
  Serial.println(trim.throttle);

  Serial.println("--------------------------------------------- ");

  delay(100);
  while (Serial.available() > 0)    byte Temp = Serial.read();
  printPrompt();
}

void printCmdError()
{
  Serial.println("Try again");
  printPrompt();
}

void printPrompt()
{
  Serial.print("> ");
}


