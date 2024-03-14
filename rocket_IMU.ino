/* 센서관련된 클래스가 정의된 cpp, 헤더파일은 Src 폴더로 넣어주시기 바랍니다*/

/* #include begin--------------------------- */
#include "Src/MPU9250.h" //#include "폴더명/헤더파일명.h"로 하시면 됩니다.
#include <SoftwareSerial.h>
/* #include end--------------------------- */

/* #define begin--------------------------- */
#define PI = 3.1415926535897932384626433832795028
/* #define end--------------------------- */

/* Global Variables initial begin--------------*/
SoftwareSerial Gps(2,3); //(RxPin, TxPin)
//softwareSerial port speed up to 115200 bps
String tempGPS;
/* Global Variables initial end--------------*/

void setup()
{
  Serial.begin(9600);
  if(Gps.available()){
    Serial.println("GPS is Ready...")
    delay(1500);
  }
}

void loop()
{
  if(Gps.available()){
    tempGPS = Gps.read();
  }
}
