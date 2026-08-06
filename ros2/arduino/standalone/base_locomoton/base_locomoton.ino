#include <PS2X_lib.h>
PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;
int RX = 0, RY = 0, LX = 0, LY = 0, up = 0, down = 0, left = 0, right = 0, sel = 0;
int p1 = 8, p2 = 9, p3 = 12, p4 = 11;
int M1(int, int, int), M2(int, int, int), M3(int, int, int), M4(int, int, int);

void setup() {
  Serial.begin (9600);

  error = ps2x.config_gamepad(47, 51, 49, 53, false, false);
  pinMode(30, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(11, OUTPUT);
}
void loop() {

  if (error == 0)
  {

    ps2x.read_gamepad(false, vibrate);
    LY = ps2x.Analog(PSS_LY);
    LX = ps2x.Analog(PSS_LX);


    int f = map(LY, 126, 0, 0, 255);
    int b = map(LY, 127, 255, 0, 255);
    int l = map(RX, 126, 0, 0, 255);
    int r = map(RX, 127, 255, 0, 255);

    if (f > 40) {
      M1(0, 1, f);
      M2(0, 1, f);
      M3(0, 1, f);
      M4(0, 1, f);
      Serial.println("forward");
    }
    else  if (b > 40) {
      M1(1, 0, b);
      M2(1, 0, b);
      M3(1, 0, b);
      M4(1, 0, b);
      Serial.println("baack");
    }
    else  if (r > 40) {
      M1(1, 0, r);
      M2(1, 0, r);
      M3(1, 0, r);
      M4(1, 0, r);
      Serial.println("baack");
    }
    else  if (l > 40) {
      M1(1, 0, l);
      M2(1, 0, l);
      M3(1, 0, l);
      M4(1, 0, l);
      Serial.println("baack");
    }
    else
    {
      M1(0, 0, 0);
      M2(0, 0, 0);
      M3(0, 0, 0);
      M4(0, 0, 0);
    }
  }
  else
  {
    M1(0, 0, 0);
    M2(0, 0, 0);
    M3(0, 0, 0);
    M4(0, 0, 0);
    Serial.println("lolyourinterface");
  }
}


int M1(int x, int y, int z)
{
  digitalWrite(30, x);
  digitalWrite(32, y);
  analogWrite(p1, z);
}
int M2(int x, int y, int z)
{
  digitalWrite(34, x);
  digitalWrite(36, y);
  analogWrite(p2, z);
}
int M3(int x, int y, int z)
{
  digitalWrite(46, x);
  digitalWrite(48, y);
  analogWrite(p3, z);
}
int M4(int x, int y, int z)
{
  digitalWrite(42, x);
  digitalWrite(44, y);
  analogWrite(p4, z);
}
