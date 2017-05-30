//napisany pod sterownik 8xMosfet + ProMini

#define    STX          0x02
#define    ETX          0x03

#define AIN1  3
#define AIN2  5
#define BIN1  6
#define BIN2  9

long previousMillis = 0;
long sendInterval = 250;      //interwał wysyłania informacji do telefonu [ms]
String buttons = "000000";    //stan przyciskow 6 5 4 3 2 1

byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int x = 200;
int y = 200;
int min_pwm = 1; //minimalna wartość PWM (55 dla 4,2V, 27 dla 8,4V)
int max_pwm = 101;
int pwm_s;
int pwm_f;
char a;
int fromA0 = 0;  //odczyt z pojedynczego ogniwa zasilania silnikow
float Vmot = 0;  //napięcie -//-
//unsigned long start;
//unsigned long czas;
//start = millis();  //pomiar czasu
//czas = millis() - start;
 
void setup()
{
  pinMode(AIN1, OUTPUT); //Sygnał PWM silnika A (AIN1) lewy
  pinMode(AIN2, OUTPUT); //Sygnał PWM silnika A (AIN2)
    
  pinMode(BIN1, OUTPUT); //Sygnał PWM silnika B (BIN1) prawy
  pinMode(BIN2, OUTPUT); //Sygnał PWM silnika B (BIN2)
  
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  Serial.begin(9600);   //monitor szeregowy
  Serial1.begin(9600);  //domyslna dla hc06 //ukryć to przed wegraniem na promini
  while(Serial1.available())  Serial1.read();
}
 
void loop()
{
  get_xy_a();
  getVmot();
  sendBlueToothData();
}

void get_xy_a()
{
  if(Serial1.available())  {                           // data received from smartphone
   //digitalWrite(13, HIGH);
   delay(1);
   cmd[0] =  Serial1.read();  
   if(cmd[0] == STX)  {
     digitalWrite(13, HIGH);
     int i=1;      
     while(Serial1.available())  {
       delay(1);
       cmd[i] = Serial1.read();
       if(cmd[i]>127 || i>7)                 break;     // Communication error
       if((cmd[i]==ETX) && (i==2 || i==7))   break;     // Button or Joystick data
       i++;
     }
     if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
     else if(i==7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
   }
 }
 digitalWrite(13, LOW); 
}

void getJoystickState(byte data[8])    {
 int joyX = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48);       // obtain the Int from the ASCII representation
 int joyY = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
 x = joyX;                                                  // Offset to avoid
 y = joyY;                                                  // transmitting negative numbers

 if(joyX<100 || joyX>300 || joyY<100 || joyY>300)     return;      // commmunication error

 if (y > 200)
  {
    forward();
    
    if (x <= 200)  //jeśli x = 0 to i tak pojedzie prosto (y*2 + 0)
    {
      analogWrite(AIN1, slow_pwm());
      analogWrite(BIN1, fast_pwm());
    }
    else           //(x > 200)
    {
      analogWrite(AIN1, fast_pwm());
      analogWrite(BIN1, slow_pwm());
    }
  } else if (y < 200)
  {
    reverse();
    
    if (x <= 200)  //jeśli x = 0 to i tak pojedzie prosto (y*2 + 0)
    {
      analogWrite(AIN2, slow_pwm());
      analogWrite(BIN2, fast_pwm());
    }
    else           //(x > 200)
    {
      analogWrite(AIN2, fast_pwm());
      analogWrite(BIN2, slow_pwm());
    }
  }
  else
  {
    idle();
    pwm_s = 0;
    pwm_f = 0;
  }
}

void getButtonState(int bStatus)  {
  
}

void forward()
{
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);
}

void reverse()
{
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
}

void idle()
{
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);
}

int slow_pwm()  //zwolnij obroty
{
  pwm_s = min_pwm + (abs(y-200) - abs(x-200));  //abs(y-200)*2 dla 4,2V
  if (pwm_s <= min_pwm) pwm_s = 0;
  if (pwm_s >= max_pwm) pwm_s = max_pwm;
  
  return pwm_s;
}

int fast_pwm()  //normalne obroty
{
  pwm_f = min_pwm + abs(y-200);  //abs(y-200)*2 dla 4,2V
  if (pwm_f <= min_pwm) pwm_f = 0;
  if (pwm_f >= max_pwm) pwm_f = max_pwm;
  
  return pwm_f;
}

void getVmot()
{
  fromA0 = analogRead(A0);
  Vmot = fromA0 * (5.0 / 1023.0);
}

void sendBlueToothData()  {
 static long previousMillis = 0;                             
 long currentMillis = millis();
 if(currentMillis - previousMillis > sendInterval) {   // send data back to smartphone
   previousMillis = currentMillis; 

// Data frame transmitted back from Arduino to Android device:
// < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >  
// < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

   Serial.print((char)STX);                                           // Start of Transmission
   Serial.print(buttons);                 Serial.print((char)0x1);   // buttons status feedback
   Serial.print(Vmot);                    Serial.print((char)0x4);   // datafield #1
   Serial.print(pwm_s);                   Serial.print((char)0x5);   // datafield #2
   Serial.print(pwm_f);                                               // datafield #3
   Serial.print((char)ETX);                                           // End of Transmission
 }
}
  
