#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> 
#include <Adafruit_Sensor.h> 
#include <Adafruit_ADXL345_U.h>
int cocount;
bool coflag=false;
const int buttonPin = 2;
int cancelcount=0;
float X,Z,Y;
bool fl;
int RXPin = 4; //pin 
int TXPin = 3;
const int buzzer = 7;
int AlcoholAnalog=A0;
int COAnalog=A1;
TinyGPSPlus gps;
SoftwareSerial SerialGPS(TXPin, RXPin);
SoftwareSerial mySerial(9, 10);
hd44780_I2Cexp lcd;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

void setup() {
  
  Serial.begin(9600);
  mySerial.begin(9600); 
  SerialGPS.begin(9600);

  lcd.begin(16,2);

  pinMode(buzzer, OUTPUT);
  Serial.println("setup");
  if(!accel.begin())
   {
      Serial.println("No valid sensor found");

   }

  lcd.print("GPS setting");
  delay(5000);
  lcd.noBacklight();
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), button_ISR, RISING);
 
}


void loop() {
  
  int Alcohol = analogRead(AlcoholAnalog);
  int CO = analogRead(COAnalog);
  sensors_event_t event; 
  accel.getEvent(&event);
  if(fl==false)
  {
        
        X=event.acceleration.x;
        Z=event.acceleration.z;
        Y=event.acceleration.y;
        fl=true;
  }
  bool c=DiffCheck(event.acceleration.x,event.acceleration.z,event.acceleration.y);
  if(c==true)
  {
      Serial.println("crashed");
      Display("crashed","!!");
      GPSData();
      
      delay(500);
  }
  Serial.println(Alcohol);
  Serial.println(CO);
  if(Alcohol > 280 && CO>200)
  {
    cocount=0;
    Display("Alcahol detected","!!");
    alcomessage("The person who  drives is drunk");
    Serial.println("alco");
    delay(15000);
  }
  if(CO>140 && Alcohol>280 && CO<200)
  {
    cocount++;
    coflag=true;
    if(cocount>2){
    Buzzer();
    Display("carbon monoxide  are detected","!!");
    Serial.println("co");
    delay(15000);
    }
  }
  if (coflag==false)
  {
    cocount=0;
  }
  coflag=false;
  delay(2000);
  cancelcount=0;
}
 

bool DiffCheck(float tx,float tz,float ty) //
{
  if((X-tx)>4||(X-tx)<-4)
    return true;
  if((Z-tz)>5||(Z-tz)<-5)
    return true;
  if((Y-ty)>4||(Y-ty)<-4)
    return true;
  return false;
}


void Buzzer()
{
  tone(buzzer, 1000);
  delay(2000);  
  noTone(buzzer);
  delay(1000);  
}
void   Display(String msg,String msg2)//Lcd Display Printing
{
  lcd.begin(20, 4);
  lcd.print(msg);
  
  lcd.setCursor(0, 1);
  lcd.print(msg2);
delay(3000);
  DisplayTurnOFF();
}

void DisplayTurnOFF() //Turn off lcd
{
  lcd.begin(20, 4);
  lcd.print("");
  lcd.noBacklight();
}
void GPSData()
{

 while (SerialGPS.available() > 0){
  
    if (gps.encode(SerialGPS.read())){
      sendMessage();}
 }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("GPS NOT DETECTED!");
    GSMMessage("Emergency and Accident is occur. Gps location is Hard to Find");
  }
}


void sendMessage() //Check the Signal Of GPS and send Message
{
  int count=0;
  bool flag=false;
  while(flag==false){
  Serial.print("Latitude:");
  Serial.println((gps.location.lat()));
  if (gps.location.isUpdated()){
    float v,l;
    Serial.print("https://maps.google.com/?q="); //junk
    v=gps.location.lat();
    Serial.print(v, 6);//junk
    Serial.print(",");//junk
    l=gps.location.lng();//junk
    Serial.println(l, 6);//junk
    String s="Emergency and Accident is occur. Location:https://maps.google.com/?q=";
    Serial.println(l, 6);
    String msg=s+v+","+l;
    Serial.println(msg);
    delay(5000);
    GSMMessage(msg);
    delay(2000);

    flag=true;

  }
  else
  {
    Serial.println("Location is not available");
    count++;
    flag=false;
  }
  if(count==10)
  {
        String Msg="Emergency and Accident is occur. Gps location is Hard to Find.";
        GSMMessage(Msg);
        delay(2000);
       flag=true;
  }
  delay(5000);
  }
}

void alcomessage(String msg) 
{

   Serial.println("cancelbutton");
   
   delay(10000);
   Serial.println(cancelcount);
   if(cancelcount==0)
    {
    mySerial.println("AT+CMGF=1");
    delay(1000);
    mySerial.println("AT+CMGS=\"8593014459\"\r");
    delay(1000);
    mySerial.print(msg);
    delay(100);
    mySerial.write(0x1a);
    }
    else
    {
       Serial.println(cancelcount);
    }
}
void GSMMessage(String msg) 
{

   Serial.println("cancelbutton");
   
   delay(10000);
   Serial.println(cancelcount);
   if(cancelcount==0)
    {
    mySerial.println("AT+CMGF=1");
    delay(1000);
    mySerial.println("AT+CMGS=\"******\"\r");
    delay(1000);
    mySerial.print(msg);
    delay(100);
    mySerial.write(0x1a);
     delay(5000);
     mySerial.println("ATD*****"); 
     delay(10000);
     mySerial.println("ATH"); 
    }
    else
    {
       Serial.println(cancelcount);
    }
}


void button_ISR()
{

  cancelcount++;
  
}
