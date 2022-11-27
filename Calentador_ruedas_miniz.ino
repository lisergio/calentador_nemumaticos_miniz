//#include <Wire.h>
//#include <LiquidCrystal_I2C.h>

#include <LiquidCrystal.h>
LiquidCrystal lcd(7,8,9,10,11,12);


#include "thermistor.h"
#include "HardwareSerial.h"

//LiquidCrystal_I2C lcd(0x3F, 20, 4);


THERMISTOR thermistorFL(A0,
                        100000,
                        3950,
                        100000);

THERMISTOR thermistorFR(A1,
                        100000,
                        3950,
                        100000);

THERMISTOR thermistorRL(A2,
                        100000,
                        3950,
                        100000);

THERMISTOR thermistorRR(A3,
                        100000,
                        3950,
                        100000);

boolean estado_calefactor = LOW;

int pulsador = 2;
boolean estado_pulsador;
int led_estado = 13;



float tempFL;
float tempFR;
float tempRL;
float tempRR;

int OutFL = 3;
int OutFR = 4;
int OutRL = 5;
int OutRR = 6;

int pot = A5;
int target_temp;

void setup()
{
  pinMode (led_estado, OUTPUT);
  pinMode (pulsador, INPUT);
  pinMode (OutFL, OUTPUT);
  pinMode (OutFR, OUTPUT);
  pinMode (OutRL, OUTPUT);
  pinMode (OutRR, OUTPUT);
  digitalWrite (OutFL, LOW);
  digitalWrite (OutFR, LOW);
  digitalWrite (OutRL, LOW);
  digitalWrite (OutRR, LOW);
  lcd.begin(20, 4);
  //lcd.init();
  //lcd.backlight();
  lcd.setCursor(0, 0);
  // Serial.begin(9600);
  lcd.setCursor(8, 2);
  lcd.print("   ");
  lcd.setCursor(8, 2);
  lcd.print("OFF");
  lcd.setCursor(2, 1);
  lcd.print("FL");
  lcd.setCursor(15, 1);
  lcd.print("FR");
  lcd.setCursor(2, 2);
  lcd.print("RL");
  lcd.setCursor(15, 2);
  lcd.print("RR");
}


void loop()
{
  target_temp = analogRead(pot);
  target_temp = map(target_temp, 0, 1024, 20, 80);

  estado_pulsador = digitalRead(pulsador);
  if (estado_pulsador == HIGH)
  {
    if (estado_calefactor == LOW)
    {
      estado_calefactor = HIGH;
      digitalWrite(led_estado, HIGH);
      lcd.setCursor(8, 2);
      lcd.print("   ");
      lcd.setCursor(8, 2);
      lcd.print("ON");
    }
    else
    {
      estado_calefactor = LOW;
      digitalWrite(led_estado, LOW);
      lcd.setCursor(8, 2);
      lcd.print("   ");
      lcd.setCursor(8, 2);
      lcd.print("OFF");

    }
    delay(500);
  }

  delay (500);


  lcd.setCursor(8, 1);
  lcd.print(target_temp);
  lcd.print((char)223);
  lcd.print("C");

  leer_temperatura();
  if (estado_calefactor == HIGH)
  {
    calentar();
  }
  else
  {
    digitalWrite(OutFL, LOW);
    digitalWrite(OutFR, LOW);
    digitalWrite(OutRL, LOW);
    digitalWrite(OutRR, LOW);
    lcd.setCursor(7, 0);
    lcd.print(" ");
    lcd.setCursor(12, 0);
    lcd.print(" ");
    lcd.setCursor(7, 3);
    lcd.print(" ");
    lcd.setCursor(12, 3);
    lcd.print(" ");
  }

}

void calentar()
{
  if (tempFL <= target_temp)
  {
    digitalWrite(OutFL, HIGH);
    lcd.setCursor(7, 0);
    lcd.print("*");
  }
  else
  {
    digitalWrite(OutFL, LOW);
    lcd.setCursor(7, 0);
    lcd.print(" ");
  }

  if (tempFR <= target_temp)
  {
    digitalWrite(OutFR, HIGH);
    lcd.setCursor(12, 0);
    lcd.print("*");
  }
  else
  {
    digitalWrite(OutFR, LOW);
    lcd.setCursor(12, 0);
    lcd.print(" ");
  }

  if (tempRL <= target_temp)
  {
    digitalWrite(OutRL, HIGH);
    lcd.setCursor(7, 3);
    lcd.print("*");
  }
  else
  {
    digitalWrite(OutRL, LOW);
    lcd.setCursor(7, 3);
    lcd.print(" ");
  }
  if (tempRR <= target_temp)
  {
    digitalWrite(OutRR, HIGH);
    lcd.setCursor(12, 3);
    lcd.print("*");
  }
  else
  {
    digitalWrite(OutRR, LOW);
    lcd.setCursor(12, 3);
    lcd.print(" ");
  }
}

void leer_temperatura()
{
  tempFL = thermistorFL.read();
  lcd.setCursor(0, 0);
  tempFL = (tempFL / 10 );
  lcd.print(tempFL);
  lcd.print((char)223);
  lcd.print("C");


  tempFR = thermistorFR.read();
  lcd.setCursor(13, 0);
  tempFR = (tempFR / 10 );
  lcd.print(tempFR);
  lcd.print((char)223);
  lcd.print("C");

  tempRL = thermistorRL.read();
  lcd.setCursor(0, 3);
  tempRL = (tempRL / 10 );
  lcd.print(tempRL);
  lcd.print((char)223);
  lcd.print("C");

  tempRR = thermistorRR.read();
  lcd.setCursor(13, 3);
  tempRR = (tempRR / 10 );
  lcd.print(tempRR);
  lcd.print((char)223);
  lcd.print("C");
}
