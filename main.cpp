#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include <util/delay.h>
#include <LiquidCrystal_I2C.h>

const int stepPin = 6;

int val;

int inputCLK = 2;

int inputDT = 3;

int speedCounter = 0;

int prevCLK = LOW;
int currentCLK;

int n = LOW;

int maxKnob = 20;

int stepperSpeed = 2000;

int maxSpeed = 600;

int minSpeed = 2000;

LiquidCrystal_I2C lcd(0x27, 16, 2);

int temp;
int kp = 7;   // half of 14 = 7;//2;
int ki = 0.7; // 0.4;
int kd = 1;   // 14;  // 2;
int PID_p = 0;
int PID_i = 0;
int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int val, cycleTimer, realTimeCounter;
long R;
double thermistor, outputVoltage, thermistorResistance;
double pidError = 0, prevError = 0;
double setTemp = 70;
double pidValue = 0;

double time, prevTime, elapsedTime;

void lcdDisplay()
{
  TIMSK1 = (1 << OCIE1A);
  sei();
  TCCR1B = (1 << CS12) | (1 << WGM12);
  OCR1A = 12500;
  // TCNT1 = 0;
}

void adc_init()
{
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // enable ADC, and set pre scalar to 128
  DIDR0 = (1 << ADC0D);
  ADMUX = (1 << REFS0);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
}

int get_adc()
{
  // using AVcc as ref
  ADCSRA |= (1 << ADSC); // start conversion
  // while ((ADCSRA & (1 << ADIF)) == 0);
  // ADCSRA |= (1 << ADIF);

  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
  return ADC;
}

double getTemp()
{
  val = get_adc(); /* store adc value on val register */
  outputVoltage = ((val * 5.0) / 1023);
  thermistorResistance = ((5 * (10.0 / outputVoltage)) - 10) * 1000;
  // R=((102300000/val) - 100000);/* calculate the resistance */
  thermistor = log(thermistorResistance); /* calculate natural log of resistance */
  /* Steinhart-Hart Thermistor Equation: */
  /*  Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)		*/
  /*  where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8  */
  thermistor = 1 / (0.001129148 + (0.000234125 * thermistor) + (0.0000000876741 * thermistor * thermistor * thermistor));
  thermistor = thermistor - 273.15; /* convert kelvin to °C */

  return thermistor;
}

void pidInit()
{
  DDRD = (1 << PD6);
  TCCR0A = (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);
  // TIMSK0 = (1 << TOIE0);
  // sei();
  TCCR0B = (1 << CS00) | (1 << CS01);
  // TCNT0 = 0;
  // OCR0A = 0;

  time = millis(); // realTimeCounter + TCNT0;
}

void pidController()
{
  pidError = setTemp - getTemp();
  PID_p = kp * pidError;
  PID_i = PID_i + (ki * pidError);
  // if (pidError > -3 && pidError < 3) {
  // ki = 20;
  // } // calculate Proportional value
  // if(-3 < pidError <3)
  // {
  //   PID_i = PID_i + (ki * pidError);
  // }

  prevTime = time;
  time = millis(); // realTimeCounter + TCNT0;
  elapsedTime = (time - prevTime) / 1000;
  // Now we can calculate the D calue
  PID_d = kd * ((pidError - prevError) / elapsedTime);
  //   if (PID_d < 0) {
  //   PID_d = 0;
  // } else if (PID_d > 255) {
  //   PID_d = 255;
  // }
  // Final total PID value is the sum of P + I + D
  pidValue = PID_p + PID_i + PID_d;

  OCR0A = pidValue;

  if (pidValue < 0)
  {
    OCR0A = 0;
  }
  if (pidValue > 255)
  {
    OCR0A = 255;
  }

  prevError = pidError;
}

void motorStep(int MAX)
{
  for (int x = 0; x < MAX; x++)
  {

    PORTD |= (1 << PD6);
    delayMicroseconds(stepperSpeed);

    PORTD &= ~(1 << PD6);
    delayMicroseconds(stepperSpeed);
  }
}

void rotaryInterrupt()
{
  EICRA |= (1 << ISC00);
  EIMSK |= (1 << INT0);
  sei();
}

void setup()
{
  lcd.init();
  lcd.backlight();
  adc_init();
  pidInit();
  DDRD |= (1 << PD6);
  PORTD |= (1 << PD2) | (1 << PD3);

  rotaryInterrupt();
}

void loop()
{
  motorStep(1);
  temp = getTemp();
  pidController();
  lcd.setCursor(0, 0);
  lcd.print("T: ");

  lcd.setCursor(3, 0);
  lcd.print(temp);
}

ISR(INT0_vect)
{
  currentCLK = digitalRead(inputCLK);

  if ((prevCLK == LOW) && (currentCLK == HIGH))
  {

    if (digitalRead(inputDT) == LOW)
    {

      speedCounter--;
    }
    else
    {

      speedCounter++;
    }
    if (speedCounter > maxKnob)
    {

      speedCounter = maxKnob;
    }
    else if (speedCounter < 0)
    {

      speedCounter = 0;
    }
    int r = speedCounter * 100;
    r = minSpeed - r;
    r = (r < maxSpeed) ? maxSpeed : r;
    stepperSpeed = r;
    Serial.println(r);
  }

  prevCLK = n;
}