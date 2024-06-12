#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include <util/delay.h>
#include <LiquidCrystal_I2C.h>

byte Degree[8] =
    {
        0b00110,
        0b01001,
        0b01001,
        0b00110,
        0b00000,
        0b00000,
        0b00000,
        0b00000};

const int stepPin = 6;

int inputCLK = 2;

int inputDT = 3;

int prevCLK = LOW;
int currentCLK;

LiquidCrystal_I2C lcd(0x27, 16, 2);

double temp;
int lastTemp = 0;
int lastOCR = 0;
int kp = 125; // half of 14 = 7;//2;
int ki = 8;   // 0.4;
int kd = 5;   // 14;  // 2;
int PID_p = 0;
int PID_i = 0;
int PID_d = 0;
int timerCount = 0;

int val, cycleTimer, realTimeCounter;
long R;
double thermistor, outputVoltage, thermistorResistance;
double pidError = 0, prevError = 0;
double setTemp = 240.0;
double pidValue = 0;

double time, prevTime, elapsedTime;

void tempDisplay()
{
  TIMSK2 = (1 << TOIE2);
  sei();
}

void adc_init()
{
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // enable ADC, and set pre scalar to 128
  DIDR0 = (1 << ADC0D);
  ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << MUX1);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
}

void stepperInit()
{
  DDRD |= (1 << PD6);
  DDRB |= (1 << PB2);
  PORTB |= (1 << PB2);
  TCCR0A = (1 << COM0A0) | (1 << WGM01);
  TCCR0B = (1 << CS02);
  OCR0A = 25;
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
  DDRB = (1 << PB3);
  TCCR2A = (1 << COM2A1) | (1 << WGM20) | (1 << WGM21);
  TCCR2B = (1 << CS22) | (1 << CS20) | (1 << CS21);

  time = millis(); // realTimeCounter + TCNT0;
}

void pidController()
{
  pidError = setTemp - getTemp();
  PID_p = 0.1 * kp * pidError;
  PID_i = 0.1 * PID_i + (ki * pidError);

  prevTime = time;
  time = millis(); // realTimeCounter + TCNT0;
  elapsedTime = (time - prevTime) / 1000;
  // Now we can calculate the D calue
  PID_d = 0.1 * kd * ((pidError - prevError) / elapsedTime);

  // Final total PID value is the sum of P + I + D
  pidValue = PID_p + PID_i + PID_d;

  OCR2A = pidValue;

  if (pidValue < 0)
  {
    OCR2A = 0;
  }
  if (pidValue > 255)
  {
    OCR2A = 255;
  }

  prevError = pidError;
}

void rotaryInit()
{
  PORTD |= (1 << PD2) | (1 << PD3);
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

  lcd.createChar(0, Degree);
  adc_init();
  pidInit();
  rotaryInit();
  stepperInit();
  tempDisplay();

  rotaryInterrupt();

  temp = getTemp();

  lcd.setCursor(4, 0);
  lcd.print("PETAMEN");
  lcd.setCursor(4, 1);
  lcd.print("MACHINE");

  _delay_ms(3000);
  lcd.clear();
}

void loop()
{
  pidController();
  lcd.setCursor(0, 0);
  lcd.print("T: ");

  if (temp != lastTemp)
  {
    lcd.setCursor(3, 0);
    lcd.print("       ");
    lcd.setCursor(3, 0);
    lcd.print(temp);

    lastTemp = temp;
  }

  lcd.setCursor(9, 0);
  lcd.print("/");
  lcd.setCursor(10, 0);
  lcd.print((uint16_t)setTemp);
  lcd.setCursor(13, 0);
  lcd.write(byte(0));
  lcd.setCursor(14, 0);
  lcd.print("C");

  lcd.setCursor(0, 1);
  lcd.print("Speed:");

  if (OCR0A != lastOCR)
  {
    lcd.setCursor(7, 1);
    lcd.print("   ");
    lcd.setCursor(7, 1);
    lcd.print(255 - OCR0A);

    lastOCR = OCR0A;
  }
}

ISR(INT0_vect)
{
  currentCLK = digitalRead(inputCLK);

  if ((prevCLK == LOW) && (currentCLK == HIGH))
  {

    if (digitalRead(inputDT) == LOW)
    {

      OCR0A -= 2;
    }
    else
    {

      OCR0A += 2;
    }
    if (OCR0A > 255)
    {

      OCR0A = 255;
    }
    else if (OCR0A < 0)
    {

      OCR0A = 0;
    }
  }
  prevCLK = LOW;
}

ISR(TIMER2_OVF_vect)
{
  timerCount++;
  if (timerCount == 91) // 61
  {
    timerCount = 0;
    temp = getTemp();
  }
}