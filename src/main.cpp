
/* Sketch emulates full duplex transmission of original protocol */
/* Courtesy of https://electro.club/forum/podklyuchenie_alternativnogo_akkumulyatora_k_syaokatu&page=23*/
/* External quartz crystal is a must! See BOM */

/* ------------------- Pinout -------------------- */
/*                   o ________                    */
/*         RESET   --|        |--  +3.3            */
/*  osc1 - PB3     --| attiny |--  PB2 - sense     */
/*  osc2 - PB4     --|   85   |--  PB1 - rx        */
/*         GND     --|________|--  PB0 - tx        */
/*                                                 */
/* ----------------------------------------------- */
#define DEBUG 0
#include "Arduino.h"
#define ANALOG_PIN A5
#define REGENERATIVE_CUTOFF 4180
#define ADC_DEPTH 6
#define ADC_SAMPLES 64 /* pow(2,ADC_DEPTH) */

/* Pre-definitions and global variables */
#define r1 1000 //верхний кОм (Rbat1)
#define r2 20 //нижний,кОм (Rbat2)

uint16_t Capacity = 6600;        /* 16 bytes of capacity, 65536 is max */
#define CAP_HI (Capacity & 0xFF)
#define CAP_LO (Capacity >> 8)

uint8_t  Message[9] = { 0,0,0,0,0,0,0,0,0 }; /* Temp message buffer */
uint8_t  iterator = 0;

float ref = 1.1;
float voltage; //действующее напряжение
float voltageAvg; //сглаженное напряжение
int charge;

int k = 1;  //начальный коэффициент сглаживания (ускоряет выход на действущее значение при старте)
int ku = 10; //постоянный коэффициент сглаживания значений напряжения

#define SERIAL_ELEMENTS 10 //элементов в батарее последовательно

//напряжение (В) на батарее при уровне заряда  (напряжение на 1 элементе * 100 * кол-во элементов последовательно)
#define v0 (2.50 * 100 * SERIAL_ELEMENTS)   // 0%
#define v5 (2.85 * 100 * SERIAL_ELEMENTS)   // 5%
#define v10 (3.10 * 100 * SERIAL_ELEMENTS)  // 10%
#define v20 (3.33 * 100 * SERIAL_ELEMENTS)  // 20%
#define v30 (3.48 * 100 * SERIAL_ELEMENTS)  // 30%
#define v90 (4.02 * 100 * SERIAL_ELEMENTS)  // 90%
#define v100 (4.15 * 100 * SERIAL_ELEMENTS) // 100%

/* ------------------------------- COMMAND LIST ------------------------------- */
/* Battery serial ESC */
uint8_t cmd1012[] = { 0x55,0xAA,0x12,0x25,0x01,0x10,'*','Y','A','E','B','L','A','N',' ', 'c','=','=','3','*',0x00,0x02 };
/* Battery serial MiHome */
uint8_t cmd1014[] = { 0x55,0xAA,0x14,0x25,0x01,0x10,'*','Y','A','E','B','L','A','N',' ', 'c','=','=','3','*',0x00,0x02,CAP_HI,CAP_LO };
/* Charge cycles count */
uint8_t   cmd1B[] = { 0x55,0xAA,0x06,0x25,0x01,0x1B,0x01,0x00,0x01,0x00 };
/* Manufacture date */
uint8_t   cmd20[] = { 0x55,0xAA,0x08,0x25,0x01,0x20,0x87,0x25,0x00,0x00,0x00,0x00 };
/* Battery status ESC */
                           /*LENG/ADDR/READ/CMND/MODE/STAT/CAPACITY /CHARGE   /CURRENT  /VOLTAGE  /TEMP    */
uint8_t   cmd30[] = { 0x55,0xAA,0x0E,0x25,0x01,0x30,0x01,0x00,0xDE,0xAD,0xBE,0x00,0x10,0x00,0xFA,0xCE,0x2A,0x2B };
/* Battery status MiHome */
                           /*LENG/ADDR/READ/CMND/CAPACITY /CHARGE   /CURRENT  /VOLTAGE  /TEMP    */
uint8_t   cmd31[] = { 0x55,0xAA,0x0C,0x25,0x01,0x31,0xDE,0xAD,0xBE,0x00,0x10,0x00,0xFA,0xCE,0x2A,0x2B };
/* Battery ping */
uint8_t   cmd3B[] = { 0x55,0xAA,0x04,0x25,0x01,0x3B,0x62,0x00 };
/* Battery cell voltages */
uint8_t  cmd40[] = { 0x55,0xAA,0x20,0x25,0x01,0x40,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE,0xCE };
/* ----------------------------------------------------------------------------- */

float NormalizeBridge()
{
  /*    -[10MOm]-|-[750kOm]-
   * 5v calibrated is 74 after division.
   * So, to get approx voltage, divide by 15    */
   /* Drop 4 least significant bits */
  // uint16_t dropped = voltage & 0x3F0;
  // return (uint16_t)(((float)dropped / 15.0) * 100.0) - VbattOffset;
  return ((float)analogRead(ANALOG_PIN) * ref / 1023.0) * ((r1 + r2) / r2) * 100.0;
}

void setup()
{
  analogReference(INTERNAL);
  pinMode(ANALOG_PIN, INPUT);
  Serial.begin(115200);
  // for (uint8_t i = 0;i < ADC_SAMPLES;i++)
  //   ADCBuffer[i] = 4100;
}

uint8_t VoltageToPercent(uint16_t voltage)
{
  /* Drop last digit
  voltage=voltage/10;
  voltage=voltage*10;
  Convert, using approximate discharge curves
  float vcell=float(((float)voltage)/1000.0);
  uint8_t percent=((45.078)*(vcell*vcell)-238.258*(vcell)+312.0);
  if(percent<0)   return 0;
  if(percent>100) return 100;
  return percent;
  ------------------------------- */

  //уровень заряда % с линеаризацией
  if (voltage <= v0) charge = 0;
  if (v0 < voltage && voltage <= v5) charge = 0 + (5 - 0) * (voltage - v0) / (v5 - v0);
  if (v5 < voltage && voltage <= v10) charge = 5 + (10 - 5) * (voltage - v5) / (v10 - v5);
  if (v10 < voltage && voltage <= v20) charge = 10 + (20 - 10) * (voltage - v10) / (v20 - v10);
  if (v20 < voltage && voltage <= v30) charge = 20 + (30 - 20) * (voltage - v20) / (v30 - v20);
  if (v30 < voltage && voltage <= v90) charge = 30 + (90 - 30) * (voltage - v30) / (v90 - v30);
  if (v90 < voltage && voltage <= v100) charge = 90 + (100 - 90) * (voltage - v90) / (v100 - v90);
  if (voltage > v100) charge = 100;
  return charge;
}

void UpdateStats()
{
  /* Get sample */
  /*uint16_t Sample=NormalizeBridge(analogRead(ANALOG_PIN));
  Serial.print("NormalizeBridge: ");
  Serial.println(Sample);*/
  /* -------------------------------- */

  /* Overvoltage protection, ! EXPERIMENTAL !
  if(Sample>REGENERATIVE_CUTOFF) cmd30[7]=0x02; //Bit 9 supposed to be overvoltage feedback
  else cmd30[7]=0x00;
   ---------- */

   /* Average
   ADCBuffer[iterator]=(uint16_t)Sample;
   iterator=(iterator==ADC_SAMPLES-1)?0:iterator+1;
   uint32_t Average=0;
   for(uint8_t i=0;i<ADC_SAMPLES;i++)
     Average+=ADCBuffer[i];
   ----------------- */

   //сглаживаем напряжение батареи
  voltage = NormalizeBridge();
  voltageAvg = voltageAvg - (voltageAvg - voltage) / k;
  if (k < ku) k++;  //постепенно увеличиваем время сглаживания до заданного, чтобы быстрее получить правильное значение на старте 
  unsigned int voltageBat = voltageAvg;

#if DEBUG == 1
  Serial.println("\tUpdateStats()");
  Serial.print("voltage: ");
  Serial.println(voltage);
  Serial.print("voltageAvg: ");
  Serial.println(voltageAvg);
  Serial.print("voltageBat: ");
  Serial.println(voltageBat);
#endif

  //uint16_t BatteryVoltage=(uint16_t)(Average>>ADC_DEPTH);
  //Serial.print("BatteryVoltage: ");
  //Serial.println(BatteryVoltage);

  /* Update battery voltage */
  cmd30[14] = cmd31[12] = voltageBat & 0xFF;
  cmd30[15] = cmd31[13] = voltageBat >> 8;


  /* Update remaining power */
  uint8_t  RemainingPower = VoltageToPercent(voltageBat);
  cmd30[10] = cmd31[8] = RemainingPower;
  /* ---------- */

  /* Update remaining capacity */
  uint16_t RemainingCapacity = (float)(Capacity) * ((float)RemainingPower / 100.0);
  cmd30[8] = cmd31[6] = RemainingCapacity & 0xFF;
  cmd30[9] = cmd31[7] = RemainingCapacity >> 8;
  /* ------------------ */

#if DEBUG == 1
  Serial.print("RemainingPower: ");
  Serial.println(RemainingPower);
  Serial.print("RemainingCapacity: ");
  Serial.println(RemainingCapacity);
#endif

}

void UpdateCells()
{

  /* Get battery voltage */
  uint16_t BatteryVoltage = NormalizeBridge();
  /* ----------------- */

#if DEBUG == 1
  Serial.println("\tUpdateCells()");

  Serial.print("BatteryVoltage: ");
  Serial.println(BatteryVoltage);
#endif

  /* Update each cell */
  for (uint8_t i = 6;i < 36;i += 2)
  {
    cmd40[i] = BatteryVoltage & 0xFF;;
    cmd40[i + 1] = BatteryVoltage >> 8;
  }
  /* ---------------- */
}

void SendCMD(uint8_t* cmd)
{
  /* Calc CRC */
  uint16_t crc = 0xFFFF;
  for (int i = 2;i < cmd[2] + 4;i++)
    crc -= cmd[i];
  /* ---------- */

  /* Prepare bytes before transmit to maintain bitrate, these values have to be calculated ! before ! transmission */
  uint8_t crc_high = crc & 0xFF;
  uint8_t crc_low = crc >> 8;
  /* ---------- */

  /* Drop CMD into Serial */
  noInterrupts();
  Serial.write(cmd, cmd[2] + 4);
  Serial.write(crc_high);
  Serial.write(crc_low);
  interrupts();
  /* -------------------- */
}

void loop()
{
  if (Serial.available() >= 9)
  {
    if (Serial.peek() == 0x55)
    {
      for (uint8_t i = 0;i < 9;i++)
        Message[i] = Serial.read();
      switch (Message[5])
      {
      case (0x10): { if (Message[6] == 0x10) SendCMD(cmd1012); else SendCMD(cmd1014);}  break;
      case (0x1B): { SendCMD(cmd1B);} break;
      case (0x20): { SendCMD(cmd20);} break;
      case (0x30): { UpdateStats(); SendCMD(cmd30);} break;
      case (0x31): { UpdateStats(); SendCMD(cmd31);} break;
      case (0x3B): { SendCMD(cmd3B);} break;
      case (0x40): { UpdateCells(); SendCMD(cmd40);} break;
      default: break;
      };
    }
    else Serial.read();
  }
}
