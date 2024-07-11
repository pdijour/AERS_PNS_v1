#include <SPI.h>
#include "protocentral_afe44xx.h"
#include <Adafruit_MCP4728.h>
#include <Wire.h>

Adafruit_MCP4728 DAC;

#define AFE44XX_CS_PIN   25
#define AFE44XX_PWDN_PIN 4

AFE44XX afe44xx(AFE44XX_CS_PIN, AFE44XX_PWDN_PIN);

afe44xx_data afe44xx_raw_data;
int32_t heart_rate_prev=0;
int32_t spo2_prev=0;

int16_t ppg_wave_ir;

void setup()
{
  Serial.begin(115200);
  Serial.println("Intilaziting AFE44xx.. ");
  
  SPI.begin();
  afe44xx.afe44xx_init();
  Serial.println("Inited...");
  // Try to initialize!
  if (!DAC.begin(0x60)) {
    Serial.println("Failed to find DAC Module");
    while (1) {
      delay(10);
    }
  }
  Serial.println("DAC Module Found!");
}

void loop()
{
    delay(5);
    
    afe44xx.get_AFE44XX_Data(&afe44xx_raw_data);
    ppg_wave_ir = (int16_t)(afe44xx_raw_data.IR_data >> 8);
    ppg_wave_ir = ppg_wave_ir;
    // Serial.println(ppg_wave_ir);

    if (afe44xx_raw_data.buffer_count_overflow)
    {
      if(afe44xx_raw_data.aRED_data > 100000)
      {
        Serial.println("Probe OFF!");
        heart_rate_prev = 0;
        spo2_prev = 0;
      }
      else if ((heart_rate_prev != afe44xx_raw_data.heart_rate) || (spo2_prev != afe44xx_raw_data.spo2))
      {
        heart_rate_prev = afe44xx_raw_data.heart_rate;
        spo2_prev = afe44xx_raw_data.spo2;

        // Serial.print("calculating sp02...");
        // Serial.print(" Sp02 : ");
        // Serial.print(afe44xx_raw_data.spo2);
        // Serial.print("% ,");
        // Serial.print("Pulse rate :");
        // Serial.print(afe44xx_raw_data.heart_rate);
        // Serial.println(" bpm");

        // Serial.print(afe44xx_raw_data.aIR_data);
        // Serial.print(", ");
        Serial.println(afe44xx_raw_data.aRED_data);
        // Serial.print(", ");
        // Serial.print(afe44xx_raw_data.IR_data);
        // Serial.print(", ");
        // Serial.println(afe44xx_raw_data.RED_data);
      } 
    }
    if(spo2_prev == 0){
      DAC.setChannelValue(MCP4728_CHANNEL_B, 0);
      DAC.setChannelValue(MCP4728_CHANNEL_C, ppg_wave_ir);
    }else{
      DAC.setChannelValue(MCP4728_CHANNEL_B, ppg_wave_ir);
      DAC.setChannelValue(MCP4728_CHANNEL_C, ppg_wave_ir);
    }
    DAC.setChannelValue(MCP4728_CHANNEL_A, map(spo2_prev,0,100,0,4095/3.3));
    DAC.setChannelValue(MCP4728_CHANNEL_D, map(spo2_prev,0,100,0,4095/3.3));
}