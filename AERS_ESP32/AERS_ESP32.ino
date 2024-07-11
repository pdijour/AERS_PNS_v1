//////////////////For BMP390 Temp+BaroPres Sensor Start///////////////////////////////////
#include <Arduino.h>
#include <Wire.h>  //I2C comms with BMP390 and Teensy
#include <Adafruit_MCP4728.h>
// #include <Adafruit_MCP4725.h>
#include <CircularBuffer.hpp>
#include <Bounce2.h>
#include <BMP388_DEV.h>                 // Include the BMP388_DEV.h library (works the same with the BMP390)
float temperature, pressure, altitude;  // Create the temperature, pressure and altitude variables
BMP388_DEV bmp388;                      // Create the BMP388 Type (works the same with the BMP390)
Adafruit_MCP4728 mcp;
#include "DFRobot_GP8403.h"

#include <elapsedMillis.h>

#define  sensorOn    23
// #define  stimTrigOut    14
#define  stimTrigIn    14
#define extTrigPin1  25
#define extTrigPin2 27
#define spo2_pin 39 // vp
#define ppg_pin 36 // vn
// #define extTrigPin1BoxWrite 26
#define extTrigPin1BoxRead 4
// #define extTrigPin2BoxWrite 5
#define extTrigPin2BoxRead 5
#define stimLED1 19
#define stimLED2 18
#define oxLED 26

elapsedMillis inspTime;
elapsedMillis expTime;
elapsedMicros noDataTime;


//////////////////For BMP390 Temp+BaroPres Sensor End///////////////////////////////////
//////////////////Flow Packet Set-up Start//////////////////
union packed_long {
  int32_t l;
  byte bb[4];
};
packed_long mydataFlow;  ///Variable declaration, 4-byte flow message stored here from SFM3300 to unpack later
//////////////////Flow Packet Set-up End//////////////////

//////////////////Pres Packet Set-up Start//////////////////
float Digoutpmax = 14745;
float Digoutpmin = 1638;
float pmax = 200; //Trace had 200, Phoebe changed to 200
float pmin = -200; //Trace had -200, Phoebe changed to -200
float scaling = (Digoutpmax - Digoutpmin) / (pmax - pmin);

float curPresRaw = 0.0;
float curPresOffset = 0.0;
float presOffsetValue = 0.0;

float baseline_pres = 0.0;
CircularBuffer<float, 200> presBuffer; // Circular buffer for storing recent pressure values
const int windowSizePres = 200; // Define the window size for averaging
float thresholdPres = 5.0;


////////////////////// NEW PRESSURE OFFSET VALUES /////////////////
float PRES_OFFSET = 0.0;
int PRES_OFFSET_STATUS = 0;
float PRES_OFFSET_SUM = 0;
elapsedMillis PRES_OFFSET_TIMER;
unsigned long PRES_OFFSET_TIMER_HOLD = 0;
int PRES_OFFSET_X;
int PRES_OFFSET_Y;
int PRES_OFFSET_COUNTER = 0;
float PRES_OFFSET_BOUNDS = 1.5; ///CHANGE BACK

////////////////////END NEW PRESSURE OFFSET VALUES ////////////////////
union pres_union {
  uint16_t l;
  byte bbb[2];
};
pres_union mydataPres;
//////////////////Pres Packet Set-up End//////////////////

/////////////General Variable Declaration//////////////////
float flowBound = 250.0;
float presBound = 150.0;

#define BUTTON_PIN_26_ZERO_OFFSET 12
// Instantiate a Bounce object
// Bounce debouncer1 = Bounce();

int buttonState = 0;
int lastButtonState = 0;

float rawFlow = 0.0;  //Raw Flow Value from SFM3300 Sensor
float BTPSFlowRaw = 0.0;
float BTPSFlow = 0.0;  //BTPS Conversion value from SFM3300 Sensor and BaroPres/Temp from BMP390
float flowOffsetValue = 0.0;

bool sendSerial = true;
bool sendSerial2 = true;

// CircularBuffer<float, 200> flowOffset;
// CircularBuffer<float, 200> presOffset;

float baroPres = 738.98;  //Nominal Value, will be updated after 5s
float ambTemp = 25.15;    //Nominal Value, will be updated after 5s

/////////////Timing Set-up//////////////////
elapsedMicros fullLoop;  // Used for getting 200Hz rate
float fullLoopHolder = 0;

#include <HardwareSerial.h>

HardwareSerial SerialA(1);

////// Parameters for stimulation ////
DFRobot_GP8403 dac(&Wire,0x5F);
#define DAC_RESOLUTION (9)

int stimGate1 = 32;
int stimGate2 = 12;
int triggerTrainPin1 = 33;
int triggerTrainPin2 = 13;

// fixed params
int n_breaths = 0;
float insp_period = 1500;
float exp_period = 1500;
int frequency = 30;
int wave_type = 2;
// side 1
int allow_stim1 = 0;
float amplitude1 = 5.0;
float base1 = 0.0;
float rise_time1 = 80.0;
float fall_time1 = 80.0;
// side 2
int allow_stim2 = 0;
float amplitude2 = 5.0;
float base2 = 0.0;
float rise_time2 = 80.0;
float fall_time2 = 80.0;
int trigger_type = 0;
float pred_delay = 0.0;
float hfov_base = 0;
int num_breaths_avg = 10;
int i_threshold = 5;
int e_threshold = -10;
int bilevel_t_ = 50;
int bilevel_v = 50;
float bias1 = 0.0;
float bias2 = 0.0;

bool stim1_enable;
bool stim2_enable;

bool sentOnce = false;
bool sentZero = false;

elapsedMicros sinceHigh1;
elapsedMicros sinceLow1;
elapsedMicros sinceHigh2;
elapsedMicros sinceLow2;

elapsedMillis signal_time;
elapsedMillis signal_time1;
elapsedMillis signal_time2;

elapsedMillis off_time;
elapsedMillis off_time_double;
elapsedMillis exp_time;
elapsedMillis off_time_burst;

elapsedMillis signal_time_burst1;
elapsedMillis signal_time_burst2;

elapsedMillis print_time;

long breathCounter = 0;

// Serial Read
const int BUFFER_SIZE = 1000; //1000
char buf[BUFFER_SIZE];
String strs[803]; //803

bool switchOn = false;

bool reading = false;

/////// End parameters for stimulation

// #define SDA_2 18
// #define SCL_2 26

bool synch_1;
bool synch_2;
float signal_time_last;
float signal_time_first;
bool first_end;
bool first_start;

///// New code: set baseline of flow/////
const int windowSize = 1000;  // 1000
CircularBuffer<float, windowSize> data;
float baseline = 0;
float threshold = 0;
float thresholdFactor = 2.0;  // Adjust the threshold factor based on your requirements

///// New code: determine ie state within Arduino ////
int iestate;

#define eBufferSize 10

CircularBuffer<float, eBufferSize> e_times;
CircularBuffer<float, 200> i_times; //200
float lastTimestamp = 0;
float currentTimestamp = 0;
float step;

float baselinePos = 0;
float thresholdPos = 0;
float baselineNeg = 0;
float thresholdNeg = 0;
float piMax = 0;
float peakFlow = 0;
float iTidal = 0;
float eTidal = 0;

bool firstPos = true;
bool firstNeg = true;
float iStart = 0;
float eStart = 0;
float lastIStart = 0;
float lastETime = 0;
float lastPeriod = 0;
float lastITime = 0;

int currentIndex = 0;

float peep = 0;


/// New code: logic for synchronous
bool first = true;
bool first_smth = true;

bool first2 = true;
bool first_smth2 = true;

/// New code: predictive algorithm
float averageETime;

/// New code: values for values tab ////
float value_2 = 0;   //Ti
float value_3 = 0;   //IE ratio
float value_4 = 0;   //PEEP
float value_5 = 0;   //PiMax
float value_6 = 0;   //TVi
float value_7 = 0;   //TVe
float value_8 = 0;   //Resp rate
float value_9 = 0;   //Minute ventilation
float value_10 = 0;  //Peak Flow
float value_11 = 0;  //Flow

bool enable = false;
bool enable2 = false;

float stim_val1;
float stim_val2;

int burst1;
int burst2;
bool start;

bool was_insp;
bool was_exp = false;

int stim_count = 0;

bool responded_1 = true;
bool responded_box_1 = true;
bool responded_2 = true;
bool responded_box_2 = true;

int burst_ext1;
int burst_ext2;
int rando = 0;

bool clear = false;
bool zeroing = false;

void setup() {

  bool status1 = mcp.begin();
  if (!status1) {
    Serial.println("Could not find a valid BME280_1 sensor, check wiring!");
  }

  // pinMode(BUTTON_PIN_26_ZERO_OFFSET, INPUT_PULLUP);
  // debouncer1.attach(BUTTON_PIN_26_ZERO_OFFSET);
  // debouncer1.interval(10);  // interval in ms
  //////////////////BMP390 setup//////////////////////
  Wire.begin();
  bmp388.begin();                            // Default initialisation, place the BMP388 into SLEEP_MODE
  bmp388.setTimeStandby(TIME_STANDBY_40MS);  // Set the standby time to 1.3 seconds
  bmp388.startNormalConversion();            // Start BMP388 continuous conversion in NORMAL_MODE

  // Serial.begin(115200);  //Debug console to PC, whatever COM shows up via USB cable
  SerialA.begin(115200, SERIAL_8N1, 16, 17);

  initializeSensor();

  if (SerialA.available()) {
    while (SerialA.available()) {
      SerialA.read();
    }
  }

  sendFlowPres();  //delay(100);

  delay(1);
  //Serial.println(Serial1.available());
  fullLoop = 0;
  noDataTime = 0;
  PRES_OFFSET_TIMER = 0;


  ///// Setup code for stimulation /////
  Serial.begin(921600);
  pinMode(triggerTrainPin1, OUTPUT);
  pinMode(triggerTrainPin2, OUTPUT);
  pinMode(stimGate1, OUTPUT);
  pinMode(stimGate2, OUTPUT);
  pinMode(stimTrigIn, INPUT_PULLUP);
  pinMode(extTrigPin1, INPUT_PULLUP);
  pinMode(extTrigPin2, INPUT_PULLUP);

  Serial.flush();
  // pinMode(stimTrigOut, OUTPUT);
  // digitalWrite(stimTrigOut, HIGH);
  // pinMode(extTrigPin1BoxWrite, OUTPUT);
  // digitalWrite(extTrigPin1BoxWrite, HIGH);
  // pinMode(extTrigPin1BoxWrite, OUTPUT);
  // digitalWrite(extTrigPin1BoxWrite, HIGH);

  pinMode(extTrigPin1BoxRead, INPUT_PULLUP);
  pinMode(extTrigPin2BoxRead, INPUT_PULLUP);

  pinMode(sensorOn, OUTPUT);
  digitalWrite(sensorOn, LOW);

  pinMode(stimLED1, OUTPUT);
  digitalWrite(stimLED1, LOW);

  pinMode(stimLED2, OUTPUT);
  digitalWrite(stimLED2, LOW);

  pinMode(oxLED, OUTPUT);
  digitalWrite(oxLED, LOW);

  dac.setDACOutRange(dac.eOutputRange10V);

  delay(1000);
  ////// End setup code for stimulation //////
}

void loop() {
  int switchValue = analogRead(stimTrigIn);
  if (switchValue == 4095 && !sentOnce) {
    switchOn = true;
    sentOnce = true;
    sentZero = false;  // Reset the sentZero flag
  } else if (switchValue != 4095 && !sentZero) {
    switchOn = false;
    sentZero = true;
    sentOnce = false;
  }

  // Serial.print(analogRead(spo2_pin));
  // Serial.print(", ");
  // Serial.println(analogRead(ppg_pin));

  if (analogRead(ppg_pin) == 0){
    digitalWrite(oxLED, LOW);
  }
  else{
    digitalWrite(oxLED, HIGH);
  }

  // mcp.setChannelValue(MCP4728_CHANNEL_A, (BTPSFlow * (2048.0 / flowBound)) + 2048.0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X);       //FlowA Considering range to be flowBound lpm, 5V from USB --> can't use because mine is 3.3V
  // mcp.setChannelValue(MCP4728_CHANNEL_B, (curPresOffset * (2048.0 / presBound)) + 2048.0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X);  //PresA Considering range to be presBound cmH2O, 5V from USB --> can't use because mine is 3.3V

  // mcp.setChannelValue(MCP4728_CHANNEL_A, (BTPSFlow * (2048.0 / flowBound)) + 2048.0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
  // //Lab Chart Arithmetic: (Ch7-2.048)/(2.048/250) --> (Ch7 - 1.024) * (250 /  1.024) but this isn't so accurate
  // mcp.setChannelValue(MCP4728_CHANNEL_B, (curPresOffset * (2048.0 / presBound)) + 2048.0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
  // //Lab Chart Arithmetic: (Ch8-2.048)/(2.048/150) --> (Ch8 - 1.024) * (150 /  1.024) but this isn't so accurate

  // mcp.setChannelValue(MCP4728_CHANNEL_A, 2048); //Use this number without arithmetic off to determine offset
  // mcp.setChannelValue(MCP4728_CHANNEL_B, 2048);

  mcp.setChannelValue(MCP4728_CHANNEL_A, (BTPSFlow * (2048.0 / flowBound)) + 2048.0); //(Ch7 - 1.608) * (250 / 1.608)
  mcp.setChannelValue(MCP4728_CHANNEL_B, (curPresOffset * (2048.0 / presBound)) + 2048.0); //(Ch8 - 1.608) * (150 /  1.608)

  if (fullLoop >= 5000) {  //Micros 5000 == 200Hz
    fullLoopHolder = fullLoop;
    fullLoop = fullLoop - 5000;
    fullLoop = 0;  ////NEW
    if (SerialA.available() >= 10) {
      for (int i = 0; i < 10; i++) {
        if (i > 2 && i < 7) {
          if (SerialA.available()) {
            mydataFlow.bb[i - 3] = SerialA.read();
          }
        } else if (i >= 7 && i < 9) {
          if (SerialA.available()) {
            mydataPres.bbb[i - 7] = SerialA.read();
          }
        } else {
          if (SerialA.available()) {
            SerialA.read();
          }
        }
      }
      rawFlow = mydataFlow.l / 1000.0;
      BTPSFlowRaw = ((mydataFlow.l / 1000.0) * (760 / (baroPres - 47)) * (310 / 293.1));
      if (zeroing){
        flowOffsetValue = baseline; // Phoebe- comment this line out to get rid of constant flow baseline updating
        presOffsetValue = baseline_pres;
        zeroing = false;
      }
      BTPSFlow = BTPSFlowRaw - flowOffsetValue;
      // flowOffset.push(BTPSFlowRaw);

      curPresRaw = ((((mydataPres.l - Digoutpmin) / scaling) + pmin) * 1.0197162129779);
      // curPresOffset = curPresRaw + PRES_OFFSET;

      presBuffer.push(curPresRaw); // Add the current pressure to the buffer
      if (presBuffer.size() >= windowSizePres) {
        float sum = 0;
        float sumSq = 0;
        int count = presBuffer.size();

        // Calculate the sum and sum of squares
        for (int i = 0; i < count; i++) {
            sum += presBuffer[i];
            sumSq += presBuffer[i] * presBuffer[i];
        }

        // Calculate the mean and standard deviation
        float mean = sum / count;
        float variance = (sumSq / count) - (mean * mean);
        float stdDev = sqrt(variance);

        // Set the threshold as a multiple of the standard deviation above the mean
        float thresholdPres = mean + (stdDev * 0.1); // Adjust the multiplier as needed

        // Calculate the low average of the values below the threshold
        float lowSum = 0;
        int lowCount = 0;
        for (int i = 0; i < count; i++) {
            if (presBuffer[i] < thresholdPres) {
                lowSum += presBuffer[i];
                lowCount++;
            }
        }

        if (lowCount > 0) { // Avoid division by zero
            float lowAverage = lowSum / lowCount;
            baseline_pres = -lowAverage; // Set PRES_OFFSET to the negative of the low average
            // Serial.print(curPresRaw);
            // Serial.print(", ");
            // Serial.println(baseline_pres);
        }
      }

      
      curPresOffset = curPresRaw + presOffsetValue; // Phoebe, add this to zero pressure: + baseline_pres 
      // Serial.println(curPresOffset);
      // presOffset.push(curPresOffset);

      // Serial.print(BTPSFlow);
      // Serial.print(", ");
      // Serial.println(curPresRaw);

      /////////////////////////////////Get Temp + Pressure from BMP390//////////////////////////////////
      if (bmp388.getMeasurements(temperature, pressure, altitude)) {
        baroPres = (pressure * 0.75006157584566);  //hPa to mmHg
        ambTemp = temperature + 273.15;            //Kelvin
      }

      ///////////////////////////////////Pressure Offset///////////////////////////////////// Serial Command can just set the pres_offset_timer to 0 to start it
      if (PRES_OFFSET_TIMER <= 5000) {
        PRES_OFFSET_COUNTER++;
        if (PRES_OFFSET_COUNTER % 50 == 0) {
          // Serial.print("Performing Pressure Offset: Time remaining: ");
          // Serial.println(5.0 - PRES_OFFSET_TIMER / 1000.0);
          noDataTime = 0;
        }
        PRES_OFFSET_SUM += curPresRaw;
      }
      else if (PRES_OFFSET_TIMER > 5000 && PRES_OFFSET_TIMER < 5050) {
        PRES_OFFSET = PRES_OFFSET_SUM / -PRES_OFFSET_COUNTER; //Invert
        PRES_OFFSET_STATUS = 1;
        if (PRES_OFFSET > PRES_OFFSET_BOUNDS || PRES_OFFSET < -PRES_OFFSET_BOUNDS) {
          PRES_OFFSET_SUM = 0;
          PRES_OFFSET_COUNTER = 0;
          PRES_OFFSET_TIMER = 0;
        }
        else {
          PRES_OFFSET_TIMER = 10000;
        }
      }
      if (PRES_OFFSET_TIMER > 9000) {
        PRES_OFFSET_TIMER = 10000;
        PRES_OFFSET_STATUS = 1;
      }

      ////////////////////////////////Printing, analogOut, and Serial Sending //////////////////////////////////////////////////
      // mcp.setChannelValue(MCP4728_CHANNEL_A, (BTPSFlow * (2048.0 / flowBound)) + 2048.0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X);       //FlowA Considering range to be flowBound lpm, 5V from USB
      // mcp.setChannelValue(MCP4728_CHANNEL_B, (curPresOffset * (2048.0 / presBound)) + 2048.0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_2X);  //PresA Considering range to be presBound cmH2O, 5V from USB
      // debouncer1.update();

      // if (debouncer1.fell()) {
      //   sensorAOffset();
      // }
      
      if (sendSerial) {  // and !reading
        if (sizeof(BTPSFlow) == sizeof(float)) {
          digitalWrite(sensorOn, HIGH);
          printf("%d,%.3f,%.3f,%d,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.1f,%.3f,%.3f,%d,%d,%d,%d\n", switchOn, BTPSFlow, curPresOffset, iestate, stim_val1, stim_val2, value_2, value_3, value_4, value_5, value_6, value_7, value_8, value_9, value_10, value_11, PRES_OFFSET_STATUS, float(stim_count), lastITime, lastETime, burst1, burst_ext1, burst2, burst_ext2);
        }
        sendFlowPres();
        delayMicroseconds(1250);  // used to be 1250 in trace's code, Phoebe changed it to 200 to increase speed
        sendSerial = !sendSerial;
        noDataTime = 0;
      } else {
        sendSerial = !sendSerial;
        sendFlowPres();
        delayMicroseconds(750);
      }
      clear = false;
    }

    else {
      if (!clear){
        clearSerialBuffer();
        clear = true;
      }
      // clearSerialBuffer();
      // initializeSensor();
      
      digitalWrite(sensorOn, LOW);
      if (sendSerial) {
        if (SerialA.available() != 20) {
          while (SerialA.available()) {
            SerialA.read();
          }
          printf("%d,%.1f,%.1f,%.1f,%d,%d,%d,%d\n", switchOn, stim_val1, stim_val2, float(stim_count), burst1, burst_ext1, burst2, burst_ext2);
          sendFlowPres();
          delayMicroseconds(1250);
        }
      }
      else{
        sendSerial = !sendSerial;
        sendFlowPres();
        delayMicroseconds(750);
      }
    }
    if (noDataTime > 2000000) {
      if (SerialA.available()) {
        while (SerialA.available()) {
          SerialA.read();
        }
      }
      sendFlowPres();  //delay(100);
      delayMicroseconds(750);
    }

    ///////////////////Determine expiration or inspiration///////////////////////////
    updateBaseline();
    determineIEstate();
    weightedAverage();
    //////////////// End exp/insp determination //////////

    ////////////Read in over Serial/////////////////////////
    int StringCount = 0;

    // check if data is available
    if (Serial.available() > 0) {
      reading = true;
      int bytesRead = Serial.readBytesUntil('\n', buf, BUFFER_SIZE);
      String str = String(buf);
      while (str.length() > 0) {
        int index = str.indexOf(',');
        if (index == -1)  // No space found
        {
          strs[StringCount++] = str;
          break;
        } else {
          strs[StringCount++] = str.substring(0, index);
          str = str.substring(index + 1);
        }
      }
      if (isDigit(strs[0][0])) {
        // same parameters
        n_breaths = strs[0].toInt();               // if 0 then not symmetric
        insp_period = strs[1].toFloat() * 1000.0;  // float in milliseconds
        exp_period = strs[2].toFloat() * 1000.0;   // float in milliseconds
        frequency = strs[3].toInt();               // Hz
        wave_type = strs[4].toInt();               // 1 = triangle, 2 = trapezoidal, 3 = sine, 4 = HFOV
        rise_time1 = strs[5].toFloat() * 1000.0;
        fall_time1 = strs[6].toFloat() * 1000.0;
        rise_time2 = rise_time1;
        fall_time2 = fall_time1;
        allow_stim1 = strs[7].toInt();
        amplitude1 = strs[8].toFloat();
        base1 = strs[9].toFloat();
        allow_stim2 = strs[10].toInt();
        amplitude2 = strs[11].toFloat();
        base2 = strs[12].toFloat();
        trigger_type = strs[13].toInt();
        pred_delay = strs[14].toFloat(); // float in milliseconds
        hfov_base = strs[15].toFloat();
        num_breaths_avg = strs[16].toInt();
        i_threshold = strs[17].toInt();
        e_threshold = strs[18].toInt();
        bilevel_t_ = strs[19].toInt();
        bilevel_v = strs[20].toInt();
        bias1 = strs[21].toFloat();
        bias2 = strs[22].toFloat();
        zeroing = strs[23].toFloat();

        if (amplitude1 < 0) {
          synch_1 = true;
        }
        if (amplitude2 < 0) {
          synch_2 = true;
        }

        if (n_breaths > 0) {  // symmetric
          breathCounter = -1;
        }

        if (allow_stim1 == 2){
          // allow_stim1 = 1;
          burst1 = 1;
        }

        if (allow_stim2 == 3){
          // allow_stim2 = 1;
          burst2 = 1;
        }

        // if (allow_stim1 == 2 && allow_stim2 == 3){
        //   // allow_stim2 = 1;
        //   burst = 3;
        // }

        if (allow_stim1 == 4 || allow_stim2 == 4){
          allow_stim1 = 0;
          allow_stim2 = 0;
          stim_count = 0;
        }
      }
    }
    reading = false;
  }
  /////////////////////////END LOOP CODE FOR STIMULATION////////////////////////////////
  ////////////Stim Gate/////////////////////////    
  if (analogRead(stimTrigIn) == 4095) {  //

    if ((digitalRead(extTrigPin1) == 0) && !responded_1){ //normally closed
      burst_ext1 = 1;
      responded_1 = true;
    }
    if ((digitalRead(extTrigPin1BoxRead) == 1) && !responded_box_1){ //normally closed
      burst_ext1 = 1;
      responded_box_1 = true;
    }
    if ((digitalRead(extTrigPin2) == 0) && !responded_2){ //normally closed
      burst_ext2 = 1;
      responded_2 = true;
    }
    if ((digitalRead(extTrigPin2BoxRead) == 1) && !responded_box_2){ //normally closed
      burst_ext2 = 1;
      responded_box_2 = true;
    }
    // else if (digitalRead(extTrigPin1) == 1 && responded){
    //   burst_ext = 0;
    //   responded = false;
    // }
    
    if ((digitalRead(extTrigPin1) != 0) && responded_1) { //(digitalRead(extTrigPin1) != 0 or digitalRead(extTrigPin1BoxRead) != 1) && 
      burst_ext1 = 0;
      responded_1 = false;
    }
    if ((digitalRead(extTrigPin1BoxRead) != 1) && responded_box_1) { //(digitalRead(extTrigPin1) != 0 or digitalRead(extTrigPin1BoxRead) != 1) && 
      burst_ext1 = 0;
      responded_box_1 = false;
    }

    if ((digitalRead(extTrigPin2) != 0) && responded_2) { //(digitalRead(extTrigPin1) != 0 or digitalRead(extTrigPin1BoxRead) != 1) && 
      burst_ext2 = 0;
      responded_2 = false;
    }
    if ((digitalRead(extTrigPin2BoxRead) != 1) && responded_box_2) { //(digitalRead(extTrigPin1) != 0 or digitalRead(extTrigPin1BoxRead) != 1) && 
      burst_ext2 = 0;
      responded_box_2 = false;
    }
    
    if ((burst1 == 1 or burst2 == 1) && (trigger_type == 1 || trigger_type == 3 || trigger_type == 4 || trigger_type == 6)){
      base1 = bias1;
      base2 = bias2;

      if (burst1 == 1) {
        allow_stim1 = 1;
        burst_stim(stim1_enable, allow_stim1, burst1, signal_time_burst1);
        signal_time1 = signal_time_burst1;
      }

      if (burst2 == 1) { // was else if before
        allow_stim2 = 1;
        burst_stim(stim2_enable, allow_stim2, burst2, signal_time_burst2);
        signal_time2 = signal_time_burst2;
      }
    }
    
    else{
      signal_time1 = signal_time;
      signal_time2 = signal_time;

      if (!allow_stim1 && !allow_stim2) {
        stim1_enable = false;
        stim2_enable = false;
        first = true;
        first_smth = true;
        enable = true;
        enable2 = true;
        was_insp = false;
      }


      // asymmetric, alternating stimulation
      if (n_breaths != 0) {  // symmetric if not 0
        if (allow_stim1 && allow_stim2) {
          alternating_stim(stim1_enable, synch_1, stim2_enable, synch_2);
        }
      }
      
      else {
        // symmetric stimulation
        if (allow_stim1 && allow_stim2) {
          indiv_stim(stim1_enable, synch_1, stim2_enable, synch_2);
        }

        // Individual stimulation
        else { 
          // stim 1
          if (allow_stim1 && !allow_stim2) {
            indiv_stim(stim1_enable, synch_1);
          }

          // stim 2
          if (allow_stim2 && !allow_stim1) {
            indiv_stim(stim2_enable, synch_2);
          }
        }
      }
    }
    

    if ((allow_stim1 && stim1_enable) or (allow_stim1 && base1 > 0) or (bias1 > 0) or ((trigger_type == 2 || trigger_type == 5 || trigger_type == 7 || trigger_type == 8) && allow_stim1 && synch_1 && !stim1_enable && signal_time < fall_time1)) {
      digitalWrite(stimGate1, HIGH);
    } else {
      digitalWrite(stimGate1, LOW);
    }

    if ((allow_stim2 && stim2_enable) or (allow_stim2 && base2 > 0) or (bias2 > 0) or ((trigger_type == 2 || trigger_type == 5 || trigger_type == 7 || trigger_type == 8) && allow_stim2 && synch_2 && !stim2_enable && signal_time < fall_time2)) {
      digitalWrite(stimGate2, HIGH);
    } else {
      digitalWrite(stimGate2, LOW);
    }

    setTriggerTrainFreq_PW(frequency, stim1_enable, allow_stim1, base1, triggerTrainPin1, signal_time, fall_time1, synch_1, bias1);
    setTriggerTrainFreq_PW(frequency, stim2_enable, allow_stim2, base2, triggerTrainPin2, signal_time, fall_time1, synch_2, bias2);
    
    if (burst1 != 1 and burst2 != 1){
      signal_time1 = signal_time;
      signal_time2 = signal_time;
    }

    // if (bias1 > 0){
    //   dac.setDACOutVoltage(bias1*1000,1);
    //   stim_val1 = bias1;
    //   digitalWrite(stimLED1, HIGH);
    // }
    // else {
    //   dac.setDACOutVoltage(0,1);
    //   stim_val1 = 0;
    //   digitalWrite(stimLED1, LOW);
    // }
    // if (bias2 > 0){
    //   dac.setDACOutVoltage(bias2*1000,0);
    //   stim_val2 = bias2;
    //   digitalWrite(stimLED2, HIGH);
    // }
    // else {
    //   dac.setDACOutVoltage(0,1);
    //   stim_val1 = 0;
    //   digitalWrite(stimLED2, LOW);
    // }

    if (allow_stim1){
      if (wave_type == 1) {
        Tri_AWG(stim_val1, rise_time1, fall_time1, amplitude1, base1, stim1_enable && allow_stim1, signal_time1, 1, stimLED1);
      }
      else if (wave_type == 2) {
        Trap_AWG(stim_val1, insp_period, rise_time1, fall_time1, amplitude1, base1, stim1_enable && allow_stim1, allow_stim1, signal_time1, 1, synch_1, stimLED1);
      }
      else if (wave_type == 3) {
        Sine_AWG(stim_val1, insp_period, amplitude1, base1, stim1_enable && allow_stim1, signal_time1, 1, stimLED1);
      }
      else if (wave_type == 4) {
        HFOV_AWG(stim_val1, rise_time1, fall_time1, amplitude1, base1, stim1_enable && allow_stim1, signal_time1, 1, hfov_base, stimLED1);
      }
      else if (wave_type == 5) {
        Bilevel_AWG(stim_val1, insp_period, rise_time1, fall_time1, amplitude1, base1, stim1_enable && allow_stim1, allow_stim1, signal_time1, 1, bilevel_t_, bilevel_v, stimLED1);
      }
    }
    else{
      dac.setDACOutVoltage(bias1*1000,1);
      stim_val1 = bias1;
      digitalWrite(stimLED1, LOW);
    }

    if (allow_stim2){
      if (wave_type == 1) {
        Tri_AWG(stim_val2, rise_time2, fall_time2, amplitude2, base2, stim2_enable && allow_stim2, signal_time2, 0, stimLED2);
      }
      else if (wave_type == 2) {
        Trap_AWG(stim_val2, insp_period, rise_time2, fall_time2, amplitude2, base2, stim2_enable && allow_stim2, allow_stim2, signal_time2, 0, synch_2, stimLED2);
      }
      else if (wave_type == 3) {
        Sine_AWG(stim_val2, insp_period, amplitude2, base2, stim2_enable && allow_stim2, signal_time2, 0, stimLED2);
      }
      else if (wave_type == 4) {
        HFOV_AWG(stim_val2, rise_time2, fall_time2, amplitude2, base2, stim2_enable && allow_stim2, signal_time2, 0, hfov_base, stimLED2);
      }
      else if (wave_type == 5) {
        Bilevel_AWG(stim_val2, insp_period, rise_time2, fall_time2, amplitude2, base2, stim2_enable && allow_stim2, allow_stim2, signal_time2, 0, bilevel_t_, bilevel_v, stimLED2);
      }
    }
    else{
      dac.setDACOutVoltage(bias2*1000,0);
      stim_val2 = bias2;
      digitalWrite(stimLED2, LOW);
    }


    // if (wave_type == 1) {
    //   Tri_AWG(stim_val1, rise_time1, fall_time1, amplitude1, base1, stim1_enable && allow_stim1, signal_time1, 1, stimLED1);
    //   Tri_AWG(stim_val2, rise_time2, fall_time2, amplitude2, base2, stim2_enable && allow_stim2, signal_time2, 0, stimLED2);
    // }
    // else if (wave_type == 2) {
    //   if (allow_stim1){
    //     Trap_AWG(stim_val1, insp_period, rise_time1, fall_time1, amplitude1, base1, stim1_enable && allow_stim1, allow_stim1, signal_time1, 1, synch_1, stimLED1);
    //   }
    //   else{
    //     dac.setDACOutVoltage(bias1*1000,1);
    //     stim_val1 = bias1;
    //     // digitalWrite(stimLED1, LOW);
    //   }
    //   if (allow_stim2){
    //     Trap_AWG(stim_val2, insp_period, rise_time2, fall_time2, amplitude2, base2, stim2_enable && allow_stim2, allow_stim2, signal_time2, 0, synch_2, stimLED2);
    //   }
    //   else{
    //     dac.setDACOutVoltage(bias2*1000,0);
    //     stim_val2 = bias2;
    //     // digitalWrite(stimLED2, LOW);
    //   }
    // } else if (wave_type == 3) {
    //   Sine_AWG(stim_val1, insp_period, amplitude1, base1, stim1_enable && allow_stim1, signal_time1, 1, stimLED1);
    //   Sine_AWG(stim_val2, insp_period, amplitude2, base2, stim2_enable && allow_stim2, signal_time2, 0, stimLED2);
    // } else if (wave_type == 4) {
    //   HFOV_AWG(stim_val1, rise_time1, fall_time1, amplitude1, base1, stim1_enable && allow_stim1, signal_time1, 1, hfov_base, stimLED1);
    //   HFOV_AWG(stim_val2, rise_time2, fall_time2, amplitude2, base2, stim2_enable && allow_stim2, signal_time2, 0, hfov_base, stimLED2);
    // } else if (wave_type == 5) {
    //   Bilevel_AWG(stim_val1, insp_period, rise_time1, fall_time1, amplitude1, base1, stim1_enable && allow_stim1, allow_stim1, signal_time1, 1, bilevel_t_, bilevel_v, stimLED1);
    //   Bilevel_AWG(stim_val2, insp_period, rise_time2, fall_time2, amplitude2, base2, stim2_enable && allow_stim2, allow_stim2, signal_time2, 0, bilevel_t_, bilevel_v, stimLED2);
    // } 


    // else if (wave_type == 6){
    //   dac.setDACOutVoltage(bias1*1000,1);
    //   dac.setDACOutVoltage(bias2*1000,0);
    //   stim_val1 = bias1;
    //   stim_val2 = bias2;
    // } else if (wave_type == 7){
    //   dac.setDACOutVoltage(bias1*1000,1);
    //   dac.setDACOutVoltage(0,0);
    //   stim_val1 = bias1;
    //   stim_val2 = 0;
    //   digitalWrite(stimLED1, LOW);
    //   digitalWrite(stimLED2, LOW);
    // } else if (wave_type == 8){
    //   dac.setDACOutVoltage(0,1);
    //   dac.setDACOutVoltage(bias2*1000,0);
    //   stim_val1 = 0;
    //   stim_val2 = bias2;
    //   digitalWrite(stimLED1, LOW);
    //   digitalWrite(stimLED2, LOW);
    // } 
    // else if (wave_type == 0){
    //   dac.setDACOutVoltage(0,1);
    //   dac.setDACOutVoltage(0,0);
    //   stim_val1 = 0;
    //   stim_val2 = 0;
    //   digitalWrite(stimLED1, LOW);
    //   digitalWrite(stimLED2, LOW);
    // }
  } else {
    digitalWrite(stimGate1, LOW);
    digitalWrite(stimGate2, LOW);
    digitalWrite(triggerTrainPin1, LOW);
    digitalWrite(triggerTrainPin2, LOW);
    dac.setDACOutVoltage(0,1);
    dac.setDACOutVoltage(0,0);
    stim_val1 = 0;
    stim_val2 = 0;
    digitalWrite(stimLED1, LOW);
    digitalWrite(stimLED2, LOW);
  }
  ////////////////////////////////////////////////////////
}

void initializeSensor() {
  boardHardwareReset();
  delay(100);
  setHeaterPowerMax();
  delay(100);
  setHeaterOn();
  delay(100);
  sendFlowPres();
  delay(100);
}

void clearSerialBuffer() {
  while (SerialA.available() > 0) {
    SerialA.read();
    // Serial.println("clear");
  }
}

/// Functions for enabling stim
void burst_stim(bool& enable, int& allow, int& burst, elapsedMillis& signal_time_burst){
  ///insp
  if (!enable){
    enable = true;
    signal_time = 0;
    signal_time_burst = 0;
  }
  ///exp
  else if (signal_time_burst >= insp_period) {
    /// in mandatory / synch trig, turn off stim when pre-set stim inspiration over, and in synch, turn off stim when not inhaling
    enable = false;
    allow = 0;
    burst = 0;
    off_time_burst = 0;
  }
}


/// Functions for enabling stim
void alternating_stim(bool& enable, bool& synch, bool& enable2, bool& synch2){
  ////////// First n breaths = stim 1 ///////////
  if (breathCounter < n_breaths){
    rando = 1;
    enable2 = false;
    synch2 = false;
    if (trigger_type != 4 && trigger_type != 8){
      if (iestate == 1){
        was_insp = true;
        was_exp = false;
      }

      if (iestate == 2 && !was_exp){
        exp_time = 0;
        was_exp = true;
      }

      /// Inhalation/inspiration period
      if (!enable && ((trigger_type == 1 && signal_time >= exp_period) or ((trigger_type == 2 or trigger_type == 3) && iestate == 1 && off_time_double > 500) or ((trigger_type == 5) && off_time >= fall_time1 + 510) or (trigger_type == 7 && iestate != 1 && off_time_double > 500) or ((trigger_type == 6) && exp_time <= (insp_period + pred_delay)))) {  //Could turn #5 into similar to #6
        /// in synch trig, it enables stim once inspiration begins
        if ((trigger_type == 3 or trigger_type == 6) && first){
          if (first_smth){
            signal_time = 0;
            first_smth = false;
            enable = false;
          }
          if ((trigger_type == 3 || trigger_type == 6) && signal_time >= pred_delay){
            enable = true;
            stim_count++;
            breathCounter++;
            signal_time = 0;
            synch = true;
            if (trigger_type == 3){
              first = false;
            }
            if (breathCounter == n_breaths) {
              enable = false;
              enable2 = true;
            }
          }
          else{
            enable = false;
            synch = false;
          }
        }

        /// in mandatory, it enables stim when expiration is over. in synchronous, it enables stim once inspiration begins
        else if (trigger_type != 3 and trigger_type != 6){
          //for mandatory, just second if block. for synch, re-starts signal_time at 0 once delay is reached, 
          if (first){
            signal_time = 0;
            first = false;
            enable = false;
          }
          if ((trigger_type != 5 && signal_time >= pred_delay) || (trigger_type == 5 && off_time >= fall_time1 + 10 + pred_delay)){
            enable = true;
            stim_count++;
            breathCounter++;
            signal_time = 0;
            synch = true;
          }
          else{
            enable = false;
            synch = false;
          }
          if (breathCounter == n_breaths) {
            enable = false;
            synch = false;
            enable2 = true;
            synch2 = true;
          }
        }
      }

      /// Not-inhalation/non-inspiration period
      else if (((trigger_type == 1  or trigger_type == 3) && enable && signal_time >= insp_period) or (trigger_type == 6 && signal_time >= insp_period) or (((trigger_type == 2 or trigger_type == 5) && was_insp) && ((iestate != 1 && enable && signal_time >= 500) || (trigger_type == 2 && signal_time >= 3000))) or (trigger_type == 7 && (iestate == 1 && enable && signal_time >= 500))) {
        /// in mandatory / synch trig, turn off stim when pre-set stim inspiration over, and in synch, turn off stim when not inhaling
        enable = false;
        signal_time = 0;
        was_insp = false;
        off_time = 0;
        off_time_double = 0;

        /// in synch trig, flag when not inhaling
        if ((trigger_type != 1 && trigger_type != 3 && iestate != 1) || trigger_type == 7){
          first = true;
          first_smth = true;
        }
      }
      /// in synch trig, flag when not inhaling
      if (trigger_type == 3 && was_exp){
        first = true;
        first_smth = true;
      }
    }

    /// Inspiration and expiration for predictive ///
    if (trigger_type == 4 or trigger_type == 8) {      
      if (trigger_type == 8){
        synch = true;
      }
      else{
        synch = false;
      }
      
      if (iestate == 1){
        was_insp = true;
        was_exp = false;
      }

      if (iestate == 2 && !was_exp){
        was_exp = true;
        was_insp = false;
      }
        
      if (iestate != 1 && first_smth){
        first_smth = false;
        exp_time = 0;
        first = false;
      }
      else if (iestate == 1){
        first_smth = true;
      }
      
      if (!enable && off_time_double > 500) {
        // rando = 1;
        /// enables stim once inspiration begins
        if (exp_time >= averageETime + pred_delay && !first && e_times.size() > 1){
          // rando = 2;
          signal_time = 0;
          first = true;
          enable = true;
          stim_count++;
          breathCounter++;
          if (breathCounter == n_breaths) {
            enable = false;
            enable2 = true;
            synch = false;
          }
        }
      }
      else if (enable) {
        // rando = 3;
        /// disables stim once inspiration ends
        if (trigger_type == 4 and signal_time >= insp_period){
          enable = false;
          off_time_double = 0;
        }
        if (trigger_type == 8 && was_insp &&  (iestate != 1 || signal_time >= 3000)){
          // rando = 4;
          enable = false;
          off_time_double = 0;
          was_insp = false;
          signal_time = 0;
        }
      }
    }
  }

  //////// Subsequent n breaths = stim 2 /////////
  else{
    rando = 2;
    enable = false;
    synch = false;
    if (trigger_type != 4 && trigger_type != 8){
      if (iestate == 1){
        was_insp = true;
        was_exp = false;
      }
      if (iestate == 2 && !was_exp){
        exp_time = 0;
        was_exp = true;
      }

      /// Inhalation/inspiration period
      if (!enable2 && ((trigger_type == 1 && signal_time >= exp_period) or ((trigger_type == 2 or trigger_type == 3) && iestate == 1 && off_time_double > 500) or ((trigger_type == 5) && off_time >= fall_time1 + 510) or (trigger_type == 7 && iestate != 1 && off_time_double > 500) or ((trigger_type == 6) && exp_time <= (insp_period + pred_delay)))) {  //Could turn #5 into similar to #6
        /// in synch trig, it enables stim once inspiration begins
        if ((trigger_type == 3 or trigger_type == 6) && first){
          if (first_smth){
            signal_time = 0;
            first_smth = false;
            enable2 = false;
          }
          if ((trigger_type == 3 || trigger_type == 6) && signal_time >= pred_delay){
            enable2 = true;
            stim_count++;
            breathCounter++;
            signal_time = 0;
            synch2 = true;
            first = false;
            if (breathCounter == n_breaths * 2) {
              breathCounter = 0;  // reset
              enable2 = false;
              enable = true;
            }
          }
          else{
            enable2 = false;
            synch2 = false;
          }
        }

        /// in mandatory, it enables stim when expiration is over. in synchronous, it enables stim once inspiration begins
        else if (trigger_type != 3 and trigger_type != 6){
          //for mandatory, just second if block. for synch, re-starts signal_time at 0 once delay is reached, 
          if (first){
            signal_time = 0;
            first = false;
            enable2 = false;
          }
          if ((trigger_type != 5 && signal_time >= pred_delay) || (trigger_type == 5 && off_time >= fall_time1 + 10 + pred_delay)){
            enable2 = true;
            signal_time = 0;
            synch2 = true;
            stim_count++;
            breathCounter++;
          }
          else{
            enable2 = false;
            synch2 = false;
          }
          if (breathCounter == n_breaths * 2) {
            breathCounter = 0;  // reset
            enable2 = false;
            synch2 = false;
            enable = true;
            synch = true;
          }
        }
      }

      /// Not-inhalation/non-inspiration period
      else if (((trigger_type == 1  or trigger_type == 3) && enable2 && signal_time >= insp_period) or (trigger_type == 6 && signal_time >= insp_period) or (((trigger_type == 2 or trigger_type == 5) && was_insp) && ((iestate != 1 && enable2 && signal_time >= 500) || (trigger_type == 2 && signal_time >= 3000))) or (trigger_type == 7 && (iestate == 1 && enable2 && signal_time >= 500))) {
        
        /// in mandatory / synch trig, turn off stim when pre-set stim inspiration over, and in synch, turn off stim when not inhaling
        enable2 = false;
        signal_time = 0;
        was_insp = false;
        off_time = 0;
        off_time_double = 0;

        /// in synch trig, flag when not inhaling
        if ((trigger_type != 1 && trigger_type != 3 && iestate != 1) || trigger_type == 7){
          first = true;
          first_smth = true;
        }
      }
      /// in synch trig, flag when not inhaling
      if (trigger_type == 3 && was_exp){
        first = true;
        first_smth = true;
      }
    }

    /// Inspiration and expiration for predictive ///
    if (trigger_type == 4 or trigger_type == 8) {
      if (trigger_type == 8){
        synch2 = true;
      }
      else{
        synch2 = false;
      }
      
      if (iestate == 1){
        was_insp = true;
        was_exp = false;
      }

      if (iestate == 2 && !was_exp){
        was_exp = true;
        was_insp = false;
      }

      if (iestate != 1 && first_smth){
        first_smth = false;
        exp_time = 0;
        first = false;
      }
      else if (iestate == 1){
        first_smth = true;
      }

      if (!enable2 && off_time_double > 500) {
        // rando = 5;
        /// enables stim once inspiration begins
        if (exp_time >= averageETime + pred_delay && !first && e_times.size() > 1){
          // rando = 6;
          signal_time = 0;
          first = false;
          enable2 = true;
          stim_count++;
          breathCounter++;
          if (breathCounter == n_breaths * 2) {
            breathCounter = 0;  // reset
            enable2 = false;
            enable = true;
            synch2 = false;
          }
        }
      }

      else if (enable2) {
        // rando = 7;
        /// enables stim once inspiration begins
        if (trigger_type == 4 and signal_time >= insp_period){
          enable2 = false;
          off_time_double = 0;
          // synch2 = false;
        }
        if (trigger_type == 8 && was_insp && (iestate != 1 || signal_time >= 3000)){
          // rando = 8;
          enable2 = false;
          off_time_double = 0;
          was_insp = false;
          signal_time = 0;
        }
      }
    }
  }
}

/// Functions for enabling stim
void indiv_stim(bool& enable, bool& synch, bool& enable2, bool& synch2){
  if (trigger_type != 4 && trigger_type != 8){
    if (iestate == 1){
      was_insp = true;
      was_exp = false;
    }

    if (iestate == 2 && !was_exp){
      exp_time = 0;
      was_exp = true;
    }

    /// Inhalation/inspiration period
    if (!enable && ((trigger_type == 1 && signal_time >= exp_period) or ((trigger_type == 2 or trigger_type == 3) && iestate == 1 && off_time_double > 500) or ((trigger_type == 5) && off_time >= fall_time1 + 510) or (trigger_type == 7 && iestate != 1 && off_time_double > 500) or ((trigger_type == 6) && exp_time <= (insp_period + pred_delay)))) {  //Could turn #5 into similar to #6
      /// in synch trig, it enables stim once inspiration begins
      if ((trigger_type == 3 or trigger_type == 6) && first){
        rando = 4;
        if (first_smth){
          signal_time = 0;
          first_smth = false;
          enable = false;
          enable2 = false;
          rando = 5;
        }
        if ((trigger_type == 3 || trigger_type == 6) && signal_time >= pred_delay){
          enable = true;
          enable2 = true;
          stim_count++;
          signal_time = 0;
          synch = true;
          synch2 = true;
          if (trigger_type == 3){
            first = false;
          }
        }
        else{
          enable = false;
          enable2 = false;
          synch = false;
          synch2 = false;
        }
      }

      /// in mandatory, it enables stim when expiration is over. in synchronous, it enables stim once inspiration begins
      else if (trigger_type != 3 and trigger_type != 6){
        //for mandatory, just second if block. for synch, re-starts signal_time at 0 once delay is reached, 
        if (first){
          signal_time = 0;
          first = false;
          enable = false;
          enable2 = false;
        }
        if ((trigger_type != 5 && signal_time >= pred_delay) || (trigger_type == 5 && off_time >= fall_time1 + 10 + pred_delay)){
          enable = true;
          enable2 = true;
          stim_count++;
          signal_time = 0;
          synch = true;
          synch2 = true;
          // rando = 1;
        }
        else{
          enable = false;
          enable2 = false;
          synch = false;
          synch2 = false;
          // rando = 3;
        }
      }
    }

    /// Not-inhalation/non-inspiration period
    else if (((trigger_type == 1  or trigger_type == 3) && enable && signal_time >= insp_period) or (trigger_type == 6 && signal_time >= insp_period) or ((trigger_type == 2 or trigger_type == 5) && was_insp && ((iestate != 1 && enable && signal_time >= 500) || (trigger_type == 2 && signal_time >= 3000))) or (trigger_type == 7 && (iestate == 1 && enable && signal_time >= 500))) {
      /// in mandatory / synch trig, turn off stim when pre-set stim inspiration over, and in synch, turn off stim when not inhaling
      enable = false;
      enable2 = false;
      signal_time = 0;
      was_insp = false;
      off_time = 0;
      off_time_double = 0;
      
      /// in synch trig, flag when not inhaling
      if ((trigger_type != 1 && trigger_type != 3 && iestate != 1) || trigger_type == 7){
        first = true;
        first_smth = true;
      }
    }
    /// in synch trig, flag when not inhaling
    if (trigger_type == 3 && was_exp){
      first = true;
      first_smth = true;
    }
  }

  /// Inspiration and expiration for predictive ///
  if (trigger_type == 4 or trigger_type == 8) {
    if (trigger_type == 8){
      synch = true;
      synch2 = true;
    }
    else{
      synch = false;
      synch2 = false;
    }
    
    if (iestate == 1){
      was_insp = true;
      was_exp = false;
    }

    if (iestate == 2 && !was_exp){
      // exp_time = 0;
      was_exp = true;
      was_insp = false;
    }
    
    if (iestate != 1 && first_smth){
      first_smth = false;
      exp_time = 0;
      first = false;
    }
    else if (iestate == 1){
      first_smth = true;
    }
    rando = 3;

    if (!enable && off_time_double > 500) {
      /// enables stim based on exp time
      if (exp_time >= averageETime + pred_delay && !first && e_times.size() > 1){
        signal_time = 0;
        first = true;
        enable = true;
        enable2 = true;
        stim_count++;
        rando = 1;
      }
    }
    else if (enable) {
      /// disables stim once inspiration ends
      if (trigger_type == 4 and signal_time >= insp_period){
        enable = false;
        enable2 = false;
        off_time_double = 0;
      }
      if (trigger_type == 8 && was_insp && (iestate != 1 || signal_time >= 3000)){
        enable = false;
        enable2 = false;
        off_time_double = 0;
        was_insp = false;
        signal_time = 0;
        rando = 2;
      }
    }
  }
}

// Overloaded version with default arguments
void indiv_stim(bool& enable, bool& synch) {
    bool dummy1, dummy2;
    indiv_stim(enable, synch, dummy1, dummy2);
}

/// Functions for determining insp vs exp ///
void updateBaseline() {
  data.push(BTPSFlowRaw);
  currentIndex = (currentIndex + 1) % windowSize;

  // Check if the circular buffer is filled with enough data points
  if (data.size() >= windowSize) {
    // Calculate the baseline using convolution
    float sum = 0;
    for (int i = 0; i < windowSize; i++) {
      sum += data[i];
    }
    baseline = sum / windowSize;

    // Calculate amplitude and update threshold
    float amplitude = abs(BTPSFlowRaw - baseline);
    threshold = thresholdFactor * amplitude;
  }
}

void determineIEstate() {
  float average = 0;
  if (data.size() >= 10) {
    float sum = 0;
    // Sum the last 5 elements
    for (int i = windowSize - 10; i < windowSize; i++) {
      sum += data[i];
    }

    // Calculate average
    average = sum / 5;
  }

  if (average > baseline + i_threshold) { //Phoebe- 10
    if (firstPos) {
      firstPos = false;
      lastIStart = iStart;
      iStart = millis();
      lastPeriod = iStart - lastIStart;
      lastETime = iStart - eStart;
      if (eStart > 0) {
        e_times.push(lastETime);
        if (i_times.size() > 1){
          updateValues();
        }
      }
      piMax = 0;
      peakFlow = 0;
      baselinePos = baseline;
      thresholdPos = threshold;
      iTidal = 0;
      lastTimestamp = 0;
    }

    iestate = 1;
    firstNeg = true;

    if (curPresRaw > piMax) { //curPresOffset
      piMax = curPresRaw; //curPresOffset
    }

    if (BTPSFlowRaw > peakFlow) {
      peakFlow = BTPSFlowRaw;
    }

    currentTimestamp = millis();
    if (lastTimestamp != 0) {
      step = (currentTimestamp - lastTimestamp) / 60;
      iTidal += (BTPSFlowRaw - baseline) * step;
    }
    lastTimestamp = currentTimestamp;
    //////////////////////
  }
  else if (average < baseline + e_threshold) { //Phoebe- 10
    if (firstNeg) {
      firstNeg = false;
      eStart = millis();
      lastITime = eStart - iStart;
      if (iStart > 0) {
        i_times.push(lastITime);
      }
      peep = 250;
      baselineNeg = baseline;
      thresholdNeg = threshold;
      eTidal = 0;
      lastTimestamp = 0;
    }

    iestate = 2;
    firstPos = true;

    if (peep > curPresRaw) { //curPresOffset
      peep = curPresRaw; //curPresOffset
    }
    
    currentTimestamp = millis();
    if (lastTimestamp != 0) {
      step = (currentTimestamp - lastTimestamp) / 60;
      eTidal += abs(BTPSFlowRaw - baseline) * step;
    }
    lastTimestamp = currentTimestamp;
  }
  else {
    iestate = 0;
  }
}
/// End fns for determins insp vs exp ///


/// Fns for predictive algorithm: collect 12 values and average
void weightedAverage() {
  int numValues = e_times.size();
  if (e_times.size() == 0){
    averageETime = 0;
    return;
  }
  int actualWeightsLength = min(num_breaths_avg, numValues);
  
  // Allocate memory for the weights array
  float* weights = new float[actualWeightsLength];
  
  // Initialize weights
  for (int i = 0; i < actualWeightsLength; i++) {
    weights[i] = 0.1;
  }
  if (actualWeightsLength >= 3) {
    weights[actualWeightsLength - 3] = 0.2;
    weights[actualWeightsLength - 2] = 0.25;
    weights[actualWeightsLength - 1] = 0.3;
  }

  // // Calculate the length of the weights array
  // int weightsLength = num_breaths_avg;

  // Calculate the weighted sum
  float weightedSum = 0;
  for (int i = 0; i < actualWeightsLength; ++i) {
    weightedSum += weights[i] * e_times[numValues - actualWeightsLength + i];
  }

  // Calculate the total weight
  float totalWeight = 0;
  for (int i = 0; i < actualWeightsLength; ++i) {
    totalWeight += weights[i];
  }

  // Calculate the weighted average
  averageETime = weightedSum / totalWeight;

  // Deallocate the dynamically allocated memory
  delete[] weights;
}

/// End fns for predictive algorithm

/// Fns for updating values in values tab////
void updateValues() {
  value_2 = lastITime / 1000;         //Ti
  value_3 = lastITime / (lastPeriod - lastITime);    //IE ratio
  value_4 = peep;                     //PEEP
  value_5 = piMax;                    //PiMax
  value_6 = iTidal;                   //TVi
  value_7 = eTidal;                   //TVe
  value_8 = 60 * 1000 / lastPeriod;   //Resp rate
  value_9 = value_6 * value_8 / 1000;   //Resp rate
  if (peakFlow != 0) {
    value_10 = peakFlow;  //Peak Flow
  }
  value_11 = peakFlow / 2;  //Flow
}

/// End fns for updating values in values tab ///


//// FUNCTIONS FOR STIMULATION //////

void calc_insp_period(float IE_ratio, int breaths_per_min) {
  IE_ratio = 0.5;
  breaths_per_min = 12;
  float breath_period = breaths_per_min / 60.0;  // 0.2 seconds
  float exp_period = breath_period / (1 + IE_ratio);
  float insp_period = exp_period * IE_ratio;
}

// trigger train function
void setTriggerTrainFreq_PW(float freq, int enableStim, int allowStim, float base, int triggerTrainPin, elapsedMillis signal_time, float fall_time, bool synch, float bias) {
  if ((allowStim && enableStim) or (allowStim && base > 0) or (bias > 0) or ((trigger_type == 2 || trigger_type == 5 || trigger_type == 7 || trigger_type == 8) && allowStim && synch && !enableStim && signal_time < fall_time)){
    double stimFrequencyDelay = (1000000 / freq);
    if (sinceLow1 >= stimFrequencyDelay && digitalRead(triggerTrainPin) == 0) {
      digitalWrite(triggerTrainPin, HIGH);
      sinceHigh1 = 0;
    }
    if (sinceHigh1 >= 1000 && digitalRead(triggerTrainPin) == 1) {
      digitalWrite(triggerTrainPin, LOW);
      sinceLow1 = 0;
    }
  }
  else {
    digitalWrite(triggerTrainPin, LOW);
  }
  
  // if (bias1 > 0 or bias2 > 0){
  //   if (bias1 > 0) {
  //     double stimFrequencyDelay = (1000000 / freq);
  //     if (sinceLow1 >= stimFrequencyDelay && digitalRead(triggerTrainPin1) == 0) {
  //       digitalWrite(triggerTrainPin1, HIGH);
  //       sinceHigh1 = 0;
  //     }
  //     if (sinceHigh1 >= 1000 && digitalRead(triggerTrainPin1) == 1) {
  //       digitalWrite(triggerTrainPin1, LOW);
  //       sinceLow1 = 0;
  //     }
  //   } 

  //   if (bias2 > 0) {
  //     double stimFrequencyDelay = (1000000 / freq);
  //     if (sinceLow1 >= stimFrequencyDelay && digitalRead(triggerTrainPin2) == 0) {
  //       digitalWrite(triggerTrainPin2, HIGH);
  //       sinceHigh1 = 0;
  //     }
  //     if (sinceHigh1 >= 1000 && digitalRead(triggerTrainPin2) == 1) {
  //       digitalWrite(triggerTrainPin2, LOW);
  //       sinceLow1 = 0;
  //     }
  //   } 
  // }
  // else if (bias1 == 0 and bias2 == 0){
  //   if ((enableStim || base != 0) && allowStim) {
  //     double stimFrequencyDelay = (1000000 / freq);
  //     if (sinceLow1 >= stimFrequencyDelay && digitalRead(triggerTrainPin) == 0) {
  //       digitalWrite(triggerTrainPin, HIGH);
  //       sinceHigh1 = 0;
  //     }
  //     if (sinceHigh1 >= 1000 && digitalRead(triggerTrainPin) == 1) {
  //       digitalWrite(triggerTrainPin, LOW);
  //       sinceLow1 = 0;
  //     }
  //   } 
  //   else if ((trigger_type == 2 || trigger_type == 5 || trigger_type == 7 || trigger_type == 8) && allowStim && !enableStim && signal_time < fall_time && synch) {
  //     double stimFrequencyDelay = (1000000 / freq);
  //     if (sinceLow1 >= stimFrequencyDelay && digitalRead(triggerTrainPin) == 0) {
  //       digitalWrite(triggerTrainPin, HIGH);
  //       sinceHigh1 = 0;
  //     }
  //     if (sinceHigh1 >= 1000 && digitalRead(triggerTrainPin) == 1) {
  //       digitalWrite(triggerTrainPin, LOW);
  //       sinceLow1 = 0;
  //     }
  //   }
  //   else {
  //     digitalWrite(triggerTrainPin, LOW);
  //   }
}

void Sine_AWG(float& stim_val, float period, float amplitude, float base, int enableStim, elapsedMillis signal_time, int dac_num, int stimLED) {  //
  float analogVoltage;
  float mas = base * 1000;
  if (enableStim) {
    double rad = (signal_time * 3.14159) / period;
    int pas = amplitude * 1000;
    analogVoltage = (pas - mas) * sin(rad) + mas;
    digitalWrite(stimLED, HIGH);
  } else {
    analogVoltage = mas;
    digitalWrite(stimLED, LOW);
  }
  dac.setDACOutVoltage(analogVoltage, dac_num);
  stim_val = analogVoltage / 1000;
}

void SawTooth_AWG(float& stim_val, float rise_time, float amplitude, float base, int enableStim, elapsedMillis signal_time, int dac_num, int stimLED) {
  float analogVoltage;
  float pas = amplitude * 1000;  // peak analog signal (0-4095) corresponding to 0-10V
  float mas = base * 1000;       // min analog signal (0-4095) corresponding to 0-10V
  if (enableStim) {
    analogVoltage = map(signal_time, 0, rise_time, mas, pas);
    digitalWrite(stimLED, HIGH);
  } else {
    analogVoltage = mas;
    digitalWrite(stimLED, LOW);
  }
  dac.setDACOutVoltage(analogVoltage, dac_num);
  stim_val = analogVoltage / 1000;
}

void HFOV_AWG(float& stim_val, float rise_time, float fall_time, float amplitude, float base, int enableStim, elapsedMillis signal_time, int dac_num, float hfov_base, int stimLED) {
  float analogVoltage;
  float pas = amplitude * 1000;  // peak analog signal (0-4095) corresponding to 0-10V
  float mas = base * 1000;       // min analog signal (0-4095) corresponding to 0-10V
  float int_base = (hfov_base/100) * (pas - mas) + mas;
  // float period = rise_time + fall_time;
  float time_in_period = fmod((float)signal_time, (rise_time + fall_time));  // Calculate the time within the current period

  if (enableStim) {
    // If within the rise_time, generate a rising sawtooth
    if (time_in_period <= rise_time) {
      analogVoltage = map(time_in_period, 0, rise_time, int_base, pas);
    }
    // Otherwise, generate a falling sawtooth for the remainder of the period
    else {
      // analogVoltage = map(time_in_period - rise_time, 0, period - rise_time, pas, mas);
      // analogVoltage = map(time_in_period, 0, fall_time, pas, mas);
      analogVoltage = int_base;
    }
    digitalWrite(stimLED, HIGH);
  } else {
    analogVoltage = mas;
    digitalWrite(stimLED, HIGH);
  }
  dac.setDACOutVoltage(analogVoltage, dac_num);
  stim_val = analogVoltage / 1000;
}

void Tri_AWG(float& stim_val, float rise_time, float fall_time, float amplitude, float base, int enableStim, elapsedMillis signal_time, int dac_num, int stimLED) {
  float analogVoltage;
  float pas = amplitude * 1000;  // peak analog signal (0-4095) corresponding to 0-10V
  float mas = base * 1000;       // min analog signal (0-4095) corresponding to 0-10V
  float period = rise_time + fall_time;
  if (enableStim) {
    if (signal_time <= rise_time) {
      analogVoltage = map(signal_time, 0, rise_time, mas, pas);
    } else {
      analogVoltage = map(signal_time, rise_time, period, pas, mas);
    }
    digitalWrite(stimLED, HIGH);
  } else {
    analogVoltage = mas;
    digitalWrite(stimLED, LOW);
  }
  dac.setDACOutVoltage(analogVoltage, dac_num);
  stim_val = analogVoltage / 1000;
}

void Bilevel_AWG(float& stim_val, float period, float rise_time, float fall_time, float amplitude, float base, int enableStim, int allow_stim, elapsedMillis signal_time, int dac_num, int bilevel_t_, int bilevel_v, int stimLED) {
  float analogVoltage;
  float pas = amplitude * 1000;  // peak analog signal (0-4095) corresponding to 0-10V
  float mas = base * 1000;       // min analog signal (0-4095) corresponding to 0-10V
  float pas_bilevel = pas * bilevel_v / 100;
  float time_first_half = period * bilevel_t_ / 100;

  if (enableStim) {
    if (signal_time <= rise_time && rise_time != 0) {
      analogVoltage = map(signal_time, 0, rise_time, mas, pas_bilevel);
    } else if (signal_time <= time_first_half) {
      analogVoltage = pas_bilevel;
    } else if (signal_time <= time_first_half + rise_time) {
      analogVoltage = map(signal_time, time_first_half, time_first_half + rise_time, pas_bilevel, pas);
    } else if (signal_time <= period - fall_time){
      analogVoltage = pas;
    } else if (fall_time != 0 && signal_time > period - fall_time){
      analogVoltage = map(signal_time, period - fall_time, period, pas, mas);
    }
    digitalWrite(stimLED, HIGH);
  } else {
    analogVoltage = mas;
    digitalWrite(stimLED, LOW);
  }
  
  dac.setDACOutVoltage(analogVoltage, dac_num);
  stim_val = analogVoltage / 1000;
}

void Trap_AWG(float& stim_val, float period, float rise_time, float fall_time, float amplitude, float base, int enableStim, int allow_stim, elapsedMillis signal_time, int dac_num, bool synch, int stimLED) {
  float analogVoltage;
  float pas = amplitude * 1000;  // peak analog signal (0-4095) corresponding to 0-10V
  float mas = base * 1000;       // min analog signal (0-4095) corresponding to 0-10V
  float high_time = period - rise_time - fall_time;

  if (trigger_type != 2 && trigger_type != 5 && trigger_type != 7 && trigger_type != 8) {
    if (enableStim) {
      if (signal_time <= rise_time && rise_time != 0) {
        analogVoltage = map(signal_time, 0, rise_time, mas, pas);
      } else if (signal_time <= rise_time + high_time) {
        analogVoltage = pas;
      } else if (fall_time != 0){
        analogVoltage = map(signal_time, period - fall_time, period, pas, mas);
      }
      digitalWrite(stimLED, HIGH);
    } 
    else {
      analogVoltage = mas;
      digitalWrite(stimLED, LOW);
    }
  }

  else if (trigger_type == 2 || trigger_type == 7 || trigger_type == 8) {
    if (enableStim) {
      if (signal_time <= rise_time && rise_time != 0) {
        analogVoltage = map(signal_time, 0, rise_time, mas, pas);
        if (first_start) {
          signal_time_first = signal_time;
          first_start = false;
        }
      } 
      else {
        first_start = true;
        first_end = true;
        analogVoltage = pas;
      }
      digitalWrite(stimLED, HIGH);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    }
    else {
      if (signal_time <= (fall_time) && allow_stim && synch && fall_time != 0) {
        analogVoltage = map(signal_time, 0, fall_time, pas, mas);
        digitalWrite(stimLED, HIGH);
      }
      else {
        analogVoltage = mas;
        digitalWrite(stimLED, LOW);
      }
    }
  }

  else if (trigger_type == 5) {
    if (enableStim) {
      if (signal_time <= rise_time && rise_time != 0) {
        analogVoltage = map(signal_time, 0, rise_time, mas, pas);
      } 
      else {
        analogVoltage = pas;
      }
      digitalWrite(stimLED, HIGH);
    } 
    else {
      if (off_time <= (fall_time) && allow_stim && synch && fall_time != 0) {
        analogVoltage = map(off_time, 0, fall_time, pas, mas);
        digitalWrite(stimLED, HIGH);
      } 
      else {
        analogVoltage = mas;
        digitalWrite(stimLED, LOW);
      }
    }
  }

  dac.setDACOutVoltage(analogVoltage, dac_num);
  stim_val = analogVoltage / 1000;
}

///// END FUNCTIONS FOR STIMULATION /////

// void sensorAOffset() {
//   flowOffsetValue = 0;
//   using index_t = decltype(flowOffset)::index_t;
//   for (index_t i = 0; i < flowOffset.size(); i++) {
//     flowOffsetValue += flowOffset[i] / flowOffset.size();
//   }
// }

void sendFlowPres() {
  SerialA.write(0xff);
  SerialA.write(0x09);
  SerialA.write((byte)0x00);
  SerialA.write(0x88);
}

void testCommand() {
  SerialA.write(0xff);
  SerialA.write(0x05);
  SerialA.write((byte)0x00);
  SerialA.write(0x3c);
}

void startSensor() {
  SerialA.write(0xff);
  SerialA.write(0x0E);
  SerialA.write((byte)0x00);
  SerialA.write(0x26);
}

void boardHardwareReset() {
  SerialA.write(0xff);
  SerialA.write(0x0B);
  SerialA.write((byte)0x00);
  SerialA.write(0x51);
}

void sensorHardReset() {
  SerialA.write(0xff);
  SerialA.write(0x0C);
  SerialA.write((byte)0x00);
  SerialA.write(0xff);
}

void setHeaterOn() {
  SerialA.write(0xff);
  SerialA.write(0x14);
  SerialA.write(0x01);
  SerialA.write(0x01);
  SerialA.write(0x9f);
}

void setHeaterPowerMax() {
  SerialA.write(0xff);
  SerialA.write(0x15);
  SerialA.write(0x01);
  SerialA.write(0x64);
  SerialA.write(0x97);
}

void setBaud() {
  SerialA.write(0xff);
  SerialA.write(0x27);
  SerialA.write(0x01);
  SerialA.write(0x07);
  SerialA.write(0x2e);
}