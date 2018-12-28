#include <Arduino.h>
#include <math.h>
#define ARM_MATH_CM0PLUS
#include <arm_math.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <ArduinoLog.h>
#include <ArduinoSound.h>

//useful macros
#define bit_test(x,n) (x & (0x01<<n))
#define bit_set(x,n) (x=x | (0x01<<n))
#define bit_clear(x,n) (x=x & ~(0x01<<n))


//Log defines
#define LOG_LEVEL   LOG_LEVEL_VERBOSE

// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
extern SERCOM sercom1;
#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2 (SC1PAD2)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3 (SC1PAD3)
// Instantiate the Serial2 class
Uart SerialGPS(&sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX); //add SERCOM1 handler code

//Pinout
/*
Acc
  Acc Int1 <-- A3/D17
  Acc Int2 <-- A4/D18
SD
  SD CD <-- D8
  SD CS --> D4

I2S Mic
  DOUT <-- A7/D9
  BCLK --> D1
  LRCLK/WS --> D0

GPS
  GPS RX --> D10
  GPS TX <-- D12

GPIO Extra
  D11
  D13

Misc
  Status LED Out --> D3
  Misc LED Out --> A5/D19
  Batt Status In <-- A1/D15
  Analog Mic In <-- A2/D16
*/
static const uint8_t pin_acc_int1_int = 17; //A3/D17
static const uint8_t pin_acc_int2_int = 18; //A4/D18
static const uint8_t pin_sd_cd = 8; //D8
static const uint8_t pin_sd_cs = 4; //D4
static const uint8_t pin_gps_rx = 10; //D10
static const uint8_t pin_gps_tx = 12; //D12
static const uint8_t pin_gpio_d11 = 11; //D11
static const uint8_t pin_gpio_d13 = 13; //D13
static const uint8_t pin_status_led = 3; //D3
static const uint8_t pin_misc_led = 19; //A5/D19
static const uint8_t pin_batt_volt = A1; //A1/D15
static const uint8_t pin_analog_mic = A2; //A2/D16

//Constants
const char dev_id[] = "GPS_Sensor";
const uint16_t acc_sensor_id = 12345;
const int16_t utc_offset = (-3) * SECS_PER_HOUR;   // Adjust your time

//Objects
TinyGPSPlus gps;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(acc_sensor_id);
AmplitudeAnalyzer amplitudeAnalyzer;

//needed for Serial communication
void SERCOM1_Handler()
{
  char uart_char;
  // Serial2.IrqHandler();
  if (sercom1.availableDataUART())
  {
    uart_char = sercom1.readDataUART();
    gps.encode(uart_char);
    // Serial.write(uart_char);
    // if (gps.time.isValid())
    // {
    //   setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
    //           gps.date.day(), gps.date.month(), gps.date.year());
    // }
  }
}

void gps_init(void)
{
  uint8_t i;
  //NAVX5 Enable AssistNow and Signal Atenuation Configuration (not supported in V18)
  uint8_t UBX_CFG_NAVX5[48] = {0xb5, 0x62, 0x06, 0x23, 0x28, 0x00, 0x02, 0x00,
                               0x4c, 0x66, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x03, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x4b, 0x07, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x01, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8e, 0xdf};
  //PMS Enable Full Power
  uint8_t UBX_CFG_PMS[16] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x94, 0x5A};

  //NAV5 Enable Automotive mode
  uint8_t UBX_CFG_NAV5[44] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x04,
                            0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
                            0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E,
                            0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0xC4};
  // //NAV5 Enable Pedestrian mode
  // uint8_t UBX_CFG_NAV5[44] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03,
  //                           0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  //                           0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E,
  //                           0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  //                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0xA2};


  //TODO: configurar tasa de actualización a 10Hz
	//deshabilita mensajes de GPS que no se van a utilizar
  // SerialGPS.println(F("$PUBX,40,RMC,0,0,0,0*47")); //RMC OFF
  // delay(1100);
  Log.notice("Init GPS...");
  SerialGPS.println(F("$PUBX,40,GGA,0,0,0,0*5A")); //GGA OFF
  SerialGPS.flush();
  SerialGPS.println("$PUBX,40,GLL,0,0,0,0*5C"); // Disable GLL
  SerialGPS.flush();
  SerialGPS.println("$PUBX,40,ZDA,0,0,0,0*44"); // Disable ZDA
  SerialGPS.flush();
  SerialGPS.println("$PUBX,40,VTG,0,0,0,0*5E"); // Disable VTG
  SerialGPS.flush();
  SerialGPS.println("$PUBX,40,GSV,0,0,0,0*59"); // Disable GSV
  SerialGPS.flush();
  SerialGPS.println("$PUBX,40,GSA,0,0,0,0*4E"); // Disable GSA
  SerialGPS.flush();
  for (i = 0; i < sizeof(UBX_CFG_PMS); i++)
  {
    SerialGPS.write(UBX_CFG_PMS[i]);
  }
  SerialGPS.flush();
  // for (i = 0; i < sizeof(UBX_CFG_NAV5); i++)
  // {
  //   SerialGPS.write(UBX_CFG_NAV5[i]);
  // }
  // SerialGPS.flush();
  // for (i = 0; i < sizeof(UBX_CFG_NAVX5); i++)
  // {
  //   SerialGPS.write(UBX_CFG_NAVX5[i]);
  // }
  // SerialGPS.flush();
  Log.notice("Done\n");
}

void acc_init(void)
{
  Log.notice("Init Acc...");
  // Serial.println("Accelerometer Test"); Serial.println("");
	/* Initialise the sensor */
	if(!accel.begin())
	{
		/* There was a problem detecting the ADXL345 ... check your connections */
    Log.error("No Acc detected");
    // Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
		while(true);
	}

	/* Set the range to whatever is appropriate for your project */
	accel.setRange(ADXL345_RANGE_16_G);
	// displaySetRange(ADXL345_RANGE_8_G);
	// displaySetRange(ADXL345_RANGE_4_G);
	// displaySetRange(ADXL345_RANGE_2_G);

  Log.notice("Done\n");
}

void mic_init(void)
{
  Log.notice("Init Mic...");

    // start I2S at 16 kHz with 32-bits per sample
    // if (!I2S.begin(I2S_PHILIPS_MODE, 16000, 32))
    // {
    //   Serial.println("Failed to initialize I2S!");
    //   while (1); // do nothing
    // }

  // setup the I2S audio input for 44.1 kHz with 32-bits per sample
  // if (!AudioInI2S.begin(44100, 24))
  if (!AudioInI2S.begin(44100, 32))
  {
    Log.error("Failed to initialize I2S input!\n");
    while (1); // do nothing
  }

  // configure the I2S input as the input for the amplitude analyzer
  if (!amplitudeAnalyzer.input(AudioInI2S))
  {
    Log.error("Failed to set amplitude analyzer input!");
    while (1); // do nothing
  }

  Log.notice("Done\n");
}

bool get_mic_read(uint32_t &mic_read)
{
  //TODO: convert value to dB
  //TODO: change ArduinoSound to get 24bit data instead of 32bit
  //TODO: add retry so it outputs a different value rather than 0
  if (amplitudeAnalyzer.available())
  {
    // read the new amplitude
    // return amplitudeAnalyzer.read();
    mic_read = amplitudeAnalyzer.read();

    // mic_read = mic_read >> 6; //got a 24bit value, reduce it to a 18bit value LSB

    // //check if it's negative or not
    // if(bit_test(mic_read,17))
    // {
    //   Serial.print("OR: ");
    //   Serial.println(mic_read);
    //   bit_clear(mic_read,17);
    //   bit_set(mic_read, 31);
    // }

    return true;
  }
  else
  {
    return false;
  }

  // mic_read = I2S.read();
  //
  // Serial.println(mic_read);
  //
  // if(mic_read == 0 || mic_read == -1)
  // {
  //   // Serial.print("O: ");
  //   // Serial.println((uint32_t)mic_read);
  //   mic_read = 0;
  //   return false;
  // }
  //
  // mic_read >>= 14; //turn into 18bit
  // // Serial.print("Mic: ");
  // // Serial.println(mic_read);
  // return true;
}

float_t get_acc_vector(float_t acc_x, float_t acc_y, float_t acc_z)
{
  //Calculates Acceleration vector value
  
  float_t acc[3];
  float_t acc_vector;

  acc[0] = acc_x;
  acc[1] = acc_y;
  acc[2] = acc_z;

  arm_power_f32(acc, 3, &acc_vector);
  arm_sqrt_f32(acc_vector, &acc_vector);

  // arm_rms_f32 (acc, 3, &acc_vector);

  return acc_vector;
}

void save_sd_data(float_t acc_x, float_t acc_y, float_t acc_z, double_t lat,
                  double_t lng, uint32_t mic_value, TinyGPSTime gps_time,
                  TinyGPSDate gps_date)
{
  File sdFile;
  char str_buffer[20];
  // time_t current_time;
  String sd_data_string;

  // if (gps_time.isValid())
  // {
  //   setTime(gps_time.hour(), gps_time.minute(), gps_time.second(),
  //           gps_date.day(), gps_date.month(), gps_date.year());
  //   adjustTime(utc_offset);
  //   // current_time = now();
  // }

  setTime(gps_time.hour(), gps_time.minute(), gps_time.second(),
          gps_date.day(), gps_date.month(), gps_date.year());
  adjustTime(utc_offset);

  sdFile = SD.open("bike.txt", FILE_WRITE);
  if (sdFile)
  {
    //"Unix Time";"ID";"Lat";"Lng";;"0";"0";"0";"0";"0";"0";"0";"0";"0";"Audio";;"PromAcc";"YEAR-MONTH-DAY hh:mm:ss"
    sd_data_string = "\""+String(now())+"\","+"\""+dev_id+"\","+"\""
                    +String(lat,6)+"\","+"\""+String(lng,6)+"\",,";
    sdFile.print(sd_data_string);
    sdFile.print("\"0\",\"0\",\"0\",\"0\",\"0\",\"0\",\"0\",\"0\",\"0\",");
    // sprintf(str_buffer,"\"%ld\";;", mic_value); //Sound
    if (mic_value > 0)
    {
      sd_data_string = "\""+String(20.0*log10((double)mic_value),3)+"\",,";
    }
    else
    {
      sd_data_string = "\"0\",,";
    }
    sdFile.print(sd_data_string);
    sd_data_string = "\""+String(get_acc_vector(acc_x,acc_y,acc_z),3)+"\",";
    sdFile.print(sd_data_string);
    sprintf(str_buffer,"\"%02d-%02d-%02d %02d:%02d:%02d\"\n", year(),
                                                              month(),
                                                              day(),
                                                              hour(),
                                                              minute(),
                                                              second());
    sdFile.print(str_buffer);
    // sd_data_string = "x:"+String(acc_x)+"y:"+String(acc_y)+"z:"+String(acc_z);
    // sdFile.println(sd_data_string);
    sdFile.close();

  	// sprintf(str_buffer,"%02d/%02d/%02d,", day(), month(), year()); //Date
  	// sdFile.print(str_buffer);
  	// sprintf(str_buffer,"%02d:%02d:%02d,", hour(), minute(), second()); //Time
  	// sdFile.print(str_buffer);
    // gps_acc_string = String(lat,3)+','+String(lng,3)+','+String(acc_x,3)+','
    //                                   +String(acc_y,3)+','+String(acc_z,3)+',';
    // sdFile.print(gps_acc_string);
    // sprintf(str_buffer,"%.3f,%.3f,", lat, lng); //Location
  	// sdFile.print(str_buffer);
    // sprintf(str_buffer,"%.3f,%.3f,%.3f,", acc_x, acc_y, acc_z); //Vibration
  	// sdFile.print(str_buffer);
  	// sdFile.close();
  }
}

void collect_data(float_t &acc_x, float_t &acc_y, float_t &acc_z, uint32_t &mic_read)
{
  sensors_event_t event;
  uint32_t mic_read_temp;

  accel.getEvent(&event);

  if (acc_x == 0 || acc_y == 0 || acc_z == 0)
  {
    acc_x = event.acceleration.x;
    acc_y = event.acceleration.y;
    acc_z = event.acceleration.z;
  }
  else
  {
    acc_x = (acc_x + event.acceleration.x)/2;
    acc_y = (acc_y + event.acceleration.y)/2;
    acc_z = (acc_z + event.acceleration.z)/2;
  }

  if (mic_read == 0)
  {
    if (!get_mic_read(mic_read))
    {
      mic_read = 0;
    }
  }
  else
  {
    if (get_mic_read(mic_read_temp))
    {
      mic_read = (mic_read + mic_read_temp)/2;
    }

    // if (get_mic_read(mic_read_temp[0]))
    // {
    //   //arm_sqrt_f32 (float32_t in, float32_t *pOut)
    //   //arm_rms_f32 (float32_t *pSrc, uint32_t blockSize, float32_t *pResult)
    //   //arm_rms_q31 (q31_t *pSrc, uint32_t blockSize, q31_t *pResult)
    //   mic_read_temp[1] = mic_read;
    //   arm_rms_q31 (mic_read_temp, sizeof(mic_read_temp), &mic_read);
    // }
  }
}

void setup(void)
{
	Serial.begin(115200L);

  // while (!Serial) {
  //     ; // wait for serial port to connect. Needed for native USB port only
  // }
  delay(3000);

	SerialGPS.begin(9600);

  //pin initialization
  pinMode(pin_sd_cs, OUTPUT);
  pinMode(pin_misc_led, OUTPUT);
  pinMode(pin_status_led, OUTPUT);

	if (!SD.begin(pin_sd_cs))
	{
  	Serial.println("SD initialization failed!");
    // #define DISABLE_LOGGING
  	// return;
	}
  else
  {
    //init logging SD
    Log.begin(LOG_LEVEL, &Serial, false);
    //Log.setPrefix(printTimestamp); // Uncomment to get timestamps as prefix
    //Log.setSuffix(printNewline); // Uncomment to get newline as suffix
  }

  Log.notice("System Init\n");
  // Serial.println("initialization done.");

  //hardware initialization
  gps_init();
  acc_init();
  mic_init();

  Log.notice("System Init Done\n");
}

void loop(void)
{
  double_t lng;
  double_t lat;
  static double_t last_lng;
  static double_t last_lat;
  static float_t acc_x = 0;
  static float_t acc_y = 0;
  static float_t acc_z = 0;
  static uint32_t mic_value = 0;

  if (gps.location.isValid())
  {
    if (gps.location.isUpdated())
    {
      //only logs data if position is updated
      
      lat = gps.location.lat();
      lng = gps.location.lng();

      lat = ceil(lat*10000.0)/10000.0;
      lng = ceil(lng*10000.0)/10000.0;

      if (last_lat != lat || last_lng != lng)
      {
        save_sd_data(acc_x, acc_y, acc_z, gps.location.lat(), gps.location.lng(), mic_value, gps.time, gps.date);
        last_lat = lat;
        last_lng = lng;
        acc_x = 0;
        acc_y = 0;
        acc_z = 0;
        mic_value = 0;
      }
      digitalWrite(pin_status_led, LOW);
      digitalWrite(pin_misc_led, HIGH);
    }
    else
    {
      collect_data(acc_x, acc_y, acc_z, mic_value);
      digitalWrite(pin_status_led, HIGH);
      digitalWrite(pin_misc_led, LOW);
    }
  }
  else
  {
    digitalWrite(pin_status_led, HIGH);
    digitalWrite(pin_misc_led, LOW);
  }
}
