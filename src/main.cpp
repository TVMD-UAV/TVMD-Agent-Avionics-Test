/****************************************************************
 * ESP32-S3 Functionality Test
 * Using the SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic)
 * Using the Adafruit BMP280 Library
 * This library is for TVMD single agent flight controller
 *
 * This sketch will output the raw data from the ICM-20948
 ***************************************************************/

#include "ICM_20948.h" // Check out this library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Adafruit_BMP280.h>

#define USE_SPI        // Uncomment this to use SPI
#define USE_IMU_INT    // Uncomment this to use the interrupt functionality of the IMU

#define SERIAL_PORT Serial

// Default HSPI
#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define IMU_SCK_PIN 36
#define IMU_MISO_PIN 37
#define IMU_MOSI_PIN 35
#define IMU_CS_PIN 38     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define IMU_INT_PIN 13    // Which pin you connect INT to

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

Adafruit_BMP280 bmp(14, &SPI_PORT);

// Some vars to control or respond to interrupts
volatile bool isrFired = false;
volatile bool sensorSleep = false;
volatile bool canToggle = false;

void icmISR(void);

void printPaddedInt16b(int16_t val);
void printRawAGMT(ICM_20948_AGMT_t agmt);
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);
void printScaledAGMT(ICM_20948_SPI *sensor);
void printBmpData(Adafruit_BMP280 &bmp);

void setup()
{
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };

#ifdef USE_SPI
  SPI_PORT.begin(IMU_SCK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, IMU_CS_PIN);
  // (int8_t sck=-1, int8_t miso=-1, int8_t mosi=-1, int8_t ss=-1);
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

#ifdef USE_IMU_INT
  pinMode(IMU_INT_PIN, INPUT_PULLUP);                                   // Using a pullup b/c ICM-20948 Breakout board has an onboard pullup as well and we don't want them to compete
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), icmISR, FALLING); // Set up a falling interrupt
#endif

  myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(IMU_CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  {
    // Here we are doing a SW reset to make sure the device starts in a known state
    myICM.swReset();
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.print(F("Software Reset returned: "));
      SERIAL_PORT.println(myICM.statusString());
    }
    delay(250);

    // Now wake the sensor up
    myICM.sleep(sensorSleep);
    myICM.lowPower(false);

    // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

    // Set Gyro and Accelerometer to a particular sample mode
    // options: ICM_20948_Sample_Mode_Continuous
    //          ICM_20948_Sample_Mode_Cycled
    myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled);
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM.statusString());

    ICM_20948_smplrt_t mySmplrt;
    mySmplrt.g = 54;
    myICM.setSampleRate(ICM_20948_Internal_Gyr, mySmplrt);
    SERIAL_PORT.print(F("setSampleRate returned: "));
    SERIAL_PORT.println(myICM.statusString());

    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

    myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                    // gpm2
                    // gpm4
                    // gpm8
                    // gpm16

    myFSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                      // dps250
                      // dps500
                      // dps1000
                      // dps2000

    myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.print(F("setFullScale returned: "));
      SERIAL_PORT.println(myICM.statusString());
    }

    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
    myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                    // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                    // acc_d111bw4_n136bw
                                    // acc_d50bw4_n68bw8
                                    // acc_d23bw9_n34bw4
                                    // acc_d11bw5_n17bw
                                    // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                    // acc_d473bw_n499bw

    myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                      // gyr_d196bw6_n229bw8
                                      // gyr_d151bw8_n187bw6
                                      // gyr_d119bw5_n154bw3
                                      // gyr_d51bw2_n73bw3
                                      // gyr_d23bw9_n35bw9
                                      // gyr_d11bw6_n17bw8
                                      // gyr_d5bw7_n8bw9
                                      // gyr_d361bw4_n376bw5

    myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.print(F("setDLPcfg returned: "));
      SERIAL_PORT.println(myICM.statusString());
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
    ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, true);
    ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
    SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: "));
    SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
    SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: "));
    SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));

    // Choose whether or not to start the magnetometer
    myICM.startupMagnetometer();
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.print(F("startupMagnetometer returned: "));
      SERIAL_PORT.println(myICM.statusString());
    }

    // Now we're going to set up interrupts. There are a lot of options, but for this test we're just configuring the interrupt pin and enabling interrupts to tell us when new data is ready
    /*
      ICM_20948_Status_e  cfgIntActiveLow         ( bool active_low );
      ICM_20948_Status_e  cfgIntOpenDrain         ( bool open_drain );
      ICM_20948_Status_e  cfgIntLatch             ( bool latching );                          // If not latching then the interrupt is a 50 us pulse

      ICM_20948_Status_e  cfgIntAnyReadToClear    ( bool enabled );                           // If enabled, *ANY* read will clear the INT_STATUS register. So if you have multiple interrupt sources enabled be sure to read INT_STATUS first

      ICM_20948_Status_e  cfgFsyncActiveLow       ( bool active_low );
      ICM_20948_Status_e  cfgFsyncIntMode         ( bool interrupt_mode );                    // Can ue FSYNC as an interrupt input that sets the I2C Master Status register's PASS_THROUGH bit

      ICM_20948_Status_e  intEnableI2C            ( bool enable );
      ICM_20948_Status_e  intEnableDMP            ( bool enable );
      ICM_20948_Status_e  intEnablePLL            ( bool enable );
      ICM_20948_Status_e  intEnableWOM            ( bool enable );
      ICM_20948_Status_e  intEnableWOF            ( bool enable );
      ICM_20948_Status_e  intEnableRawDataReady   ( bool enable );
      ICM_20948_Status_e  intEnableOverflowFIFO   ( uint8_t bm_enable );
      ICM_20948_Status_e  intEnableWatermarkFIFO  ( uint8_t bm_enable );
  */
    myICM.cfgIntActiveLow(true);  // Active low to be compatible with the breakout board's pullup resistor
    myICM.cfgIntOpenDrain(false); // Push-pull, though open-drain would also work thanks to the pull-up resistors on the breakout
    myICM.cfgIntLatch(true);      // Latch the interrupt until cleared
    SERIAL_PORT.print(F("cfgIntLatch returned: "));
    SERIAL_PORT.println(myICM.statusString());

    myICM.intEnableRawDataReady(true); // enable interrupts on raw data ready
    // Enable interrupts on dmp ready
    // myICM.intEnableDMP(true);
    SERIAL_PORT.print(F("intEnableRawDataReady returned: "));
    SERIAL_PORT.println(myICM.statusString());

    //  // Note: weirdness with the Wake on Motion interrupt being always enabled.....
    //  uint8_t zero_0 = 0xFF;
    //  ICM_20948_execute_r( &myICM._device, AGB0_REG_INT_ENABLE, (uint8_t*)&zero_0, sizeof(uint8_t) );
    //  SERIAL_PORT.print("INT_EN was: 0x"); SERIAL_PORT.println(zero_0, HEX);
    //  zero_0 = 0x00;
    //  ICM_20948_execute_w( &myICM._device, AGB0_REG_INT_ENABLE, (uint8_t*)&zero_0, sizeof(uint8_t) );

    SERIAL_PORT.println();
    SERIAL_PORT.println(F("Configuration complete!"));
  }


  // BMP
  {
    unsigned status = bmp.begin();
    if (!status) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                        "try a different address!"));
      Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
    }

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }
}

void loop()
{
  if (isrFired)
  { // If our isr flag is set then clear the interrupts on the ICM
    isrFired = false;
    myICM.getAGMT();            // get the A, G, M, and T readings
    printScaledAGMT(&myICM);    // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    myICM.clearInterrupts();  // This would be efficient... but not compatible with Uno
  }

  // if (myICM.dataReady())
  // {
  //   myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
  //                            //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
  //   printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
  //   delay(30);
  // }
  // else
  // {
  //   SERIAL_PORT.println("Waiting for data");
  //   delay(500);
  // }

  printBmpData(bmp);
}

void icmISR(void)
{
  isrFired = true; // Can't use I2C within ISR on 328p, so just set a flag to know that data is available
}

// Below here are some helper functions to print the data nicely!
void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printBmpData(Adafruit_BMP280 &bmp) {
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.println();
}