/*
   SparkFun 9DOF Magnetometer Calibration Code
  This code will give the offsets and scale factors for the
  SparkFun 9DOF stick Magnetometer which can be used for
  calibration.
  By: Thomas Horning
***This Code is 100% open source. It is free to be used and altered***
  ////////////////////////////////////////////////
  Steps:
  1. Upload code to Arduino with connected SparkFun
  9DOF sensor stick
  2. Rotate sensor between 1 Hz and 2 Hz in the direction given
  3. Add offset and scale to your desired code as shown in printHeading() function
*/

// Declarations
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

// Library
LSM9DS1 imu;

// I2C setup
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

/* Sketch Output Settings
   Comment out the definitions not to be used
   Uncomment the tools that should be used
*/
//Prints calculated values in printMag() function
#define PRINT_CALCULATED

//Prints raw values in printMag() function
//#define PRINT_RAW

//Set the delays for data prints
#define PRINT_SPEED 150 // 250 ms between prints

//Print corrected Magnetic component data after calibration
//#define MAG_DATA

//Print corrected, calculated heading data after calibration
//#define HEADING

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. These links will
// give you the declination of your location (East is positive
// West is negative):
// http://www.magnetic-declination.com
// http://www.ngdc.noaa.gov/geomag-web/#declination

#define DECLINATION 8.58 // Declination (degrees) in Boulder, CO.

//Global variables used in calibration calculations
float Xoffset = 0;
float Yoffset = 0;
float Xscale = 0;
float Yscale = 0;

float minX = 0;
float maxX = 0;
float minY = 0;
float maxY = 0;

float offX = 0;
float offY = 0;
float scalX = 0;
float scalY = 0;

int j = 0;

void setup()
{

  Serial.begin(115200);

  //LSM9DS1 communication set-up
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  // Verification of connection with compass
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1)
      ;
  }

}

void loop()
{
  // Tells user how to move compass during calibration
  Serial.println("=====================");
  Serial.println("Begining the sensor calibration. ");

  for (j; j < 4; j++) {
    switch (j) {
      case 0:
        Serial.println("Figure 8: Hold the compass flat, with Z-axis pointing up and drive in a Figure 8 while keeping it level");
        Serial.println("Complete a circuit once very 3 or so seconds");
        break;
      case 1:
        Serial.println("Rotate around the X-Axis");
        Serial.println("Complete a rotation once very 3 or so seconds");
        break;
      case 2:
        Serial.println("Rotate around the Y-Axis");
        Serial.println("Complete a rotation once very 3 or so seconds");
        break;
      case 3:
        Serial.println("Rotate around the Z-Axis");
        Serial.println("Complete a rotation once very 3 or so seconds");
        break;
    }

    delay(4000);
    Serial.println("Starting this Calibration Step");

    // Begin Calibration Steps
    MagCalibration();
  }

  Serial.println("============================");
  Serial.println("----Calibration Complete----");

  //Print final offsets and scale factors. These values should be added to code that uses this compass.
  Serial.print("Final Offsets: X = "); Serial.print(Xoffset); Serial.print("\tY = "); Serial.println(Yoffset);
  Serial.print("Final Scales: X = "); Serial.print(Xscale); Serial.print("\tY = "); Serial.println(Yscale);
  delay(8000);

  // If MAG_DATA was declared, Show component Data 250 times (This is good data to plot)
#ifdef MAG_DATA
  Serial.println("Printing Corrected Magnetic Component Data");
  for (int k = 0; k < 250; k++) {
    printMag();   // Print "M: mx, my, mz"
    Serial.println();
    delay(PRINT_SPEED);
  }

  // If HEADING was declared, Show calculated heading Data 250 times
#elif HEADING
  Serial.println("Printing Corrected Magnetic Heading Data");
  for (int l = 0; l < 250; l++) {
    printHeading(); //Changed to calculated magnetic values for ease of use
    delay(PRINT_SPEED);
  }
#endif
}

// Function that produces offset and scale factors from magnetic readings
void MagCalibration()
{
  float avgX, avgY, radAvg;
  offX = Xoffset;
  offY = Yoffset;
  scalX = Xscale;
  scalY = Yscale;

  // Find the max and min values of 200 data points
  for (int i = 0; i < 200; i++)
  {
    imu.readMag();
    if (imu.calcMag(imu.mx) < minX) minX = imu.calcMag(imu.mx);
    if (imu.calcMag(imu.mx) > maxX) maxX = imu.calcMag(imu.mx);
    if (imu.calcMag(imu.my) < minY) minY = imu.calcMag(imu.my);
    if (imu.calcMag(imu.my) > maxY) maxY = imu.calcMag(imu.my);
    // Calculate offsets and add to running total
    offX += (maxX + minX) / 2;
    offY += (maxY + minY) / 2;
    // Calculate the average for each axis
    avgX = (maxX - minX) / 2;
    avgY = (maxY - minY) / 2;
    // Calculate average radius
    radAvg = (avgX + avgY) / 2;
    // Add each scale value to running total
    scalX += radAvg / avgX;
    scalY += radAvg / avgY;

    //Print information to screen
    Serial.print(imu.calcMag(imu.mx));
    Serial.print(":");
    Serial.print(imu.calcMag(imu.my));
    Serial.print(":");
    Serial.print(offX / (i + 1));
    Serial.print(":");
    Serial.print(offY / (i + 1));
    Serial.print(":");
    Serial.print(scalX / (i + 1));
    Serial.print(":");
    Serial.print(scalY / (i + 1));
    Serial.print("\n");
    delay(PRINT_SPEED);

  }
  // Divide running totals by number of values added
  Xoffset = offX / 201;
  Yoffset = offY / 201;
  Xscale = scalX / 201;
  Yscale = scalY / 201;
}

//Prints Magnetic information to serial
void printMag()
{
  // Update Mag data
  imu.readMag();


  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  //Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print((imu.calcMag(imu.mx) - Xoffset), 2);
  Serial.print("\t");
  Serial.print((imu.calcMag(imu.my) - Yoffset), 2);
  Serial.print("\t");
  //Serial.println(imu.calcMag(imu.mz), 2);
  //Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print("\t");
  Serial.print(imu.my);
  Serial.print("\t");
  Serial.println(imu.mz);
#endif
}

// Calculates and prints heading in degrees
void printHeading()
{

  float x, y;
  imu.readMag();
  // Subtract calculated offsets from magnetometer data
  x = imu.calcMag(imu.mx) - Xoffset;
  y = imu.calcMag(imu.my) - Yoffset;

  // Scaling correction
  x *= Xscale;
  y *= Yscale;

  // Calculate heading
  float heading;

  if (y == 0)
    heading = (x < 0) ? PI : 0;
  else
    heading = atan2(y, x);

  heading -= DECLINATION * (PI / 180);

  if (heading > 2 * PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += (2 * PI);

  // Convert everything from radians to degrees:
  heading *= (180.0 / PI);

  Serial.print("Heading: "); Serial.println(heading, 2);
}
