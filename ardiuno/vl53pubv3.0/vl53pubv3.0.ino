// compiles and runs for RESOLUTION =64
//rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=57600 
// and of course : roscore on noetic

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

#define RESOLUTION    64
ros::NodeHandle nh;
std_msgs::Float32MultiArray vl53_range;
ros::Publisher vl53_range_pub("vl53_range1", &vl53_range);

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

void setup() {
  nh.getHardware()->setBaud(57600); // ros setup
  nh.initNode();              // init ROS
  nh.advertise(vl53_range_pub);
  
  Wire.begin(); //This resets to 100kHz I2C  // i2c setup
  Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 
  myImager.begin();  // todo: set to flash an led for success or fail bd
  myImager.setResolution(8*8); //Enable all 64 pads
  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width
  myImager.startRanging();
}   // end of setup

void loop() {
  int i, w;
  float value[RESOLUTION] ;
  
 //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      w=0;
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
                    
          value[w] = measurementData.distance_mm[x + y] ;
          w++;

        }
      }
    }
  }

  vl53_range.data = value;
  vl53_range.data_length = RESOLUTION;
  vl53_range_pub.publish(&vl53_range);

  nh.spinOnce(); 
  delay(1000);       
}
