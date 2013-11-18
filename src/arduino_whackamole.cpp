#include <ros/ros.h>
#include <SerialStream.h>
#include <iostream>
#include <string>
#include <signal.h>

// http://sglez.org/2008/08/05/interfacing-arduino-with-c-and-libserial/

std::string port = "/dev/ttyUSB1";
LibSerial::SerialStream arduino;

static bool keepRunning = true;

void intHandler(int dummy=0) {
    keepRunning = false;
}

void openArduino()
{
  arduino.Open(port);
  arduino.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
  arduino.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arduino_whackamole_node");
  ros::NodeHandle n;

  ros::NodeHandle np("~");

  if (!np.getParam("port",port))
    ROS_WARN("Error getting port, sticking with default");

  // Subscribes to whack-a-mole mole state commands
  //ros::Subscriber mole_states_sub = n.subscribe("whackamole/mole_states", 1000, moleStatesCallback);

  ROS_INFO("Opening arduino on port %s for serial communication", port.c_str());

  openArduino();

  if (!arduino.IsOpen()) {
    ROS_ERROR("arduino not open, closing.\nsuggest: ls /dev/ttyUSB*");
    return -1;
  }


  ros::Rate loop_rate(10.0);

  while (ros::ok() && arduino.IsOpen() && keepRunning)
  {
    if (arduino.good()) {
      ROS_INFO("sending:  %s","H");
      arduino.put('H');
    }
    if (arduino.good()) {
      char c;
      c = arduino.get();
      ROS_INFO("received: %c",c);
      //char line[256];
      //arduino >> line;
      //ROS_INFO("got word: %s",line);
    }
    loop_rate.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Closing arduino");
  arduino.Close();

}