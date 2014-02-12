#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <SerialStream.h>
#include <iostream>
#include <string>
#include <signal.h>
#include <time.h>
#include <stdlib.h>

// http://sglez.org/2008/08/05/interfacing-arduino-with-c-and-libserial/

std::string port = "/dev/ttyUSB0";
LibSerial::SerialStream arduino;

static bool keepRunning = true;

bool newCommand=0;

int pos=1;

// Array holding the current mole states
static int moleStates[7];

void intHandler(int dummy=0) {
    keepRunning = false;
}

void openArduino()
{
  arduino.Open(port);
  arduino.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
  arduino.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
}

void molePosCallback(const std_msgs::Int16::ConstPtr& msg)
{
  if (msg->data>=0 && msg->data<3) {
    pos=msg->data;
    newCommand=1;
  }
}

void moleStateCallback(const std_msgs::Int32MultiArray::ConstPtr& newStates)
{
  // Sets the mole state values equal to the new values received in the message.
  int i = 0;
  for(std::vector<int>::const_iterator iter = newStates->data.begin(); iter != newStates->data.end(); iter++) {
    moleStates[i] = *iter;
    i++;
  }
  newCommand = 1;
}

int main(int argc, char **argv)
{
  srand(time(NULL));
  int r = rand();

  ros::init(argc, argv, "arduino_whackamole_node");
  ros::NodeHandle n;

  ros::NodeHandle np("~");

  //ros::Subscriber sub = n.subscribe("whackamole/molepos", 1000, molePosCallback);
  ros::Subscriber sub = n.subscribe("whackamole/mole_states", 1000, moleStateCallback);

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


  ros::Rate loop_rate(50.0);

  while (ros::ok() && arduino.IsOpen() && keepRunning)
  {
    if (arduino.good() && newCommand) {
      newCommand = 0;
      //ROS_INFO("sending:  %d",pos);
      ROS_INFO("Sending:");
      int i;
      int molePos;
      for (i=0; i<7; i++) {
        //arduino.put((char)(((rand())%3)+1));
        if(moleStates[i] < 0) {
          molePos = 1;
        } else if(moleStates[i] == 0) {
          molePos = 2;
        } else {
          molePos = 3;
        }
        ROS_INFO("%d", molePos);
        arduino.put((char)(molePos));
      }
      arduino.put((char)0);
    }
    /*if (arduino.good()) {
      char c;
      c = arduino.get();
      ROS_INFO("received: %c",c);
      //char line[256];
      //arduino >> line;
      //ROS_INFO("got word: %s",line);
    }*/
    loop_rate.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Closing arduino");
  arduino.Close();

}
