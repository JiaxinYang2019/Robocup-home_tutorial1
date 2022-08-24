#include <ros/ros.h>

#include <rover/rover.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "rover_node");

  rover::Rover rover;

  ros::NodeHandle nh;
  //#>>>>TODO: initialize the rover  ????

  //#>>>>TODO: Check if its return value is true, else stop the node here and search the error  ????

  if (rover.initialize(nh) == false)
      return -1;//#>>>>TODO: call the initialize(...) function

  ros::Rate rate(60);
  ros::Time time;
  ros::Time prev_time = ros::Time::now();
  ros::Duration duration;

  while(ros::ok())
  {
    time = ros::Time::now();
    duration = time - prev_time;
    prev_time = time;

    rover.update(time, duration);//#>>>>TODO: call the update(...) function

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};