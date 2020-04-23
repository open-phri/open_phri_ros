#include "ros/ros.h"
#include <OpenPHRI/OpenPHRI.h>

#include <physical_quantities/units/units.h>
#include <iostream>
#include <string>

// #include <OpenPHRI/drivers/vrep_driver.h>
#include "open_phri_ros/add.h"

int add(string type_of_constraint, int constraint_value){

    

    return 0;
}

int main(int argc, char **argv)
  {
   ros::init(argc, argv, "add");
   ros::NodeHandle n;
  
   ros::ServiceServer service = n.advertiseService("add", add);
   ROS_INFO("Ready to add constraint");
   ros::spin();
   return 0;
   }