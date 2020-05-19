#include "ros/ros.h"
#include <OpenPHRI/OpenPHRI.h>

#include <physical_quantities/units/units.h>
#include <iostream>
#include <string>

// #include <OpenPHRI/drivers/vrep_driver.h>
#include "open_phri_ros/add.h"

// bool add(string type_of_constraint, scalar::Velocity max_velocity){ // impossible de faire switch avec string?

bool add(open_phri_ros::add::Request &req, open_phri_ros::add:Response &res){ // impossible de faire switch avec string?


if(req.type_of_constraint=="velocity_constraint") {
    //phri::VelocityConstraint::setMaximumVelocity(&max_velocity);
    //controller().add<phri::VelocityConstraint>("max_velocity", max_velocity);

    auto constraint = std::make_shared<scalar::Velocity>(req.max_velocity);  
    controller().addConstraint("max_velocity", constraint,false);
    res.stateConstraint=true;
}

    return true; // return res.stateConstraint
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

   /* ------------ QUESTIONS ------------ 
    
 -> Dans add.srv, pour les contraintes, quel type mettre ? Pour le moment, scalar::Velocity...
 -> Verifier si le shared pointeur existe ? Pour le creer une premiere fois et apres reecrire dessus si il y a une nouvelle contrainte qui arrive
 -> On passe en entrée de la fonction add un scalar::Velocity ou un pointeur créé par le controlleur (je pense pas)
 -> addConstraint ou utiliser template add<T>()?
 
     */