
#include <OpenPHRI/OpenPHRI.h>
#include <OpenPHRI/drivers/vrep_driver.h>

#include <physical_quantities/units/units.h>

#include <ros/ros.h>

#include <iostream>

int main(int argc, char* argv[]) {
    using namespace spatial::literals;
    using namespace units::literals;
    ros::init(argc, argv, "open_phri/controller");

    constexpr double time_step = 10.0_ms;
    constexpr size_t joint_count = 7;

    auto robot = phri::Robot{"end-effector"_frame, "base"_frame, "LBR4p", joint_count};
    auto model = phri::RobotModel{robot, "robot_models/kuka_lwr4.yaml", "end-effector"};
    auto controller = phri::SafetyController{robot};
    auto driver = phri::VREPDriver(robot, time_step); 
    auto clock = phri::Clock{time_step}; 
    auto data_logger = phri::DataLogger{"/tmp", clock.getTime(), true};

    data_logger.logSafetyControllerData(&controller);
    data_logger.logRobotData(&robot);
    driver.setScene("vrep_scenes/KukaLWR4.ttt");
    driver.start();

    // Set the task space damping matrix
    robot.control().task().damping().diagonal().setConstant(100.);

	// Initialize the application. Exit on failure.
	if (driver.init()) {
		ROS_INFO("Starting main loop");
	} else {
		ROS_ERROR("Initialization failed");
		std::exit(-1);
	}

	// Run the main loop
	while (ros::ok()) {
        if(not driver.syncThenRead()) {
            ROS_ERROR("Cannot read data from the robot");
            break;
        }
        // TODO reconfigure the controller according to possible service calls
        ros::NodeHandle n;
        ros::ServiceClient controllerClient = n.serviceClient<open_phri_ros::add>("add"); //"controllerClient" mauvais nom?
        open_phri_ros::add srv; 
        srv.request.type_of_constraint = "velocity_constraint";
        srv.request.max_velocity = 2; //pas sur du format - pour le moment, simple valeur

        if (controllerClient.call(srv))
        {
         ROS_INFO("Constraint added");
        }
        else
        {
        ROS_ERROR("Failed to call service add");
        }

        model.forwardKinematics();
        controller();
        if(not driver.send()) {
            ROS_ERROR("Cannot send data to the robot");
            break;
        }
        ros::spinOnce();
	}

	ROS_INFO("Exiting");
    driver.stop();
}

