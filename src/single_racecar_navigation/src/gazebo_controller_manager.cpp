#include "single_racecar_navigation/gazebo_controller_manager.h"


static const std::vector<std::string> controllerList = {"joint_state_controller", "left_rear_wheel_velocity_controller", "right_rear_wheel_velocity_controller",
                                                            "left_front_wheel_velocity_controller", "right_front_wheel_velocity_controller", "left_steering_hinge_position_controller",
                                                            "right_steering_hinge_position_controller"};

bool startControllers(ros::NodeHandle &nodeHandle, std::string serviceName) {
    ros::ServiceClient switchController = nodeHandle.serviceClient<controller_manager_msgs::SwitchController>(serviceName);
    controller_manager_msgs::SwitchController switchControllerMsg;
    switchControllerMsg.request.start_controllers = controllerList;
    switchControllerMsg.request.strictness = switchControllerMsg.request.STRICT;
    ros::service::waitForService(serviceName, -1);
    switchController.call(switchControllerMsg);

    if (switchControllerMsg.response.ok){
        ROS_INFO_STREAM("Controller start correctly");
        return true;
    } else {
        ROS_ERROR_STREAM("Error occured trying to start controller");
        return false;
    }
}

bool stopControllers(ros::NodeHandle &nodeHandle, std::string serviceName) {

    ros::ServiceClient switchController = nodeHandle.serviceClient<controller_manager_msgs::SwitchController>(serviceName);
    controller_manager_msgs::SwitchController switchControllerMsg;
    switchControllerMsg.request.stop_controllers = controllerList;
    switchControllerMsg.request.strictness = switchControllerMsg.request.STRICT;
    switchControllerMsg.request.start_asap = false;
    ros::service::waitForService(serviceName, -1);
    switchController.call(switchControllerMsg);

    if (switchControllerMsg.response.ok){
        ROS_INFO_STREAM("Controller stop correctly");
        return true;
    } else {
        ROS_ERROR_STREAM("Error occured trying to stop controller");
        return false;
    }
}

bool ResetRobotBySystem(ros::NodeHandle &nodeHandle, std::string robotName, float pos_x, float pos_y, float pos_z, float pos_th) {
    int deleteModelId = system(std::string("rosservice call gazebo/delete_model '{model_name: " + robotName + "}'").c_str());
    ROS_INFO("delete state: %d", deleteModelId);
   
    int urdfStateId = system(std::string("rosrun gazebo_ros spawn_model -urdf -x " + std::to_string(pos_x) + " -y " + std::to_string(pos_y) + " -z " + std::to_string(pos_z) + " -Y " + std::to_string(pos_th) + " -model " + robotName + " -param robot_description -unpause").c_str());
    ROS_INFO("spawn model state: %d", urdfStateId);

     int controllersStateId = system(std::string("rosrun controller_manager spawner __ns:=/" + robotName + " joint_state_controller "\
           "left_rear_wheel_velocity_controller right_rear_wheel_velocity_controller "\
           "left_front_wheel_velocity_controller right_front_wheel_velocity_controller "\
           "left_steering_hinge_position_controller right_steering_hinge_position_controller").c_str());
    ROS_INFO("controller statu: %d", controllersStateId);
    sleep(1);

    stopControllers(nodeHandle, "/" + robotName + "/controller_manager/switch_controller");
    startControllers(nodeHandle, "/" + robotName + "/controller_manager/switch_controller");
    return true;
}

bool ResetRobotByService(ros::NodeHandle &nodeHandle, 
                         ros::ServiceClient &modelStateClient, 
                         ros::ServiceClient &jointStateClient, 
                         std::string robotName) {
    stopControllers(nodeHandle, "/" + robotName + "/controller_manager/switch_controller");

    gazebo_msgs::ModelState modelState;
    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::SetModelConfiguration setjointstate;

    modelState.model_name = robotName;
    modelState.reference_frame = "map";

    geometry_msgs::Twist model_twist;
    geometry_msgs::Pose model_pose;
    model_twist.linear.x = 0.0;
    model_twist.linear.y = 0.0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;
    model_pose.position.x =0;
    model_pose.position.y =0;
    model_pose.position.z =0.3;
    model_pose.orientation.w = 1;
    model_pose.orientation.x = 0;
    model_pose.orientation.y = 0;
    model_pose.orientation.z = 0;

    modelState.twist = model_twist;
    modelState.pose = model_pose;

    setmodelstate.request.model_state = modelState;
    setjointstate.request.model_name = robotName;
    setjointstate.request.urdf_param_name = "robot_description";
    setjointstate.request.joint_names = {"left_rear_wheel_velocity_controller", "right_rear_wheel_velocity_controller",
                                        "left_front_wheel_velocity_controller", "right_front_wheel_velocity_controller", 
                                        "left_steering_hinge_position_controller", "right_steering_hinge_position_controller"};
    double hip_angle = 0.3;
    double thigh_angle = 1.1;
    double calf_angle = -2.2;

    setjointstate.request.joint_positions = {-hip_angle,thigh_angle,calf_angle,
                                             hip_angle,thigh_angle,calf_angle,
                                             -hip_angle,thigh_angle,calf_angle,
                                             hip_angle,thigh_angle,calf_angle};

    ros::service::waitForService("/gazebo/set_model_state", -1);
    modelStateClient.call(setmodelstate);
    ros::service::waitForService("/gazebo/set_model_configuration", -1);
     if (jointStateClient.call(setjointstate))
    {
        ROS_INFO("BRILLIANT!!!");
        startControllers(nodeHandle, "/" + robotName + "/controller_manager/switch_controller");
        return true;
    } else
    {
        ROS_ERROR("Failed to set joints");
        return false;
    }
}