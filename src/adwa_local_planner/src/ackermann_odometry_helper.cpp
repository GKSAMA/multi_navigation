#include "adwa_local_planner/ackermann_odometry_helper.h"
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

AckermannOdometry::AckermannOdometry(std::string odom_topic, int num_avg_samples):
    odom_topic_(odom_topic)
{
    last_states_.resize(num_avg_samples);
}

AckermannOdometry::~AckermannOdometry()
{

}

void printOdom(const nav_msgs::Odometry& msg)
{
    std::cout << "OdomMSg: ";
    std::cout << "linearX: " << msg.twist.twist.linear.x
              << " linearY: " << msg.twist.twist.linear.y
              << " linearZ: " << msg.twist.twist.linear.z
              << " angularX: " << msg.twist.twist.angular.x
              << " angularY: " << msg.twist.twist.angular.y
              << " angularZ: " << msg.twist.twist.angular.z
              << std::endl;
}

void printState(const AckermannState& msg)
{
    std::cout << "AckermannState: ";
    std::cout << "trans_speed: " << msg.trans_speed
              << " steer_angle: " << msg.steer_angle
              << " steer_speed: " << msg.steer_speed
              << std::endl;
}

void AckermannOdometry::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    static int current_index = 0;
    ROS_INFO_ONCE("odom received!");
    std::lock_guard<std::mutex> odomGuard(odom_mutex_);
    // translation speed -> linear.x.
    // steer angle -> angular.z
    // steer speed -> linear.z
    this->last_states_[current_index].trans_speed = msg->twist.twist.linear.x;
    this->last_states_[current_index].steer_angle = msg->twist.twist.angular.z;
    this->last_states_[current_index].steer_speed = msg->twist.twist.linear.z;
    current_index = (current_index + 1) % this->last_states_.size();
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.linear.z = msg->twist.twist.linear.z;
    base_odom_.twist.twist.angular.x = msg->twist.twist.angular.x;
    base_odom_.twist.twist.angular.y = msg->twist.twist.angular.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    // printOdom(base_odom_);
    // std::cout << "last state : ";
    // printState(this->last_states_[current_index]);
}

void AckermannOdometry::getOdom(nav_msgs::Odometry &base_odom)
{
    std::lock_guard<std::mutex> odomGuard(odom_mutex_);
    base_odom = base_odom_;
}

void AckermannOdometry::getRobotVel(geometry_msgs::PoseStamped& robot_vel)
{
    std::lock_guard<std::mutex> odomGuard(odom_mutex_);
    robot_vel.header.frame_id = base_odom_.child_frame_id;
    robot_vel.header.stamp = ros::Time();
    robot_vel.pose.position.x = base_odom_.twist.twist.linear.x;
    robot_vel.pose.position.y = base_odom_.twist.twist.linear.y;
    robot_vel.pose.position.z = base_odom_.twist.twist.linear.z;
    tf2::Quaternion q;
    q.setRPY(0, 0, base_odom_.twist.twist.angular.z);
    tf2::convert(q, robot_vel.pose.orientation);
    // robot_vel.pose.orientation = tf::createQuaternionFromYaw(base_odom_.twist.twist.angular.z);
}

void AckermannOdometry::getAckermannState(AckermannState &state)
{
    unsigned int i = 0;
    // std::cout << "get odom from : " << odom_topic_ << std::endl;
    // printOdom(base_odom_);
    std::lock_guard<std::mutex> odomGuard(odom_mutex_);
    state.trans_speed = 0;
    state.steer_angle = 0;
    state.steer_speed = 0;
    for(i = 0; i < this->last_states_.size(); ++i){
        state.trans_speed += this->last_states_[i].trans_speed;
        state.steer_angle += this->last_states_[i].steer_angle;
        state.steer_speed += this->last_states_[i].steer_speed;
    }
    state.trans_speed /= this->last_states_.size();
    state.steer_angle /= this->last_states_.size();
    state.steer_speed /= this->last_states_.size();
    // std::cout << "current state: ";
    // printState(state);
}

void AckermannOdometry::setOdomTopic(std::string odom_topic)
{
    if(odom_topic != odom_topic_){
        odom_topic_ = odom_topic;
        if(odom_topic != ""){
            ros::NodeHandle nh;
            // std::cout << "odom nodehandle name : " << ros::this_node::getName() <<  std::endl;
            odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom_topic_, 1, boost::bind(&AckermannOdometry::odomCB, this, _1));
        } else {
            odom_sub_.shutdown();
        }
    }
}

void AckermannOdometry::setAverageSamples(int num_avg_samples)
{
    std::lock_guard<std::mutex> odomGuard(odom_mutex_);
    if(num_avg_samples != this->last_states_.size()){
        last_states_.resize(num_avg_samples);
    }
}

std::string AckermannOdometry::getOdomTopic() const
{
    return this->odom_topic_;
}

int AckermannOdometry::getAverageSamples()
{
    return this->last_states_.size();
}