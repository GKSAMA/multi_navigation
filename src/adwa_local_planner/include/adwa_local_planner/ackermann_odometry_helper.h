#ifndef ACKERMANN_ODOMETRY_HELPER
#define ACKERMANN_ODOMETRY_HELPER

#include <thread>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

struct AckermannState {
    double trans_speed;
    double steer_angle;
    double steer_speed;
};

class AckermannOdometry
{
public:
    AckermannOdometry(std::string odom_topic = "", int num_avg_samples = 10);
    ~AckermannOdometry();

    void getOdom(nav_msgs::Odometry &base_odom);
    void getRobotVel(geometry_msgs::PoseStamped &robot_vel);
    void getAckermannState(AckermannState &state);
    std::string getOdomTopic() const;
    int getAverageSamples();

    void setOdomTopic(std::string odom_topic);
    void setAverageSamples(int num_average_samples);

    void odomCB(const nav_msgs::Odometry::ConstPtr& msg);

private:
    std::string odom_topic_;
    // circular queue
    std::vector<AckermannState> last_states_;
    ros::Subscriber odom_sub_;
    nav_msgs::Odometry base_odom_;
    std::mutex odom_mutex_;
    std::string frame_id_;
};

#endif // ACKERMANN_ODOMETRY_HELPER