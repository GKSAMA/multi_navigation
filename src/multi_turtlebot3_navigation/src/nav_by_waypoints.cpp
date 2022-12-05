//
// Created by gk on 2022/3/7.
//

#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "tf/transform_listener.h"
#include "actionlib/client/simple_action_client.h"
#include "signal.h"
#include "iostream"
#include "fstream"
#include "vector"
#include "string"
#include "multi_turtlebot3_navigation/utils.h"
#include "multi_turtlebot3_navigation/RunState.h"
#include "sys/shm.h"

using namespace std;

#define path "/home/gk/Documents/multi_turtlebot3_navigation/src/multi_turtlebot3_navigation/config/waypoints.data"
// #define path "/home/gk/Documents/multi_turtlebot3_navigation/src/multi_turtlebot3_navigation/config/waypoints_.data"
#define timedatapath "/home/gk/Documents/DataRecord/segtime.data"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
vector<double> goals_data;
int cnt;    // the number of points
int robots = 4; //the number of robots
string robotNameSpace = "tb3";

void DoShutdown(int sig){
    ROS_INFO("Navigation has ended");
    ros::shutdown();
    exit(sig);
};

void ReadGoal(vector<double> &p){
    ifstream ifs;
    ifs.open(path,ios::in);
    double d;
    while(ifs >> d){
        p.push_back(d);
    }
    cnt = int(p.size()/4);
    cout << "Get " << cnt << " group(s) waypoints" <<endl;
}

void saveTime(double time, int segment, int idx){
    ofstream ofs;
    ofs.open(timedatapath,ios::out|ios::app);
    ofs << time << " " << segment << " " << idx << endl;
    ofs.close();
}

int main(int argc, char** argv){
    ros::init(argc,argv, "send_goal");
    ros::NodeHandle nh;
    vector<ros::Publisher> pubs;
    vector<MoveBaseClient*> acs;
    for(int i = 0; i < robots; ++i){
        ros::Publisher pub = nh.advertise<geometry_msgs::Twist>('/' + robotNameSpace + '_' + char('0' + i) + "/cmd_vel", 1);
        pubs.emplace_back(pub);
        string acns = '/' + robotNameSpace + '_' + char('0' + i) + "/move_base";
        MoveBaseClient *ac = new MoveBaseClient(acns, true);
        acs.push_back(ac);
    }
    // ros::Publisher tb0_pub = nh.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel", 1);
    // ros::Publisher tb1_pub = nh.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel", 1);
    // ros::Publisher tb2_pub = nh.advertise<geometry_msgs::Twist>("/tb3_2/cmd_vel", 1);
    // ros::Publisher tb3_pub = nh.advertise<geometry_msgs::Twist>("/tb3_3/cmd_vel", 1);
    tf::TransformListener listener;
    // tf::StampedTransform transform0, transform1, transform2, transform3;
    vector<tf::StampedTransform> transforms(robots);
    ros::Duration(1.0).sleep();
    void *shared_memory = (void*)0;
    struct MultiNavUtil::runstate *rs;
    int shmid;
    srand((unsigned int)getpid());
    shmid = shmget((key_t)1234,sizeof(struct MultiNavUtil::runstate),0666 | IPC_CREAT);
    if(shmid == -1){
        std::cout << "shmget failed!\n";
        exit(EXIT_FAILURE);
    }
    shared_memory = shmat(shmid,(void *)0,0);
    rs = (struct MultiNavUtil::runstate*)shared_memory;
    rs->isStart = false;
    rs->isEnd = false;
    rs->waypoint = 0;
    rs->idx = -2;

    try {
        for(int i = 0; i < robots; ++i){
            listener.lookupTransform("/map", robotNameSpace + '_' + char('0' + i) + "/base_footprint", ros::Time(0), transforms[i]);
        }
        // listener.lookupTransform("/map", "tb3_0/base_footprint", ros::Time(0), transform0);
        // listener.lookupTransform("/map", "tb3_1/base_footprint", ros::Time(0), transform1);
        // listener.lookupTransform("/map", "tb3_2/base_footprint", ros::Time(0), transform2);
        // listener.lookupTransform("/map", "tb3_3/base_footprint", ros::Time(0), transform3);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    // MoveBaseClient ac0("/tb3_0/move_base", true);
    // MoveBaseClient ac1("/tb3_1/move_base", true);
    // MoveBaseClient ac2("/tb3_2/move_base", true);
    // MoveBaseClient ac3("/tb3_3/move_base", true);
    while (!acs[0]->waitForServer(ros::Duration(5.0)) && !acs[0]->waitForServer(ros::Duration(5.0)) && !acs[0]->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the move_base action server to bring up");
    vector<move_base_msgs::MoveBaseGoal> goals;
    vector<move_base_msgs::MoveBaseGoal> homes;
    move_base_msgs::MoveBaseGoal goal;
    for(int i = 0; i < robots; ++i){
        move_base_msgs::MoveBaseGoal home;
        MultiNavUtil::setPose(home, transforms[i]);
        homes.emplace_back(home);
    }
    // MultiNavUtil::setPose(home0, transform0);
    // MultiNavUtil::setPose(home1, transform1);
    // MultiNavUtil::setPose(home2, transform2);
    // MultiNavUtil::setPose(home3, transform3);
    ReadGoal(goals_data);
    for (int i = 0; i < 4 * cnt; i += 4){
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = goals_data[i];
        goal.target_pose.pose.position.y = goals_data[i + 1];
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = goals_data[i + 3];
        goal.target_pose.pose.orientation.w = goals_data[i + 2];
        goals.push_back(goal);
    }
    vector<bool> succeeds;
    vector<ros::Time> starts;
    vector<ros::Time> ends;
    vector<double> times;
    for(int i = 0; i < robots; ++i){
        succeeds.emplace_back(true);
        starts.emplace_back(ros::Time(0));
        ends.emplace_back(ros::Time(0));
        times.emplace_back(0.0);
    }
    // bool succeed0(true),succeed1(true),succeed2(true), succeed3(true);
    // ros::Time start0(0),start1(0),start2(0),start3(0),end0(0),end1(0),end2(0),end3(0);
    // double time0(0.0),time1(0.0),time2(0.0),time3(0.0);

    for(int robot = 0; robot < robots; ++robot){
        signal(SIGINT, DoShutdown);
        if(succeeds[robot]){
            for (int i = 0; i < 7; ++i) {
                cout << goals[i].target_pose.pose.position.x <<" " <<goals[i].target_pose.pose.position.y<<endl;
                if(i == 0){
                    cout << "Robot " << robot << " go to the start point" <<endl;
                    acs[robot]->sendGoal(goals[i]);
                    acs[robot]->waitForResult();
                    if(acs[robot]->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                        succeeds[robot] = false;
                        break;
                    }
                    ros::Duration(2.0).sleep();
                    continue;
                }else if(i == 6){
                    cout << "Robot "<< robot <<" Go to the home point" <<endl;
                    acs[robot]->sendGoal(homes[robot]);
                    acs[robot]->waitForResult();
                    if(acs[robot]->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                        succeeds[robot] = false;
                        break;
                    }
                    ros::Duration(2.0).sleep();
                    continue;
                }
                cout << "Robot "<< robot <<" go to the "<< i <<" point" <<endl;
                MultiNavUtil::setRunstate(rs,true,false,i,robot);
                // if(i == 6) ac0.sendGoal(home0);
                // else ac0.sendGoal(goals[i]);
                acs[robot]->sendGoal(goals[i]);
                starts[robot] = ros::Time::now();
                acs[robot]->waitForResult();
                if(acs[robot]->getState()==actionlib::SimpleClientGoalState::SUCCEEDED){
                    // cout << "Robot0 has reached the "<< i+1 <<" point" << endl;
                    ends[robot] = ros::Time::now();
                    times[robot] = (ends[robot] - starts[robot]).toSec();
                    ROS_INFO("Robot %d use %f seconds to reach the %d point", robot, times[robot],i);
                    MultiNavUtil::setRunstate(rs,false,true,i,robot);
                    saveTime(times[robot],i,robot);
                    ros::Duration(2.0).sleep();
                }else{
                    succeeds[robot] = false;
                    break;
                }
            }
        } else {
            succeeds[robot] = false;
            cout << "Robot"<<robot<<" failed to reach the waypoints!" << endl;
            break;
        }
        MultiNavUtil::stopRobot(pubs[robot]);
    }
    cout << "Mission Complete!" << endl;
    // delete shared memory
    shmdt(shared_memory);
    shmctl(shmid, IPC_RMID, 0);
    return 0;
}
