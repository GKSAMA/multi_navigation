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
#define timedatapath "/home/gk/Documents/DataRecord/segtime.data"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
vector<double> goals_data;
int cnt;    // the number of points

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
    ros::Publisher tb0_pub = nh.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel", 1);
    ros::Publisher tb1_pub = nh.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel", 1);
    ros::Publisher tb2_pub = nh.advertise<geometry_msgs::Twist>("/tb3_2/cmd_vel", 1);
    ros::Publisher tb3_pub = nh.advertise<geometry_msgs::Twist>("/tb3_3/cmd_vel", 1);
    // ros::Publisher runstate_pub = nh.advertise<multi_turtlebot3_navigation::RunState>("runstate",1);
    tf::TransformListener listener;
    tf::StampedTransform transform0, transform1, transform2, transform3;
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
        listener.lookupTransform("/map", "tb3_0/base_footprint", ros::Time(0), transform0);
        listener.lookupTransform("/map", "tb3_1/base_footprint", ros::Time(0), transform1);
        listener.lookupTransform("/map", "tb3_2/base_footprint", ros::Time(0), transform2);
        listener.lookupTransform("/map", "tb3_3/base_footprint", ros::Time(0), transform3);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    MoveBaseClient ac0("/tb3_0/move_base", true);
    MoveBaseClient ac1("/tb3_1/move_base", true);
    MoveBaseClient ac2("/tb3_2/move_base", true);
    MoveBaseClient ac3("/tb3_3/move_base", true);
    while (!ac0.waitForServer(ros::Duration(5.0)) && !ac0.waitForServer(ros::Duration(5.0)) && !ac0.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the move_base action server to bring up");
    vector<move_base_msgs::MoveBaseGoal> goals;
    move_base_msgs::MoveBaseGoal goal, home0, home1, home2, home3;
    MultiNavUtil::setPose(home0, transform0);
    MultiNavUtil::setPose(home1, transform1);
    MultiNavUtil::setPose(home2, transform2);
    MultiNavUtil::setPose(home3, transform3);
    ReadGoal(goals_data);
    for (int i = 0; i < 4 * cnt; i += cnt){
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
    bool succeed0(true),succeed1(true),succeed2(true), succeed3(true);
    ros::Time start0(0),start1(0),start2(0),start3(0),end0(0),end1(0),end2(0),end3(0);
    double time0(0.0),time1(0.0),time2(0.0),time3(0.0);
    // multi_turtlebot3_navigation::RunState runstate;
    while (true){
        signal(SIGINT,DoShutdown);
        
        for (int i = 0; i < 5; ++i) {
            cout << "Robot 0 Go to the "<< i+1 <<" point" <<endl;
            MultiNavUtil::setRunstate(rs,true,false,i+1,0);
            if(i != 4) ac0.sendGoal(goals[i]);
            else ac0.sendGoal(home0);
            start0 = ros::Time::now();
            ac0.waitForResult();
            if(ac0.getState()==actionlib::SimpleClientGoalState::SUCCEEDED){
                // cout << "Robot0 has reached the "<< i+1 <<" point" << endl;
                end0 = ros::Time::now();
                time0 = (end0 - start0).toSec();
                ROS_INFO("Robot 0 use %f seconds to reach the %d point", time0,i+1);
                MultiNavUtil::setRunstate(rs,false,true,i+1,0);
                saveTime(time0,i+1,0);
                ros::Duration(2.0).sleep();
            }else{
                succeed0 = false;
                break;
            }
        }
        if(succeed0){
            // stop robot0 and calculate the distance of its trajectory
            MultiNavUtil::stopRobot(tb0_pub);

            // start robot1 navigation
            
            for (int i = 0; i < 5; ++i) {
                cout << "Robot 1 Go to the "<< i+1 <<" point" <<endl;
                MultiNavUtil::setRunstate(rs,true,false,i+1,1);
                if(i != 4) ac1.sendGoal(goals[i]);
                else ac1.sendGoal(home1);
                start1 = ros::Time::now();
                ac1.waitForResult();
                if(ac1.getState()==actionlib::SimpleClientGoalState::SUCCEEDED){
                    // cout << "Robot1 has reached the "<< i+1 <<" point" << endl;
                    end1 = ros::Time::now();
                    time1 = (end1 - start1).toSec();
                    ROS_INFO("Robot 1 use %f seconds to reach the %d point", time1,i+1);
                    MultiNavUtil::setRunstate(rs,false,true,i+1,1);
                    saveTime(time1,i+1,1);
                    ros::Duration(2.0).sleep();
                }else{
                    succeed1 = false;
                    break;
                }
            }
        }else{
            cout << "Robot0 failed to reach the waypoints!" << endl;
            break;
        }
        if(succeed1){
            MultiNavUtil::stopRobot(tb1_pub);
            // start robot2 navigation 
            for (int i = 0; i < 5; ++i) {
                cout << "Robot 2 Go to the "<< i+1 <<" point" <<endl;
                MultiNavUtil::setRunstate(rs,true,false,i+1,2);
                if(i != 4) ac2.sendGoal(goals[i]);
                else ac2.sendGoal(home2);
                start2 = ros::Time::now();
                ac2.waitForResult();
                if(ac2.getState()==actionlib::SimpleClientGoalState::SUCCEEDED){
                    // cout << "Robot2 has reached the "<< i+1 <<" point" << endl;
                    end2 = ros::Time::now();
                    time2 = (end2 - start2).toSec();
                    ROS_INFO("Robot 2 use %f seconds to reach the %d point", time2,i+1);
                    MultiNavUtil::setRunstate(rs,false,true,i+1,2);
                    saveTime(time2,i+1,2);
                    ros::Duration(2.0).sleep();
                }else{
                    succeed2 = false;
                    break;
                }
            }
        }else {
            cout << "Robot1 failed to reach the waypoints!" << endl;
            break;
        }
        if (succeed2){
            MultiNavUtil::stopRobot(tb2_pub);
            // start robot3 Navigation
            for (int i = 0; i < 5; ++i){
                cout << "Robot 3 Go to the " << i + 1 << " point" <<endl;
                MultiNavUtil::setRunstate(rs, true, false, i + 1, 3);
                if (i != 4) ac3.sendGoal(goals[i]);
                else ac3.sendGoal(home3);
                start3 = ros::Time::now();
                ac3.waitForResult();
                if (ac3.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    end3 = ros::Time::now();
                    time3 = (end3 - start3).toSec();
                    ROS_INFO("Robot 3 use %f seconds to reach the %d point", time3, i + 1);
                    MultiNavUtil::setRunstate(rs, false, true, i + 1, 3);
                    saveTime(time3, i + 1, 3);
                    ros::Duration(2.0).sleep();
                }else{
                    succeed3 = false;
                    break;
                }
            }
        } else{
            cout << "Robot2 failed to reach the waypoints!" << endl;
            break;
        }
        if(!succeed3){
            cout << "Robot2 failed to reach the waypoints!" << endl;
            break;
        }
        MultiNavUtil::stopRobot(tb3_pub);
        cout << "Mission Complete!" << endl;
        // delete shared memory
        shmdt(shared_memory);
        shmctl(shmid, IPC_RMID, 0);
        break;
    }
    return 0;
}
