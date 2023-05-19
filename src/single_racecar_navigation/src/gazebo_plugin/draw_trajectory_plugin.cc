/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-11-24 22:09:46
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2022-12-03 17:19:25
 * @FilePath: /src/single_racecar_navigation/src/gazebo_plugin/draw_trajectory_plugin.cc
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Visual.hh>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

namespace gazebo
{
    class DrawTrajectoryPlugin : public VisualPlugin
    {
        public:
        DrawTrajectoryPlugin() : VisualPlugin(){
            ros::NodeHandle nh;
            radius_pub = nh.advertise<geometry_msgs::Point>("/footprint/turn_radius", 1);
        }
        ~DrawTrajectoryPlugin()
        {
        }

        void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
        {
            visual = _visual;
            // auto plotElem = _sdf->GetElement("plot");
            // auto linkName = plotElem->Get<std::string>("link");
            // auto link = model->GetLink(linkName);
            // plot.link = link;
            if (_sdf->HasElement("position"))
            {
                position = _sdf->Get<ignition::math::Vector3d>("position");
            }
            else
            {
                position = ignition::math::Vector3d(0, 0, 0);
            }

            std::string material;
            if (_sdf->HasElement("material"))
            {
                material = _sdf->Get<std::string>("material");
            }
            
            if(_sdf->HasElement("id")){
                id = _sdf->Get<int>("id");
            }
            if(id == 1 && startPoint == ignition::math::Vector3d::Zero){
                startPoint = visual->WorldPose().Pos() + position;
            }
            // this->updateConnection = event::Events::ConnectPreRender(std::bind(&DrawTrajectoryPlugin::OnUpdate, this));
            this->updateConnection = event::Events::ConnectRender(std::bind(&DrawTrajectoryPlugin::OnUpdate, this));

            prevPoint = visual->WorldPose().Pos();
            markerMsg.set_ns("visual_draw_trajectory/");
            markerMsg.set_id(id);
            markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
            markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
            ignition::msgs::Material *matMsg = markerMsg.mutable_material();
            matMsg->mutable_script()->set_name(material);
            ignition::msgs::Time *timeMsg = markerMsg.mutable_lifetime();
            timeMsg->set_sec(500);
            timeMsg->set_nsec(0);
        }
        
        void OnUpdate()
        {
            ignition::math::Vector3d point = visual->WorldPose().Pos() + position;
            // ignition::math::Vector3d point = plot.link->WorldPose().Pos() + position;
            if (point != ignition::math::Vector3d(0, 0, 0) && point != prevPoint)
            {
                ignition::msgs::Set(markerMsg.add_point(), point);
                //ignition::msgs::Set(markerMsg.mutable_scale(), ignition::math::Vector3d(1, 1, 1));
                if (markerMsg.point_size() > step)
                {
                    node.Request("/marker", markerMsg);
                    // markerMsg.mutable_point()->DeleteSubrange(0, step);
                }
                prevPoint = point;
            } else if(id == 1){
                float r = startPoint.Distance(point) / 2.0;
                // std::cout << "r = " << r << std::endl;
                geometry_msgs::Point p;
                p.x = r;
                radius_pub.publish(p);
            }
        }

        private:
        rendering::VisualPtr visual;
        ignition::math::Vector3d position;
        event::ConnectionPtr updateConnection;
        ignition::transport::Node node;
        ignition::msgs::Marker markerMsg;
        ignition::math::Vector3d prevPoint;
        ignition::math::Vector3d startPoint;
        int step = 2;
        int id = 0;
        ros::Publisher radius_pub;
    };
    GZ_REGISTER_VISUAL_PLUGIN(DrawTrajectoryPlugin)
}