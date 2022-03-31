//
// Created by gk on 2022/2/22.
//

#include "eb_local_planner/eb_local_planner.h"
namespace eb_local_planner{
    EBPlanner::EBPlanner() :costmap_ros_(NULL),initialized_(false){}
    EBPlanner::EBPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros):costmap_ros_(NULL),initialized_(false) { initialize(name,costmap_ros); }
    EBPlanner::~EBPlanner() { delete world_model_; }

    void EBPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            world_model_ = new base_local_planner::CostmapModel(*costmap_);
            footprint_spec_ = costmap_ros_->getRobotFootprint();
            ros::NodeHandle pn("~/"+name);
            elastic_band_.clear();
            initialized_ = true;
            visualization_ = false;
        }else{
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
    }
    void EBPlanner::reconfigure(EBPlannerConfig& config){
        min_bubble_overlap_ = config.eband_min_relative_overlap;
        tiny_bubble_distance_ = config.eband_tiny_bubble_distance;
        tiny_bubble_expansion_ = config.eband_tiny_bubble_expansion;
        internal_force_gain_ = config.eband_internal_force_gain;
        external_force_gain_ = config.eband_external_force_gain;
        num_optim_iterations_ = config.num_iterations_eband_optimization;
        max_recursion_depth_approx_equi_ = config.eband_equilibrium_approx_max_recursion_depth;
        equilibrium_relative_overshoot_ = config.eband_equilibrium_relative_overshoot;
        significant_force_ = config.eband_significant_force_lower_bound;
        costmap_weight_ = config.costmap_weight;
    }
    void EBPlanner::setVisualization(boost::shared_ptr<EBVisualization> eb_visual){
        eb_visual_ = eb_visual;
        visualization_ = true;
    }
    bool EBPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if (global_plan.size() < 2){
            ROS_ERROR("Attempt to pass empty path to optimization. Valid path needs to have at least 2 Frames. This one has %d.", ((int) global_plan.size()) );
            return false;
        }
        global_plan_ = global_plan;
        if (global_plan.front().header.frame_id != costmap_ros_->getGlobalFrameID()){
            ROS_ERROR("Elastic Band expects plan for optimization in the %s frame, the plan was sent in the %s frame.",
                      costmap_ros_->getGlobalFrameID().c_str(), global_plan.front().header.frame_id.c_str());
        }
        ROS_DEBUG("Converting Plan to Band");
        if (!convertPlanToBand(global_plan_,elastic_band_)){
            ROS_WARN("Conversion from plan to elastic band failed. Plan probably not collision free. Plan not set for optimization");
            // TODO try to do local repairs of band
            return false;
        }
        ROS_DEBUG("Refining Band");
        if (!refineBand(elastic_band_)){
            ROS_WARN("Band is broken. Could not close gaps in converted path. Path not set. Global replanning needed");
            return false;
        }
        ROS_DEBUG("Refinement done - Band set.");
        return true;
    }
    bool EBPlanner::getPlan(std::vector<geometry_msgs::PoseStamped>& global_plan){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if (elastic_band_.empty()){
            ROS_WARN("Band is empty. There was no path successfully set so far.");
            return false;
        }
        if (!convertBandToPlan(global_plan,elastic_band_)){
            ROS_WARN("Conversion from Elastic Band to path failed.");
            return false;
        }
        return true;
    }
    bool EBPlanner::getBand(std::vector<Bubble>& elastic_band){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        elastic_band = elastic_band_;
        if (elastic_band_.empty()){
            ROS_WARN("Band is empty");
            return false;
        }
        return true;
    }
    bool EBPlanner::addFrames(const std::vector<geometry_msgs::PoseStamped>& plan_to_add, const AddAtPosition& add_frames_at){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if (elastic_band_.size()<1){
            ROS_WARN("Attempt to connect path to empty band. path not connected. Use SetPath instead");
            return false;
        }
        if (plan_to_add.empty()){
            ROS_WARN("Attempt to connect empty path to band. Nothing to do here.");
            return false;
        }
        if (plan_to_add.at(0).header.frame_id != costmap_ros_->getGlobalFrameID()){
            ROS_ERROR("Elastic Band expects robot pose for optimization in the %s frame, the pose was sent in the %s frame.",
                      costmap_ros_->getGlobalFrameID().c_str(), plan_to_add.at(0).header.frame_id.c_str());
            return false;
        }

        std::vector<Bubble> band_to_add;
        if (!convertPlanToBand(plan_to_add,band_to_add)){
            ROS_DEBUG("Conversion from plan to elastic band failed. Plan not appended");
            // TODO try to do local repairs of band
            return false;
        }
        ROS_DEBUG("Checking for connections between current band and new bubbles");
        bool connected = false;
        int bubble_connect = -1;
        if(add_frames_at == add_front){
            for (int i = ((int)elastic_band_.size()-1); i >= 0 ; i--) {
                if (checkOverlap(band_to_add.back(),elastic_band_.at(i))){
                    bubble_connect = i;
                    connected = true;
                    break;
                }
            }
        }else{
            for (int i = 0; i < ((int) elastic_band_.size() - 1); ++i) {
                if (checkOverlap(band_to_add.front(),elastic_band_.at(i))){
                    bubble_connect = i;
                    connected = true;
                    break;
                }
            }
        }
        std::vector<Bubble> tmp_band;
        std::vector<Bubble>::iterator tmp_iter1,tmp_iter2;
        tmp_band.assign(band_to_add.begin(),band_to_add.end());

        if(connected){
            ROS_DEBUG("Connections found - composing new band by connecting new frames to bubble %d", bubble_connect);
            if (add_frames_at == add_front){
                tmp_iter1 = elastic_band_.begin() + bubble_connect;
                ROS_ASSERT( (tmp_iter1 >= elastic_band_.begin()) && (tmp_iter1 < elastic_band_.end()) );
                tmp_band.insert(tmp_band.end(),tmp_iter1,elastic_band_.end());
            }else{
                tmp_iter1 = elastic_band_.begin() + bubble_connect + 1;
                ROS_ASSERT( (tmp_iter1 > elastic_band_.begin()) && (tmp_iter1 <= elastic_band_.end()) );
                tmp_band.insert(tmp_band.begin(), elastic_band_.begin(), tmp_iter1);
            }
            elastic_band_ = tmp_band;
            return true;
        }

        ROS_DEBUG("No direct connection found - Composing tmp band and trying to fill gap");
        if (add_frames_at == add_front){
            tmp_band.insert(tmp_band.end(),elastic_band_.begin(),elastic_band_.end());
            tmp_iter1 = tmp_band.begin() + ((int)band_to_add.size() - 1);
            tmp_iter2 = tmp_iter1 + 1;
        }else{
            tmp_band.insert(tmp_band.end(),elastic_band_.begin(),elastic_band_.end());
            tmp_iter1 = tmp_band.begin() + ((int)elastic_band_.size() - 1);
            tmp_iter2 = tmp_iter1 + 1;
        }
        ROS_ASSERT( tmp_iter1 >= tmp_band.begin() );
        ROS_ASSERT( tmp_iter2 < tmp_band.end() );
        ROS_ASSERT( tmp_iter1 < tmp_iter2 );
        if(!fillGap(tmp_band, tmp_iter1, tmp_iter2)){
            ROS_DEBUG("Could not connect robot pose to band - Failed to fill gap.");
            return false;
        }
        elastic_band_ = tmp_band;

        return true;
    }
    bool EBPlanner::optimizeBand(){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if(elastic_band_.empty()){
            ROS_ERROR("Band is empty. Probably Band has not been set yet");
            return false;
        }
        ROS_DEBUG("Starting optimization of band");
        if(!optimizeBand(elastic_band_)){
            ROS_DEBUG("Aborting Optimization. Changes discarded.");
            return false;
        }
        ROS_DEBUG("Elastic Band - Optimization successfull!");
        return true;
    }
    bool EBPlanner::optimizeBand(std::vector<Bubble>& band){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if (band.front().center.header.frame_id != costmap_ros_->getGlobalFrameID()){
            ROS_ERROR("Elastic Band expects plan for optimization in the %s frame, the plan was sent in the %s frame.",
                      costmap_ros_->getGlobalFrameID().c_str(), band.front().center.header.frame_id.c_str());
            return false;
        }

        double distance;
        for (int i = 0; i < ((int) band.size()); ++i) {
            if (!calcObstacleKinematicDistance(band.at(i).center.pose,distance)){
                ROS_DEBUG("Optimization (Elastic Band) - Calculation of Distance failed. Frame %d of %d Probably outside map coordinates.",
                          i, ((int) band.size()) );
                return false;
            }
            if (distance == 0.0){
                ROS_DEBUG("Optimization (Elastic Band) - Calculation of Distance failed. Frame %d of %d in collision. Plan invalid. Trying to refine band.",
                          i, ((int) band.size()) );
                // TODO if frame in collision try to repair band instead of aborting everything
                return false;
            }
            band.at(i).expansion = distance;
        }

        if (!refineBand(band)){
            ROS_DEBUG("Elastic Band is broken. Could not close gaps in band. Global replanning needed.");
            return false;
        }
        std::vector<Bubble> tmp_band = band;
        for (int i = 0; i < num_optim_iterations_; ++i) {
            ROS_DEBUG("Inside optimization: Cycle no. %d", i);
            if (!modifyBandArtificialForce(tmp_band)){
                ROS_DEBUG("Optimization failed while trying to modify Band.");
                return false;
            }
            if (!refineBand(tmp_band)){
                ROS_DEBUG("Optimization failed while trying to refine modified band");
                return false;
            }
        }
        band = tmp_band;
        return true;
    }

    bool EBPlanner::refineBand(std::vector<Bubble>& band){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if(band.size() < 2){
            ROS_WARN("Attempt to convert empty band to plan. Valid band needs to have at least 2 Frames. This one has %d.", ((int) band.size()) );
            return false;
        }
        bool success;
        std::vector<Bubble> tmp_band;
        std::vector<Bubble>::iterator start_iter,end_iter;
        tmp_band = band;
        start_iter = tmp_band.begin();
        end_iter = tmp_band.end() - 1;
        success = removeAndFill(tmp_band,start_iter,end_iter);
        if(!success)
            ROS_DEBUG("Band is broken. Could not close gaps.");
        else{
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Recursive filling and removing DONE");
#endif
            band = tmp_band;
        }
        return success;
    }
    bool EBPlanner::removeAndFill(std::vector<Bubble>& band, std::vector<Bubble>::iterator& start_iter,std::vector<Bubble>::iterator& end_iter){
        bool overlap;
        std::vector<Bubble>::iterator tmp_iter;
        int mid_int, diff_int;
#ifdef DEBUG_EBAND_
        int debug_dist_start,debug_dist_iters;
        debug_dist_start = std::distance(band.begin(),start_iter);
        debug_dist_iters = std::distance(start_iter,end_iter);
        ROS_DEBUG("Refining Recursive - Check if Bubbles %d and %d overlapp. Total size of band %d.", debug_dist_start, (debug_dist_start + debug_dist_iters), ((int) band.size()) );
#endif
        ROS_ASSERT( start_iter >= band.begin() );
        ROS_ASSERT( end_iter < band.end() );
        ROS_ASSERT( start_iter < end_iter );
        overlap = checkOverlap(*start_iter,*end_iter);
        if (overlap){
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Refining Recursive - Bubbles overlapp, check for redundancies");
#endif
            if ((start_iter+1) < end_iter){
#ifdef DEBUG_EBAND_
                ROS_DEBUG("Refining Recursive - Bubbles overlapp, removing Bubbles %d to %d.", (debug_dist_start + 1), (debug_dist_start + debug_dist_iters -1));
#endif
                tmp_iter = band.erase((start_iter+1),end_iter);

                end_iter = tmp_iter;
            }
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Refining Recursive - Bubbles overlapp - DONE");
#endif
            return true;
        }
        if ((start_iter + 1) < end_iter){
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Refining Recursive - Bubbles do not overlapp, go one recursion deeper");
#endif
            mid_int = std::distance(start_iter,end_iter);
            mid_int = mid_int/2;
            tmp_iter = start_iter + mid_int;
            diff_int = (int)std::distance(tmp_iter,end_iter);
            ROS_ASSERT( start_iter >= band.begin() );
            ROS_ASSERT( end_iter < band.end() );
            ROS_ASSERT( start_iter < end_iter );
            if (!removeAndFill(band,start_iter,tmp_iter)){
                return false;
            }
            end_iter = tmp_iter + diff_int;
            ROS_ASSERT( start_iter >= band.begin() );
            ROS_ASSERT( end_iter < band.end() );
            ROS_ASSERT( (start_iter < tmp_iter) && (tmp_iter < end_iter) );

            diff_int = (int)std::distance(start_iter,tmp_iter);
            if (!removeAndFill(band,tmp_iter,end_iter)){
                return false;
            }
            start_iter = tmp_iter - diff_int;
            ROS_ASSERT( start_iter >= band.begin() );
            ROS_ASSERT( end_iter < band.end() );
            ROS_ASSERT( (start_iter < tmp_iter) && (tmp_iter < end_iter) );

            if (checkOverlap(*(tmp_iter-1),*(tmp_iter+1))){
#ifdef DEBUG_EBAND_
                ROS_DEBUG("Refining Recursive - Removing middle bubble");
#endif
                diff_int = (int)std::distance((tmp_iter+1),end_iter);
                tmp_iter = band.erase(tmp_iter);
                end_iter = tmp_iter + diff_int;
            }
            ROS_ASSERT( start_iter >= band.begin() );
            ROS_ASSERT( end_iter < band.end() );
            ROS_ASSERT( (start_iter < tmp_iter) && (tmp_iter < end_iter) );
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Refining Recursive - Bubbles do not overlapp, go one recursion deeper DONE");
#endif
            return true;
        }
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Refining Recursive - Gap detected, fill recursive");
#endif
        if (!fillGap(band,start_iter,end_iter)){
            ROS_DEBUG("Failed to fill gap between bubble %d and %d.", (int) distance(band.begin(), start_iter), (int) distance(band.begin(), end_iter) );
            return false;
        }
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Refining Recursive - Gap detected, fill recursive DONE");
#endif
        return true;
    }
    bool EBPlanner::fillGap(std::vector<Bubble>& band, std::vector<Bubble>::iterator& start_iter,std::vector<Bubble>::iterator& end_iter){
        double distance = 0.0;
        Bubble interpolated_bubble;
        geometry_msgs::PoseStamped interpolated_center;
        std::vector<Bubble>::iterator tmp_iter;
        int diff_int,start_num,end_num;

        ROS_ASSERT( start_iter >= band.begin() );
        ROS_ASSERT( end_iter < band.end() );
        ROS_ASSERT( start_iter < end_iter );
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - interpolate");
#endif
        if (!interpolateBubbles(start_iter->center,end_iter->center,interpolated_center)){
            start_num = std::distance(band.begin(),start_iter);
            end_num = std::distance(band.begin(),end_iter);
            ROS_DEBUG("Interpolation failed while trying to fill gap between bubble %d and %d.", start_num, end_num);
            return false;
        }
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - calc expansion of interpolated bubble");
#endif
        if (!calcObstacleKinematicDistance(interpolated_center.pose,distance)){
            start_num = std::distance(band.begin(),start_iter);
            end_num = std::distance(band.begin(),end_iter);
            ROS_DEBUG("Calculation of Distance failed for interpolated bubble - failed to fill gap between bubble %d and %d.", start_num, end_num);
            return false;
        }
        if (distance <= tiny_bubble_expansion_){
            start_num = std::distance(band.begin(),start_iter);
            end_num = std::distance(band.begin(),end_iter);
            ROS_DEBUG("Interpolated Bubble in Collision - failed to fill gap between bubble %d and %d.", start_num, end_num);
            // TODO this means only that there is an obstacle on the direct interconnection between the bubbles - think about repair or rescue strategies -
            return false;
        }
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - inserting interpolated bubble at (%f, %f), with expansion %f", interpolated_center.pose.position.x, interpolated_center.pose.position.y, distance);
#endif
        interpolated_bubble.center = interpolated_center;
        interpolated_bubble.expansion = distance;
        tmp_iter = band.insert(end_iter,interpolated_bubble);
        start_iter = tmp_iter - 1;
        end_iter = tmp_iter + 1;
        ROS_ASSERT( start_iter >= band.begin() );
        ROS_ASSERT( end_iter < band.end() );
        ROS_ASSERT( (start_iter < tmp_iter) && (tmp_iter < end_iter) );
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - check overlap interpolated bubble and first bubble");
#endif
        if (!checkOverlap(*start_iter,*tmp_iter)){
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Fill recursive - gap btw. interpolated and first bubble - fill recursive");
#endif
            if (!fillGap(band,start_iter,tmp_iter)){
                return false;
            }
            end_iter = tmp_iter + 1;
        }
        ROS_ASSERT( start_iter >= band.begin() );
        ROS_ASSERT( end_iter < band.end() );
        ROS_ASSERT( (start_iter < tmp_iter) && (tmp_iter < end_iter) );
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - check overlap interpolated bubble and second bubble");
#endif
        if (!checkOverlap(*tmp_iter,*end_iter)){
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Fill recursive - gap btw. interpolated and second bubble - fill recursive");
#endif
            diff_int = (int)std::distance(start_iter,tmp_iter);
            if (!fillGap(band,tmp_iter,end_iter)){
                return false;
            }
            start_iter = tmp_iter - diff_int;
        }
        ROS_ASSERT( start_iter >= band.begin() );
        ROS_ASSERT( end_iter < band.end() );
        ROS_ASSERT( (start_iter < tmp_iter) && (tmp_iter < end_iter) );
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - gap closed");
#endif
        return true;
    }
    bool EBPlanner::modifyBandArtificialForce(std::vector<Bubble>& band){
        if (band.empty()){
            ROS_ERROR("Trying to modify an empty band.");
            return false;
        }
        if (band.size() <= 2){
            return true;
        }
        std::vector<geometry_msgs::WrenchStamped> internal_forces,external_forces, forces;
        geometry_msgs::WrenchStamped wrench;
#ifdef DEBUG_EBAND_
        //publish original band
        if(visualization_)
            eband_visual_->publishBand("bubbles", band);
#endif
        wrench.header.stamp = ros::Time::now();
        wrench.header.frame_id = band[0].center.header.frame_id;
        wrench.wrench.force.x = 0.0;
        wrench.wrench.force.y = 0.0;
        wrench.wrench.force.z = 0.0;
        wrench.wrench.torque.x = 0.0;
        wrench.wrench.torque.y = 0.0;
        wrench.wrench.torque.z = 0.0;
        internal_forces.assign(band.size(),wrench);
        external_forces = internal_forces;
        forces = internal_forces;
        // TODO log timings of planner
        // instantiate variables for timing
        //ros::Time time_stamp1, time_stamp2;
        //ros::Duration duration;
        //time_stamp1 = ros::Time::now();
        int i = 1;
        bool forward = true;
        while((i > 0) && (i < (int)band.size() - 1)){
            ROS_DEBUG("Modifying bubble %d.", i);
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Calculating internal force for bubble %d.", i);
#endif
            if (!calcInternalForces(i,band,band.at(i),internal_forces.at(i))){
                ROS_DEBUG("Calculation of internal forces failed");
                return false;
            }
#ifdef DEBUG_EBAND_
            if(visualization_)
            // publish internal forces
            eband_visual_->publishForce("internal_forces", i, eband_visual_->blue, internal_forces[i], band[i]);
            // Log out debug info about next step
            ROS_DEBUG("Calculating external force for bubble %d.", i);
#endif
            if (!calcExternalForces(i,band.at(i),external_forces.at(i))){
                ROS_DEBUG("Calculation of external forces failed");
                return false;
            }
#ifdef DEBUG_EBAND_
            if(visualization_)
            //publish external forces
            eband_visual_->publishForce("external_forces", i, eband_visual_->red, external_forces[i], band[i]);
            // Log out debug info about next step
            ROS_DEBUG("Superposing internal and external forces");
#endif
            forces.at(i).wrench.force.x = internal_forces.at(i).wrench.force.x + external_forces.at(i).wrench.force.x;
            forces.at(i).wrench.force.y = internal_forces.at(i).wrench.force.y + external_forces.at(i).wrench.force.y;
            forces.at(i).wrench.force.z = internal_forces.at(i).wrench.force.z + external_forces.at(i).wrench.force.z;

            forces.at(i).wrench.torque.x = internal_forces.at(i).wrench.torque.x + external_forces.at(i).wrench.torque.x;
            forces.at(i).wrench.torque.y = internal_forces.at(i).wrench.torque.y + external_forces.at(i).wrench.torque.y;
            forces.at(i).wrench.torque.z = internal_forces.at(i).wrench.torque.z + external_forces.at(i).wrench.torque.z;
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Superpose forces: (x, y, theta) = (%f, %f, %f)", forces.at(i).wrench.force.x, forces.at(i).wrench.force.y, forces.at(i).wrench.torque.z);
            ROS_DEBUG("Supressing tangential forces");
#endif
            if (!suppressTangentialForces(i,band,forces.at(i))){
                ROS_DEBUG("Supression of tangential forces failed");
                return false;
            }
#ifdef DEBUG_EBAND_
            if(visualization_)
            //publish resulting forces
            eband_visual_->publishForce("resulting_forces", i, eband_visual_->green, forces[i], band[i]);
#endif
            ROS_DEBUG("Applying forces to modify band");
            if (!applyForces(i,band,forces)){
                ROS_DEBUG("Band is invalid - Stopping Modification");
                return false;
            }
#ifdef DEBUG_EBAND_
            if(visualization_){
            // publish band with changed bubble at resulting position
            eband_visual_->publishBand("bubbles", band);
            ros::Duration(0.01).sleep();
          }
#endif
            if (forward){
                i++;
                if(i == ((int)band.size() - 1)){
                    forward = false;
                    i--;
                    ROS_DEBUG("Optimization Elastic Band - Forward cycle done, starting backward cycle");
                }
            }else{
                i--;
            }
        }
        return true;
    }
    bool EBPlanner::applyForces(int bubble_num, std::vector<Bubble>& band, std::vector<geometry_msgs::WrenchStamped> forces){
        if(band.size() <= 2){
            return true;
        }
        geometry_msgs::Pose2D bubble_pose2D,new_bubble_pose2D;
        geometry_msgs::Pose bubble_pose,new_bubble_pose;
        geometry_msgs::Twist bubble_jump;
        Bubble new_bubble = band.at(bubble_num);
        double distance;

        bubble_pose = band.at(bubble_num).center.pose;
        PoseToPose2D(bubble_pose,bubble_pose2D);
        bubble_jump.linear.x = band.at(bubble_num).expansion * forces.at(bubble_num).wrench.force.x;
        bubble_jump.linear.y = band.at(bubble_num).expansion * forces.at(bubble_num).wrench.force.y;
        bubble_jump.linear.z = 0.0;
        bubble_jump.angular.x = 0.0;
        bubble_jump.angular.y = 0.0;
        bubble_jump.angular.z = band.at(bubble_num).expansion/ getCircumscribedRadius(*costmap_ros_)*forces.at(bubble_num).wrench.torque.z;
        bubble_jump.angular.z = angles::normalize_angle(bubble_jump.angular.z);
        new_bubble_pose2D.x = bubble_pose2D.x + bubble_jump.linear.x;
        new_bubble_pose2D.y = bubble_pose2D.y + bubble_jump.linear.y;
        new_bubble_pose2D.theta = bubble_pose2D.theta + bubble_jump.angular.z;
        new_bubble_pose2D.theta = angles::normalize_angle(new_bubble_pose2D.theta);
        Pose2DToPose(new_bubble_pose2D,new_bubble_pose);
        new_bubble.center.pose = new_bubble_pose;
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Try moving bubble %d at (%f, %f, %f) by (%f, %f, %f).", bubble_num, bubble_pose2D.x, bubble_pose2D.y, bubble_pose2D.theta,
        bubble_jump.linear.x, bubble_jump.linear.y, bubble_jump.angular.z);
#endif
        if (!calcObstacleKinematicDistance(new_bubble_pose,distance)){
            ROS_DEBUG("Calculation of Distance failed. Frame %d of %d Probably outside map. Discarding Changes", bubble_num, ((int) band.size()) );

#ifdef DEBUG_EBAND_
            if(visualization_)
        eband_visual_->publishBubble("bubble_hypo", bubble_num, eband_visual_->red, new_bubble);
#endif
            return true;
        }
        if (distance <= tiny_bubble_expansion_){
            ROS_DEBUG("Calculation of Distance failed. Frame %d of %d in collision. Plan invalid. Discarding Changes", bubble_num, ((int) band.size()) );
#ifdef DEBUG_EBAND_
            if(visualization_)
                eband_visual_->publishBubble("bubble_hypo", bubble_num, eband_visual_->red, new_bubble);
#endif
            return true;
        }
        new_bubble.expansion = distance;
        geometry_msgs::WrenchStamped new_bubble_force = forces.at(bubble_num);
        if (!getForcesAt(bubble_num,band,new_bubble,new_bubble_force)){
            ROS_DEBUG("Cannot calculate forces on bubble %d at new position - discarding changes", bubble_num);
            return true;
        }
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Check for zero-crossings in force on bubble %d", bubble_num);
#endif
        double checksum_zero,abs_new_force,abs_old_force;
        checksum_zero = (new_bubble_force.wrench.force.x * forces.at(bubble_num).wrench.force.x)+
                (new_bubble_force.wrench.force.y * forces.at(bubble_num).wrench.force.y)+
                (new_bubble_force.wrench.torque.z * forces.at(bubble_num).wrench.torque.z);
        if(checksum_zero < 0.0){
            ROS_DEBUG("Detected zero-crossings in force on bubble %d. Checking total change in force.", bubble_num);
            abs_new_force = sqrt((new_bubble_force.wrench.force.x * new_bubble_force.wrench.force.x)+
                                 (new_bubble_force.wrench.force.y * new_bubble_force.wrench.force.y)+
                                 (new_bubble_force.wrench.torque.z * new_bubble_force.wrench.torque.z));
            abs_old_force = sqrt((forces.at(bubble_num).wrench.force.x * forces.at(bubble_num).wrench.force.x)+
                                (forces.at(bubble_num).wrench.force.y * forces.at(bubble_num).wrench.force.y)+
                                (forces.at(bubble_num).wrench.torque.z * forces.at(bubble_num).wrench.torque.z));
            if ((abs_new_force > equilibrium_relative_overshoot_ * abs_old_force) && (abs_new_force > significant_force_)){
                ROS_DEBUG("Detected significante change in force (%f to %f) on bubble %d. Entering Recursive Approximation.", abs_old_force, abs_new_force, bubble_num);
                int curr_recursion_depth = 0;
                geometry_msgs::Twist new_step_width;
                Bubble curr_bubble = band.at(bubble_num);
                geometry_msgs::WrenchStamped curr_bubble_force = forces.at(bubble_num);
                new_step_width.linear.x = 0.5*bubble_jump.linear.x;
                new_step_width.linear.y = 0.5*bubble_jump.linear.y;
                new_step_width.linear.z = 0.5*bubble_jump.linear.z;
                new_step_width.angular.x = 0.5*bubble_jump.angular.x;
                new_step_width.angular.y = 0.5*bubble_jump.angular.y;
                new_step_width.angular.z = 0.5*bubble_jump.angular.z;
                if (moveApproximateEquilibrium(bubble_num,band,curr_bubble,curr_bubble_force,new_step_width,curr_recursion_depth)){
                    new_bubble = curr_bubble;
#ifdef DEBUG_EBAND_
                    geometry_msgs::Pose2D curr_bubble_pose2D;
                    PoseToPose2D(curr_bubble.center.pose, curr_bubble_pose2D);
                    ROS_DEBUG("Instead - Try moving bubble %d at (%f, %f, %f) by (%f, %f, %f) to (%f, %f, %f).",
                                bubble_num, bubble_pose2D.x, bubble_pose2D.y, bubble_pose2D.theta,
                                new_step_width.linear.x, new_step_width.linear.y, new_step_width.angular.z,
                                curr_bubble_pose2D.x, curr_bubble_pose2D.y, curr_bubble_pose2D.theta);
#endif
                }
            }
        }
        // TODO use this routine not only to check whether gap can be filled but also to fill gap (if possible)
        std::vector<Bubble> tmp_band = band;
        std::vector<Bubble>::iterator start_iter,end_iter;
        tmp_band.at(bubble_num) = new_bubble;

        start_iter = tmp_band.begin();
        start_iter = start_iter + bubble_num -1;
        end_iter = start_iter + 1;
        if (!checkOverlap(*start_iter,*end_iter)){
            if (!fillGap(tmp_band,start_iter,end_iter)){
                ROS_DEBUG("Bubble at new position cannot be connected to neighbour. Discarding changes.");
                return true;
            }
        }
        tmp_band = band;
        tmp_band.at(bubble_num) = new_bubble;
        start_iter = tmp_band.begin();

        start_iter = start_iter + bubble_num;
        end_iter = start_iter + 1;
        if (!checkOverlap(*start_iter,*end_iter)){
            if (!fillGap(tmp_band,start_iter,end_iter)){
                ROS_DEBUG("Bubble at new position cannot be connected to neighbour. Discarding changes.");
                return true;
            }
        }
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Frame %d of %d: Check successful - bubble and band valid. Applying Changes", bubble_num, ((int) band.size()) );
#endif
        band.at(bubble_num) = new_bubble;
        return true;
    }
    bool EBPlanner::moveApproximateEquilibrium(const int& bubble_num,const std::vector<Bubble>& band, Bubble& curr_bubble,
                                    const geometry_msgs::WrenchStamped& curr_bubble_force, geometry_msgs::Twist& curr_step_width,
                                    const int& curr_recursion_depth){
        double distance;
        Bubble new_bubble = curr_bubble;
        geometry_msgs::Pose2D new_bubble_pose2D, curr_bubble_pose2D;
        geometry_msgs::WrenchStamped new_bubble_force = curr_bubble_force;

        PoseToPose2D(curr_bubble.center.pose, curr_bubble_pose2D);
        PoseToPose2D(new_bubble.center.pose, new_bubble_pose2D);
        new_bubble_pose2D.x = curr_bubble_pose2D.x + curr_step_width.linear.x;
        new_bubble_pose2D.y = curr_bubble_pose2D.y + curr_step_width.linear.y;
        new_bubble_pose2D.theta = curr_bubble_pose2D.theta + curr_step_width.angular.z;
        new_bubble_pose2D.theta = angles::normalize_angle(new_bubble_pose2D.theta);
        Pose2DToPose(new_bubble_pose2D,new_bubble.center.pose);
        if (!calcObstacleKinematicDistance(new_bubble.center.pose,distance)) return false;
        if (distance == 0.0) return false;
        new_bubble.expansion = distance;
        if (!getForcesAt(bubble_num,band,new_bubble,new_bubble_force)) return false;
        curr_bubble = new_bubble;
        if (curr_recursion_depth >= max_recursion_depth_approx_equi_) return true;
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Check for zero-crossings in force on bubble %d - Recursion %d", bubble_num, curr_recursion_depth);
#endif
        double checksum_zero,abs_new_force,abs_old_force;
        int new_recursion_depth;
        geometry_msgs::Twist new_step_width;
        checksum_zero = (new_bubble_force.wrench.force.x * curr_bubble_force.wrench.force.x)+
                        (new_bubble_force.wrench.force.y * curr_bubble_force.wrench.force.y)+
                        (new_bubble_force.wrench.force.z * curr_bubble_force.wrench.force.z);
        if(checksum_zero < 0.0){
            ROS_DEBUG("Detected zero-crossings in force on bubble %d. Checking total change in force.", bubble_num);
            abs_new_force = sqrt((new_bubble_force.wrench.force.x * new_bubble_force.wrench.force.x)+
                                 (new_bubble_force.wrench.force.y * new_bubble_force.wrench.force.y)+
                                 (new_bubble_force.wrench.force.z * new_bubble_force.wrench.force.z));
            abs_old_force = sqrt((curr_bubble_force.wrench.force.x * curr_bubble_force.wrench.force.x)+
                                 (curr_bubble_force.wrench.force.y * curr_bubble_force.wrench.force.y)+
                                 (curr_bubble_force.wrench.force.z * curr_bubble_force.wrench.force.z));
            if ((abs_new_force > equilibrium_relative_overshoot_ * abs_old_force) && (abs_new_force > significant_force_)){
#ifdef DEBUG_EBAND_
                ROS_DEBUG("Detected significant change in force (%f to %f) on bubble %d - Recursion %d. Going one Recursion deeper.", abs_old_force, abs_new_force, bubble_num, curr_recursion_depth);
#endif
                new_recursion_depth = curr_recursion_depth + 1;
                new_step_width.linear.x = -0.5*curr_step_width.linear.x;
                new_step_width.linear.y = -0.5*curr_step_width.linear.y;
                new_step_width.linear.z = -0.5*curr_step_width.linear.z;
                new_step_width.angular.x = -0.5*curr_step_width.angular.x;
                new_step_width.angular.y = -0.5*curr_step_width.angular.y;
                new_step_width.angular.z = -0.5*curr_step_width.angular.z;
                if (moveApproximateEquilibrium(bubble_num,band,new_bubble,new_bubble_force,new_step_width,new_recursion_depth)){
                    curr_bubble = new_bubble;
                }
            }else{
#ifdef DEBUG_EBAND_
                ROS_DEBUG("No zero-crossings in force on bubble %d - Recursion %d. Continue walk in same direction. Going one recursion deeper.", bubble_num, curr_recursion_depth);
#endif
                new_recursion_depth = curr_recursion_depth + 1;
                new_step_width.linear.x = 0.5*curr_step_width.linear.x;
                new_step_width.linear.y = 0.5*curr_step_width.linear.y;
                new_step_width.linear.z = 0.5*curr_step_width.linear.z;
                new_step_width.angular.x = 0.5*curr_step_width.angular.x;
                new_step_width.angular.y = 0.5*curr_step_width.angular.y;
                new_step_width.angular.z = 0.5*curr_step_width.angular.z;
                if (moveApproximateEquilibrium(bubble_num,band,new_bubble,new_bubble_force,new_step_width,new_recursion_depth)){
                    curr_bubble = new_bubble;
                }
            }
        }
        return true;
    }
    bool EBPlanner::interpolateBubbles(geometry_msgs::PoseStamped start_center, geometry_msgs::PoseStamped end_center,geometry_msgs::PoseStamped& interpolated_center){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        geometry_msgs::Pose2D start_pose2D,end_pose2D,interpolated_pose2D;
        double delta_theta;
        interpolated_center.header = start_center.header;
        // TODO make this in a better way
        // - for instance use "slerp" to interpolate directly between quaternions
        // - or work with pose2D right from the beginnning
        PoseToPose2D(start_center.pose,start_pose2D);
        PoseToPose2D(end_center.pose,end_pose2D);
        delta_theta = end_pose2D.theta - start_pose2D.theta;
        delta_theta = angles::normalize_angle(delta_theta) / 2.0;
        interpolated_pose2D.theta = angles::normalize_angle(start_pose2D.theta + delta_theta);
        interpolated_pose2D.x = 0.0;
        interpolated_pose2D.y = 0.0;
        Pose2DToPose(interpolated_pose2D,interpolated_center.pose);
        interpolated_center.pose.position.x = (start_center.pose.position.x + end_center.pose.position.x)/2.0;
        interpolated_center.pose.position.y = (start_center.pose.position.y + end_center.pose.position.y)/2.0;
        interpolated_center.pose.position.z = (start_center.pose.position.z + end_center.pose.position.z)/2.0;
        // TODO ideally this would take into account kinematics of the robot and for instance use splines

        return true;
    }
    bool EBPlanner::checkOverlap(Bubble bubble1,Bubble bubble2){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        double distance=0.0;
        if (!calcBubbleDistance(bubble1.center,bubble2.center,distance)){
            ROS_ERROR("failed to calculate Distance between two bubbles. Aborting check for overlap!");
            return false;
        }
        if (distance >= min_bubble_overlap_ * (bubble1.expansion + bubble2.expansion)) return false;
        // TODO this does not account for kinematic properties -> improve

        return true;
    }
    bool EBPlanner::calcBubbleDistance(geometry_msgs::PoseStamped start_center, geometry_msgs::PoseStamped end_center,double& distance){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        geometry_msgs::Pose2D start_pose2D,end_pose2D,diff_pose2D;
        // TODO make this in a better way
        // - or work with pose2D right from the beginnning
        PoseToPose2D(start_center.pose,start_pose2D);
        PoseToPose2D(end_center.pose,end_pose2D);
        diff_pose2D.theta = end_pose2D.theta - start_pose2D.theta;
        diff_pose2D.theta = angles::normalize_angle(diff_pose2D.theta);
        diff_pose2D.x = end_pose2D.x - start_pose2D.x;
        diff_pose2D.y = end_pose2D.y - start_pose2D.y;
        double angle_to_pseudo_vel = diff_pose2D.theta * getCircumscribedRadius(*costmap_ros_);
        distance = sqrt(diff_pose2D.x * diff_pose2D.x + diff_pose2D.y * diff_pose2D.y);
        // TODO take into account kinematic properties of body

        return true;
    }
    bool EBPlanner::calcBubbleDifference(geometry_msgs::PoseStamped start_center, geometry_msgs::PoseStamped end_center,geometry_msgs::Twist& difference){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        geometry_msgs::Pose2D start_pose2D,end_pose2D,diff_pose2D;
        // TODO make this in a better way
        // - or work with pose2D right from the beginnning
        PoseToPose2D(start_center.pose,start_pose2D);
        PoseToPose2D(end_center.pose,end_pose2D);
        diff_pose2D.theta = end_pose2D.theta - start_pose2D.theta;
        diff_pose2D.theta = angles::normalize_angle(diff_pose2D.theta);
        diff_pose2D.x = end_pose2D.x - start_pose2D.x;
        diff_pose2D.y = end_pose2D.y - start_pose2D.y;
        difference.linear.x = diff_pose2D.x;
        difference.linear.y = diff_pose2D.y;
        difference.linear.z = 0.0;
        difference.angular.x = 0.0;
        difference.angular.y = 0.0;
        difference.angular.z = diff_pose2D.theta * getCircumscribedRadius(*costmap_ros_);
        // TODO take into account kinematic properties of body

        return true;
    }
    bool EBPlanner::calcObstacleKinematicDistance(geometry_msgs::Pose center_pose, double& distance){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        unsigned int cell_x, cell_y;
        unsigned int disc_cost;
        double weight = costmap_weight_;

        if (!costmap_->worldToMap(center_pose.position.x,center_pose.position.y,cell_x,cell_y)) disc_cost = 1;
        else disc_cost = costmap_->getCost(cell_x,cell_y);
        if (disc_cost == costmap_2d::LETHAL_OBSTACLE){
            distance = 0.0;
        }else{
            if (disc_cost == 0) disc_cost = 1;
            else if (disc_cost == 255) disc_cost = 1; // can't understand
            double factor = ((double)disc_cost)/(costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1);
            distance = -log(factor)/weight;
        }
        return true;
    }
    bool EBPlanner::getForcesAt(int bubble_num,std::vector<Bubble> band,Bubble curr_bubble,geometry_msgs::WrenchStamped& forces){
        geometry_msgs::WrenchStamped internal_force, external_force;
        if (!calcInternalForces(bubble_num,band,curr_bubble,internal_force)){
            ROS_DEBUG("Calculation of internal forces failed");
            return false;
        }
        if (!calcExternalForces(bubble_num,curr_bubble,external_force)){
            ROS_DEBUG("Calculation of external forces failed");
            return false;
        }
        forces.wrench.force.x = internal_force.wrench.force.x + external_force.wrench.force.x;
        forces.wrench.force.y = internal_force.wrench.force.y + external_force.wrench.force.y;
        forces.wrench.force.z = internal_force.wrench.force.z + external_force.wrench.force.z;
        forces.wrench.torque.x = internal_force.wrench.torque.x + external_force.wrench.torque.x;
        forces.wrench.torque.y = internal_force.wrench.torque.y + external_force.wrench.torque.y;
        forces.wrench.torque.z = internal_force.wrench.torque.z + external_force.wrench.torque.z;
        if (!suppressTangentialForces(bubble_num,band,forces)){
            ROS_DEBUG("Suppression of tangential forces failed");
            return false;
        }
        return true;
    }
    bool EBPlanner::calcInternalForces(int bubble_num,std::vector<Bubble> band, Bubble curr_bubble,geometry_msgs::WrenchStamped& forces){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if (band.size() <= 2) return true;
        double distance1,distance2;
        geometry_msgs::Twist difference1,difference2;
        geometry_msgs::Wrench wrench;
        ROS_ASSERT(bubble_num > 0);
        ROS_ASSERT(bubble_num < ((int)band.size() - 1));
        if (!calcBubbleDistance(curr_bubble.center,band[bubble_num-1].center,distance1)){
            ROS_ERROR("Failed to calculate Distance between two bubbles. Aborting calculation of internal forces!");
            return false;
        }
        if (!calcBubbleDistance(curr_bubble.center,band[bubble_num+1].center,distance2)){
            ROS_ERROR("Failed to calculate Distance between two bubbles. Aborting calculation of internal forces!");
            return false;
        }
        if (!calcBubbleDifference(curr_bubble.center,band[bubble_num-1].center,difference1)){
            ROS_ERROR("Failed to calculate Difference between two bubbles. Aborting calculation of internal forces!");
            return false;
        }
        if (!calcBubbleDifference(curr_bubble.center,band[bubble_num+1].center,difference2)){
            ROS_ERROR("Failed to calculate Difference between two bubbles. Aborting calculation of internal forces!");
            return false;
        }
        if (distance1 <= tiny_bubble_distance_) distance1 = 1000000.0;
        if (distance2 <= tiny_bubble_distance_) distance2 = 1000000.0;

        wrench.force.x = internal_force_gain_ * (difference1.linear.x / distance1 + difference2.linear.x / distance2);
        wrench.force.y = internal_force_gain_ * (difference1.linear.y / distance1 + difference2.linear.y / distance2);
        wrench.force.z = internal_force_gain_ * (difference1.linear.z / distance1 + difference2.linear.z / distance2);
        wrench.torque.x = internal_force_gain_ * (difference1.angular.x / distance1 + difference2.angular.x / distance2);
        wrench.torque.y = internal_force_gain_ * (difference1.angular.y / distance1 + difference2.angular.y / distance2);
        wrench.torque.z = internal_force_gain_ * (difference1.angular.z / distance1 + difference2.angular.z / distance2);
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Calculating internal forces: (x, y, theta) = (%f, %f, %f)", wrench.force.x, wrench.force.y, wrench.torque.z);
#endif
        forces.wrench = wrench;
        return true;
    }
    bool EBPlanner::calcExternalForces(int bubble_num,Bubble curr_bubble,geometry_msgs::WrenchStamped forces){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        double distance1,distance2;
        geometry_msgs::Pose edge;
        geometry_msgs::Pose2D edge_pose2D;
        geometry_msgs::Wrench wrench;

        edge = curr_bubble.center.pose;
        edge.position.x = edge.position.x + curr_bubble.expansion;
        if (!calcObstacleKinematicDistance(edge,distance1)){
            ROS_DEBUG("Bubble %d probably at edge of map - cannot retrieve distance information to calculate external forces", bubble_num);
            return true;
        }
        edge.position.x = edge.position.x - 2.0 * curr_bubble.expansion;
        if (!calcObstacleKinematicDistance(edge,distance2)){
            ROS_DEBUG("Bubble %d probably at edge of map - cannot retrieve distance information to calculate external forces", bubble_num);
            return true;
        }
        if (curr_bubble.expansion <= tiny_bubble_expansion_){
            wrench.force.x = -external_force_gain_ * (distance2-distance1)/(2.0*tiny_bubble_expansion_);
            ROS_DEBUG("Calculating external forces on broken band. Bubble should have been removed. Local Planner probably ill configured");
        }else
            wrench.force.x = -external_force_gain_ * (distance2-distance1)/(2.0*curr_bubble.expansion);
        // TODO above equations skip term to make forces continuous at end of influence region - test to add corresponding term

        edge = curr_bubble.center.pose;
        edge.position.y = edge.position.y + curr_bubble.expansion;
        if (!calcObstacleKinematicDistance(edge,distance1)){
            ROS_DEBUG("Bubble %d probably at edge of map - cannot retrieve distance information to calculate external forces", bubble_num);
            return true;
        }
        edge.position.y = edge.position.y - 2.0 * curr_bubble.expansion;
        if (!calcObstacleKinematicDistance(edge,distance2)){
            ROS_DEBUG("Bubble %d probably at edge of map - cannot retrieve distance information to calculate external forces", bubble_num);
            return true;
        }
        if (curr_bubble.expansion <= tiny_bubble_expansion_){
            wrench.force.y = -external_force_gain_ * (distance2-distance1)/(2.0*tiny_bubble_expansion_);
            ROS_DEBUG("Calculating external forces on broken band. Bubble should have been removed. Local Planner probably ill configured");
        }else
            wrench.force.y = -external_force_gain_ * (distance2-distance1)/(2.0*curr_bubble.expansion);
        // TODO above equations skip term to make forces continuous at end of influence region - test to add corresponding term

        wrench.force.z = 0.0;

        wrench.torque.x = 0.0;
        wrench.torque.y = 0.0;
        PoseToPose2D(curr_bubble.center.pose,edge_pose2D);
        edge_pose2D.theta = edge_pose2D.theta + (curr_bubble.expansion/ getCircumscribedRadius(*costmap_ros_)); //cant understand
        edge_pose2D.theta = angles::normalize_angle(edge_pose2D.theta);
        // Pose2DToPose(edge_pose2D,edge);
        PoseToPose2D(edge,edge_pose2D);
        if (!calcObstacleKinematicDistance(edge,distance1)){
            ROS_DEBUG("Bubble %d probably at edge of map - cannot retrieve distance information to calculate external forces", bubble_num);
            return true;
        }
        edge_pose2D.theta = edge_pose2D.theta - 2.0*(curr_bubble.expansion/ getCircumscribedRadius(*costmap_ros_));
        edge_pose2D.theta = angles::normalize_angle(edge_pose2D.theta);
        // Pose2DToPose(edge_pose2D,edge);
        PoseToPose2D(edge,edge_pose2D);
        if (!calcObstacleKinematicDistance(edge,distance2)){
            ROS_DEBUG("Bubble %d probably at edge of map - cannot retrieve distance information to calculate external forces", bubble_num);
            return true;
        }
        if (curr_bubble.expansion <= tiny_bubble_expansion_){
            wrench.torque.z = -external_force_gain_ * (distance2 - distance1)/(2.0*tiny_bubble_expansion_);
            ROS_DEBUG("Calculating external forces on broken band. Bubble should have been removed. Local Planner probably ill configured");
        }else
            wrench.torque.z = -external_force_gain_ * (distance2 - distance1)/(2.0*curr_bubble.expansion);
        // TODO above equations skip term to make forces continuous at end of influence region - test to add corresponsing term

#ifdef DEBUG_EBAND_
        ROS_DEBUG("Calculating external forces: (x, y, theta) = (%f, %f, %f)", wrench.force.x, wrench.force.y, wrench.torque.z);
#endif
        forces.wrench = wrench;
        return true;
    }
    bool EBPlanner::suppressTangentialForces(int bubble_num,std::vector<Bubble> band,geometry_msgs::WrenchStamped& forces){
        if (band.size() <= 2) return true;
        double scalar_fd,scalar_dd;
        geometry_msgs::Twist difference;
        ROS_ASSERT(bubble_num > 0);
        ROS_ASSERT(bubble_num < ((int)band.size() - 1));
        if (!calcBubbleDifference(band[bubble_num+1].center,band[bubble_num-1].center,difference)) return false;
        scalar_fd = forces.wrench.force.x * difference.linear.x + forces.wrench.force.y * difference.linear.y +
                forces.wrench.force.z * difference.linear.z + forces.wrench.torque.x * difference.angular.x +
                forces.wrench.torque.y * difference.angular.y + forces.wrench.torque.z * difference.angular.z;
        scalar_dd = difference.linear.x * difference.linear.x + difference.linear.y * difference.linear.y +
                difference.linear.z * difference.linear.z + difference.angular.x * difference.angular.x +
                difference.angular.y * difference.angular.y + difference.angular.z * difference.angular.z;
        if (scalar_dd <= tiny_bubble_distance_)
            ROS_DEBUG("Calculating tangential forces for redundant bubbles. Bubble should have been removed. Local Planner probably ill configured");
        forces.wrench.force.x = forces.wrench.force.x - scalar_fd/scalar_dd * difference.linear.x;
        forces.wrench.force.y = forces.wrench.force.y - scalar_fd/scalar_dd * difference.linear.y;
        forces.wrench.force.z = forces.wrench.force.z - scalar_fd/scalar_dd * difference.linear.z;
        forces.wrench.torque.x = forces.wrench.torque.x - scalar_fd/scalar_dd * difference.angular.x;
        forces.wrench.torque.y = forces.wrench.torque.y - scalar_fd/scalar_dd * difference.angular.y;
        forces.wrench.torque.z = forces.wrench.torque.z - scalar_fd/scalar_dd * difference.angular.z;
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Supressing tangential forces: (x, y, theta) = (%f, %f, %f)",
        forces.wrench.force.x, forces.wrench.force.y, forces.wrench.torque.z);
#endif
        return true;
    }
    bool EBPlanner::convertPlanToBand(std::vector<geometry_msgs::PoseStamped> plan,std::vector<Bubble>& band){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        double distance = 0.0;
        std::vector<Bubble> tmp_band;
        ROS_DEBUG("Copying plan to band - Conversion started: %d frames to convert.", ((int) plan.size()) );
        tmp_band = band;
        tmp_band.resize(plan.size());
        for (int i = 0; i < ((int) plan.size()); ++i) {
#ifdef DEBUG_EBAND_
            ROS_DEBUG("Checking Frame %d of %d", i, ((int) plan.size()) );
#endif
            tmp_band[i].center = plan[i];
            if (!calcObstacleKinematicDistance(tmp_band[i].center.pose,distance)){
                ROS_WARN("Calculation of Distance between bubble and nearest obstacle failed. Frame %d of %d outside map", i, ((int) plan.size()) );
                return false;
            }
            if (distance <= 0.0){
                ROS_WARN("Calculation of Distance between bubble and nearest obstacle failed. Frame %d of %d in collision. Plan invalid", i, ((int) plan.size()) );
                // TODO if frame in collision try to repair band instaed of aborting averything
                return false;
            }
            tmp_band[i].expansion = distance;
        }
        band = tmp_band;
        ROS_DEBUG("Successfully converted plan to band");
        return true;
    }
    bool EBPlanner::convertBandToPlan(std::vector<geometry_msgs::PoseStamped>& plan,std::vector<Bubble> band){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        std::vector<geometry_msgs::PoseStamped> tmp_plan;
        tmp_plan.resize(band.size());
        for (int i = 0; i < ((int) band.size()); ++i) {
            tmp_plan[i] = band[i].center;
        }
        plan = tmp_plan;
        return true;
    }
}