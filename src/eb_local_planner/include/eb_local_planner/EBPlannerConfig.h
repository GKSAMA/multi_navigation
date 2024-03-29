//
// Created by gk on 2022/2/22.
//

#ifndef EB_LOCAL_PLANNER_EBPLANNERCONFIG_H
#define EB_LOCAL_PLANNER_EBPLANNERCONFIG_H

#if __cplusplus >= 201103L
#define DYNAMIC_RECONFIGURE_FINAL final
#else
#define DYNAMIC_RECONFIGURE_FINAL
#endif

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace eb_local_planner
{
    class EBPlannerConfigStatics;

    class EBPlannerConfig
    {
    public:
        class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
        {
        public:
            AbstractParamDescription(std::string n, std::string t, uint32_t l,
                                     std::string d, std::string e)
            {
                name = n;
                type = t;
                level = l;
                description = d;
                edit_method = e;
            }

            virtual void clamp(EBPlannerConfig &config, const EBPlannerConfig &max, const EBPlannerConfig &min) const = 0;
            virtual void calcLevel(uint32_t &level, const EBPlannerConfig &config1, const EBPlannerConfig &config2) const = 0;
            virtual void fromServer(const ros::NodeHandle &nh, EBPlannerConfig &config) const = 0;
            virtual void toServer(const ros::NodeHandle &nh, const EBPlannerConfig &config) const = 0;
            virtual bool fromMessage(const dynamic_reconfigure::Config &msg, EBPlannerConfig &config) const = 0;
            virtual void toMessage(dynamic_reconfigure::Config &msg, const EBPlannerConfig &config) const = 0;
            virtual void getValue(const EBPlannerConfig &config, boost::any &val) const = 0;
        };

        typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
        typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

        // Final keyword added to class because it has virtual methods and inherits
        // from a class with a non-virtual destructor.
        template <class T>
        class ParamDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractParamDescription
        {
        public:
            ParamDescription(std::string a_name, std::string a_type, uint32_t a_level,
                             std::string a_description, std::string a_edit_method, T EBPlannerConfig::* a_f) :
                    AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
                    field(a_f)
            {}

            T EBPlannerConfig::* field;

            virtual void clamp(EBPlannerConfig &config, const EBPlannerConfig &max, const EBPlannerConfig &min) const
            {
                if (config.*field > max.*field)
                    config.*field = max.*field;

                if (config.*field < min.*field)
                    config.*field = min.*field;
            }

            virtual void calcLevel(uint32_t &comb_level, const EBPlannerConfig &config1, const EBPlannerConfig &config2) const
            {
                if (config1.*field != config2.*field)
                    comb_level |= level;
            }

            virtual void fromServer(const ros::NodeHandle &nh, EBPlannerConfig &config) const
            {
                nh.getParam(name, config.*field);
            }

            virtual void toServer(const ros::NodeHandle &nh, const EBPlannerConfig &config) const
            {
                nh.setParam(name, config.*field);
            }

            virtual bool fromMessage(const dynamic_reconfigure::Config &msg, EBPlannerConfig &config) const
            {
                return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
            }

            virtual void toMessage(dynamic_reconfigure::Config &msg, const EBPlannerConfig &config) const
            {
                dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
            }

            virtual void getValue(const EBPlannerConfig &config, boost::any &val) const
            {
                val = config.*field;
            }
        };

        class AbstractGroupDescription : public dynamic_reconfigure::Group
        {
        public:
            AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
            {
                name = n;
                type = t;
                parent = p;
                state = s;
                id = i;
            }

            std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
            bool state;

            virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
            virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
            virtual void updateParams(boost::any &cfg, EBPlannerConfig &top) const= 0;
            virtual void setInitialState(boost::any &cfg) const = 0;


            void convertParams()
            {
                for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
                {
                    parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
                }
            }
        };

        typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
        typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

        // Final keyword added to class because it has virtual methods and inherits
        // from a class with a non-virtual destructor.
        template<class T, class PT>
        class GroupDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractGroupDescription
        {
        public:
            GroupDescription(std::string a_name, std::string a_type, int a_parent, int a_id, bool a_s, T PT::* a_f) : AbstractGroupDescription(a_name, a_type, a_parent, a_id, a_s), field(a_f)
            {
            }

            GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
            {
                parameters = g.parameters;
                abstract_parameters = g.abstract_parameters;
            }

            virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
            {
                PT* config = boost::any_cast<PT*>(cfg);
                if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
                    return false;

                for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
                {
                    boost::any n = &((*config).*field);
                    if(!(*i)->fromMessage(msg, n))
                        return false;
                }

                return true;
            }

            virtual void setInitialState(boost::any &cfg) const
            {
                PT* config = boost::any_cast<PT*>(cfg);
                T* group = &((*config).*field);
                group->state = state;

                for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
                {
                    boost::any n = boost::any(&((*config).*field));
                    (*i)->setInitialState(n);
                }

            }

            virtual void updateParams(boost::any &cfg, EBPlannerConfig &top) const
            {
                PT* config = boost::any_cast<PT*>(cfg);

                T* f = &((*config).*field);
                f->setParams(top, abstract_parameters);

                for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
                {
                    boost::any n = &((*config).*field);
                    (*i)->updateParams(n, top);
                }
            }

            virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
            {
                const PT config = boost::any_cast<PT>(cfg);
                dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

                for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
                {
                    (*i)->toMessage(msg, config.*field);
                }
            }

            T PT::* field;
            std::vector<EBPlannerConfig::AbstractGroupDescriptionConstPtr> groups;
        };

        class DEFAULT
        {
        public:
            DEFAULT()
            {
                state = true;
                name = "Default";
            }

            void setParams(EBPlannerConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
            {
                for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
                {
                    boost::any val;
                    (*_i)->getValue(config, val);

                    if("xy_goal_tolerance"==(*_i)->name){xy_goal_tolerance = boost::any_cast<double>(val);}
                    if("yaw_goal_tolerance"==(*_i)->name){yaw_goal_tolerance = boost::any_cast<double>(val);}
                    if("rot_stopped_vel"==(*_i)->name){rot_stopped_vel = boost::any_cast<double>(val);}
                    if("trans_stopped_vel"==(*_i)->name){trans_stopped_vel = boost::any_cast<double>(val);}
                    if("marker_lifetime"==(*_i)->name){marker_lifetime = boost::any_cast<double>(val);}
                    if("eband_min_relative_overlap"==(*_i)->name){eband_min_relative_overlap = boost::any_cast<double>(val);}
                    if("eband_tiny_bubble_distance"==(*_i)->name){eband_tiny_bubble_distance = boost::any_cast<double>(val);}
                    if("eband_tiny_bubble_expansion"==(*_i)->name){eband_tiny_bubble_expansion = boost::any_cast<double>(val);}
                    if("eband_internal_force_gain"==(*_i)->name){eband_internal_force_gain = boost::any_cast<double>(val);}
                    if("eband_external_force_gain"==(*_i)->name){eband_external_force_gain = boost::any_cast<double>(val);}
                    if("num_iterations_eband_optimization"==(*_i)->name){num_iterations_eband_optimization = boost::any_cast<int>(val);}
                    if("eband_equilibrium_approx_max_recursion_depth"==(*_i)->name){eband_equilibrium_approx_max_recursion_depth = boost::any_cast<int>(val);}
                    if("eband_equilibrium_relative_overshoot"==(*_i)->name){eband_equilibrium_relative_overshoot = boost::any_cast<double>(val);}
                    if("eband_significant_force_lower_bound"==(*_i)->name){eband_significant_force_lower_bound = boost::any_cast<double>(val);}
                    if("costmap_weight"==(*_i)->name){costmap_weight = boost::any_cast<double>(val);}
                    if("max_vel_lin"==(*_i)->name){max_vel_lin = boost::any_cast<double>(val);}
                    if("max_vel_th"==(*_i)->name){max_vel_th = boost::any_cast<double>(val);}
                    if("min_vel_lin"==(*_i)->name){min_vel_lin = boost::any_cast<double>(val);}
                    if("min_vel_th"==(*_i)->name){min_vel_th = boost::any_cast<double>(val);}
                    if("min_in_place_vel_th"==(*_i)->name){min_in_place_vel_th = boost::any_cast<double>(val);}
                    if("in_place_trans_vel"==(*_i)->name){in_place_trans_vel = boost::any_cast<double>(val);}
                    if("k_prop"==(*_i)->name){k_prop = boost::any_cast<double>(val);}
                    if("k_damp"==(*_i)->name){k_damp = boost::any_cast<double>(val);}
                    if("Ctrl_Rate"==(*_i)->name){Ctrl_Rate = boost::any_cast<double>(val);}
                    if("max_acceleration"==(*_i)->name){max_acceleration = boost::any_cast<double>(val);}
                    if("virtual_mass"==(*_i)->name){virtual_mass = boost::any_cast<double>(val);}
                    if("max_translational_acceleration"==(*_i)->name){max_translational_acceleration = boost::any_cast<double>(val);}
                    if("max_rotational_acceleration"==(*_i)->name){max_rotational_acceleration = boost::any_cast<double>(val);}
                    if("rotation_correction_threshold"==(*_i)->name){rotation_correction_threshold = boost::any_cast<double>(val);}
                    if("differential_drive"==(*_i)->name){differential_drive = boost::any_cast<bool>(val);}
                    if("bubble_velocity_multiplier"==(*_i)->name){bubble_velocity_multiplier = boost::any_cast<double>(val);}
                    if("rotation_threshold_multiplier"==(*_i)->name){rotation_threshold_multiplier = boost::any_cast<double>(val);}
                    if("disallow_hysteresis"==(*_i)->name){disallow_hysteresis = boost::any_cast<bool>(val);}
                }
            }

            double xy_goal_tolerance;
            double yaw_goal_tolerance;
            double rot_stopped_vel;
            double trans_stopped_vel;
            double marker_lifetime;
            double eband_min_relative_overlap;
            double eband_tiny_bubble_distance;
            double eband_tiny_bubble_expansion;
            double eband_internal_force_gain;
            double eband_external_force_gain;
            int num_iterations_eband_optimization;
            int eband_equilibrium_approx_max_recursion_depth;
            double eband_equilibrium_relative_overshoot;
            double eband_significant_force_lower_bound;
            double costmap_weight;
            double max_vel_lin;
            double max_vel_th;
            double min_vel_lin;
            double min_vel_th;
            double min_in_place_vel_th;
            double in_place_trans_vel;
            double k_prop;
            double k_damp;
            double Ctrl_Rate;
            double max_acceleration;
            double virtual_mass;
            double max_translational_acceleration;
            double max_rotational_acceleration;
            double rotation_correction_threshold;
            bool differential_drive;
            double bubble_velocity_multiplier;
            double rotation_threshold_multiplier;
            bool disallow_hysteresis;

            bool state;
            std::string name;


        }groups;



//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double xy_goal_tolerance;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double yaw_goal_tolerance;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double rot_stopped_vel;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double trans_stopped_vel;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double marker_lifetime;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double eband_min_relative_overlap;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double eband_tiny_bubble_distance;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double eband_tiny_bubble_expansion;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double eband_internal_force_gain;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double eband_external_force_gain;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        int num_iterations_eband_optimization;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        int eband_equilibrium_approx_max_recursion_depth;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double eband_equilibrium_relative_overshoot;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double eband_significant_force_lower_bound;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double costmap_weight;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double max_vel_lin;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double max_vel_th;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double min_vel_lin;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double min_vel_th;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double min_in_place_vel_th;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double in_place_trans_vel;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double k_prop;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double k_damp;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double Ctrl_Rate;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double max_acceleration;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double virtual_mass;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double max_translational_acceleration;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double max_rotational_acceleration;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double rotation_correction_threshold;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        bool differential_drive;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double bubble_velocity_multiplier;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        double rotation_threshold_multiplier;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
        bool disallow_hysteresis;
//#line 228 "/opt/ros/melodic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

        bool __fromMessage__(dynamic_reconfigure::Config &msg)
        {
            const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
            const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

            int count = 0;
            for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
                if ((*i)->fromMessage(msg, *this))
                    count++;

            for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
            {
                if ((*i)->id == 0)
                {
                    boost::any n = boost::any(this);
                    (*i)->updateParams(n, *this);
                    (*i)->fromMessage(msg, n);
                }
            }

            if (count != dynamic_reconfigure::ConfigTools::size(msg))
            {
                ROS_ERROR("EBPlannerConfig::__fromMessage__ called with an unexpected parameter.");
                ROS_ERROR("Booleans:");
                for (unsigned int i = 0; i < msg.bools.size(); i++)
                    ROS_ERROR("  %s", msg.bools[i].name.c_str());
                ROS_ERROR("Integers:");
                for (unsigned int i = 0; i < msg.ints.size(); i++)
                    ROS_ERROR("  %s", msg.ints[i].name.c_str());
                ROS_ERROR("Doubles:");
                for (unsigned int i = 0; i < msg.doubles.size(); i++)
                    ROS_ERROR("  %s", msg.doubles[i].name.c_str());
                ROS_ERROR("Strings:");
                for (unsigned int i = 0; i < msg.strs.size(); i++)
                    ROS_ERROR("  %s", msg.strs[i].name.c_str());
                // @todo Check that there are no duplicates. Make this error more
                // explicit.
                return false;
            }
            return true;
        }

        // This version of __toMessage__ is used during initialization of
        // statics when __getParamDescriptions__ can't be called yet.
        void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
        {
            dynamic_reconfigure::ConfigTools::clear(msg);
            for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
                (*i)->toMessage(msg, *this);

            for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
            {
                if((*i)->id == 0)
                {
                    (*i)->toMessage(msg, *this);
                }
            }
        }

        void __toMessage__(dynamic_reconfigure::Config &msg) const
        {
            const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
            const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
            __toMessage__(msg, __param_descriptions__, __group_descriptions__);
        }

        void __toServer__(const ros::NodeHandle &nh) const
        {
            const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
            for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
                (*i)->toServer(nh, *this);
        }

        void __fromServer__(const ros::NodeHandle &nh)
        {
            static bool setup=false;

            const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
            for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
                (*i)->fromServer(nh, *this);

            const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
            for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
                if (!setup && (*i)->id == 0) {
                    setup = true;
                    boost::any n = boost::any(this);
                    (*i)->setInitialState(n);
                }
            }
        }

        void __clamp__()
        {
            const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
            const EBPlannerConfig &__max__ = __getMax__();
            const EBPlannerConfig &__min__ = __getMin__();
            for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
                (*i)->clamp(*this, __max__, __min__);
        }

        uint32_t __level__(const EBPlannerConfig &config) const
        {
            const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
            uint32_t level = 0;
            for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
                (*i)->calcLevel(level, config, *this);
            return level;
        }

        static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
        static const EBPlannerConfig &__getDefault__();
        static const EBPlannerConfig &__getMax__();
        static const EBPlannerConfig &__getMin__();
        static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
        static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

    private:
        static const EBPlannerConfigStatics *__get_statics__();
    };

    template <> // Max and min are ignored for strings.
    inline void EBPlannerConfig::ParamDescription<std::string>::clamp(EBPlannerConfig &config, const EBPlannerConfig &max, const EBPlannerConfig &min) const
    {
        (void) config;
        (void) min;
        (void) max;
        return;
    }

    class EBPlannerConfigStatics
    {
        friend class EBPlannerConfig;

        EBPlannerConfigStatics()
        {
            EBPlannerConfig::GroupDescription<EBPlannerConfig::DEFAULT, EBPlannerConfig> Default("Default", "", 0, 0, true, &EBPlannerConfig::groups);
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.xy_goal_tolerance = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.xy_goal_tolerance = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.xy_goal_tolerance = 0.1;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("xy_goal_tolerance", "double", 0, "Distance tolerance for reaching the goal pose", "", &EBPlannerConfig::xy_goal_tolerance)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("xy_goal_tolerance", "double", 0, "Distance tolerance for reaching the goal pose", "", &EBPlannerConfig::xy_goal_tolerance)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.yaw_goal_tolerance = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.yaw_goal_tolerance = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.yaw_goal_tolerance = 0.05;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("yaw_goal_tolerance", "double", 0, "Orientation tolerance for reaching the desired goal pose", "", &EBPlannerConfig::yaw_goal_tolerance)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("yaw_goal_tolerance", "double", 0, "Orientation tolerance for reaching the desired goal pose", "", &EBPlannerConfig::yaw_goal_tolerance)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.rot_stopped_vel = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.rot_stopped_vel = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.rot_stopped_vel = 0.01;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("rot_stopped_vel", "double", 0, "Angular velocity lower bound that determines if the robot should stop to avoid limit-cycles or locks", "", &EBPlannerConfig::rot_stopped_vel)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("rot_stopped_vel", "double", 0, "Angular velocity lower bound that determines if the robot should stop to avoid limit-cycles or locks", "", &EBPlannerConfig::rot_stopped_vel)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.trans_stopped_vel = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.trans_stopped_vel = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.trans_stopped_vel = 0.01;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("trans_stopped_vel", "double", 0, "Linear velocity lower bound that determines if the robot should stop to avoid limit-cycles or locks", "", &EBPlannerConfig::trans_stopped_vel)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("trans_stopped_vel", "double", 0, "Linear velocity lower bound that determines if the robot should stop to avoid limit-cycles or locks", "", &EBPlannerConfig::trans_stopped_vel)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.marker_lifetime = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.marker_lifetime = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.marker_lifetime = 0.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("marker_lifetime", "double", 0, "Lifetime of eband visualization markers", "", &EBPlannerConfig::marker_lifetime)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("marker_lifetime", "double", 0, "Lifetime of eband visualization markers", "", &EBPlannerConfig::marker_lifetime)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.eband_min_relative_overlap = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.eband_min_relative_overlap = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.eband_min_relative_overlap = 0.7;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_min_relative_overlap", "double", 0, "Min distance that denotes connectivity between consecutive bubbles", "", &EBPlannerConfig::eband_min_relative_overlap)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_min_relative_overlap", "double", 0, "Min distance that denotes connectivity between consecutive bubbles", "", &EBPlannerConfig::eband_min_relative_overlap)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.eband_tiny_bubble_distance = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.eband_tiny_bubble_distance = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.eband_tiny_bubble_distance = 0.01;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_tiny_bubble_distance", "double", 0, "Bubble geometric bound regarding tiny bubble distance", "", &EBPlannerConfig::eband_tiny_bubble_distance)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_tiny_bubble_distance", "double", 0, "Bubble geometric bound regarding tiny bubble distance", "", &EBPlannerConfig::eband_tiny_bubble_distance)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.eband_tiny_bubble_expansion = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.eband_tiny_bubble_expansion = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.eband_tiny_bubble_expansion = 0.01;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_tiny_bubble_expansion", "double", 0, "Bubble geometric bound regarding tiny bubble expansion", "", &EBPlannerConfig::eband_tiny_bubble_expansion)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_tiny_bubble_expansion", "double", 0, "Bubble geometric bound regarding tiny bubble expansion", "", &EBPlannerConfig::eband_tiny_bubble_expansion)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.eband_internal_force_gain = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.eband_internal_force_gain = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.eband_internal_force_gain = 1.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_internal_force_gain", "double", 0, "Force gain of forces between consecutive bubbles that tend to stretch the elastic band", "", &EBPlannerConfig::eband_internal_force_gain)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_internal_force_gain", "double", 0, "Force gain of forces between consecutive bubbles that tend to stretch the elastic band", "", &EBPlannerConfig::eband_internal_force_gain)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.eband_external_force_gain = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.eband_external_force_gain = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.eband_external_force_gain = 2.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_external_force_gain", "double", 0, "Force gain of forces that tend to move the bubbles away from obstacles", "", &EBPlannerConfig::eband_external_force_gain)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_external_force_gain", "double", 0, "Force gain of forces that tend to move the bubbles away from obstacles", "", &EBPlannerConfig::eband_external_force_gain)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.num_iterations_eband_optimization = 1;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.num_iterations_eband_optimization = 2147483647;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.num_iterations_eband_optimization = 3;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<int>("num_iterations_eband_optimization", "int", 0, "Number of iterations for eband optimization", "", &EBPlannerConfig::num_iterations_eband_optimization)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<int>("num_iterations_eband_optimization", "int", 0, "Number of iterations for eband optimization", "", &EBPlannerConfig::num_iterations_eband_optimization)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.eband_equilibrium_approx_max_recursion_depth = 1;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.eband_equilibrium_approx_max_recursion_depth = 2147483647;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.eband_equilibrium_approx_max_recursion_depth = 4;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<int>("eband_equilibrium_approx_max_recursion_depth", "int", 0, "Number of iterations for reaching the equilibrium between internal and external forces", "", &EBPlannerConfig::eband_equilibrium_approx_max_recursion_depth)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<int>("eband_equilibrium_approx_max_recursion_depth", "int", 0, "Number of iterations for reaching the equilibrium between internal and external forces", "", &EBPlannerConfig::eband_equilibrium_approx_max_recursion_depth)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.eband_equilibrium_relative_overshoot = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.eband_equilibrium_relative_overshoot = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.eband_equilibrium_relative_overshoot = 0.75;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_equilibrium_relative_overshoot", "double", 0, "Maximum relative equlibrium overshoot", "", &EBPlannerConfig::eband_equilibrium_relative_overshoot)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_equilibrium_relative_overshoot", "double", 0, "Maximum relative equlibrium overshoot", "", &EBPlannerConfig::eband_equilibrium_relative_overshoot)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.eband_significant_force_lower_bound = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.eband_significant_force_lower_bound = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.eband_significant_force_lower_bound = 0.15;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_significant_force_lower_bound", "double", 0, "Minimum magnitude of force that is considered significant and used in the calculations", "", &EBPlannerConfig::eband_significant_force_lower_bound)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("eband_significant_force_lower_bound", "double", 0, "Minimum magnitude of force that is considered significant and used in the calculations", "", &EBPlannerConfig::eband_significant_force_lower_bound)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.costmap_weight = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.costmap_weight = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.costmap_weight = 10.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("costmap_weight", "double", 0, "Costmap weight factor used in the calculation of distance to obstacles", "", &EBPlannerConfig::costmap_weight)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("costmap_weight", "double", 0, "Costmap weight factor used in the calculation of distance to obstacles", "", &EBPlannerConfig::costmap_weight)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.max_vel_lin = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.max_vel_lin = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.max_vel_lin = 0.75;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("max_vel_lin", "double", 0, "Maximum linear velocity", "", &EBPlannerConfig::max_vel_lin)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("max_vel_lin", "double", 0, "Maximum linear velocity", "", &EBPlannerConfig::max_vel_lin)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.max_vel_th = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.max_vel_th = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.max_vel_th = 1.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("max_vel_th", "double", 0, "Maximum angular velocity", "", &EBPlannerConfig::max_vel_th)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("max_vel_th", "double", 0, "Maximum angular velocity", "", &EBPlannerConfig::max_vel_th)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.min_vel_lin = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.min_vel_lin = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.min_vel_lin = 0.1;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("min_vel_lin", "double", 0, "Minimum linear velocity", "", &EBPlannerConfig::min_vel_lin)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("min_vel_lin", "double", 0, "Minimum linear velocity", "", &EBPlannerConfig::min_vel_lin)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.min_vel_th = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.min_vel_th = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.min_vel_th = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("min_vel_th", "double", 0, "Minimum angular velocity", "", &EBPlannerConfig::min_vel_th)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("min_vel_th", "double", 0, "Minimum angular velocity", "", &EBPlannerConfig::min_vel_th)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.min_in_place_vel_th = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.min_in_place_vel_th = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.min_in_place_vel_th = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("min_in_place_vel_th", "double", 0, "Minimum in-place angular velocity", "", &EBPlannerConfig::min_in_place_vel_th)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("min_in_place_vel_th", "double", 0, "Minimum in-place angular velocity", "", &EBPlannerConfig::min_in_place_vel_th)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.in_place_trans_vel = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.in_place_trans_vel = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.in_place_trans_vel = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("in_place_trans_vel", "double", 0, "Minimum in place linear velocity", "", &EBPlannerConfig::in_place_trans_vel)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("in_place_trans_vel", "double", 0, "Minimum in place linear velocity", "", &EBPlannerConfig::in_place_trans_vel)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.k_prop = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.k_prop = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.k_prop = 4.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("k_prop", "double", 0, "Proportional gain of the PID controller", "", &EBPlannerConfig::k_prop)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("k_prop", "double", 0, "Proportional gain of the PID controller", "", &EBPlannerConfig::k_prop)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.k_damp = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.k_damp = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.k_damp = 3.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("k_damp", "double", 0, "Damping gain of the PID controller", "", &EBPlannerConfig::k_damp)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("k_damp", "double", 0, "Damping gain of the PID controller", "", &EBPlannerConfig::k_damp)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.Ctrl_Rate = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.Ctrl_Rate = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.Ctrl_Rate = 10.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("Ctrl_Rate", "double", 0, "Control rate", "", &EBPlannerConfig::Ctrl_Rate)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("Ctrl_Rate", "double", 0, "Control rate", "", &EBPlannerConfig::Ctrl_Rate)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.max_acceleration = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.max_acceleration = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.max_acceleration = 0.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("max_acceleration", "double", 0, "Maximum allowable acceleration", "", &EBPlannerConfig::max_acceleration)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("max_acceleration", "double", 0, "Maximum allowable acceleration", "", &EBPlannerConfig::max_acceleration)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.virtual_mass = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.virtual_mass = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.virtual_mass = 0.75;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("virtual_mass", "double", 0, "Virtual mass", "", &EBPlannerConfig::virtual_mass)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("virtual_mass", "double", 0, "Virtual mass", "", &EBPlannerConfig::virtual_mass)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.max_translational_acceleration = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.max_translational_acceleration = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.max_translational_acceleration = 0.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("max_translational_acceleration", "double", 0, "Maximum linear acceleration", "", &EBPlannerConfig::max_translational_acceleration)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("max_translational_acceleration", "double", 0, "Maximum linear acceleration", "", &EBPlannerConfig::max_translational_acceleration)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.max_rotational_acceleration = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.max_rotational_acceleration = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.max_rotational_acceleration = 1.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("max_rotational_acceleration", "double", 0, "Maximum angular acceleration", "", &EBPlannerConfig::max_rotational_acceleration)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("max_rotational_acceleration", "double", 0, "Maximum angular acceleration", "", &EBPlannerConfig::max_rotational_acceleration)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.rotation_correction_threshold = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.rotation_correction_threshold = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.rotation_correction_threshold = 0.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("rotation_correction_threshold", "double", 0, "Rotation correction threshold", "", &EBPlannerConfig::rotation_correction_threshold)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("rotation_correction_threshold", "double", 0, "Rotation correction threshold", "", &EBPlannerConfig::rotation_correction_threshold)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.differential_drive = 0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.differential_drive = 1;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.differential_drive = 1;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<bool>("differential_drive", "bool", 0, "Denotes whether to use the differential drive hack", "", &EBPlannerConfig::differential_drive)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<bool>("differential_drive", "bool", 0, "Denotes whether to use the differential drive hack", "", &EBPlannerConfig::differential_drive)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.bubble_velocity_multiplier = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.bubble_velocity_multiplier = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.bubble_velocity_multiplier = 2.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("bubble_velocity_multiplier", "double", 0, "Multiplier of bubble radius", "", &EBPlannerConfig::bubble_velocity_multiplier)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("bubble_velocity_multiplier", "double", 0, "Multiplier of bubble radius", "", &EBPlannerConfig::bubble_velocity_multiplier)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.rotation_threshold_multiplier = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.rotation_threshold_multiplier = std::numeric_limits<double>::infinity();
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.rotation_threshold_multiplier = 1.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("rotation_threshold_multiplier", "double", 0, "Multiplier of rotation threshold", "", &EBPlannerConfig::rotation_threshold_multiplier)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<double>("rotation_threshold_multiplier", "double", 0, "Multiplier of rotation threshold", "", &EBPlannerConfig::rotation_threshold_multiplier)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __min__.disallow_hysteresis = 0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __max__.disallow_hysteresis = 1;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __default__.disallow_hysteresis = 0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.abstract_parameters.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<bool>("disallow_hysteresis", "bool", 0, "Determines whether to try getting closer to the goal, in case of going past the tolerance", "", &EBPlannerConfig::disallow_hysteresis)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __param_descriptions__.push_back(EBPlannerConfig::AbstractParamDescriptionConstPtr(new EBPlannerConfig::ParamDescription<bool>("disallow_hysteresis", "bool", 0, "Determines whether to try getting closer to the goal, in case of going past the tolerance", "", &EBPlannerConfig::disallow_hysteresis)));
//#line 246 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            Default.convertParams();
//#line 246 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
            __group_descriptions__.push_back(EBPlannerConfig::AbstractGroupDescriptionConstPtr(new EBPlannerConfig::GroupDescription<EBPlannerConfig::DEFAULT, EBPlannerConfig>(Default)));
//#line 366 "/opt/ros/melodic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

            for (std::vector<EBPlannerConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
            {
                __description_message__.groups.push_back(**i);
            }
            __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
            __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
            __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
        }
        std::vector<EBPlannerConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
        std::vector<EBPlannerConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
        EBPlannerConfig __max__;
        EBPlannerConfig __min__;
        EBPlannerConfig __default__;
        dynamic_reconfigure::ConfigDescription __description_message__;

        static const EBPlannerConfigStatics *get_instance()
        {
            // Split this off in a separate function because I know that
            // instance will get initialized the first time get_instance is
            // called, and I am guaranteeing that get_instance gets called at
            // most once.
            static EBPlannerConfigStatics instance;
            return &instance;
        }
    };

    inline const dynamic_reconfigure::ConfigDescription &EBPlannerConfig::__getDescriptionMessage__()
    {
        return __get_statics__()->__description_message__;
    }

    inline const EBPlannerConfig &EBPlannerConfig::__getDefault__()
    {
        return __get_statics__()->__default__;
    }

    inline const EBPlannerConfig &EBPlannerConfig::__getMax__()
    {
        return __get_statics__()->__max__;
    }

    inline const EBPlannerConfig &EBPlannerConfig::__getMin__()
    {
        return __get_statics__()->__min__;
    }

    inline const std::vector<EBPlannerConfig::AbstractParamDescriptionConstPtr> &EBPlannerConfig::__getParamDescriptions__()
    {
        return __get_statics__()->__param_descriptions__;
    }

    inline const std::vector<EBPlannerConfig::AbstractGroupDescriptionConstPtr> &EBPlannerConfig::__getGroupDescriptions__()
    {
        return __get_statics__()->__group_descriptions__;
    }

    inline const EBPlannerConfigStatics *EBPlannerConfig::__get_statics__()
    {
        const static EBPlannerConfigStatics *statics;

        if (statics) // Common case
            return statics;

        boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

        if (statics) // In case we lost a race.
            return statics;

        statics = EBPlannerConfigStatics::get_instance();

        return statics;
    }


}

#undef DYNAMIC_RECONFIGURE_FINAL


#endif //EB_LOCAL_PLANNER_EBPLANNERCONFIG_H
