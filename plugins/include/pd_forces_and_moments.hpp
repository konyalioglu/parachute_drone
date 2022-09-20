#ifndef _PD_FORCES_AND_MOMENTS_HPP
#define _PD_FORCES_AND_MOMENTS_HPP


#include <stdio.h>
#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <ignition/math/Vector3.hh>
#include "rosgraph_msgs/Clock.h"
#include "std_msgs/Float64MultiArray.h"
#include <eigen3/Eigen/Dense>
#include <cmath>



namespace gazebo
{


class ParachuteDroneForcesAndMoments : public ModelPlugin {
    public:
        ParachuteDroneForcesAndMoments();

        ~ParachuteDroneForcesAndMoments();

        void PublishForcesAndMoments();

        void calculateForcesAndMoments();

        void calculateGravitationalForces();

        void calculateAerodynamicForcesAndMoments();

        void calculatePropulsiveForcesAndMoments();


    protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        void OnUpdate(const common::UpdateInfo & /*_info*/);


    private:
        std::string prop_topic_name_ = "/parachute_drone/joint_vel";
        std::string clock_topic_name_;
        std::string joint_name_1_;
        std::string joint_name_2_;
        std::string link_name_;

        physics::WorldPtr world_;
        physics::ModelPtr model_;
        physics::LinkPtr link_;
        physics::JointPtr joint_;
        physics::EntityPtr parent_link_;
        event::ConnectionPtr updateConnection_;


        /// Physical Parameters
        double mass_;
        double g_;
        double cog_x_;
        double cog_y_;
        double cog_z_;
        ignition::math::Vector3d R_COG_;


        /// Propulsion System Parameters
        double ct_;
        double ch_;
        double w1_;
        double w2_;
        ignition::math::Vector3d PROP_R_1;
        ignition::math::Vector3d PROP_R_2;
        

        /// Aerodynamic Parameters
        double Cd_;
        double S_;
        double rho_;
        double Cdd_;
        double dref_;
        double Vt_;
        double FD_;
        double cop_x_;
        double cop_y_;
        double cop_z_;
        double r_motor1_y_;
        double r_motor2_y_;
        ignition::math::Vector3d R_COP_;
        
        /// Other Parameters
        ignition::math::Vector3d R_MOMENT_AERO;
        double curr_time;
        double prev_time;
        double freq;
                
                
        ignition::math::Vector3d FA_;
        ignition::math::Vector3d MA_;
        ignition::math::Vector3d FG_;
        ignition::math::Vector3d MG_;
        ignition::math::Vector3d FP_;
        ignition::math::Vector3d MP_;


        struct pd_total_forces_and_moments{
            double X;
            double Y;
            double Z;
            double L;
            double M;
            double N;
        } TOTAL_FM_;

        double psi;
        double theta;
        double phi;

        ignition::math::Vector3d LIN_VEL_NED_;
        ignition::math::Vector3d ANG_VEL_NED_;
        ignition::math::Vector3d EUL_ANG_NED_;
        ignition::math::Vector3d DRAG_FORCE_UNIT_VECTOR_;

        ignition::math::Vector3d LIN_VEL_GAZEBO_;
        ignition::math::Vector3d ANG_VEL_GAZEBO_;
        ignition::math::Vector3d EUL_ANG_GAZEBO_;
        ignition::math::Pose3d WORLD_COG_POSE_;
        //ignition::math::Vector3d EUL_ANG_GAZEBO_;

        ///std::unique_ptr<ros::NodeHandle> rosNode;
        ros::NodeHandle* node_handle_;

        ros::Subscriber prop_sub_;

        ros::CallbackQueue rosQueue;

        void callBackFunction(const std_msgs::Float64MultiArray::ConstPtr& msg);

        
      
};

}
#endif