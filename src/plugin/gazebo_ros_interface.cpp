/*
 * Copyright (C) 2020, Jonathan Cacace.
 * Email id : jonathan.cacace@gmail.com
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include <ros/ros.h>
#include <gazebo/gazebo_client.hh>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>

#include "CommandMotorSpeed.pb.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"

using namespace std;
	
/**
 * \brief Obtains a parameter from sdf.
 * \param[in] sdf Pointer to the sdf object.
 * \param[in] name Name of the parameter.
 * \param[out] param Param Variable to write the parameter to.
 * \param[in] default_value Default value, if the parameter not available.
 * \param[in] verbose If true, gzerror if the parameter is not available.
 */
template<class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value, const bool& verbose =
                     false) {
  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  }
  else {
    param = default_value;
    if (verbose)
      gzerr << "[rotors_gazebo_plugins] Please specify a value for parameter \"" << name << "\".\n";
  }
  return false;
}

namespace gazebo {

    enum outputFrame { NED, NOU, ENU };
 
    class GazeboROSInterface : public ModelPlugin {

        static const unsigned n_out_max = 16;

        private: ros::NodeHandle* _node_handle;
      	private: transport::NodePtr _gz_node;
        private: physics::ModelPtr model_;
        physics::WorldPtr world_;
        std::string motor_velocity_reference_pub_topic_;
        std::string output_odometry_frame;
        std::string _namespace;

        ros::Publisher _odom_pub;
        ros::Publisher _local_pose_pub;
        ros::Publisher _local_vel_pub;
        ros::Subscriber _local_pose_sub;
        ros::Subscriber _motor_vel_sub;

        Eigen::Matrix<double, 3,3> R_nou2ned;
        common::Time last_time_;
        common::Time last_imu_time_;
        common::Time last_actuator_time_;
      
        Eigen::VectorXd input_reference_;
        transport::PublisherPtr motor_velocity_reference_pub_;

        ignition::math::Quaterniond q_br = ignition::math::Quaterniond(0, 1, 0, 0);
        ignition::math::Quaterniond q_ng = ignition::math::Quaterniond(0, 0.70711, 0.70711, 0);

        private: event::ConnectionPtr updateConnection;

        std::vector< float > _cmd_vel;
        string _model_name; 
        float _motor_size;
        outputFrame frame;
        
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {	
            
            _node_handle = new ros::NodeHandle();	
            model_ = _parent;

            world_ = model_->GetWorld();
            
            _gz_node = transport::NodePtr(new transport::Node());
            _gz_node->Init(model_->GetName());
            
            getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_, motor_velocity_reference_pub_topic_);
            getSdfParam<std::string>(_sdf, "odomFrame", output_odometry_frame, output_odometry_frame);

            if( output_odometry_frame == "NED"  ) { 
                frame = NED;
            }
            else if( output_odometry_frame == "ENU" ) { 
                frame = ENU;
            }
            else if( output_odometry_frame == "NOU" ) {
                frame = NOU;
            }
            
            if( output_odometry_frame != "NED" &&  output_odometry_frame != "NOU" ) {
                cout << "Output odometry frame: " << output_odometry_frame << " not supported" << endl << "Switch on NED odometry frame" << endl;
                frame = NED;
            }
                        
            getSdfParam<float>(_sdf, "motorSize", _motor_size, _motor_size);
            getSdfParam<string>(_sdf, "robotNamespace", _namespace, _namespace);
            
            _cmd_vel.resize( n_out_max );
            for( int i=0; i< n_out_max; i++ ) _cmd_vel[i] = 0.0;
        
            input_reference_.resize(n_out_max);
            _model_name = model_->GetName();         
            _motor_vel_sub = _node_handle->subscribe( model_->GetName() + "/cmd/motor_vel", 1, &GazeboROSInterface::MotorVelCallback, this);
            motor_velocity_reference_pub_ = _gz_node->Advertise<gz_mav_msgs::CommandMotorSpeed>("/gazebo/default/" + _namespace  + "/gazebo/command/motor_speed", 1);
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboROSInterface::OnUpdate, this));
            _local_pose_sub = _node_handle->subscribe("/gazebo/model_states", 1, &GazeboROSInterface::GzModelsCallback, this);
            _local_pose_pub = _node_handle->advertise<geometry_msgs::PoseStamped>( model_->GetName() + "/local_pose", 0);
            _local_vel_pub = _node_handle->advertise<geometry_msgs::TwistStamped>( model_->GetName() + "/local_vel", 0);
            _odom_pub = _node_handle->advertise<nav_msgs::Odometry>( model_->GetName() + "/odometry", 0);
    }


    void GzModelsCallback( gazebo_msgs::ModelStates model_data ) {

        geometry_msgs::PoseStamped pose;
        geometry_msgs::TwistStamped vel;

        bool found = false;
        int index = 0;
        while( !found && index < model_data.name.size() ) {
            if( model_data.name[index] != _model_name  ) {
                index++;
            }
            else found = true;            
        }

        if (found) {
            
            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "world";

            if ( frame == NOU ) {

                pose.pose.position.x = model_data.pose[index].position.x;
                pose.pose.position.y = model_data.pose[index].position.y;
                pose.pose.position.z = model_data.pose[index].position.z;

                ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
                    model_data.pose[index].orientation.w,
                    model_data.pose[index].orientation.x,
                    model_data.pose[index].orientation.y,
                    model_data.pose[index].orientation.z);

                pose.pose.orientation.x = q_gr.X();
                pose.pose.orientation.y = q_gr.Y();
                pose.pose.orientation.z = q_gr.Z();
                pose.pose.orientation.w = q_gr.W();
        
                _local_pose_pub.publish(pose);

                vel.twist.linear.x = model_data.twist[index].linear.x;
                vel.twist.linear.y = model_data.twist[index].linear.y;
                vel.twist.linear.z = model_data.twist[index].linear.z;

                vel.twist.angular.x = model_data.twist[index].angular.x;
                vel.twist.angular.y = model_data.twist[index].angular.y;
                vel.twist.angular.z = model_data.twist[index].angular.z;

                _local_vel_pub.publish(vel);


                odom.pose.pose.position.x = pose.pose.position.x;
                odom.pose.pose.position.y = pose.pose.position.y;
                odom.pose.pose.position.z = pose.pose.position.z;

                odom.pose.pose.orientation.w = pose.pose.orientation.w;
                odom.pose.pose.orientation.x = pose.pose.orientation.x;
                odom.pose.pose.orientation.y = pose.pose.orientation.y;
                odom.pose.pose.orientation.z = pose.pose.orientation.z;

                odom.twist.twist = vel.twist;        
            }
            else if( frame == NED ) {
                R_nou2ned << 1, 0, 0, 0, -1, 0, 0, 0, -1;
                Eigen::Vector3d pos_ned, pos_nou;
                Eigen::Vector3d lin_vel_ned, lin_vel_nou;
                Eigen::Vector3d ang_vel_ned, ang_vel_nou;
                Eigen::Matrix3d R_body2world_nou, R_body2world_ned;

                pos_nou << model_data.pose[index].position.x,
                        model_data.pose[index].position.y,
                        model_data.pose[index].position.z;
                    
                pos_ned = R_nou2ned * pos_nou;

                pose.pose.position.x = pos_ned(0);
                pose.pose.position.y = pos_ned(1);
                pose.pose.position.z = pos_ned(2);

                Eigen::Quaterniond q_att_nou(
                    model_data.pose[index].orientation.w,
                    model_data.pose[index].orientation.x,
                    model_data.pose[index].orientation.y,
                    model_data.pose[index].orientation.z);

                R_body2world_nou = q_att_nou.toRotationMatrix();
                R_body2world_ned = R_nou2ned * R_body2world_nou * R_nou2ned.transpose();

                Eigen::Quaterniond q_att_ned( R_body2world_ned );

                pose.pose.orientation.x = q_att_ned.x();
                pose.pose.orientation.y = q_att_ned.y();
                pose.pose.orientation.z = q_att_ned.z();
                pose.pose.orientation.w = q_att_ned.w();

                _local_pose_pub.publish(pose);

                lin_vel_nou << model_data.twist[index].linear.x,
                            model_data.twist[index].linear.y,
                            model_data.twist[index].linear.z;

                lin_vel_ned = R_nou2ned * lin_vel_nou;

                vel.twist.linear.x = lin_vel_ned(0);
                vel.twist.linear.y = lin_vel_ned(1);
                vel.twist.linear.z = lin_vel_ned(2);

                ang_vel_nou << model_data.twist[index].angular.x,
                            model_data.twist[index].angular.y,
                            model_data.twist[index].angular.z;

                ang_vel_ned = R_nou2ned * ang_vel_nou;

                vel.twist.angular.x = ang_vel_ned(0);
                vel.twist.angular.y = ang_vel_ned(1);
                vel.twist.angular.z = ang_vel_ned(2);

                _local_vel_pub.publish(vel);
                odom.pose.pose.position.x = pose.pose.position.x;
                odom.pose.pose.position.y = pose.pose.position.y;
                odom.pose.pose.position.z = pose.pose.position.z;

                odom.pose.pose.orientation.w = pose.pose.orientation.w;
                odom.pose.pose.orientation.x = pose.pose.orientation.x;
                odom.pose.pose.orientation.y = pose.pose.orientation.y;
                odom.pose.pose.orientation.z = pose.pose.orientation.z;

                odom.twist.twist = vel.twist;                        
            }
            _odom_pub.publish( odom );            
        }
    }

    /*
     * Motor Order:
     *     3    1
     *       ^
     *     0    2
     *
     */
    public: void MotorVelCallback( std_msgs::Float32MultiArray motor_vel_ ) {
#if GAZEBO_MAJOR_VERSION >= 9
        last_actuator_time_ = world_->SimTime();
#else
        last_actuator_time_ = world_->GetSimTime();
#endif


        if( motor_vel_.data.size() != _motor_size) {
            cout << "Error: mismatch motor velocity and the number of motor size: " << motor_vel_.data.size() << " / " << _motor_size << endl;
        }
        else {
            for( int i=0; i<motor_vel_.data.size(); i++ ) {
                _cmd_vel[i] = motor_vel_.data[i];
            }
        }
    }

    // Called by the world update start event
    public: void OnUpdate()  {
#if GAZEBO_MAJOR_VERSION >= 9
        common::Time current_time = world_->SimTime();
#else
        common::Time current_time = world_->GetSimTime();
#endif

        double dt = (current_time - last_time_).Double();
        gz_mav_msgs::CommandMotorSpeed turning_velocities_msg;
        for (int i = 0; i < input_reference_.size(); i++) {
            if ( (last_actuator_time_ == 0 || (current_time - last_actuator_time_).Double() > 0.2)) {
                turning_velocities_msg.add_motor_speed(0);
            } //No power 
            else {
                turning_velocities_msg.add_motor_speed( _cmd_vel[i] );
            }
        }

        motor_velocity_reference_pub_->Publish(turning_velocities_msg);
        last_time_ = current_time;
	
        }
    
};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboROSInterface)
}
