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


#include "SITLGps.pb.h"
#include "Imu.pb.h"
#include "Groundtruth.pb.h"
#include "CommandMotorSpeed.pb.h"

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <tf/transform_broadcaster.h>
#include <ctime>
//#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <random>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32MultiArray.h"

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
    typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> ImuPtr;
    typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;
    typedef const boost::shared_ptr<const sensor_msgs::msgs::SITLGps> GpsPtr;

    class GazeboROSInterface : public ModelPlugin {

        //Ros variables		
        private: ros::NodeHandle* _node_handle;
      	private: transport::NodePtr _gz_node;
        private: physics::ModelPtr model_;
        physics::WorldPtr world_;
        double alt_home = 488.0;   // meters
        double imu_update_interval_ = 0.004;
        static const unsigned n_out_max = 16;

        transport::NodePtr node_handle_;
        std::string motor_velocity_reference_pub_topic_;
        std::string imu_sub_topic_;
        std::string gps_sub_topic_;
        std::string vision_sub_topic_;
        std::string lidar_sub_topic_;
        std::string opticalFlow_sub_topic_;
        std::string sonar_sub_topic_;
        std::string irlock_sub_topic_;
        std::string groundtruth_sub_topic_;

        std::vector<physics::JointPtr> joints_;
        std::vector<common::PID> pids_;
        double input_offset_[n_out_max];
        double input_scaling_[n_out_max];
        double zero_position_disarmed_[n_out_max];
        double zero_position_armed_[n_out_max];
        std::string joint_control_type_[n_out_max];
        std::string gztopic_[n_out_max];
        transport::PublisherPtr joint_control_pub_[n_out_max];
        transport::SubscriberPtr imu_sub_;
        transport::SubscriberPtr lidar_sub_;
        transport::SubscriberPtr sonar_sub_;
        transport::SubscriberPtr opticalFlow_sub_;
        transport::SubscriberPtr irlock_sub_;
        transport::SubscriberPtr gps_sub_;
        transport::SubscriberPtr groundtruth_sub_;
        transport::SubscriberPtr vision_sub_;

        ros::Publisher _imu_pub;
        ros::Publisher _gps_pub;
        ros::Subscriber _local_pose_sub;
        ros::Subscriber _motor_vel_sub;
        ros::Publisher _local_pose_pub;
        ros::Publisher _local_vel_pub;

        common::Time last_time_;
        common::Time last_imu_time_;
        common::Time last_actuator_time_;
        std::default_random_engine rand_;
        std::normal_distribution<float> randn_;
        
        ignition::math::Vector3d gravity_W_;
        ignition::math::Vector3d velocity_prev_W_;
        ignition::math::Vector3d mag_d_;

        double groundtruth_lat_rad;
        double groundtruth_lon_rad;
        double groundtruth_altitude;
        Eigen::VectorXd input_reference_;
        transport::PublisherPtr motor_velocity_reference_pub_;

        unsigned _rotor_count;
        //ordine del quat -> w,x,y,z
        ignition::math::Quaterniond q_br = ignition::math::Quaterniond(0, 1, 0, 0); //rotazione di -pi intorno ad X
        ignition::math::Quaterniond q_ng = ignition::math::Quaterniond(0, 0.70711, 0.70711, 0); // x: -3.1415927, y: 0, z: -1.5707963 
        
        //ignition::math::Quaterniond qr_ned = ignition::math::Quaterniond(0, 1, 0, 0); //da terna robot a terna ned
        //Quaternion for ENU to NOU
        //ignition::math::Quaterniond q_enu2nou = ignition::math::Quaterniond(0.70711, 0.0, 0.0, 0.70711);
        Eigen::Matrix<double, 3,3> R_nou2ned;
        

        private: event::ConnectionPtr updateConnection;
        double baro_rnd_y2_;
        bool baro_rnd_use_last_;
        //float _cmd_vel[4];
        std::vector< float > _cmd_vel;
        string _model_name; 
        float _motor_size;
        bool _motor_cmd_ready;
        
        public: void GroundtruthCallback(GtPtr& groundtruth_msg) {
            // update groundtruth lat_rad, lon_rad and altitude
            groundtruth_lat_rad = groundtruth_msg->latitude_rad();
            groundtruth_lon_rad = groundtruth_msg->longitude_rad();
            groundtruth_altitude = groundtruth_msg->altitude();
            // the rest of the data is obtained directly on this interface and sent to
            // the FCU
        }


        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {	
            
            _node_handle = new ros::NodeHandle();	
            model_ = _parent;

            world_ = model_->GetWorld();
            const char *env_alt = std::getenv("PX4_HOME_ALT");
            if (env_alt) {
                gzmsg << "Home altitude is set to " << env_alt << ".\n";
                alt_home = std::stod(env_alt);
            }
            

            //_cmd_vel[0] = _cmd_vel[1] = _cmd_vel[2] = _cmd_vel[3] = 0.0;
         
         
            _gz_node = transport::NodePtr(new transport::Node());
            _gz_node->Init(model_->GetName());
            
            getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_, motor_velocity_reference_pub_topic_);
            getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
            getSdfParam<std::string>(_sdf, "gpsSubTopic", gps_sub_topic_, gps_sub_topic_);
            getSdfParam<std::string>(_sdf, "visionSubTopic", vision_sub_topic_, vision_sub_topic_);
            getSdfParam<std::string>(_sdf, "lidarSubTopic", lidar_sub_topic_, lidar_sub_topic_);
            getSdfParam<std::string>(_sdf, "opticalFlowSubTopic", opticalFlow_sub_topic_, opticalFlow_sub_topic_);
            getSdfParam<std::string>(_sdf, "sonarSubTopic", sonar_sub_topic_, sonar_sub_topic_);
            getSdfParam<std::string>(_sdf, "irlockSubTopic", irlock_sub_topic_, irlock_sub_topic_);

            
            getSdfParam<float>(_sdf, "motorSize", _motor_size, _motor_size);
            _cmd_vel.resize( n_out_max );
            for( int i=0; i< n_out_max; i++ ) _cmd_vel[i] = 0.0;
            groundtruth_sub_topic_ = "/groundtruth";

        
            // set input_reference_ from inputs.control
            input_reference_.resize(n_out_max);
            joints_.resize(n_out_max);
            pids_.resize(n_out_max);
            for (int i = 0; i < n_out_max; ++i) {
                pids_[i].Init(0, 0, 0, 0, 0, 0, 0);
                input_reference_[i] = 0;
            }


            _model_name = model_->GetName();
            
            if (_sdf->HasElement("control_channels")) {
                
                sdf::ElementPtr control_channels = _sdf->GetElement("control_channels");
                sdf::ElementPtr channel = control_channels->GetElement("channel");
                while (channel) {
                    if (channel->HasElement("input_index")) {
                        int index = channel->Get<int>("input_index");
                        if (index < n_out_max) {
                            input_offset_[index] = channel->Get<double>("input_offset");
                            input_scaling_[index] = channel->Get<double>("input_scaling");
                            zero_position_disarmed_[index] = channel->Get<double>("zero_position_disarmed");
                            zero_position_armed_[index] = channel->Get<double>("zero_position_armed");

                            if (channel->HasElement("joint_control_type")) {
                                joint_control_type_[index] = channel->Get<std::string>("joint_control_type");
                            }
                            else {
                                gzwarn << "joint_control_type[" << index << "] not specified, using velocity.\n";
                                joint_control_type_[index] = "velocity";
                            }

                            // start gz transport node handle
                            if (joint_control_type_[index] == "position_gztopic") {
                                // setup publisher handle to topic
                                if (channel->HasElement("gztopic"))
                                    gztopic_[index] = "~/" + model_->GetName() + channel->Get<std::string>("gztopic");
                                else
                                    gztopic_[index] = "control_position_gztopic_" + std::to_string(index);
#if GAZEBO_MAJOR_VERSION >= 7 && GAZEBO_MINOR_VERSION >= 4
                                /// only gazebo 7.4 and above support Any
                                joint_control_pub_[index] = node_handle_->Advertise<gazebo::msgs::Any>(
                                gztopic_[index]);
#else
                                joint_control_pub_[index] = node_handle_->Advertise<gazebo::msgs::GzString>(
                                gztopic_[index]);
#endif
                            }

                            if (channel->HasElement("joint_name")) {
                                std::string joint_name = channel->Get<std::string>("joint_name");
                                joints_[index] = model_->GetJoint(joint_name);
                                if (joints_[index] == nullptr) {
                                    gzwarn << "joint [" << joint_name << "] not found for channel["
                                    << index << "] no joint control for this channel.\n";
                                }
                                else {
                                    gzdbg << "joint [" << joint_name << "] found for channel["
                                    << index << "] joint control active for this channel.\n";
                                }
                            }
                            else {
                                gzdbg << "<joint_name> not found for channel[" << index
                                << "] no joint control will be performed for this channel.\n";
                            }

                            // setup joint control pid to control joint
                            if (channel->HasElement("joint_control_pid")) {
                                sdf::ElementPtr pid = channel->GetElement("joint_control_pid");
                                double p = 0;
                                if (pid->HasElement("p"))
                                    p = pid->Get<double>("p");
                                double i = 0;
                                if (pid->HasElement("i"))
                                    i = pid->Get<double>("i");
                                double d = 0;
                                if (pid->HasElement("d"))
                                    d = pid->Get<double>("d");
                                double iMax = 0;
                                if (pid->HasElement("iMax"))
                                    iMax = pid->Get<double>("iMax");
                                double iMin = 0;
                                if (pid->HasElement("iMin"))
                                    iMin = pid->Get<double>("iMin");
                                double cmdMax = 0;
                                if (pid->HasElement("cmdMax"))
                                    cmdMax = pid->Get<double>("cmdMax");
                                double cmdMin = 0;
                                if (pid->HasElement("cmdMin"))
                                    cmdMin = pid->Get<double>("cmdMin");
                                pids_[index].Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
                            }
                        }
                        else {
                            gzerr << "input_index[" << index << "] out of range, not parsing.\n";
                        }
                    }
                    else {
                        gzerr << "no input_index, not parsing.\n";
                        break;
                    }
                    channel = channel->GetNextElement("channel");
                    
                }
                
            }


            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboROSInterface::OnUpdate, this));

            // Input
            //lidar_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + lidar_sub_topic_, &GazeboROSInterface::LidarCallback, this);
            //opticalFlow_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + opticalFlow_sub_topic_, &GazeboROSInterface::OpticalFlowCallback, this);
            //sonar_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + sonar_sub_topic_, &GazeboROSInterface::SonarCallback, this);
            //irlock_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + irlock_sub_topic_, &GazeboROSInterface::IRLockCallback, this);
            //imu_sub_ = _gz_node->Subscribe("/gazebo/default/tarot_standalone/imu", &GazeboROSInterface::ImuCallback, this);           
            groundtruth_sub_ = _gz_node->Subscribe("~/" + model_->GetName() + groundtruth_sub_topic_, &GazeboROSInterface::GroundtruthCallback, this);
            _local_pose_sub = _node_handle->subscribe("/gazebo/model_states", 0, &GazeboROSInterface::GzModelsCallback, this);
            gps_sub_ = _gz_node->Subscribe("~/" + model_->GetName() + gps_sub_topic_, &GazeboROSInterface::GpsCallback, this);
            _motor_vel_sub = _node_handle->subscribe( model_->GetName() + "/cmd/motor_vel", 0, &GazeboROSInterface::MotorVelCallback, this);
            
            //vision_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + vision_sub_topic_, &GazeboROSInterface::VisionCallback, this);
    
            // Output
            _imu_pub = _node_handle->advertise<sensor_msgs::Imu>("/tarot/imu/data", 0);
            _gps_pub = _node_handle->advertise<sensor_msgs::NavSatFix>("tarot/gps/fix", 0);      

            motor_velocity_reference_pub_ = _gz_node->Advertise<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + motor_velocity_reference_pub_topic_, 1);
            _local_pose_pub = _node_handle->advertise<geometry_msgs::PoseStamped>( model_->GetName() + "/local_pose", 0);
            _local_vel_pub = _node_handle->advertise<geometry_msgs::TwistStamped>( model_->GetName() + "/local_vel", 0);

            _rotor_count = 5;
            #if GAZEBO_MAJOR_VERSION >= 9
                last_time_ = world_->SimTime();
                last_imu_time_ = world_->SimTime();
                gravity_W_ = world_->Gravity();
            #else
                last_time_ = world_->GetSimTime();
                last_imu_time_ = world_->GetSimTime();
                gravity_W_ = ignitionFromGazeboMath(world_->GetPhysicsEngine()->GetGravity());
            #endif

            if (_sdf->HasElement("imu_rate")) {
                imu_update_interval_ = 1 / _sdf->GetElement("imu_rate")->Get<int>();
            }

    
                 
    
    }

    public: void ImuCallback(ImuPtr& imu_message) {
#if GAZEBO_MAJOR_VERSION >= 9
        common::Time current_time = world_->SimTime();
#else
        common::Time current_time = world_->GetSimTime();
#endif
        double dt = (current_time - last_imu_time_).Double();

        ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
            imu_message->orientation().w(),
            imu_message->orientation().x(),
            imu_message->orientation().y(),
            imu_message->orientation().z());


        ignition::math::Quaterniond q_gb = q_gr*q_br.Inverse();
        ignition::math::Quaterniond q_nb = q_ng*q_gb;

    #if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Vector3d pos_g = model_->WorldPose().Pos();
    #else
        ignition::math::Vector3d pos_g = ignitionFromGazeboMath(model_->GetWorldPose().pos);
    #endif
        ignition::math::Vector3d pos_n = q_ng.RotateVector(pos_g);

        //float declination = get_mag_declination(groundtruth_lat_rad, groundtruth_lon_rad);
        //ignition::math::Quaterniond q_dn(0.0, 0.0, declination);
        //ignition::math::Vector3d mag_n = q_dn.RotateVector(mag_d_);

    #if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Vector3d vel_b = q_br.RotateVector(model_->RelativeLinearVel());
        ignition::math::Vector3d vel_n = q_ng.RotateVector(model_->WorldLinearVel());
        ignition::math::Vector3d omega_nb_b = q_br.RotateVector(model_->RelativeAngularVel());
    #else
        ignition::math::Vector3d vel_b = q_br.RotateVector(ignitionFromGazeboMath(model_->GetRelativeLinearVel()));
        ignition::math::Vector3d vel_n = q_ng.RotateVector(ignitionFromGazeboMath(model_->GetWorldLinearVel()));
        ignition::math::Vector3d omega_nb_b = q_br.RotateVector(ignitionFromGazeboMath(model_->GetRelativeAngularVel()));
    #endif

        ignition::math::Vector3d accel_b = q_br.RotateVector(ignition::math::Vector3d(
        imu_message->linear_acceleration().x(),
        imu_message->linear_acceleration().y(),
        imu_message->linear_acceleration().z()));
        ignition::math::Vector3d gyro_b = q_br.RotateVector(ignition::math::Vector3d(
        imu_message->angular_velocity().x(),
        imu_message->angular_velocity().y(),
        imu_message->angular_velocity().z()));


        if (imu_update_interval_!=0 && dt >= imu_update_interval_) {
           
            sensor_msgs::Imu imu_data;
            imu_data.orientation.w = q_nb.W();
            imu_data.orientation.x = q_nb.X();
            imu_data.orientation.y = q_nb.Y();
            imu_data.orientation.z = q_nb.Z();

            imu_data.linear_acceleration.x = accel_b.X();
            imu_data.linear_acceleration.y = accel_b.Y();
            imu_data.linear_acceleration.z = accel_b.Z();

            imu_data.angular_velocity.x = gyro_b.X();
            imu_data.angular_velocity.y = gyro_b.Y();
            imu_data.angular_velocity.z = gyro_b.Z();

            last_imu_time_ = current_time;

            _imu_pub.publish( imu_data );
        }

    }

    void GzModelsCallback( gazebo_msgs::ModelStates model_data ) {
        //cout << "GzModelsCallback" << endl;
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
           
        }
    }

    void GpsCallback(GpsPtr& gps_msg){
        sensor_msgs::NavSatFix gps;
        gps.latitude = gps_msg->latitude_deg();
        gps.longitude = gps_msg->longitude_deg();
        gps.altitude = gps_msg->altitude();
        _gps_pub.publish( gps );
    }

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

        mav_msgs::msgs::CommandMotorSpeed turning_velocities_msg;
        for (int i = 0; i < input_reference_.size(); i++) {
            if (last_actuator_time_ == 0 || (current_time - last_actuator_time_).Double() > 0.2) {
                turning_velocities_msg.add_motor_speed(0);
            } //No power 
            else {
                //cout << "vel " << i << " " <<  _cmd_vel[i] << endl;
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
