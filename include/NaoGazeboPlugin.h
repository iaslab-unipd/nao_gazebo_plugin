/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-, Stefano Michieletto
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * NaoGazeboPlugin.h
 * Authors:	Davide Zanin [davidezanin@gmail.com]
 * 			Stefano Michieletto [stefano.michieletto@dei.unipd.it]
 *
 */

#ifndef NAO_GAZEBO_PLUGIN_H
#define NAO_GAZEBO_PLUGIN_H

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <controller_manager/controller_manager.h>
#include <sdf/sdf_config.h>

// Std
#include <stdio.h>
#include <sstream>
#include <cmath>
#include <unistd.h>
#include <set>
#include <iostream>
#include <vector>

// Boost
#include <boost/unordered_map.hpp>

// NAO Gazebo Plugin
#include <RobotMovementPID.h>


namespace gazebo{

class NaoGazeboPlugin : public ModelPlugin{

public:

	/**
	 * Gazebo's plugin Constructor
	 */
	NaoGazeboPlugin();

	/**
	 * Gazebo's plugin Destructor
	 */
	virtual ~NaoGazeboPlugin();

	/**
	 * This method loads the Nao's robot model and the Gazebo's plugin
	 * @param _parent Pointer to the model
	 * @param _sdf Pointer to the sdf world file
	 */
        virtual void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

	/**
	 * This method updates the model position inside Gazebo
	 */
	virtual void OnUpdate();


private:
	/**
	 * This callback sets new target angles for the robot joints
	 * @param jointState The jointState message containing the new target angles for the robot
	 */
	void cmdCallback(const sensor_msgs::JointState::ConstPtr& joint_state);


private:
	ros::Publisher
//		angles_pub_,
		feedback_states_pub_;

	ros::Subscriber
		cmd_angles_sub_;

	std::vector<double>
		joint_angles_,
		joint_velocities_;

	trajectory_msgs::JointTrajectoryPoint 
		desired_,
		actual_;

	control_msgs::FollowJointTrajectoryFeedback 
		feedback_;

	ros::Time
		start_movement_;

	std::map<std::string,unsigned int>
		joint_map_;

	RobotMovementPID 
		*pid_;

	// Gazebo's plugin variables
	// \brief Pointer to the model
	physics::ModelPtr
		parent_model_;
	// \brief urdf model
	urdf::Model
		urdf_model_;
	std::vector<gazebo::physics::JointPtr>
		joints_;
	// \brief pointer to ros node
	ros::NodeHandle
		*rosnode_;
	// \brief set topic name of robot description parameter
	std::string
		robot_namespace_;
	// \brief Pointer to the update event connection
	event::ConnectionPtr
		update_connection_;

	double 
		pgain_,
		dgain_,
		update_rate_,
		prev_update_time_;

};
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(NaoGazeboPlugin)

}

#endif // NAO_GAZEBO_PLUGIN_H
