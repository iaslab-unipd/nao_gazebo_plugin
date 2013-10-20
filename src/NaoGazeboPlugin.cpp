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
 * NaoGazeboPlugin.cpp
 * Authors:	Davide Zanin [davidezanin@gmail.com]
 * 			Stefano Michieletto [stefano.michieletto@dei.unipd.it]
 *
 */

#include <NaoGazeboPlugin.h>

static const double JOINT_MAX_FORCE = 100.0;
static const std::string PATH_URDF = "/urdf/nao_robot_v4.urdf";
static const std::string PATH_NAO_DESCRIPTION = "nao_description";

namespace gazebo {

NaoGazeboPlugin::NaoGazeboPlugin() : pgain_(4.0), dgain_(0.0)
{
	rosnode_ = NULL;
	pid_ = NULL;
	update_rate_ = 0;
	prev_update_time_ = 0;
};

NaoGazeboPlugin::~NaoGazeboPlugin()
{
	delete pid_;
	delete rosnode_;
}

void NaoGazeboPlugin::cmdCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
	pid_->setMove(joint_state);

	for (unsigned int i = 0; i < joint_state->position.size(); i++)
	{
        desired_.positions[joint_map_.at(joint_state->name[i])] = joint_state->position[i];
    }

    feedback_.desired = desired_;
	start_movement_ = ros::Time::now();
}

void NaoGazeboPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf){

	ROS_INFO("Gazebo's plugin is loading...");

	parent_model_ = parent;
	if (!parent_model_)
		ROS_ERROR("NaoGazeboPlugin controller requires a Model as its parent");

	if (!sdf->HasElement("robotNamespace"))
		ROS_ERROR("Missing <robotNamespace>");
	else
		robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();

	int argc = 0;
	char** argv = NULL;

	//load URDF and extract joint
	if (!urdf_model_.initFile(ros::package::getPath(PATH_NAO_DESCRIPTION) + PATH_URDF))
	{
		ROS_ERROR("load urdf error!");
	}

	// set the update rate
	update_rate_ = common::Time(0, common::Time::SecToNano(0.001)).Double();
	// initialize the prevUpdateTime
	prev_update_time_ = common::Time::GetWallTime().Double();

	for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator joint_it = urdf_model_.joints_.begin(); joint_it != urdf_model_.joints_.end(); joint_it++)
	{
		// Save the joint name
		std::string joint_name = (*joint_it).first;
		ROS_INFO("name=%s", joint_name.c_str());

		if ((*joint_it).second->type != urdf::Joint::UNKNOWN && (*joint_it).second->type != urdf::Joint::FIXED)
		{
			feedback_.joint_names.push_back(joint_name);
			joint_map_.insert(std::pair<std::string,unsigned int>(joint_name,feedback_.joint_names.size()-1));
			gazebo::physics::JointPtr gazebo_joint = parent_model_->GetJoint(joint_name);
			joints_.push_back(gazebo_joint);
			if (gazebo_joint)
			{
				ROS_INFO("Add joint: %s", joint_name.c_str());
				joint_angles_.push_back(0.0);
				joint_velocities_.push_back(0.0);

				gazebo_joint->SetVelocity(0,0);
				gazebo_joint->SetMaxForce(0, JOINT_MAX_FORCE);
				gazebo::math::Angle angle(joint_angles_[joint_angles_.size()-1]);
				gazebo_joint->SetAngle(0, angle);
			}
           	else
           	{
				ROS_WARN("A joint named \"%s\" is not part of Mechanism Controlled joints.\n", joint_name.c_str());
           	}
		}
	}

	ROS_INFO("Total inserted joints: %lu", joints_.size());

	ros::init(argc,argv,"gazebo_ros",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
	rosnode_ = new ros::NodeHandle(robot_namespace_); // namespace comes from gazebo_model spawner

	//resize of the actual and desired msg with Nao's joints number
	actual_.positions.resize(joints_.size());
	desired_.positions.resize(joints_.size());
	start_movement_ = ros::Time::now();

	//angles_pub_ = rosnode_->advertise<std_msgs::Float64>("nao/angles", 1);
	feedback_states_pub_ = rosnode_->advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
	cmd_angles_sub_ = rosnode_->subscribe<sensor_msgs::JointState>("joint_states", 1, &NaoGazeboPlugin::cmdCallback, this);

	pid_ = new RobotMovementPID(joint_map_);
	pid_->setPGain(pgain_);
	pid_->setDGain(dgain_);

	ROS_INFO("Starting NaoGazeboPlugin plugin in namespace: %s", robot_namespace_.c_str());
	update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&NaoGazeboPlugin::OnUpdate, this));
	ROS_INFO("Gazebo's plugin loading ended!");
}


void NaoGazeboPlugin::OnUpdate()
{
	//increases Gazebo's real time factor
	if (common::Time::GetWallTime().Double() - prev_update_time_ < update_rate_)
	{
		return;
	}

	pid_->getNextInterpolationVelocities(joint_angles_, joint_velocities_);

	for (unsigned int i = 0; i < joints_.size(); i++)
	{
		physics::JointPtr joint = joints_[i];

		joint->SetForce(0, joint_velocities_[i]);
		joint->SetVelocity(0, joint_velocities_[i]);

		joint_angles_[i] = joint->GetAngle(0).Radian();
		joint_velocities_[i] = joint->GetVelocity(0);

		actual_.positions[i] = joint->GetAngle(0).Radian();
	}

	actual_.time_from_start = ros::Time::now() - start_movement_;
	feedback_.actual = actual_;
	feedback_states_pub_.publish(feedback_);

	prev_update_time_ = common::Time::GetWallTime().Double();
}

}
