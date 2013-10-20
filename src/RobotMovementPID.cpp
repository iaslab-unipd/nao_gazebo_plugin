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
 * RobotMovementPID.cpp
 * Authors: Andrea Bisson [andreabisson@hotmail.it]
 * 			Stefano Michieletto [stefano.michieletto@dei.unipd.it]
 *
 */

#include <RobotMovementPID.h>


RobotMovementPID::RobotMovementPID(const std::map<std::string,unsigned int>& joint_map)
{
	joint_map_ = joint_map;
    for (int i = 0; i < joint_map_.size(); i++)
    {
        target_joint_angles_.push_back(0.0);
    }

    joints_modified_ = false;
    dgain_ = 0.0;
    pgain_ = 1.0;
    interpolation_time_ = 0;
}

RobotMovementPID::~RobotMovementPID() {
}

void RobotMovementPID::setDGain(const double dgain)
{
    dgain_ = dgain;
}


void RobotMovementPID::setPGain(const double pgain)
{
    pgain_ = pgain;
}


void RobotMovementPID::getNextInterpolationVelocities(std::vector<double>& actual_joint_angles,std::vector<double>& actual_joint_velocities)
{
	double
		damping_force,
		diff_force;

    for (unsigned int i = 0; i < joint_map_.size(); i++)
    {
        damping_force = dgain_ * (0 - actual_joint_velocities[i]);
        diff_force = pgain_ * (target_joint_angles_[i] - actual_joint_angles[i]);

        actual_joint_velocities[i] = diff_force + damping_force;
    }
}


void RobotMovementPID::setMove(const sensor_msgs::JointState::ConstPtr& joint_state)
{
    for (unsigned int i = 0; i < joint_state->position.size(); i++)
    {
        if (fabs(target_joint_angles_[joint_map_.at(joint_state->name[i])] - joint_state->position[i]) > EPSILON)
        {
            joints_modified_ = true;
            break;
        }
    }
    if (!joints_modified_)
        return;

    for (unsigned int i = 0; i < joint_state->position.size(); i++)
    {
    	target_joint_angles_[joint_map_.at(joint_state->name[i])] = joint_state->position[i];
    }
}
