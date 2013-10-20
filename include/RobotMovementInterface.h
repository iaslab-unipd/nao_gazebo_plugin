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
 * RobotMovementInterface.h
 * Authors: Andrea Bisson [andreabisson@hotmail.it]
 * 			Stefano Michieletto [stefano.michieletto@dei.unipd.it]
 *
 */

#ifndef ROBOTMOVEMENTINTERFACE_H
#define	ROBOTMOVEMENTINTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

#define DEG2RAD 0.017453293
#define RAD2DEG 57.295779513
#define EPSILON 0.005

class RobotMovementInterface {
    
public:
    virtual ~RobotMovementInterface(){};
    
    /**
     * This method must calculate the new joints angles values.
     * @param actualJointsAngles This joints array contains the actual joint model values. The method updates this array with new calculated values.
     */
    virtual void getNextInterpolationAngles(std::vector<double>& actual_joint_angles) = 0;
    
    /**
     * This method must calculate the new joints velocity values.
     * @param actualJointsAngles The current joints angles array.
     * @param actualJointsVelocities The new joints velocity computed. This method saves the results in this variable.
     */
    virtual void getNextInterpolationVelocities(std::vector<double>& actual_joint_angles, std::vector<double>& actual_joint_velocities) = 0;
    
    /**
     * This method must set up the new requested joints values.
     * @param jointState The message that contains the new joints values.
     */
    virtual void setMove(const sensor_msgs::JointState::ConstPtr& joint_state) = 0;
};

#endif	/* ROBOTMOVEMENTINTERFACE_H */
