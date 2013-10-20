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
 * RobotMovementPID.h
 * Authors: Andrea Bisson [andreabisson@hotmail.it]
 * 			Stefano Michieletto [stefano.michieletto@dei.unipd.it]
 *
 */

#ifndef ROBOTMOVEMENTPID_H
#define	ROBOTMOVEMENTPID_H

#include <RobotMovementInterface.h>

class RobotMovementPID : public RobotMovementInterface {
public:
    /**
     * RobotMovementPID class constructor. 
     * By default, proportional and derivative gain of the PID controller are set to 1 and 0 respectively.
     * @param jointsNumber
     */
    RobotMovementPID(const std::map<std::string,unsigned int>& joint_map);

    /**
     * RobotMovementPID class destructor.
     */
    virtual ~RobotMovementPID();

    virtual void getNextInterpolationAngles(std::vector<double>& actual_joint_angles) {};

    /**
     * This method updates the speed of the robot joints by taking the actual joints angles and velocities. 
     * The new velocities are saved into the actualJointsVelocities array.
     * @param actualJointsAngles The array of the robot current angles. The dimension of this array must be equal to jointsNumber.
     * @param actualJointsVelocities The array of the robot current velocities. The dimension of this array must be equal to jointsNumber.
     */
    virtual void getNextInterpolationVelocities(std::vector<double>& actual_joint_angles,std::vector<double>& actual_joint_velocities);

    /**
     * This method sets new robot joints angles. 
     * The new target angles are set only if they are different from the previous target angles.
     * @param jointState The jointState message containing the target angles.
     */
    virtual void setMove(const sensor_msgs::JointState::ConstPtr& joint_state);

    /**
     * Changes the proportional gain of the PID controller of the robot.
     * @param pgain The new value of the proportional gain.
     */
    void setPGain(const double pgain);

    /**
     * Changes the derivative gain of the PID controller of the robot.
     * @param dgain The new value of the derivative gain.
     */
    void setDGain(const double dgain);


private:
    std::vector<double>
    	target_joint_angles_;

    std::map<std::string,unsigned int>
    	joint_map_;

    bool
    	joints_modified_;

    double
    	interpolation_time_,
    	dgain_,
    	pgain_;
};

#endif	/* ROBOTMOVEMENTPID_H */

