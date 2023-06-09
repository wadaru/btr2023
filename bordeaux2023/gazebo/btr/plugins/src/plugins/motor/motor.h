/***************************************************************************
 *  motor.h - Plugin for controling a model through a simulated motor
 *
 *  Created: Wed Jan 29 16:08:32 2014
 *  Copyright  2014  Frederik Zwilling
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <list>
#include <stdio.h>
#include <string.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <thread>
// #include "std_msgs/Float32.h"


namespace gazebo {
/** @class Motor
   * Motor plugin for Gazebo
   * @author Frederik Zwilling
   */
class Motor : public ModelPlugin
{
public:
	///Constructor
	Motor();

	///Destructor
	~Motor();

	//Overridden ModelPlugin-Functions
	virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
	virtual void OnUpdate(const common::UpdateInfo &);
	virtual void Reset();

private:
	/// Pointer to the Gazebo model
	physics::ModelPtr model_;
	/// Pointer to the update event connection
	event::ConnectionPtr update_connection_;
	///Node for communication to fawkes
	transport::NodePtr node_;
	///name of the motor and the communication channel
	std::string name_;

	//Motor Stuff:
	void on_motor_move_msg(ConstVector3dPtr &msg);

	///Suscriber for MotorMove Interfaces from Fawkes
	transport::SubscriberPtr motor_move_sub_;

	//current movement commands:
	float vx_;
	float vy_;
	float vomega_;

	std::unique_ptr<ros::NodeHandle> rosNode;
	ros::Subscriber rosSub;
	ros::CallbackQueue rosQueue;
	// std::thread rosQueueThread;
	boost::thread rosQueueThread;
	// void OnRosMsg(const std_msgs::Float32ConstPtr &_msg);
	void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg);
	void QueueThread();

};
} // namespace gazebo

