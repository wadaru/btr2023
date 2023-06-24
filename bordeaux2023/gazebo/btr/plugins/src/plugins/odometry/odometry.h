/***************************************************************************
 *  odometry.h - provides odometry simulation of object model
 *
 *  Created: Fri Feb 20 18:05:07 2015
 *  Copyright  2015 Stefan Profanter
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
#include <boost/thread/mutex.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <list>
#include <stdio.h>
#include <string.h>

#include <ros/ros.h>
// #include </nav_msgs/Odometry.h>
#include "/opt/ros/noetic/include/nav_msgs/Odometry.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <thread>
#include <tf/transform_broadcaster.h>
#include "ResetOdometry.h"

namespace gazebo {
/**
    * Provides odometry simulation of object model
    * @author Stefan Profanter
    */
class Odometry : public ModelPlugin
{
public:
	Odometry();
	~Odometry();

	//Overridden ModelPlugin-Functions
	virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
	virtual void OnUpdate(const common::UpdateInfo &);
	virtual void Reset();

private:
	/// Pointer to the gazbeo model
	physics::ModelPtr model_;
	/// Pointer to the update event connection
	event::ConnectionPtr update_connection_;
	///Node for communication to fawkes
	transport::NodePtr node_;
	///name of the gps and the communication channel
	std::string name_;

	///time variable to send in intervals
	double last_sent_time_;

	//Odometry Stuff:

	///Mutex for thread safe estimate update
	boost::mutex readingsMutex;

	///Estimated positions
	float estimate_x;
	float estimate_y;
	float estimate_omega;

	///Set odometry callback
	void on_set_odometry_msg(ConstVector3dPtr &msg);

	///Suscriber for SetOdometry
	transport::SubscriberPtr set_odometry_sub_;

	///Functions for sending information to fawkes:
	void send_position();

	///Publisher for Odometry position
	transport::PublisherPtr odometry_pub_;

	ros::Publisher odom_pub;
	std::unique_ptr<ros::NodeHandle> rosNode;
	ros::ServiceServer rosAdv;
	ros::CallbackQueue rosQueue;
	// std::thread rosQueueThread;
	// boost::shared_ptr<boost::thread> rosQueueThread;
	boost::thread rosQueueThread;
	// void OnRosMsg(const nav_msgs::OdometryConstPtr &_msg);
	bool OnRosMsg(robotino_msgs::ResetOdometry::Request &req,
                      robotino_msgs::ResetOdometry::Response &res);
	void QueueThread();
	ros::Time current_time, last_time;
	ros::Rate r();
};
} // namespace gazebo
