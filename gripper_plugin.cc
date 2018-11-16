/*
 
  Copyright (C) 2018 Anders Bogga and Emil Johansson. All rights reserved.
 
  BSD license:
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:
  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
  3. Neither the name of the copyright holder nor the names of
     contributors to this software may be used to endorse or promote
     products derived from this software without specific prior written
     permission.
 
  THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
	/// \brief A plugin to control the gripper.	
	class GripperPlugin : public ModelPlugin {
	
	/// \brief Constructor	
	public: GripperPlugin() {}

	public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{
		this->model = _model; 					// Store model pointer
		this->sdf = _sdf;					// Store sdf pointer

		this->joint1 = this->model->GetJoint("cylinder_grip1");
		this->joint2 = this->model->GetJoint("cylinder_grip2");
		this->joint3 = this->model->GetJoint("cylinder_grip3");
		this->joint4 = this->model->GetJoint("cylinder_grip4");
		this->joint5 = this->model->GetJoint("cylinder_grip5");
		this->joint6 = this->model->GetJoint("cylinder_grip6");
		
		this->pid1 = common::PID(10, 1, 1.1);
		this->pid2 = common::PID(10, 1, 1.1);
		this->pid3 = common::PID(10, 1, 1.1);
		this->pid4 = common::PID(10, 1, 1.1);
		this->pid5 = common::PID(10, 1, 1.1);
		this->pid6 = common::PID(10, 1, 1.1);

		this->model->GetJointController()->SetPositionPID(this->joint1->GetScopedName(), this->pid1);
		this->model->GetJointController()->SetPositionPID(this->joint2->GetScopedName(), this->pid2);
		this->model->GetJointController()->SetPositionPID(this->joint3->GetScopedName(), this->pid3);
		this->model->GetJointController()->SetPositionPID(this->joint4->GetScopedName(), this->pid4);
		this->model->GetJointController()->SetPositionPID(this->joint5->GetScopedName(), this->pid5);
		this->model->GetJointController()->SetPositionPID(this->joint6->GetScopedName(), this->pid6);

		this->model->GetJointController()->SetPositionTarget(this->joint1->GetScopedName(), 0);
		this->model->GetJointController()->SetPositionTarget(this->joint2->GetScopedName(), 0);
		this->model->GetJointController()->SetPositionTarget(this->joint5->GetScopedName(), 0);

		this->model->GetJointController()->SetPositionTarget(this->joint3->GetScopedName(), 0);
		this->model->GetJointController()->SetPositionTarget(this->joint4->GetScopedName(), 0);
		this->model->GetJointController()->SetPositionTarget(this->joint6->GetScopedName(), 0);
		
// ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤ ROS-related ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
		if (!ros::isInitialized())					// Initialize ros, if it has not already been initialized.
		{	
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_client",
			ros::init_options::NoSigintHandler);
		}

		this->rosNode.reset(new ros::NodeHandle("gazebo_client"));	// Create a ROS node

		ros::SubscribeOptions so1 =					// Create a ROS topic
			ros::SubscribeOptions::create<std_msgs::Float32>(
			"/" + this->model->GetName() + "/grip_rad", 1,
			boost::bind(&GripperPlugin::OnRosMsg_grip, this, _1),
			ros::VoidPtr(), &this->rosQueue1);

		ros::SubscribeOptions so2 =					// Create a ROS topic
			ros::SubscribeOptions::create<std_msgs::Float32>(
			"/" + this->model->GetName() + "/longgrip_rad", 1,
			boost::bind(&GripperPlugin::OnRosMsg_longgrip, this, _1),
			ros::VoidPtr(), &this->rosQueue2);

		this->rosSub1 = this->rosNode->subscribe(so1);			// Subscribe to the ROS topic.
		this->rosSub2 = this->rosNode->subscribe(so2);			// Subscribe to the ROS topic.

		this->rosQueueThread1 =						// Spin up the queue helper thread.
		  std::thread(std::bind(&GripperPlugin::QueueThread, this));
		this->rosQueueThread2 =						// Spin up the queue helper thread.
		  std::thread(std::bind(&GripperPlugin::QueueThread, this));
// ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤ /ROS-related ¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
	}

	// -------GAZEBO-related-------
	private: physics::JointPtr joint1; 					// Pointer to joint1.
	private: physics::JointPtr joint2; 					// Pointer to joint2.
	private: physics::JointPtr joint3; 					// Pointer to joint3.
	private: physics::JointPtr joint4; 					// Pointer to joint3.
	private: physics::JointPtr joint5; 					// Pointer to joint3.
	private: physics::JointPtr joint6; 					// Pointer to joint3.
	private: common::PID pid1;						// A PID controller for joint1.
	private: common::PID pid2;						// A PID controller for joint2.
	private: common::PID pid3;						// A PID controller for joint3.
	private: common::PID pid4;						// A PID controller for joint3.
	private: common::PID pid5;						// A PID controller for joint3.
	private: common::PID pid6;						// A PID controller for joint3.
	
	private: physics::ModelPtr model; 					// Pointer to the model.
	private: sdf::ElementPtr sdf;
	
	// -------ROS-related-------
	private: std::unique_ptr<ros::NodeHandle> rosNode; 			// A node use for ROS transport
	private: ros::Subscriber rosSub1;					// A ROS subscriber
	private: ros::Subscriber rosSub2;					// A ROS subscriber
	private: ros::CallbackQueue rosQueue1;					// A ROS callbackqueue that helps process messages
	private: ros::CallbackQueue rosQueue2;					// A ROS callbackqueue that helps process messages
	private: std::thread rosQueueThread1;					// A thread the keeps running the rosQueue
	private: std::thread rosQueueThread2;					// A thread the keeps running the rosQueue

	// ROS callback function	
	public: void OnRosMsg_longgrip(const std_msgs::Float32ConstPtr &_msg)
	{
		this->model->GetJointController()->SetPositionTarget(this->joint1->GetScopedName(), _msg->data);
		this->model->GetJointController()->SetPositionTarget(this->joint2->GetScopedName(), _msg->data);
		this->model->GetJointController()->SetPositionTarget(this->joint5->GetScopedName(), _msg->data);
	}

	public: void OnRosMsg_grip(const std_msgs::Float32ConstPtr &_msg)
	{
		this->model->GetJointController()->SetPositionTarget(this->joint3->GetScopedName(), _msg->data);
		this->model->GetJointController()->SetPositionTarget(this->joint4->GetScopedName(), _msg->data);
		this->model->GetJointController()->SetPositionTarget(this->joint6->GetScopedName(), _msg->data);
	}

	// ROS helper function that processes messages
	private: void QueueThread()
	{
		static const double timeout = 0.01;
		while (this->rosNode->ok())
		{
			this->rosQueue1.callAvailable(ros::WallDuration(timeout));
			this->rosQueue2.callAvailable(ros::WallDuration(timeout));
		}
	}

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GripperPlugin)
}
#endif
