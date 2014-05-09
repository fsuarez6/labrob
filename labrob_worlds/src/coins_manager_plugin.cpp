/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014
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
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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
 *********************************************************************/

/**
 *  \author Francisco Suarez Ruiz
 *  \desc   Plugin for animation of all the coins within the maze
 */
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <labrob_msgs/StatusMessage.h>


#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

using std::endl;

namespace gazebo
{
class CoinsManager : public ModelPlugin
{
private:
  physics::WorldPtr       world_;
  physics::ModelPtr       model_;
  physics::ModelPtr       robot_model_;
  physics::Link_V         links_;
  physics::Joint_V        joints_;
  
  // ROS STUFF
  ros::NodeHandle*        rosnode_;
  ros::Publisher          status_publisher_;
  ros::Publisher          coins_publisher_;
  ros::Publisher          complete_publisher_;
  
  labrob_msgs::StatusMessage  status_msg_;
  
  double                  spin_velocity_;
  double                  coin_radius_;
  int                     total_coins_;
  bool                    course_complete_;
  std::string             plugin_namespace_;
  std::string             robot_name_;
  std::string             goal_coin_;
  event::ConnectionPtr    update_connection_;
   // Update Rate
  double update_rate_;
  double update_period_;
  common::Time            last_update_time_;

  /* Constructor */
  public: CoinsManager() : ModelPlugin() {
  }
  
  /* Destructor */
  public: virtual ~CoinsManager()
  {
    delete rosnode_;
    event::Events::DisconnectWorldUpdateBegin(update_connection_);
  }
  
  /* Load the plugin */
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model and world
    model_ = _parent;
    world_ = model_->GetWorld();
    // Get joints and links
    joints_ = model_->GetJoints();
    links_ = model_->GetLinks();
    // Set the coin radius
    coin_radius_ = 1.0;
    
    // Read all the plugin parameters
    robot_name_ = "labrob";
    if (!_sdf->HasElement("robot_name"))
      ROS_WARN_STREAM("coins_manager_plugin missing <robot_name> parameter, using [" << robot_name_ << "]");
    else
      robot_name_ = _sdf->GetElement("robot_name")->Get<std::string>();
      
    goal_coin_ = "coin_1";
    if (!_sdf->HasElement("goal_coin"))
      ROS_WARN_STREAM("coins_manager_plugin missing <goal_coin> parameter, using [" << goal_coin_ << "]");
    else
      goal_coin_ = _sdf->GetElement("goal_coin")->Get<std::string>();
    
    update_rate_ = 1.0;
    if (!_sdf->HasElement("update_rate"))
      ROS_WARN("coins_manager_plugin missing <update_rate>, defaults to %f", update_rate_);
    else
      update_rate_ = _sdf->GetElement("update_rate")->Get<double>();

    spin_velocity_ = 1.570796327;
    if (!_sdf->HasElement("spin_velocity"))
      ROS_WARN_STREAM("coins_manager_plugin missing <spin_velocity> parameter, using [" << spin_velocity_ << "] rad/s");
    else
      spin_velocity_ = _sdf->GetElement("spin_velocity")->Get<double>();
      
    // Make spin the coins
    for (int i = 0; i < links_.size(); ++i)
    {
      // Randomize the spin direction
      double spin_dir = (rand() % 2 > 0) ? 1.0:-1.0;
      links_[i]->SetAngularVel(math::Vector3(0, 0, spin_velocity_*spin_dir));
    }
    total_coins_ = links_.size();
    ROS_DEBUG_STREAM("Found [" << total_coins_ << "] coins!");
    
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    
    plugin_namespace_ = "/";
    rosnode_ = new ros::NodeHandle(plugin_namespace_);
    
    // ROS: Setup publishers
    coins_publisher_ = rosnode_->advertise<std_msgs::Int32>("coins_remaining", 1);
    complete_publisher_ = rosnode_->advertise<std_msgs::Bool>("course_complete", 1);
    status_publisher_ = rosnode_->advertise<labrob_msgs::StatusMessage>("ed27fa22584967f31278es75efd0e16", 1);
    
    // Initialize update rate stuff
    
    if (update_rate_ > 0.0)
      update_period_ = 1.0 / update_rate_;
    else
      update_period_ = 0.0;
    last_update_time_ = world_->GetSimTime();
    
    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&CoinsManager::UpdateChild, this));
      
    ROS_INFO("Starting CoinsManager Plugin (ns = %s)!", plugin_namespace_.c_str());
    course_complete_ = false;
    
    // Populate the initial status msg
    status_msg_.collected = 0;
    status_msg_.total = links_.size();
    status_msg_.complete = false;
  
  }
  
  /* Update the plugin */
  private: void UpdateChild()
  {
    math::Pose robot_pose;
    math::Vector3 position_diff;
    double distance;
    common::Time current_time = world_->GetSimTime();
    double seconds_since_last_update = (current_time - last_update_time_).Double();
    
    if (seconds_since_last_update > update_period_)
    {
      // Put the 'intelligence' in here!
      if (!robot_model_)    // Wait for the robot to be spawn
        robot_model_ = world_->GetModel(robot_name_);
      else
      {
        // Check for the position of the robot with respect to the coins
        robot_pose = robot_model_->GetWorldPose();
        for (int i = 0; i < links_.size(); ++i)
        {
          // TODO: Should use links_[i]->GetBoundingBox() and check the intersection
          math::Pose coin_pose = links_[i]->GetWorldPose();
          position_diff = coin_pose.pos - robot_pose.pos;
          distance = sqrt(pow(position_diff.x, 2) + pow(position_diff.y, 2));
          // If the robot is close enough delete the coin
          if (distance < coin_radius_)
          {
            // Check if We got the goal coin
            if (goal_coin_.compare(links_[i]->GetName()) == 0)
            {
              course_complete_ = true;
              status_msg_.complete = true;
            }
            model_->RemoveChild(links_[i]);
            links_.erase(links_.begin() + i);
            status_msg_.collected += 1;
          }
        }
      }
      // Publish topics
      std_msgs::Int32 coins_msg;
      coins_msg.data = links_.size();
      coins_publisher_.publish(coins_msg);
      std_msgs::Bool complete_msg;
      complete_msg.data = course_complete_;
      complete_publisher_.publish(complete_msg);
      status_publisher_.publish(status_msg_);
      
      last_update_time_+= common::Time(update_period_);
    }
  }

};
GZ_REGISTER_MODEL_PLUGIN(CoinsManager)
}
