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
  
  double                  spin_velocity_;
  double                  coin_radius_;
  int                     total_coins_;
  std::string             robot_name_;
  event::ConnectionPtr    update_connection_;
  common::Time            last_time_;

  /* Constructor */
  public: CoinsManager() : ModelPlugin() {
  }
  
  /* Destructor */
  public: virtual ~CoinsManager()
  {
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
    
    // Set the coins spin velocity
    spin_velocity_ = 1.570796327;
    if (!_sdf->HasElement("spin_velocity"))
      gzwarn << "coins_manager_plugin missing <spin_velocity> parameter, using [" << spin_velocity_ << "] rad."<< endl;
    else
      spin_velocity_ = _sdf->GetElement("spin_velocity")->Get<double>();
      
    for (int i = 0; i < links_.size(); ++i)
    {
      // Randomize the spin direction
      double spin_dir = (rand() % 2 > 0) ? 1.0:-1.0;
      links_[i]->SetAngularVel(math::Vector3(0, 0, spin_velocity_*spin_dir));
    }
    total_coins_ = links_.size();
    gzdbg << "Found [" << total_coins_ << "] coins!" << endl;
    
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      gzerr << "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)" << endl;
      return;
    }
    // Reset Time
    last_time_ = world_->GetSimTime();
    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&CoinsManager::UpdateChild, this));
    
    // Wait for the robot to be spawn
    robot_name_ = "labrob";
    if (!_sdf->HasElement("robot_name"))
      gzwarn << "coins_manager_plugin missing <robot_name> parameter, using [" << robot_name_ << "] rad."<< endl;
    else
      robot_name_ = _sdf->GetElement("robot_name")->Get<std::string>();
  
    BlueMsg("coins_manager_plugin successfully loaded!");
  }
  
  /* Update the plugin */
  // TODO: Need to publish interest information to ROS: Total coins, collected coins, reached the goal?
  private: void UpdateChild()
  {
    math::Pose robot_pose;
    math::Vector3 position_diff;
    double distance;
    common::Time current_time = world_->GetSimTime();
    
    if (current_time > last_time_)
    {
      // Put the 'intelligence' in here!
      if (!robot_model_)
        robot_model_ = world_->GetModel(robot_name_);
      else
      {
        // Check for the position of the robot with respect to the coins
        robot_pose = robot_model_->GetWorldPose();
        for (int i = 0; i < links_.size(); ++i)
        {
          // TODO: Should use GetBoundingBox() and check the intersection
          math::Pose coin_pose = links_[i]->GetWorldPose();
          position_diff = coin_pose.pos - robot_pose.pos;
          distance = sqrt(pow(position_diff.x, 2) + pow(position_diff.y, 2));
          // If the robot is close enough delete the coin
          if (distance < coin_radius_)
          {
            model_->RemoveChild(links_[i]);
            links_.erase(links_.begin() + i);
          }
        }
      }      
      last_time_ = current_time;
    }
  }
  
  /* Custom log messages */
  private: void BlueMsg(std::string msg) {
    gzmsg << "\033[94m" << msg << "\033[0m" << endl; }
  private: void GreenMsg(std::string msg) {
    gzmsg << "\033[92m" << msg << "\033[0m" << endl; }

};
GZ_REGISTER_MODEL_PLUGIN(CoinsManager)
}
