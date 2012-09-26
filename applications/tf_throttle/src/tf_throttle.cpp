/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#include <boost/thread/mutex.hpp>

#include <map>  

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>

namespace pr2_interactive_manipulation
{

class TFThrottle
{
protected:

  class TransformInfo {
  public:	   
    geometry_msgs::TransformStamped transform_;
    bool changed_;
    ros::Time last_published_;

    TransformInfo(geometry_msgs::TransformStamped trf) 
    {
      changed_ = true;
      last_published_ = ros::Time(0);
      transform_ = trf;
    }
  };
  
  std::map<std::string, TransformInfo> transforms_;
  ros::NodeHandle root_nh_;
  ros::NodeHandle priv_nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  boost::mutex mutex_;
  ros::Timer sync_timer_;
  ros::Duration republish_time_;
  bool use_diff_;
  double linear_change_threshold_;
  double angular_change_threshold_;

  bool changed(const geometry_msgs::TransformStamped &t1,
	       const geometry_msgs::TransformStamped &t2)
  {
    if (!use_diff_) return true;
    tf::Transform tf1, tf2;
    tf::transformMsgToTF(t1.transform, tf1);
    tf::transformMsgToTF(t2.transform, tf2);

    if (linear_change_threshold_ == 0.0 || angular_change_threshold_ == 0.0 || 
	tf1.getOrigin().distance(tf2.getOrigin()) > linear_change_threshold_ ||
	tf1.getRotation().angle(tf2.getRotation()) > angular_change_threshold_) return true;
    return false;
  }

  void callback(const tf::tfMessageConstPtr &msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    for (size_t i=0; i<msg->transforms.size(); i++)
    {
      std::string key = msg->transforms[i].header.frame_id + msg->transforms[i].child_frame_id;
      std::map<std::string, TransformInfo>::iterator it = transforms_.find(key);
      if (it == transforms_.end())
      {
	transforms_.insert( std::pair<std::string, TransformInfo>(key, TransformInfo(msg->transforms[i])) );
      }
      else
      {
	ros::Time tf_time(msg->transforms[i].header.stamp);
	if (it->second.changed_ && msg->transforms[i].header.stamp > it->second.transform_.header.stamp)
	{
	  it->second.transform_ = msg->transforms[i];
	}
	else if (ros::Duration(tf_time - it->second.last_published_) > ros::Duration(republish_time_))
	{
	  it->second.transform_ = msg->transforms[i];
	  it->second.changed_ = true;
	}
	else if ( changed(it->second.transform_, msg->transforms[i]) )
	{
	  ROS_DEBUG_STREAM("Transform has changed: " << key);
	  it->second.transform_ = msg->transforms[i];
	  it->second.changed_ = true;
	}
      }      
    }
  }

  void sync()
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (transforms_.empty()) return;
    tf::tfMessage msg;
    std::map<std::string, TransformInfo>::iterator it;
    for (it=transforms_.begin(); it!=transforms_.end(); it++)
    {
      if (it->second.changed_)
      {
	it->second.changed_ = false;
	it->second.last_published_ = ros::Time(it->second.transform_.header.stamp);
	msg.transforms.push_back(it->second.transform_);
      }
    }
    pub_.publish(msg);
  }

public:
  TFThrottle() : root_nh_(""), priv_nh_("~")
  {
    std::string pub_topic;
    priv_nh_.param<std::string>("publish_topic", pub_topic, "/tf_throttled");
    pub_ = root_nh_.advertise<tf::tfMessage>(pub_topic, 10);
    std::string sub_topic;
    priv_nh_.param<std::string>("subscribe_topic", sub_topic, "/tf");
    sub_ = root_nh_.subscribe(sub_topic, 10, &TFThrottle::callback, this);    
    double rate;
    priv_nh_.param<double>("rate", rate, 10.0);
    sync_timer_ =  root_nh_.createTimer(ros::Duration(1.0/rate), boost::bind( &TFThrottle::sync, this ) );
    double repub_time;
    priv_nh_.param<double>("republish_time", repub_time, 0.5);
    republish_time_ = ros::Duration(repub_time);    
    priv_nh_.param<bool>("use_diff", use_diff_, true);
    priv_nh_.param<double>("angular_change_threshold", angular_change_threshold_, 0.02);
    priv_nh_.param<double>("linear_change_threshold", linear_change_threshold_, 1.0);
    ROS_INFO("TF throttle started; listening to %s and publishing on %s at %f Hz", sub_topic.c_str(),
	     pub_topic.c_str(), rate);
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_throttle");
  pr2_interactive_manipulation::TFThrottle throttle;
  ros::spin();
  return 0;
}
