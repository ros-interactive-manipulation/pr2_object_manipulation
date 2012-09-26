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

class TFRepublish
{
protected:

  class TransformInfo {
  public:	   
    geometry_msgs::TransformStamped transform_;
    ros::Time last_received_;

    TransformInfo(geometry_msgs::TransformStamped trf) 
    {
      last_received_ = ros::Time(0);
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
  ros::Duration expiration_time_;

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
	if (msg->transforms[i].header.stamp > it->second.transform_.header.stamp)
	{
	  it->second.transform_ = msg->transforms[i];
	}
      }      
    }
  }

  void sync()
  {
    mutex_.lock();
    tf::tfMessage msg;
    ros::Time time_now = ros::Time::now();
    ros::Time time_latest = ros::Time(0);
    //prune the list of transforms, and also find out the most recent one
    std::map<std::string, TransformInfo>::iterator it = transforms_.begin();
    while (it!=transforms_.end())
    {
      if ( time_now - it->second.transform_.header.stamp > expiration_time_ )
      {
        ROS_WARN_STREAM("TF Republisher: dropping transform " << it->first);
        transforms_.erase(it++);
      }
      else if ( (it++)->second.transform_.header.stamp > time_latest )
      {
        time_latest = it->second.transform_.header.stamp;
      }
    }
    if (transforms_.empty()) {mutex_.unlock(); return;}
    //publish all transforms we have; all get the time stamp of the most recent one
    for (it=transforms_.begin(); it!=transforms_.end(); it++)
    {
      msg.transforms.push_back(it->second.transform_);
      msg.transforms.back().header.stamp = time_latest;
    }
    mutex_.unlock();
    pub_.publish(msg);
  }

public:
  TFRepublish() : root_nh_(""), priv_nh_("~")
  {
    std::string pub_topic;
    priv_nh_.param<std::string>("publish_topic", pub_topic, "/tf_republished");
    pub_ = root_nh_.advertise<tf::tfMessage>(pub_topic, 10);
    std::string sub_topic;
    priv_nh_.param<std::string>("subscribe_topic", sub_topic, "/tf_throttled");
    sub_ = root_nh_.subscribe(sub_topic, 10, &TFRepublish::callback, this);    
    double rate;
    priv_nh_.param<double>("rate", rate, 100.0);
    sync_timer_ =  root_nh_.createTimer(ros::Duration(1.0/rate), boost::bind( &TFRepublish::sync, this ) );
    double expiration_time;
    priv_nh_.param<double>("expiration_time", expiration_time, 1.1);
    expiration_time_ = ros::Duration(expiration_time);    

    ROS_INFO("TF republish started; listening to %s and publishing on %s at %f Hz", sub_topic.c_str(),
	     pub_topic.c_str(), rate);
  }


};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_republish");
  pr2_interactive_manipulation::TFRepublish republish;
  ros::spin();
  return 0;
}
