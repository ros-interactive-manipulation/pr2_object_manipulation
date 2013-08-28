/*********************************************************************
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

// Author(s): Kaijen Hsiao

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <object_manipulation_msgs/FindContainerAction.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Geometry>
#include <string>
#include <vector>

typedef pcl::PointXYZRGB PointT;

//* A node for finding vertical and horizontal surfaces in point clouds
/**
* The FindContainerNode provides an action for splitting point clouds
* into vertical and horizontal surfaces (the 'walls' and 'contents' of
* a container).
*/

class FindContainerNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  actionlib::SimpleActionServer<object_manipulation_msgs::FindContainerAction> as_;
  std::string action_name_;
  tf::TransformListener tfl_;
  ros::Publisher pub_cloud_, pub_contents_, pub_container_;
  ros::Publisher pub_marker_, pub_clusters_;

  // the direction taken to be 'vertical' (can really be horizontal in the world)
  geometry_msgs::Vector3 opening_dir_;

public:
  explicit FindContainerNode(std::string name):
    pnh_("~"),
    as_(nh_, name, boost::bind(&FindContainerNode::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    pub_cloud_ = 
      pnh_.advertise<sensor_msgs::PointCloud2>("cloud", 1, true);
    pub_contents_ = 
      pnh_.advertise<sensor_msgs::PointCloud2>("contents", 1, true);
    pub_container_ = 
      pnh_.advertise<sensor_msgs::PointCloud2>("container", 1, true);
    pub_marker_ = 
      pnh_.advertise<visualization_msgs::Marker>("box", 1, true);
    pub_clusters_ = 
      pnh_.advertise<visualization_msgs::MarkerArray>("clusters_array", 
                                                      100, true);
    ROS_INFO("%s: Server ready.", action_name_.c_str());
  }

  ~FindContainerNode(void)
  {
  }

  /*!
  Split a PointCloud into parts that are closer to horizontal and parts
  that are closer to vertical
  */
  void splitCloudRegions(const pcl::PointCloud<PointT>::Ptr &cloud_in,
			 const pcl::PointCloud<PointT>::Ptr &horizontal_cloud,
			 const pcl::PointCloud<PointT>::Ptr &vertical_cloud)
  {
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Estimate point normals
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::MovingLeastSquares<PointT, pcl::PointNormal> ne;
    ne.setSearchMethod(tree);
    ne.setComputeNormals(true);
    ne.setInputCloud(cloud_in);
    ne.setSearchRadius(0.012);
    ne.process(mls_points);

    for (size_t i = 0; i < mls_points.points.size(); i++)
    {

      PointT point;
      point.x = mls_points.points[i].x;
      point.y = mls_points.points[i].y;
      point.z = mls_points.points[i].z;
      //point.rgba = mls_points.points[i].rgba;

      // check if opening_dir is within 45 deg of horizontal
      if (fabs(mls_points[i].normal_x*opening_dir_.x + 
	       mls_points[i].normal_y*opening_dir_.y + 
	       mls_points[i].normal_z*opening_dir_.z) > 0.7)
      {
        horizontal_cloud->points.push_back(point);
      }
      else
      {
        vertical_cloud->points.push_back(point);
      }
    }

    horizontal_cloud->width = horizontal_cloud->points.size();
    horizontal_cloud->height = 1;
    horizontal_cloud->is_dense = false;

    vertical_cloud->width = vertical_cloud->points.size();
    vertical_cloud->height = 1;
    vertical_cloud->is_dense = false;

  }

  /*!
  Create a filtering object and use it to remove outliers
  */
  void removeOutliers(const pcl::PointCloud<PointT>::Ptr &cloud_in,
                      const pcl::PointCloud<PointT>::Ptr &cloud_out)
  {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_in);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_out);
  }

  /*!
  Use Euclidean clustering to find clusters in a point cloud
  */
  void findClusters(const pcl::PointCloud<PointT>::Ptr &cloud, 
		    std::vector< pcl::PointCloud<PointT>::Ptr > &clusters)
  {
    if (cloud->points.size() == 0) return;

    // Create a KdTree for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.01);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = 
	   cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(*cloud, *it, *cloud_cluster);
      clusters.push_back(cloud_cluster);
      j++;
    }
  }

  /*!
  Make an rviz Marker out of a pcl PointCloud
  */
  visualization_msgs::Marker 
  makeMarkerFromCloud(const pcl::PointCloud<PointT>::Ptr &cloud_ptr, 
		      const std::string &ns, int id = 0, float scale = 0.03)
  {
    pcl::PointCloud<PointT>& cloud = *cloud_ptr;
    ROS_DEBUG("Making a marker with %d points.", (int)cloud.points.size());

    visualization_msgs::Marker marker;
    marker.action = marker.ADD;
    marker.header = cloud.header;
    marker.id = id;
    marker.ns = ns;
    marker.pose.orientation.w = 1;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.frame_locked = false;

    if (cloud.points.size())
    {
      marker.scale.x = scale;
      marker.scale.y = scale;
      marker.scale.z = scale;
      marker.type = visualization_msgs::Marker::SPHERE_LIST;

      int num_points = cloud.points.size();
      marker.points.resize(num_points);
      marker.colors.resize(num_points);

      for (int i = 0; i < num_points; i++)
      {
        marker.points[i].x = cloud.points[i].x;
        marker.points[i].y = cloud.points[i].y;
        marker.points[i].z = cloud.points[i].z;
        marker.colors[i].r = cloud.points[i].r/255.;
        marker.colors[i].g = cloud.points[i].g/255.;
        marker.colors[i].b = cloud.points[i].b/255.;
        marker.colors[i].a = 1.0;
      }
    }

    return marker;
  }

  /*!
  Find the axis-aligned bounding box of a PointCloud
  */
  void findBoundingBox(const pcl::PointCloud<PointT>::Ptr &cloud,
                       geometry_msgs::Vector3 &box_dims,
                       geometry_msgs::PoseStamped &box_pose)
  {
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    box_dims.x = max_pt.x - min_pt.x;
    box_dims.y = max_pt.y - min_pt.y;
    box_dims.z = max_pt.z - min_pt.z;

    box_pose.pose.orientation.w = 1.0;
    box_pose.header = cloud->header;
    box_pose.pose.position.x = (max_pt.x + min_pt.x)/2.0;
    box_pose.pose.position.y = (max_pt.y + min_pt.y)/2.0;
    box_pose.pose.position.z = (max_pt.z + min_pt.z)/2.0;
  }

  /*!
  Filter out points in a PointCloud that are not within axis-aligned
  dims around a pose
  */
  int boxFilter(const pcl::PointCloud<PointT>::Ptr &cloud, 
		const pcl::PointCloud<PointT>::Ptr &cloud_filtered,
		const geometry_msgs::Vector3 &dims,
		const geometry_msgs::Pose &pose)
  {
    Eigen::Vector4f center(pose.position.x, pose.position.y, 
			   pose.position.z, 0);

    // TODO: maybe allow for box pose that is not aligned with its header frame
    Eigen::Vector4f min_pt;
    min_pt = center - Eigen::Vector4f(dims.x/2, dims.y/2, dims.z/2, 0);
    Eigen::Vector4f max_pt;
    max_pt = center + Eigen::Vector4f(dims.x/2, dims.y/2, dims.z/2, 0);

    std::vector<int> indices;
    pcl::getPointsInBox(*cloud, min_pt, max_pt, indices);

    pcl::copyPointCloud(*cloud, indices, *cloud_filtered);
    return cloud_filtered->points.size();
  }

  /*!
  Draw an rviz box 
  */
  void drawBox(const geometry_msgs::Vector3 &box_dims,
               const geometry_msgs::PoseStamped &box_pose, 
	       const std::string &ns = "box")
  {
    visualization_msgs::Marker marker;
    marker.header = box_pose.header;
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE; 
    marker.action = (int32_t)visualization_msgs::Marker::ADD;
    marker.pose = box_pose.pose;
    marker.scale = box_dims;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(0);
    marker.frame_locked = false;
    pub_marker_.publish(marker);
  }

  /*!
  Callback function for the FindContainerAction
  */
  void 
  executeCB(const object_manipulation_msgs::FindContainerGoalConstPtr &goal)
  {
    object_manipulation_msgs::FindContainerResult result;

    ROS_INFO("%s: Processing a goal request!", action_name_.c_str());

    opening_dir_ = goal->opening_dir;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud_ds(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr contents(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr container(new pcl::PointCloud<PointT>());

    if (goal->cloud.data.size() == 0)
    {
      ROS_ERROR("%s: No points in input cloud!", action_name_.c_str());
      as_.setAborted();
      return;
    }

    // Remove NaNs from the cloud
    pcl::fromROSMsg(goal->cloud, *cloud);
    std::vector<int> non_NaN_indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, non_NaN_indices);
    cloud->header.stamp = ros::Time(0);

    // Transform the cloud into box_pose frame
    tf::StampedTransform T;
    Eigen::Matrix4f M;
    tfl_.waitForTransform(goal->box_pose.header.frame_id, 
			  cloud->header.frame_id, ros::Time(0), 
			  ros::Duration(2.0));
    tfl_.lookupTransform(goal->box_pose.header.frame_id, 
			 cloud->header.frame_id, ros::Time(0), T);
    pcl_ros::transformAsMatrix(T, M);
    pcl::transformPointCloud(*cloud, *cloud, M);
    cloud->header.frame_id = goal->box_pose.header.frame_id;

    // Draw the region of interest box in rviz
    drawBox(goal->box_dims, goal->box_pose, "input_bounds");

    // Filter out points not in the region of interest
    if (!boxFilter(cloud, cloud_filtered, goal->box_dims, goal->box_pose.pose))
    {
      ROS_ERROR("%s: No cloud points found in input bounding box.", 
		action_name_.c_str());
      as_.setAborted();
      return;
    }

    contents->header = goal->box_pose.header;
    container->header = goal->box_pose.header;
    cloud_ds->header = goal->box_pose.header;
    cloud_filtered->header = goal->box_pose.header;

    // Down-sample the cloud on a grid
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(cloud_filtered);
    vox.setLeafSize(0.002, 0.002, 0.002);
    vox.setFilterFieldName("z");
    vox.setFilterLimits(-100, 100);
    vox.filter(*cloud_ds);

    // Remove outlier points (speckles)
    removeOutliers(cloud_ds, cloud_ds);
    if (cloud_ds->points.size() == 0)
    {
      ROS_ERROR("%s: No cloud points remain after outlier removal.", 
		action_name_.c_str());
      as_.setSucceeded(result);
    }

    // Estimate normals and separate horizontal and vertical surfaces
    splitCloudRegions(cloud_ds, contents, container);

    // Find the bounding box for all the points
    findBoundingBox(cloud_ds, result.box_dims, result.box_pose);

    // Split the contents into clusters 
    std::vector< pcl::PointCloud<PointT>::Ptr > clusters;
    findClusters(contents, clusters);
    result.clusters.resize(clusters.size());

    // Publish the clouds and box
    visualization_msgs::MarkerArray cluster_markers;
    for (size_t i = 0; i < clusters.size(); i++)
    {
      pcl::toROSMsg(*(clusters[i]), result.clusters[i]);
      cluster_markers.markers.push_back(makeMarkerFromCloud(clusters[i], 
							    "clusters", 
							    i, 0.003));
    }
    pub_clusters_.publish(cluster_markers);
    drawBox(result.box_dims, result.box_pose, "container_box");
    pcl::toROSMsg(*contents, result.contents);
    pcl::toROSMsg(*container, result.container);
    pub_contents_.publish(result.contents);
    pub_container_.publish(result.container);

    // Return the result
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    as_.setSucceeded(result);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_container_action");

  FindContainerNode find_it(ros::this_node::getName());
  ros::spin();

  return 0;
}
