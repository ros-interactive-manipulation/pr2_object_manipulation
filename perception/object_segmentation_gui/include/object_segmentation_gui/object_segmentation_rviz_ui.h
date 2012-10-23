/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OBJECT_SEGMENTATION_RVIZ_UI
#define OBJECT_SEGMENTATION_RVIZ_UI

#include <QWidget>

#include <ros/ros.h>

#include "interactive_perception_msgs/ObjectSegmentationGuiAction.h"
#include <actionlib/server/simple_action_server.h>

#include "object_segmentation_gui/object_segmenter.h"
#include "object_segmentation_gui/table_transform.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <OGRE/OgreManualObject.h>

#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>

namespace rviz_interaction_tools {
class ImageOverlay;
class DisparityRenderer;
}

namespace rviz {
class WindowManagerInterface;
class RenderPanel;
class DisplayContext;
}

namespace Ui {
class MainFrame;
}

namespace object_segmentation_gui {
  
  class ObjectSegmentationRvizUI : public QWidget
  {
    Q_OBJECT
    typedef geometry_msgs::Point32    Point;
    
  public:
    ObjectSegmentationRvizUI( rviz::DisplayContext* context, QWidget* parent = 0 );
    virtual ~ObjectSegmentationRvizUI();
    
    virtual void update(float wall_dt, float ros_dt);

    //start listening to action goals
    void startActionServer( ros::NodeHandle &node_handle );
    
    //stop action server, cancel current goal & hide if necessary
    void stopActionServer();

  protected Q_SLOTS:
    virtual void acceptButtonClicked();
    virtual void cancelButtonClicked();

    virtual void resetButtonClicked();
    virtual void restartButtonClicked();
    virtual void segmentButtonClicked();

    virtual void withSurfaceChecked() ;
    virtual void withDisparityChecked() ;
    virtual void withColorChecked() ;
    virtual void withHolesChecked() ;
    virtual void uniformChecked() ;

    virtual void gradWeightChanged( int new_value );

  protected:

    // callback for new action server goals
    void acceptNewGoal();

    // callback for action server preempt (cancel) requests
    void preempt();
  
    /** Reimplemented from QObject to filter mouse events from render_panel_. */
    virtual bool eventFilter( QObject* watched_object, QEvent* event );

    //cleanup ogre scene, hide window
    void cleanupAndHide();

    // stop segmentation by sending a stop message
    void stopSegmentation();

    // configure the RenderPanel 
    void setupRenderPanel( rviz::DisplayContext* context );

    rviz::WindowManagerInterface* window_manager_;
    rviz::RenderPanel* render_panel_;

    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* scene_root_;

    rviz_interaction_tools::ImageOverlay* image_overlay_;
    
    struct ClickInfo
    {
      int x_, y_;
      int down_x_, down_y_;
      bool down_;
      bool dragged_;
    };
    
    void addColorCode();
    
    void reconstructAndClusterPointCloud(interactive_perception_msgs::ObjectSegmentationGuiResult &result);
    
    void filterOutliersAndDownsample(std::vector<sensor_msgs::PointCloud> &clusters);
    
    ClickInfo click_info_;
    Ogre::ManualObject* box_object_;
    
    actionlib::SimpleActionServer<interactive_perception_msgs::ObjectSegmentationGuiAction> *object_segmentation_server_;
    
  private:
  
    enum ResetType{PLAIN, COLOR, SURFACE, DISP, HOLES, UNIFORM};

    //! The node handle
    ros::NodeHandle nh_;
    
     //! The node handle
    ros::NodeHandle priv_nh_;
    
    //! Publishing images (for debug only)
    image_transport::Publisher image_pub_;

    // overlays current result from segmenter with image from camera
    void overlaySegmentationMask();

    // calculates the number of points in each segment
    void getClusterSize();
    
    // creates an image of selected region to be overlayed with current camera image
    void addToMasks(const ObjectSegmenter::Box2D &select_);

    // upates the select box during mouse dragging
    void updateSelectBox(int start_x, int start_y, int stop_x, int stop_y);

    // creates an RGB image from point cloud that comes in
    void fillRgbImage(sensor_msgs::Image &rgb_img,
		      const sensor_msgs::PointCloud2 &point_cloud);
 
    void initStorage(const sensor_msgs::Image &image);

    void segment();

    void reset();
    void includeFlags( ObjectSegmenter::Action &ac );
    void resetVars( );

    int sumLabels( const sensor_msgs::Image &image);
    
    // Parameters of object segmentation
    double grad_weight_;
    double window_size_;
    double ball_size_;
    int    n_iter_;
    bool   use_gpu_;

    // Segmentation object
    ObjectSegmenter* object_segmenter_;
    
    // Number of foreground hypotheses
    int num_fg_hypos_;

    // stores most recent selected regions or points
    std::deque<ObjectSegmenter::Box2D> region_queue_;

    // stores previous selected regions or points for potential replaying
    std::deque<ObjectSegmenter::Box2D> previous_queue_;

    // transform table in (x, y, d) space to (x, y, z) space
    TableTransform  table_transformer_;
    
    // stores sensor data
    sensor_msgs::Image          current_image_;
    stereo_msgs::DisparityImage current_disparity_image_;
    sensor_msgs::PointCloud2    current_point_cloud_;
    sensor_msgs::CameraInfo     current_camera_info_;
    
    // segmentation mask
    sensor_msgs::Image          labels_;

    // initialisation mask
    sensor_msgs::Image          inits_;

    // stores feedback image in display
    sensor_msgs::Image   texture_buffer_;

    bool running_;
    bool paused_;

    // Color code for each segment (RGB)
    std::vector<int> color_code_;
    
    /** Number of points in each segment 
     *  Number of pixels might be bigger since parts of the image 
     *  without disparity information can be included in the segment.
     */
    std::vector<int> segm_size_;
    
    /** Point clusters */
    std::vector<sensor_msgs::PointCloud> clusters_;
    
    /** Point belonging to table plane*/
    sensor_msgs::PointCloud table_points_;
    
    //! Publisher for markers
    ros::Publisher marker_pub_;
    //! Used to remember the number of markers we publish so we can delete them later
    int num_markers_published_;
    //! The current marker being published
    int current_marker_id_;
    
    //! parameters for table plane detection
    int inlier_threshold_;
    double up_direction_;

    //! statistical outlier removal parameters
    double mean_k_;
    double std_;

    //! voxel grid filtering parameters
    double clustering_voxel_size_;

    Ui::MainFrame *ui_; // UI object created by uic from main_frame.ui
  };

}

#endif
