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

#include <QMouseEvent>

#include "object_segmentation_gui/object_segmentation_rviz_ui.h"

#include "rviz_interaction_tools/image_tools.h"
#include "rviz_interaction_tools/image_overlay.h"
#include "rviz_interaction_tools/camera_tools.h"
#include "rviz_interaction_tools/unique_string_manager.h"

#include <rviz/window_manager_interface.h>
#include <rviz/render_panel.h>
#include <rviz/validate_floats.h>
#include <rviz/frame_manager.h>
#include "rviz/display_context.h"

#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>

#include "tabletop_object_detector/marker_generator.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>

#include "ui_main_frame.h" // generated by uic during build from main_frame.ui

namespace enc = sensor_msgs::image_encodings;

// Does not work right now, probably colliding with OGRE in Rviz
//#define USE_CUDA

#define MAX_FG 6

namespace object_segmentation_gui
{

typedef pcl::PointXYZ PCL_Point;

static const char
    * HELP_TEXT =
        "Left Mouse Button: Click on objects or drag a rectangle to segment them. \n\
     Segment Button: Start Segmentation. \n\
     Reset Button: Reset Segmentation and Seeds. \n\
     Cancel Button: Cancel Segmentation. \n\
     OK Button: Accept Segmentation. ";

ObjectSegmentationRvizUI::ObjectSegmentationRvizUI( rviz::DisplayContext* context, QWidget* parent )
  : QWidget( parent )
  , object_segmentation_server_(0)
  , nh_("")
  , priv_nh_("~")
  , n_iter_(2)
  , use_gpu_(false)
  , object_segmenter_(0)
  , num_fg_hypos_(0)
  , running_(false)
  , paused_(true)
  , num_markers_published_(1)
  , current_marker_id_(1)
  , ui_( new Ui::MainFrame )
{
  // Set up UI and make connections.
  ui_->setupUi( this );

  render_panel_ = ui_->render_panel_;
  render_panel_->installEventFilter( this ); // process mouse events on the render_panel_ here, in eventFilter().

  connect( ui_->accept_button_, SIGNAL( clicked() ), this, SLOT( acceptButtonClicked() ));
  connect( ui_->cancel_button_, SIGNAL( clicked() ), this, SLOT( cancelButtonClicked() ));
  connect( ui_->reset_button_, SIGNAL( clicked() ), this, SLOT( resetButtonClicked() ));
  connect( ui_->restart_button_, SIGNAL( clicked() ), this, SLOT( restartButtonClicked() ));
  connect( ui_->segment_button_, SIGNAL( clicked() ), this, SLOT( segmentButtonClicked() ));

  connect( ui_->with_surface_, SIGNAL( toggled( bool )), this, SLOT( withSurfaceChecked() ));
  connect( ui_->with_disparity_, SIGNAL( toggled( bool )), this, SLOT( withDisparityChecked() ));
  connect( ui_->with_color_, SIGNAL( toggled( bool )), this, SLOT( withColorChecked() ));
  connect( ui_->with_holes_, SIGNAL( toggled( bool )), this, SLOT( withHolesChecked() ));
  connect( ui_->uniform_, SIGNAL( toggled( bool )), this, SLOT( uniformChecked() ));

  connect( ui_->grad_weight_slider_, SIGNAL( valueChanged( int )), this, SLOT( gradWeightChanged( int )));
  connect( ui_->grad_weight_slider_, SIGNAL( valueChanged( int )), ui_->grad_weight_value_label_, SLOT( setNum( int )));

  //! Publisher for images (debugging only)
  //    image_transport::ImageTransport it_(nh_);
  //    image_pub_ = it_.advertise("rgb_out", 1);

  rviz_interaction_tools::UniqueStringManager usm;

  //construct basic ogre scene
  scene_manager_ = Ogre::Root::getSingleton().createSceneManager(
      Ogre::ST_GENERIC, usm.unique("ObjectSegmentationRvizUI"));

  scene_root_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  image_overlay_ = new rviz_interaction_tools::ImageOverlay(scene_root_,
      Ogre::RENDER_QUEUE_OVERLAY - 2);

  //put a render panel into the window
  setupRenderPanel(context);

  setWindowTitle( "Interactive Object Segmentation" );
  ui_->bottom_label_->setText( HELP_TEXT );

  click_info_.down_ = false;
  click_info_.down_x_ = click_info_.down_y_ = 0;

  marker_pub_ = nh_.advertise<visualization_msgs::Marker> (nh_.resolveName(
      "tabletop_segmentation_markers"), 10);

  box_object_ = scene_manager_->createManualObject("SelectRegionIndicator");
  box_object_->setUseIdentityProjection(true);
  box_object_->setUseIdentityView(true);
  box_object_->setDynamic(true);
  box_object_->begin("BaseWhiteNoLighting",
      Ogre::RenderOperation::OT_LINE_STRIP);
  box_object_->position(0.0, 0.0, 0.0);
  box_object_->position(0.0, 0.0, 0.0);
  box_object_->position(0.0, 0.0, 0.0);
  box_object_->position(0.0, 0.0, 0.0);
  box_object_->index(0);
  box_object_->index(1);
  box_object_->index(2);
  box_object_->index(3);
  box_object_->index(0);
  box_object_->end();
  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  box_object_->setBoundingBox(aabInf);
  //click indicator will be rendered last
  box_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  scene_root_->attachObject(box_object_);

  priv_nh_.param<int> ("inlier_threshold", inlier_threshold_, 300);
  priv_nh_.param<double> ("up_direction", up_direction_, -1.0);

  priv_nh_.param<double> ("mean_k", mean_k_, 50.0);
  priv_nh_.param<double> ("std", std_, 1.0);

  priv_nh_.param<double> ("clustering_voxel_size", clustering_voxel_size_,
      0.003);

  nh_.param<double> ("grad_weight", grad_weight_, 80.0);

  nh_.param<double> ("window_size", window_size_, 0.10);

  nh_.param<double> ("ball_size", ball_size_, 0.10);

  object_segmenter_ = new ObjectSegmenter(grad_weight_, n_iter_, use_gpu_,
      window_size_, ball_size_);

}

ObjectSegmentationRvizUI::~ObjectSegmentationRvizUI()
{
  if (object_segmentation_server_)
    stopActionServer();

  render_panel_->getRenderWindow()->setActive(false);
  delete render_panel_;
  delete image_overlay_;
  delete object_segmenter_;
}

void ObjectSegmentationRvizUI::setupRenderPanel(
    rviz::DisplayContext* context )
{
  render_panel_->initialize( context->getSceneManager(), context );

  render_panel_->setAutoRender(false);
  render_panel_->getViewport()->setOverlaysEnabled(false);
  render_panel_->getViewport()->setClearEveryFrame(true);
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive(true);
}

void ObjectSegmentationRvizUI::startActionServer( ros::NodeHandle &node_handle )
{
  if (object_segmentation_server_)
  {
    ROS_ERROR( "ObjectSegmentationGuiAction server already started!" );
    return;
  }

  ROS_INFO("Starting ObjectSegmentationGuiAction server.");

  //create non-threaded action server
  object_segmentation_server_ = new actionlib::SimpleActionServer<
    interactive_perception_msgs::ObjectSegmentationGuiAction>(node_handle, "segmentation_popup", false);

  object_segmentation_server_->registerGoalCallback(boost::bind(
      &ObjectSegmentationRvizUI::acceptNewGoal, this));
  object_segmentation_server_->registerPreemptCallback(boost::bind(
      &ObjectSegmentationRvizUI::preempt, this));

  object_segmentation_server_->start();
}

void ObjectSegmentationRvizUI::stopActionServer()
{
  if (!object_segmentation_server_)
  {
    ROS_ERROR("ObjectSegmentationGuiAction server cannot be stopped because it is not running.");
    return;
  }

  //if we're currently being active, we have to cancel everything, clean up & hide the window
  if (object_segmentation_server_->isActive())
  {
    ROS_WARN("Aborting ObjectSegmentationGuiAction goal.");
    stopSegmentation();
    object_segmentation_server_->setAborted();
    cleanupAndHide();
  }

  ROS_INFO("Stopping ObjectSegmentationGuiAction server.");
  delete object_segmentation_server_;
  object_segmentation_server_ = 0;
}

void ObjectSegmentationRvizUI::acceptNewGoal()
{

  // only if segmentation is running labeling can be accepted
  ui_->accept_button_->setEnabled(running_);

  // only if segmentation has been run at least once,
  // same seed point cloud be reused
  ui_->restart_button_->setEnabled(!previous_queue_.empty());

  // only if segmentation is not running, it can be started
  ui_->segment_button_->setEnabled(!running_);

  const interactive_perception_msgs::ObjectSegmentationGuiGoal::ConstPtr &goal =
      object_segmentation_server_->acceptNewGoal();

  rviz_interaction_tools::updateCamera(render_panel_->getCamera(),
      goal->camera_info);

  ROS_ASSERT( goal->point_cloud.height ==
      goal->disparity_image.image.height &&
      goal->point_cloud.width ==
      goal->disparity_image.image.width);

  current_point_cloud_ = goal->point_cloud;
  current_camera_info_ = goal->camera_info;
  current_disparity_image_ = goal->disparity_image;
  fillRgbImage(current_image_, current_point_cloud_);
  //    image_pub_.publish(current_image_);

  float baseline = current_disparity_image_.T;
  table_transformer_.setParams(current_camera_info_, baseline, up_direction_);

  //store data for later use in update()
  //we can't directly put in in ogre because
  //this runs in a different thread
  image_overlay_->setImage(current_image_);
  image_overlay_->update();

  show();

  click_info_.down_x_ = click_info_.down_y_ = 0;

  // initialise segmentation
  object_segmenter_->setNewData(current_image_, current_disparity_image_);

  // set checkboxes accordingly
  bool with_colors, with_color_holes, uniform, with_disparities, with_surface;
  object_segmenter_->getCurrentSegmFlags(with_colors, with_color_holes,
      uniform, with_disparities, with_surface);

  ui_->with_surface_->setChecked(with_surface);
  ui_->with_surface_->setEnabled(true);
  ui_->with_disparity_->setChecked(with_disparities);
  ui_->with_disparity_->setEnabled(true);
  ui_->with_color_->setChecked(with_colors);
  ui_->with_color_->setEnabled(true);
  ui_->with_holes_->setChecked(with_color_holes);
  ui_->with_holes_->setEnabled(true);
  ui_->uniform_->setChecked(uniform);
  ui_->uniform_->setEnabled(true);

  ui_->grad_weight_slider_->setValue( int( grad_weight_ ));

  initStorage(current_image_);

}

void ObjectSegmentationRvizUI::initStorage( const sensor_msgs::Image &image )
{
  // initialise mask container
  inits_.header.frame_id = "narrow_stereo_optical_frame";
  inits_.header.stamp = ros::Time::now();
  inits_.height = image.height;
  inits_.width = image.width;
  inits_.encoding = enc::MONO8;
  inits_.is_bigendian = false;
  inits_.step = image.width;
  size_t size = inits_.step * inits_.height;
  inits_.data.resize(size);

  texture_buffer_.header = image.header;
  texture_buffer_.height = image.height;
  texture_buffer_.width = image.width;
  texture_buffer_.encoding = enc::RGB8;
  texture_buffer_.is_bigendian = false;
  texture_buffer_.step = 3 * texture_buffer_.width;
  size = texture_buffer_.step * texture_buffer_.height;
  texture_buffer_.data.resize(size);
}

void ObjectSegmentationRvizUI::preempt()
{
  stopSegmentation();
  object_segmentation_server_->setPreempted();
  cleanupAndHide();
}

void ObjectSegmentationRvizUI::cleanupAndHide()
{
  image_overlay_->clear();
  ui_->segment_button_->setText("Segment");
  paused_ = true;
  resetVars();
  previous_queue_.clear();
  hide();
}

void ObjectSegmentationRvizUI::stopSegmentation()
{
  ObjectSegmenter::Action action;
  action.type_ = ObjectSegmenter::STOP;
  object_segmenter_->queueAction(action);
}

void ObjectSegmentationRvizUI::resetVars()
{
  for (size_t i = 0; i < clusters_.size(); ++i)
    clusters_[i].points.clear();

  clusters_.clear();
  segm_size_.clear();
  table_points_.points.clear();
  num_fg_hypos_ = 0;
  color_code_.clear();

  size_t size = inits_.step * inits_.height;
  inits_.data.clear();
  inits_.data.resize(size, 0);

  labels_.data.clear();
  labels_.data.resize(size, 0);

  image_overlay_->setImage(current_image_);
  image_overlay_->update();

  region_queue_.clear();
  //previous_queue_.clear();

  running_ = false;
  paused_ = true;

  // only if segmentation is running labeling can be accepted
  ui_->accept_button_->setEnabled(running_);

  // only if segmentation has been run at least once,
  // same seed point cloud be reused
  ui_->restart_button_->setEnabled(!previous_queue_.empty());

  // only if segmentation is not running, it can be started
  ui_->segment_button_->setEnabled(!running_);
}

void ObjectSegmentationRvizUI::update( float wall_dt, float ros_dt )
{
  render_panel_->getRenderWindow()->update();

  // if button pressed
  if (click_info_.down_)
    updateSelectBox(click_info_.down_x_, click_info_.down_y_, click_info_.x_,
        click_info_.y_);
  else
    updateSelectBox(0.0, 0.0, 0.0, 0.0);

  if (num_fg_hypos_ > 0)
  {
    if (running_)
    {
      object_segmenter_->getCurrentResult(labels_);
      /*
       ROS_INFO("Sum of labels %d given %d foreground hypos",
       sumLabels(labels_), num_fg_hypos_);
       */
      //	image_pub_.publish(labels_);
    }
    overlaySegmentationMask();
  }

  // only if segmentation has been run at least once,
  // same seed point cloud be reused
  ui_->restart_button_->setEnabled(!previous_queue_.empty());
}

int ObjectSegmentationRvizUI::sumLabels( const sensor_msgs::Image &labels )
{
  int sum = 0;
  for (unsigned int i = 0; i < labels.width; ++i)
    for (unsigned int j = 0; j < labels.height; ++j)
    {
      int adr = j * labels.width + i;
      sum += labels.data[adr];
    }

  return sum;
}

bool ObjectSegmentationRvizUI::eventFilter( QObject* watched_object, QEvent* event )
{
  // Only handle events for render_panel_.
  if( watched_object != render_panel_ )
  {
    return QWidget::eventFilter( watched_object, event );
  }

  // Explicitly ignore wheel events in render_panel_.
  if( event->type() == QEvent::Wheel )
  {
    return true;
  }

  // Let parent class handle other non-mouse events.
  if( event->type() != QEvent::MouseMove &&
      event->type() != QEvent::MouseButtonPress &&
      event->type() != QEvent::MouseButtonRelease &&
      event->type() != QEvent::MouseButtonDblClick )
  {
    return QWidget::eventFilter( watched_object, event );
  }

  // It must be a mouse event at this point.
  QMouseEvent* mevent = static_cast<QMouseEvent*>( event );

  int x = mevent->x();
  int y = mevent->y();
  //convert to coordinates in the original image resolution
  x = floor(x * current_image_.width / render_panel_->width());
  y = floor(y * current_image_.height / render_panel_->height());

  if( mevent->type() == QEvent::MouseButtonPress &&
      mevent->button() == Qt::LeftButton )
  {
    click_info_.down_ = true;

    ROS_DEBUG("Good click at (%d,%d)", x, y);
    click_info_.down_x_ = x;
    click_info_.down_y_ = y;

    click_info_.x_ = x;
    click_info_.y_ = y;
  }
  else if( mevent->type() == QEvent::MouseButtonRelease &&
           mevent->button() == Qt::LeftButton )
  {
    if (click_info_.down_x_ > click_info_.x_)
      swap(click_info_.down_x_, click_info_.x_);

    if (click_info_.down_y_ > click_info_.y_)
      swap(click_info_.down_y_, click_info_.y_);

    float dist_clicks = dist(click_info_.down_x_, click_info_.down_y_,
                             click_info_.x_, click_info_.y_);

    ObjectSegmenter::Action action;

    Point tl;
    tl.x = click_info_.down_x_;
    tl.y = click_info_.down_y_;

    Point br;

    if (click_info_.dragged_ && dist_clicks > 2)
    {

      br.x = click_info_.x_;
      br.y = click_info_.y_;

      click_info_.dragged_ = false;
      action.type_ = ObjectSegmenter::REGION;

    } else
    {

      br.x = -1;
      br.y = -1;

      click_info_.dragged_ = false;
      action.type_ = ObjectSegmenter::CLICK;
    }

    ObjectSegmenter::Box2D region;
    region.p_tl_ = tl;
    region.p_br_ = br;

    if (!running_ && region_queue_.size() < MAX_FG)
    {
      region_queue_.push_back(region);
      previous_queue_.push_back(region);
      addToMasks(region);
      num_fg_hypos_++;
      addColorCode();
    } else if (num_fg_hypos_ < MAX_FG)
    {
      previous_queue_.push_back(region);
      action.box_ = region;
      object_segmenter_->queueAction(action);
      num_fg_hypos_++;
      addColorCode();
    }
    else      ROS_WARN("Maximum Number of Segments reached.");

    click_info_.down_ = false;

  }
  else if( mevent->type() == QEvent::MouseMove &&
           mevent->buttons() == Qt::LeftButton )
  {
    if (click_info_.down_)
    {
      click_info_.dragged_ = true;

      click_info_.x_ = x;
      click_info_.y_ = y;
    }
  }
  // Return true so mouse events not handled here are ignored, and
  // events handled here are not also interpreted by anyone else.
  return true;
}

void ObjectSegmentationRvizUI::addColorCode()
{
  // Generate random colour for segment

  int zero = rand()&2;
  for(int i=0; i<3; i++)
  if(i==zero)
  color_code_.push_back(0);
  else
  color_code_.push_back(rand()&255);
}

void ObjectSegmentationRvizUI::addToMasks(const ObjectSegmenter::Box2D &select_)
{

  int w = inits_.width;

  int size = 0;

  // region selected 
  if(!(select_.p_br_.x == -1 && select_.p_br_.y == -1))
  {

    for(int y=select_.p_tl_.y; y<select_.p_br_.y; ++y)
    {
      for(int x=select_.p_tl_.x; x<select_.p_br_.x; ++x)
      {
        int i = y*w + x;
        inits_.data[i]=num_fg_hypos_ + 2; // the same as the labels from the segmentation
        size ++;
      }
    }

    ROS_DEBUG("Added rectangle of size %d", size);

  } else
  {
    // point selected - region initialisation done automatically

    int radius = 10;

    for(int y=select_.p_tl_.y-radius; y<select_.p_tl_.y+radius; ++y)
    {
      for(int x=select_.p_tl_.x-radius; x<select_.p_tl_.x+radius; ++x)
      {

        if(dist((int)select_.p_tl_.x, (int)select_.p_tl_.y, x, y)<=radius)
        {
          int i = y*w + x;
          inits_.data[i]=num_fg_hypos_ + 2;
          size++;
        }
      }
    }

    ROS_DEBUG("Added circle of size %d", size);
  }
}

void ObjectSegmentationRvizUI::fillRgbImage(sensor_msgs::Image &rgb_img,
    const sensor_msgs::PointCloud2 &point_cloud)
{

  ROS_DEBUG("Width and Height: (%d %d)",point_cloud.height, point_cloud.width );

  rgb_img.header = point_cloud.header;
  rgb_img.height = point_cloud.height;
  rgb_img.width = point_cloud.width;
  rgb_img.encoding = enc::RGB8;
  rgb_img.is_bigendian = false;
  rgb_img.step = 3 * rgb_img.width;
  size_t size = rgb_img.step * rgb_img.height;
  rgb_img.data.resize(size);

  for(unsigned int x=0; x<rgb_img.width; ++x)
  {
    for(unsigned int y=0; y<rgb_img.height; ++y)
    {
      int i = y * rgb_img.width + x;

      float rgb;
      memcpy ( &rgb, &point_cloud.data[i * point_cloud.point_step + point_cloud.fields[3].offset], sizeof (float));
      float r, g, b;
      transformRgb(rgb, r, g, b);

      int wide_i = y * rgb_img.step + x*3;
      rgb_img.data[wide_i+0] = round(r*255.0f);
      rgb_img.data[wide_i+1] = round(g*255.0f);
      rgb_img.data[wide_i+2] = round(b*255.0f);

    }
  }
}

void ObjectSegmentationRvizUI::overlaySegmentationMask()
{

  sensor_msgs::Image::ConstPtr overlay_ptr;

  if( !running_)
  {
    overlay_ptr = boost::make_shared<const sensor_msgs::Image> (inits_);
  } else
  {
    overlay_ptr = boost::make_shared<const sensor_msgs::Image> (labels_);
  }

  // decrease in gray shade for surface 
  //    uint8_t darker = 50;

  //copy image to a new buffer so we can paint the segmentation mask

  int count = 0;

  int w = current_image_.width;
  int h = current_image_.height;

  for (int i=0; i<h; i++)
  {
    for (int j=0; j<w; j++)
    {
      unsigned int adr = i *w + j;
      // foreground segments start at label 2, background=0, flat surface=1
      int label = (int)(overlay_ptr->data[adr]);

      for(int c=0; c<3; c++)
      {
        uint8_t val;
        // we want to color all foreground segments
        if(label>1)
        {
          // only one channel will be changed,
          // the other will have 0 and colour info has to be kept
          ROS_ASSERT(label-2<num_fg_hypos_);
          ROS_ASSERT(3 * (label-2) + c<(int)color_code_.size());
          if( color_code_.at(3 * (label-2) + c)==0)
          val = current_image_.data[3 * adr + c];
          else
          val = color_code_.at(3 * (label-2) + c);

          count ++;

        } /*else if(label==1){
         val = current_image_.data[3 * adr + c];
         val = val>darker ? val-darker : 0;
         }
         */
        else
        {
          val = current_image_.data[3 * adr + c];
        }

        texture_buffer_.data.at( 3 * adr + c ) = val;
      }
    }
  }

  image_overlay_->setImage(texture_buffer_);
  image_overlay_->update();
}

void ObjectSegmentationRvizUI::updateSelectBox(int start_x,
    int start_y,
    int stop_x,
    int stop_y)
{

  if(start_x>stop_x)
  swap(start_x, stop_x);

  if(start_y>stop_y)
  swap(start_y, stop_y);

  float x0 = start_x / ((float)current_image_.width / 2.0 ) - 1.0;
  float y0 = - (start_y / ((float)current_image_.height / 2.0 ) - 1.0);

  float x1 = stop_x / ((float)current_image_.width / 2.0 ) - 1.0;
  float y1 = - (stop_y / ((float)current_image_.height / 2.0 ) - 1.0);

  box_object_->beginUpdate(0);
  box_object_->position(x0, y0, 0.0);
  box_object_->position(x0, y1, 0.0);
  box_object_->position(x1, y1, 0.0);
  box_object_->position(x1, y0, 0.0);
  box_object_->index(0);
  box_object_->index(1);
  box_object_->index(2);
  box_object_->index(3);
  box_object_->index(0);
  box_object_->end();
}

void ObjectSegmentationRvizUI::acceptButtonClicked()
{
  if( !running_) return;

  interactive_perception_msgs::ObjectSegmentationGuiResult result;

  while(!object_segmenter_->getCurrentResult( labels_))
  {
    ROS_INFO("Waiting for final labeling");
  }
  float alpha, beta, gamma;
  object_segmenter_->getCurrentSurface( alpha, beta, gamma);

  // stop segmentation thread
  stopSegmentation();

  getClusterSize();

  if ( segm_size_[0] < (int)inlier_threshold_)
  {
    ROS_INFO("Plane detection has %d inliers, below min threshold of %d",
        segm_size_[0],
        inlier_threshold_);
    result.result = result.NO_TABLE;
    return;
  }

  for(int i=1; i<(int)segm_size_.size(); ++i)
  if(segm_size_[i] == 0)
  {
    ROS_INFO("Segment %d has 0 points.", segm_size_[i]);
    num_fg_hypos_--;
  }

  reconstructAndClusterPointCloud(result);

  // get table parameters in 3d
  tabletop_object_detector::Table table = table_transformer_.get3DTable(alpha, beta, gamma, table_points_,
      clusters_[0].header);
  result.table = table;

  result.result = result.SUCCESS;

  ROS_INFO("ObjectSegmentation was successful.");

  object_segmentation_server_->setSucceeded(result);

  cleanupAndHide();
}

void ObjectSegmentationRvizUI::cancelButtonClicked()
{
  stopSegmentation();
  object_segmentation_server_->setAborted();
  cleanupAndHide();
}

void ObjectSegmentationRvizUI::resetButtonClicked()
{
  reset( );
  previous_queue_.clear();
}

void ObjectSegmentationRvizUI::reset( )
{
  ObjectSegmenter::Action action;
  action.type_ = ObjectSegmenter::RESET;
  includeFlags(action);
  object_segmenter_->queueAction(action);
  image_overlay_->clear();

  ui_->segment_button_->setText("Segment");
  paused_ = true;

  resetVars();
}

void ObjectSegmentationRvizUI::restartButtonClicked()
{
  // copy old seeds into new queue
  reset();
  // ObjectSegmenter::Action action;
  // action.type_ = ObjectSegmenter::RESET;
  // includeFlags(action);
  // object_segmenter_->queueAction(action);
  // image_overlay_->clear();


  // region_queue_.clear();

  for(int i=0; i<(int)previous_queue_.size();++i)
  {
    if( num_fg_hypos_ < MAX_FG )
    {
      region_queue_.push_back(previous_queue_[i]);
      addToMasks(previous_queue_[i]);
      num_fg_hypos_++;
      addColorCode();
    } else
    {
      ROS_INFO("Maximum number of segments reached");
    }
  }

  segment();
  //    std::string pause("Pause");
  ui_->segment_button_->setText("Pause");
  paused_ = false;
}

void ObjectSegmentationRvizUI::includeFlags( ObjectSegmenter::Action &ac)
{
  // keep the current segmentation flags
  ac.with_colors_ = ui_->with_color_->isChecked();
  ac.with_color_holes_ = ui_->with_holes_->isChecked();
  ac.uniform_ = ui_->uniform_->isChecked();
  ac.with_disparities_ = ui_->with_disparity_->isChecked();
  ac.with_surface_ = ui_->with_surface_->isChecked();

}

void ObjectSegmentationRvizUI::segmentButtonClicked()
{
  if(paused_)
  {
    segment();
    ui_->segment_button_->setText("Pause");
    paused_ = false;
  } else
  {
    // copy old seeds into new queue
    ObjectSegmenter::Action action;
    action.type_ = ObjectSegmenter::PAUSE;
    object_segmenter_->queueAction(action);
    ui_->segment_button_->setText("Segment");
    paused_ = true;
  }
}

void ObjectSegmentationRvizUI::segment()
{

  if(region_queue_.empty())
  {
    // no region has been selected or click executed
    ObjectSegmenter::Action action;
    ObjectSegmenter::Box2D dummy;
    dummy.p_tl_.x = -1;
    dummy.p_tl_.y = -1;
    dummy.p_br_.x = -1;
    dummy.p_br_.y = -1;
    action.type_ = ObjectSegmenter::CLICK;
    action.box_ = dummy;
    object_segmenter_->queueAction(action);

  } else
  {
    while(!region_queue_.empty())
    {
      ObjectSegmenter::Action action;
      if(region_queue_[0].p_br_.x==-1 && region_queue_[0].p_br_.y==-1)
      {
        action.type_ = ObjectSegmenter::CLICK;
      } else
      {
        action.type_ = ObjectSegmenter::REGION;
      }

      action.box_ = region_queue_[0];

      object_segmenter_->queueAction(action);
      region_queue_.pop_front();
    }
  }

  running_=true;
  // only if segmentation is running labeling can be accepted
  ui_->accept_button_->setEnabled(running_);

  // only if segmentation has been run at least once, 
  // same seed point cloud be reused
  ui_->restart_button_->setEnabled(!previous_queue_.empty());

}

void ObjectSegmentationRvizUI::withSurfaceChecked()
{ reset();}

void ObjectSegmentationRvizUI::withDisparityChecked()
{
  if(!ui_->with_disparity_->isChecked())
  {
    ui_->with_color_->setChecked(true);
    ui_->with_holes_->setEnabled(true);
    ui_->uniform_->setEnabled(true);
  }
  reset();
}

void ObjectSegmentationRvizUI::withColorChecked()
{
  if(!ui_->with_color_->isChecked())
  {
    ui_->with_disparity_->setChecked(true);
    ui_->with_holes_->setEnabled(false);
    ui_->uniform_->setEnabled(false);
  } else
  {
    ui_->with_holes_->setEnabled(true);
    ui_->uniform_->setEnabled(true);
  }

  reset();
}

void ObjectSegmentationRvizUI::withHolesChecked()
{ reset();}

void ObjectSegmentationRvizUI::uniformChecked()
{ reset();}

void ObjectSegmentationRvizUI::gradWeightChanged( int new_value )
{
  grad_weight_ = (double) new_value;
  ObjectSegmenter::Action action;
  action.type_ = ObjectSegmenter::GRADCHANGE;
  action.grad_weight_ = grad_weight_;
  object_segmenter_->queueAction(action);
}

void ObjectSegmentationRvizUI::getClusterSize()
{
  int w = labels_.width;
  int h = labels_.height;

  // take into account surface and foreground hypotheses
  segm_size_.resize(num_fg_hypos_+1,0);

  for (int i=0; i<h; i++)
  {
    for (int j=0; j<w; j++)
    {

      unsigned int adr = i *w + j;
      unsigned int label = labels_.data[adr];
      // take foreground objects
      if(rviz_interaction_tools::hasDisparityValue(current_disparity_image_,
              i, j))
      {
        if(label>1)
        segm_size_.at(label-1)++;
        else if(label==1)
        segm_size_.at(0)++;
      }
    }
  }
}

void ObjectSegmentationRvizUI::reconstructAndClusterPointCloud( interactive_perception_msgs::ObjectSegmentationGuiResult &result)
{
  clusters_.clear();
  clusters_.resize(num_fg_hypos_);
  for(int i=0; i<num_fg_hypos_; ++i)
  {
    clusters_[i].header.frame_id = current_point_cloud_.header.frame_id;
    clusters_[i].header.stamp = ros::Time::now();
    clusters_[i].points.reserve(segm_size_[i+1]);
  }

  table_points_.header.frame_id = current_point_cloud_.header.frame_id;
  table_points_.header.stamp = ros::Time::now();
  table_points_.points.reserve(segm_size_[0]);

  int nan_cluster =0;

  for(unsigned int x=0; x<current_point_cloud_.width; ++x)
  {
    for(unsigned int y=0; y<current_point_cloud_.height; ++y)
    {
      int i = y * current_point_cloud_.width + x;
      float nan;
      memcpy (&nan,
          &current_point_cloud_.data[i * current_point_cloud_.point_step + current_point_cloud_.fields[0].offset],
          sizeof (float));
      if(!isnan(nan))
      {
        geometry_msgs::Point32 p;
        float x, y, z;
        memcpy (&x,
            &current_point_cloud_.data[i
            * current_point_cloud_.point_step
            + current_point_cloud_.fields[0].offset],
            sizeof (float));
        memcpy (&y,
            &current_point_cloud_.data[i
            * current_point_cloud_.point_step
            + current_point_cloud_.fields[1].offset],
            sizeof (float));
        memcpy (&z,
            &current_point_cloud_.data[i
            * current_point_cloud_.point_step
            + current_point_cloud_.fields[2].offset],
            sizeof (float));
        p.x = x;
        p.y = y;
        p.z = z;

        unsigned int label = labels_.data[i];
        if(label>1)
        {
          clusters_[label-2].points.push_back( p );
        } else if(label==1)
        {
          table_points_.points.push_back( p );
        }
      } else
      {
        nan_cluster++;
      }
    }
  }

  filterOutliersAndDownsample(clusters_);

  result.clusters = clusters_;
}

void ObjectSegmentationRvizUI::filterOutliersAndDownsample(std::vector<sensor_msgs::PointCloud> &clusters)
{
  std::vector<sensor_msgs::PointCloud> filtered_clusters;
  filtered_clusters.reserve(clusters.size());

  for (size_t i=0; i<clusters.size(); i++)
  {

    // annoying conversion to different point cloud formats
    sensor_msgs::PointCloud2 converted_cloud;
    sensor_msgs::convertPointCloudToPointCloud2(clusters[i], converted_cloud);

    pcl::PointCloud<PCL_Point> cloud_t;
    pcl::fromROSMsg (converted_cloud, cloud_t);
    pcl::PointCloud<PCL_Point>::ConstPtr cloud_ptr
    = boost::make_shared<const pcl::PointCloud<PCL_Point> > (cloud_t);

    // statistical outlier removal
    pcl::StatisticalOutlierRemoval<PCL_Point> sor;
    sor.setInputCloud (cloud_ptr);
    sor.setMeanK (mean_k_);
    sor.setStddevMulThresh (std_);
    pcl::PointCloud<PCL_Point>::Ptr
    cloud_filtered(new pcl::PointCloud<PCL_Point> ());
    sor.filter (*cloud_filtered);

    // and annoying back conversion into old point cloud format
    sensor_msgs::PointCloud2 filt_converted_cloud;
    pcl::toROSMsg ( *cloud_filtered, filt_converted_cloud);
    sensor_msgs::PointCloud2::ConstPtr filt_converted_cloud_ptr
    = boost::make_shared<const sensor_msgs::PointCloud2> (filt_converted_cloud);

    // downsampling of point clouds
    pcl::VoxelGrid<sensor_msgs::PointCloud2> dvx;
    dvx.setInputCloud(filt_converted_cloud_ptr);
    dvx.setLeafSize (clustering_voxel_size_,
        clustering_voxel_size_,
        clustering_voxel_size_);
    sensor_msgs::PointCloud2::Ptr
    cloud_downsampled(new sensor_msgs::PointCloud2());
    dvx.filter (*cloud_downsampled);

    sensor_msgs::PointCloud cloud_tmp;
    sensor_msgs::convertPointCloud2ToPointCloud(*cloud_downsampled, cloud_tmp);

    filtered_clusters.push_back(cloud_tmp);
  }

  clusters.clear();
  clusters = filtered_clusters;

}

} // namespace object_segmentation_gui
