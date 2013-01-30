/*
 * Note: this is a modified version of the RViz camera display and
 * intended to be included in RViz once it's finished.
 */

/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <boost/bind.hpp>

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRectangle.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreRenderOperation.h>


#include <tf/transform_listener.h>

#include "rviz/bit_allocator.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/render_panel.h"
#include "rviz/uniform_string_stream.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"
#include "rviz/properties/display_group_visibility_property.h"
#include "rviz/load_resource.h"
#include "rviz/view_manager.h"
#include "rviz/window_manager_interface.h"
#include "rviz/panel_dock_widget.h"

#include <image_transport/camera_common.h>

#include "pr2_interactive_manipulation/awesome_camera_display.h"

namespace rviz
{

const QString AwesomeCameraDisplay::BACKGROUND( "background" );
const QString AwesomeCameraDisplay::OVERLAY( "overlay" );
const QString AwesomeCameraDisplay::BOTH( "background and overlay" );

bool validateFloats(const sensor_msgs::CameraInfo& msg)
{
  bool valid = true;
  valid = valid && validateFloats( msg.D );
  valid = valid && validateFloats( msg.K );
  valid = valid && validateFloats( msg.R );
  valid = valid && validateFloats( msg.P );
  return valid;
}

void makeRect( std::string material_name, Ogre::ManualObject* rect )
{
  rect->begin( material_name, Ogre::RenderOperation::OT_TRIANGLE_FAN );
  rect->position(      1.0,  1.0, -1.0 );
  rect->textureCoord(  1.0, 0.0 );
  rect->position(     -1.0, 1.0, -1.0 );
  rect->textureCoord( 0.0,  0.0 );
  rect->position(    -1.0, -1.0, -1.0 );
  rect->textureCoord( 0.0,  1.0 );
  rect->position(     1.0, -1.0, -1.0 );
  rect->textureCoord(  1.0,  1.0 );
  rect->end();
}

AwesomeCameraDisplay::AwesomeCameraDisplay()
  : ImageDisplayBase()
  , texture_()
  , caminfo_tf_filter_( 0 )
  , new_caminfo_( false )
  , render_panel_( 0 )
  , force_render_( false )
  , render_panel_dock_widget_(0)
{
  show_panel_property_ = new BoolProperty( "Show Panel", true, "Show Extra Panel with the camera image, in addition to main view.",
                                           this, SLOT( showPanelPropertyChanged() ) );

  image_position_property_ = new EnumProperty( "Image Rendering", BOTH,
                                               "Render the image behind all other geometry or overlay it on top, or both.",
                                               this, SLOT( forceRender() ));
  image_position_property_->addOption( BACKGROUND );
  image_position_property_->addOption( OVERLAY );
  image_position_property_->addOption( BOTH );

  alpha_property_ = new FloatProperty( "Overlay Alpha", 0.5,
                                       "The amount of transparency to apply to the camera image when rendered as overlay.",
                                       this, SLOT( updateAlpha() ));
  alpha_property_->setMin( 0 );
  alpha_property_->setMax( 1 );

  zoom_property_ = new FloatProperty( "Zoom Factor", 1.0,
                                      "Set a zoom factor below 1 to see a larger part of the world, above 1 to magnify the image.",
                                      this, SLOT( forceRender() ));
  zoom_property_->setMin( 0.00001 );
  zoom_property_->setMax( 100000 );

  near_clip_property_ = new FloatProperty( "Near Clip Distance", 0.01f,
                                      "Anything closer to the camera that this threshold will not get rendered.",
                                      this, SLOT( forceRender() ));
  near_clip_property_->setMin( 0.0001 );
  near_clip_property_->setMax( 100000 );
}

AwesomeCameraDisplay::~AwesomeCameraDisplay()
{
  unsubscribe();
  caminfo_tf_filter_->clear();

  delete render_panel_;
  delete bg_screen_rect_;
  delete fg_screen_rect_;

  bg_scene_node_->getParentSceneNode()->removeAndDestroyChild( bg_scene_node_->getName() );
  fg_scene_node_->getParentSceneNode()->removeAndDestroyChild( fg_scene_node_->getName() );

  delete caminfo_tf_filter_;

  context_->visibilityBits()->freeBits(vis_bit_);
}

void AwesomeCameraDisplay::onInitialize()
{
  caminfo_tf_filter_ = new tf::MessageFilter<sensor_msgs::CameraInfo>( *context_->getTFClient(), fixed_frame_.toStdString(),
                                                                       queue_size_property_->getInt(), update_nh_ );

  bg_scene_node_ = scene_node_->createChildSceneNode();
  fg_scene_node_ = scene_node_->createChildSceneNode();

  {
    static int count = 0;
    UniformStringStream ss;
    ss << "CameraDisplayRect" << count++;

    // image texture material
    ss << "Material";
    bg_material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    bg_material_->setDepthWriteEnabled(false);
    bg_material_->setDepthCheckEnabled(false);
    bg_material_->setReceiveShadows(false);

    bg_material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState* tu = bg_material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_.getTexture()->getName());
    tu->setTextureFiltering( Ogre::TFO_NONE );
    tu->setAlphaOperation( Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0 );

    bg_material_->setCullingMode(Ogre::CULL_NONE);
    bg_material_->setSceneBlending( Ogre::SBT_REPLACE );

    //background rectangle
    bg_screen_rect_ = new Ogre::ManualObject(ss.str()+"_bg");
    makeRect( bg_material_->getName(), bg_screen_rect_ );
    bg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);

    bg_scene_node_->attachObject(bg_screen_rect_);
    bg_scene_node_->setVisible(false);

    //overlay rectangle
    fg_material_ = bg_material_->clone( ss.str()+"fg" );
    fg_material_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );

    fg_screen_rect_ = new Ogre::ManualObject(ss.str()+"_fg");
    makeRect( fg_material_->getName(), fg_screen_rect_ );
    fg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);

    fg_scene_node_->attachObject(fg_screen_rect_);
    fg_scene_node_->setVisible(false);
  }

  updateAlpha();

  // we need to listen to the main window rendering so we can fiddle with its visibility bits
  context_->getViewManager()->getRenderPanel()->getRenderWindow()->addListener(this);

  render_panel_ = new RenderPanel();
  render_panel_->getRenderWindow()->addListener( this );
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive( false );
  render_panel_->resize( 640, 480 );
  render_panel_->initialize( context_->getSceneManager(), context_ );

  //setAssociatedWidget( render_panel_ );

  WindowManagerInterface* wm = context_->getWindowManager();
  if( wm )
  {
    render_panel_dock_widget_ = wm->addPane( getName(), render_panel_ );
    connect( render_panel_dock_widget_, SIGNAL( visibilityChanged( bool ) ), this, SLOT( panelVisibilityChanged( bool )));
    render_panel_dock_widget_->setIcon( getIcon() );
  }

  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance( 0.01f );

  caminfo_tf_filter_->connectInput(caminfo_sub_);
  caminfo_tf_filter_->registerCallback(boost::bind(&AwesomeCameraDisplay::caminfoCallback, this, _1));
  context_->getFrameManager()->registerFilterForTransformStatusCheck(caminfo_tf_filter_, this);

  vis_bit_ = context_->visibilityBits()->allocBit();
  render_panel_->getViewport()->setVisibilityMask( vis_bit_ );

  visibility_property_ = new DisplayGroupVisibilityProperty(
      vis_bit_, context_->getRootDisplayGroup(), this, "Visibility", true,
      "Changes the visibility of other Displays in the camera view.");

  visibility_property_->setIcon( loadPixmap("package://rviz/icons/visibility.svg",true) );

  this->addChild( visibility_property_, 0 );
}

void AwesomeCameraDisplay::panelVisibilityChanged( bool visible )
{
  show_panel_property_->setBool(visible);
}

void AwesomeCameraDisplay::showPanelPropertyChanged()
{
  if ( render_panel_dock_widget_ )
  {
    render_panel_dock_widget_->setVisible( show_panel_property_->getBool() );
  }
}

void AwesomeCameraDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  //determine if the main view sits inside the camera
  Ogre::Camera* main_cam = context_->getViewManager()->getCurrent()->getCamera();
  Ogre::Camera* my_cam = render_panel_->getCamera();
  float dist = (main_cam->getDerivedPosition() - my_cam->getDerivedPosition()).length();

  Ogre::Viewport* vp = evt.source->getViewport(0);
  if ( vp == render_panel_->getViewport() || dist < 0.02 )
  {
    vp->setVisibilityMask( vis_bit_ );
    QString image_position = image_position_property_->getString();
    bg_scene_node_->setVisible( image_position == BACKGROUND || image_position == BOTH );
    fg_scene_node_->setVisible( image_position == OVERLAY || image_position == BOTH );
  }
  else
  {
    vp->setVisibilityMask( 0xffffffff );
    bg_scene_node_->setVisible( false );
    fg_scene_node_->setVisible( false );
  }

  // apply visibility bit to all other displays
  visibility_property_->update();
}

void AwesomeCameraDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  Ogre::Viewport* vp = evt.source->getViewport(0);
  vp->setVisibilityMask( 0xffffffff );
  bg_scene_node_->setVisible( false );
  fg_scene_node_->setVisible( false );
}

void AwesomeCameraDisplay::onEnable()
{
  subscribe();
  render_panel_->getRenderWindow()->setActive(true);
}

void AwesomeCameraDisplay::onDisable()
{
  render_panel_->getRenderWindow()->setActive(false);
  unsubscribe();
  clear();
}

void AwesomeCameraDisplay::subscribe()
{
  if ( (!isEnabled()) || (topic_property_->getTopicStd().empty()) )
  {
    return;
  }

  std::string target_frame = fixed_frame_.toStdString();
  ImageDisplayBase::enableTFFilter(target_frame);

  ImageDisplayBase::subscribe();

  std::string topic = topic_property_->getTopicStd();
  std::string caminfo_topic = image_transport::getCameraInfoTopic(topic_property_->getTopicStd());

  try
  {
    caminfo_sub_.subscribe( update_nh_, caminfo_topic, 1 );
    setStatus( StatusProperty::Ok, "Camera Info", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( StatusProperty::Error, "Camera Info", QString( "Error subscribing: ") + e.what() );
  }
}

void AwesomeCameraDisplay::unsubscribe()
{
  ImageDisplayBase::unsubscribe();
  caminfo_sub_.unsubscribe();
}

void AwesomeCameraDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();

  Ogre::Pass* pass = fg_material_->getTechnique( 0 )->getPass( 0 );
  if( pass->getNumTextureUnitStates() > 0 )
  {
    Ogre::TextureUnitState* tex_unit = pass->getTextureUnitState( 0 );
    tex_unit->setAlphaOperation( Ogre::LBX_MODULATE, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha );
  }
  else
  {
    fg_material_->setAmbient( Ogre::ColourValue( 0.0f, 1.0f, 1.0f, alpha ));
    fg_material_->setDiffuse( Ogre::ColourValue( 0.0f, 1.0f, 1.0f, alpha ));
  }

  force_render_ = true;
  context_->queueRender();
}

void AwesomeCameraDisplay::forceRender()
{
  force_render_ = true;
  context_->queueRender();
}

void AwesomeCameraDisplay::updateQueueSize()
{
  caminfo_tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
  ImageDisplayBase::updateQueueSize();
}

void AwesomeCameraDisplay::clear()
{
  texture_.clear();
  force_render_ = true;
  context_->queueRender();

  new_caminfo_ = false;
  current_caminfo_.reset();

  setStatus( StatusProperty::Warn, "Camera Info",
             "No CameraInfo received on [" + QString::fromStdString( caminfo_sub_.getTopic() ) + "].  Topic may not exist.");
  setStatus( StatusProperty::Warn, "Image", "No Image received");

  render_panel_->getCamera()->setPosition( Ogre::Vector3( 999999, 999999, 999999 ));
}

void AwesomeCameraDisplay::update( float wall_dt, float ros_dt )
{
  try
  {
    if( texture_.update() || force_render_ )
    {
      updateCamera();
      force_render_ = false;
    }
  }
  catch( UnsupportedImageEncoding& e )
  {
    setStatus( StatusProperty::Error, "Image", e.what() );
  }

  render_panel_->getRenderWindow()->update();
}

void AwesomeCameraDisplay::updateCamera()
{
  sensor_msgs::CameraInfo::ConstPtr info;
  sensor_msgs::Image::ConstPtr image;
  {
    boost::mutex::scoped_lock lock( caminfo_mutex_ );

    info = current_caminfo_;
    image = texture_.getImage();
  }

  if( !info || !image )
  {
    return;
  }

  if( !validateFloats( *info ))
  {
    setStatus( StatusProperty::Error, "Camera Info", "Contains invalid floating point values (nans or infs)" );
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  context_->getFrameManager()->getTransform( image->header.frame_id, ros::Time(0), position, orientation );

  //printf( "CameraDisplay:updateCamera(): pos = %.2f, %.2f, %.2f.\n", position.x, position.y, position.z );

  // convert vision (Z-forward) frame to ogre frame (Z-out)
  orientation = orientation * Ogre::Quaternion( Ogre::Degree( 180 ), Ogre::Vector3::UNIT_X );

  float img_width = info->width;
  float img_height = info->height;

  // If the image width is 0 due to a malformed caminfo, try to grab the width from the image.
  if( img_width == 0 )
  {
    ROS_DEBUG( "Malformed CameraInfo on camera [%s], width = 0", qPrintable( getName() ));

    img_width = texture_.getWidth();
  }

  if (img_height == 0)
  {
    ROS_DEBUG( "Malformed CameraInfo on camera [%s], height = 0", qPrintable( getName() ));

    img_height = texture_.getHeight();
  }

  if( img_height == 0.0 || img_width == 0.0 )
  {
    setStatus( StatusProperty::Error, "Camera Info",
               "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0)" );
    return;
  }

  double fx = info->K[0];
  double fy = info->K[4];

  scene_node_->setScale( 0.5*img_width / fx, 0.5*img_height / fy, 1.0 );

  float win_width = render_panel_->width();
  float win_height = render_panel_->height();
  float zoom_x = zoom_property_->getFloat();
  float zoom_y = zoom_x;

  // Preserve aspect ratio
  if( win_width != 0 && win_height != 0 )
  {
    float img_aspect = (img_width/fx) / (img_height/fy);
    float win_aspect = win_width / win_height;

    if ( img_aspect > win_aspect )
    {
      zoom_y = zoom_y / img_aspect * win_aspect;
    }
    else
    {
      zoom_x = zoom_x / win_aspect * img_aspect;
    }
  }

  // Add the camera's translation relative to the left camera (from P[3]);
  double tx = -1 * (info->P[3] / fx);
  Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  position = position + (right * tx);

  double ty = -1 * (info->P[7] / fy);
  Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
  position = position + (down * ty);

  if( !validateFloats( position ))
  {
    setStatus( StatusProperty::Error, "Camera Info", "CameraInfo/P resulted in an invalid position calculation (nans or infs)" );
    return;
  }

  render_panel_->getCamera()->setPosition( position );
  render_panel_->getCamera()->setOrientation( orientation );

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  // calculate the projection matrix
  double cx = info->P[2];
  double cy = info->P[6];

  double far_plane = 1000.0;
  double near_plane = near_clip_property_->getFloat();

  Ogre::Matrix4 proj_matrix;
  proj_matrix = Ogre::Matrix4::ZERO;
 
  proj_matrix[0][0]= 2.0 * fx/img_width * zoom_x;
  proj_matrix[1][1]= 2.0 * fy/img_height * zoom_y;

  proj_matrix[0][2]= 2.0 * (0.5 - cx/img_width) * zoom_x;
  proj_matrix[1][2]= 2.0 * (cy/img_height - 0.5) * zoom_y;

  proj_matrix[2][2]= -(far_plane+near_plane) / (far_plane-near_plane);
  proj_matrix[2][3]= -2.0*far_plane*near_plane / (far_plane-near_plane);

  proj_matrix[3][2]= -1;

  render_panel_->getCamera()->setCustomProjectionMatrix( true, proj_matrix );

  setStatus( StatusProperty::Ok, "Camera Info", "OK" );

#if 0
  static Axes* debug_axes = new Axes(scene_manager_, 0, 0.2, 0.01);
  debug_axes->setPosition(position);
  debug_axes->setOrientation(orientation);
#endif
}

void AwesomeCameraDisplay::processMessage(const sensor_msgs::Image::ConstPtr& msg)
{
  texture_.addMessage(msg);
}

void AwesomeCameraDisplay::caminfoCallback( const sensor_msgs::CameraInfo::ConstPtr& msg )
{
  boost::mutex::scoped_lock lock( caminfo_mutex_ );
  current_caminfo_ = msg;
  new_caminfo_ = true;
}

void AwesomeCameraDisplay::fixedFrameChanged()
{
  std::string targetFrame = fixed_frame_.toStdString();
  caminfo_tf_filter_->setTargetFrame(targetFrame);
  ImageDisplayBase::fixedFrameChanged();
}

void AwesomeCameraDisplay::reset()
{
  ImageDisplayBase::reset();
  clear();
}

} // namespace rviz
