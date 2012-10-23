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

#include "pr2_interactive_manipulation/camera_focus_frontend.h"

#include <rviz_view_controllers/CameraPlacement.h>

#include "object_manipulator/tools/camera_configurations.h"

#include <rviz/window_manager_interface.h>
#include <rviz/display_context.h>
#include <rviz/display.h>
#include <rviz/view_controller.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/load_resource.h>

#include <QMainWindow>
#include <QToolBar>
#include <QAction>
#include <QPainter>

namespace pr2_interactive_manipulation {

CameraFocusFrontend::CameraFocusFrontend() :
  root_nh_(""),
  toolbar_( 0 )
{
  topic_prop_ = new rviz::RosTopicProperty( "Command topic", "/rviz/camera_placement",
      QString::fromStdString(ros::message_traits::datatype<rviz_view_controllers::CameraPlacement>() ),
      "Topic on which to send out camera placement messages.", this , SLOT( updateTopic() ) );
}

CameraFocusFrontend::~CameraFocusFrontend()
{
  delete toolbar_actions_;
  delete toolbar_;
}

void CameraFocusFrontend::addFocusButton( QString id )
{
  // load icon
  QPixmap icon_fg = rviz::loadPixmap( "package://pr2_interactive_manipulation_frontend/icons/"+id+".png" );
  icon_fg = icon_fg.scaled( 32, 32 );

  // load background image
  QPixmap icon_bg = rviz::loadPixmap( "package://pr2_interactive_manipulation_frontend/icons/icon_bg.svg" );

  // draw icon on top of bg
  QPainter painter( &icon_bg );
  painter.drawPixmap( 0, 0, icon_fg );

  // add action button to toolbar
  QString name  = icon_bg.isNull() ? id : "";
  QAction* action = new QAction( icon_bg, id, toolbar_ );
  action->setData( QVariant( id ) ); // store the focus id in the action
  toolbar_actions_->addAction( action );
  toolbar_->addAction( action );
}

void CameraFocusFrontend::onDisable()
{
  toolbar_->setVisible(false);
}

void CameraFocusFrontend::onEnable()
{
  toolbar_->setVisible(true);
}

void CameraFocusFrontend::onInitialize()
{
  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  QMainWindow* main_win = dynamic_cast<QMainWindow*>( window_context );

  if ( !main_win )
  {
    setStatusStd( rviz::StatusProperty::Error, "Initialization", "This display needs a QMainWindow as Window Manager." );
    return;
  }

  toolbar_ = main_win->addToolBar( "CameraFocus" );
  toolbar_->setObjectName( "CameraFocus" );
  toolbar_->setToolButtonStyle( Qt::ToolButtonTextBesideIcon );

  toolbar_actions_ = new QActionGroup( this );
  connect( toolbar_actions_, SIGNAL( triggered( QAction* )), this, SLOT( changeView( QAction* )));

  addFocusButton("left");
  addFocusButton("right");
  addFocusButton("facing");
  addFocusButton("top");
  //addFocusButton("front");
  addFocusButton("overhead");
  addFocusButton("kinect");

  updateTopic();
}

void CameraFocusFrontend::changeView( QAction* action )
{
  processButtonClick( action->data().toString().toStdString() );
}

void CameraFocusFrontend::updateTopic()
{
  pub_ = root_nh_.advertise<rviz_view_controllers::CameraPlacement>(topic_prop_->getStdString(), 5);
}

void CameraFocusFrontend::processButtonClick(const std::string &name)
{
  rviz_view_controllers::CameraPlacement cp;
  try
  {
    if(object_manipulator::cameraConfigurations().get_camera_placement(name.c_str(), cp.eye, cp.focus, cp.up))
    {
      cp.attached_frame = "";
      cp.time_from_start = ros::Duration(0.6);
      ROS_DEBUG_STREAM("Publishing camera placement request [" << name << "]: " << cp);
      pub_.publish(cp);
    }
    else
    {
      ROS_ERROR("There was an error processing pre-defined camera position [%s]", name.c_str());
    }
  }
  catch(...)
  {
    ROS_ERROR("There was an unhandled exception processing pre-defined camera position [%s]", name.c_str());
  }
}

}  // namespace



