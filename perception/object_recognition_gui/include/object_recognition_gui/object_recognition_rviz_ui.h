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

#ifndef OBJECT_RECOGNITION_RVIZ_UI
#define OBJECT_RECOGNITION_RVIZ_UI

#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTextBrowser>

#include "object_recognition_gui/ObjectRecognitionGuiAction.h"

#include <boost/thread/mutex.hpp>

#include <actionlib/server/simple_action_server.h>

#include "object_recognition_gui/ObjectRecognitionGuiAction.h"


namespace rviz_interaction_tools {
class MeshObjectSwitcher;
class ImageOverlay;
}

namespace rviz {
class WindowManagerInterface;
class RenderPanel;
class DisplayContext;
}

namespace Ogre {
class SceneNode;
class RaySceneQuery;
class SceneManager;
}

namespace Ui {
class ObjectRecognitionFrame;
}

namespace object_recognition_gui {

class MouseEventSignallingRenderPanel;

class ObjectRecognitionRvizUI : public QWidget
{
Q_OBJECT
public:
  ObjectRecognitionRvizUI(rviz::DisplayContext* context);
  virtual ~ObjectRecognitionRvizUI();

  //called regularly; updates the display.
  void update(float wall_dt, float ros_dt);

  //start listening to action goals
  void startActionServer( ros::NodeHandle &node_handle );

  //stop action server, cancel current goal & hide if necessary
  void stopActionServer();

  void showBoundingBoxes( bool show );
  bool getShowBoundingBoxes();

protected Q_SLOTS:
  void acceptButtonClicked();
  void cancelButtonClicked();
  void onRenderWindowMouseEvent( QMouseEvent* event );
  void updateBoundingBoxes();

protected:

  // callback for new action server goals
  void acceptNewGoal();

  // callback for action server preempt (cancel) requests
  void preempt();


  //cleanup ogre scene, hide window
  void cleanupAndHide();

  //create the different red and green ogre materials
  void createMaterials();

  // Configure the RenderPanel created by the UI file.
  void setupRenderPanel( rviz::DisplayContext* context );

  //get meshes from message into ogre
  void parseMeshes(const std::vector<object_recognition_gui::ModelHypothesisList> &model_hyp_list);

  //ogre stuff
  MouseEventSignallingRenderPanel* render_panel_;
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* scene_root_;

  //used for object selection
  Ogre::RaySceneQuery* ray_scene_query_;

  //displays the camera image
  rviz_interaction_tools::ImageOverlay *image_overlay_;

  //displays the model hypothesis meshes
  std::vector< rviz_interaction_tools::MeshObjectSwitcher* > mesh_switchers_;

  actionlib::SimpleActionServer<ObjectRecognitionGuiAction> *object_recognition_server_;

private:
  Ui::ObjectRecognitionFrame *ui_; // UI object created by uic from object_recognition_frame.ui
};

}

#endif
