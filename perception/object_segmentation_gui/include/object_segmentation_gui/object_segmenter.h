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

#ifndef SEGMENTATION_INTERFACE
#define SEGMENTATION_INTERFACE

#include "object_segmentation_gui/ObjectSegmentationGuiAction.h"
#include "object_segmentation_gui/utils.h"

#include "active_realtime_segmentation/fgbgsegment.h"
#include "active_realtime_segmentation/pyra/tpimage.h"

#include <sensor_msgs/Image.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <deque>

namespace object_segmentation_gui {
  
  typedef geometry_msgs::Point32    Point;

  class ObjectSegmenter
  {
  public:
    
    enum ActionType{PAUSE, RESET, CLICK, REGION, STOP, GRADCHANGE};

    struct Box2D
    {
      Point p_tl_;
      Point p_br_;
    };
    
    struct Action
    {
      ActionType type_;
      Box2D box_;
      
      // cues and model flags for segmentation
      bool with_colors_;
      bool with_color_holes_;
      bool uniform_;
      bool with_disparities_;
      bool with_surface_;

      float grad_weight_;
      
    };

    ObjectSegmenter( float grad_weight, int n_iter, 
		     bool use_gpu, float w_size = 0.20f, 
		     float b_size = 0.20f);
    ~ObjectSegmenter();
    
    void setNewData( const sensor_msgs::Image &image_, 
		     const stereo_msgs::DisparityImage &disparity_);
    
    void queueAction( const Action &action);
    
    bool getCurrentResult( sensor_msgs::Image &image);
    void getCurrentSurface(float &alpha, float &beta, float &gamma);

    void getCurrentSegmFlags( bool &with_colors, 
			      bool &with_color_holes_,
			      bool &uniform_,
			      bool &with_disparities_,
			      bool &with_surface_ );

    void setCurrentSegmFlags( bool with_colors, 
			      bool with_color_holes,
			      bool uniform,
			      bool with_disparities,
			      bool with_surface );

  protected:
    
  private:
    
    // execute by segmentation thread
    void doSegment();

    void stopThread();

    void setCurrentResult();
    bool dequeueAction( Action &action);
    void clearQueue();

    void waitForAction();

    void pause();

    // start thread
    template <typename Callable>
      void startThread( Callable f );
    
    // stop segmentation thread
    
    // produce mask image for segment initialisation
    void fillInitMask( Image<uint8_t> &init_mask, int start_x, int start_y, int stop_x, int stop_y);
    
    // copy segmentation labels to ROS image
    void fillImage( sensor_msgs::Image &image, const Image<uint8_t> &segm_mask);
    
    bool validResult( const Image<uint8_t> &segm_mask);
   
    int w_;
    int h_;
    int grad_weight_;
    int drange_;

    float window_size_;
    float ball_size_;

    bool use_gpu_;
    int  n_iter_;

    bool init_;

    int num_fg_hypos_;

    FgBgSegment *fgbgsegment;
    
    // RGB and Disparity images with correct memory alignment for GPU
    Image<uint8_t> gpu_image;
    Image<float> gpu_disparities;

    // mask for rectangular initialisation
    Image<uint8_t> init_mask;

    // stores labling results
    Image<uint8_t> segm_mask;
    float alpha_, beta_, gamma_; 
    
    // stores most recent control message
    std::deque<Action> action_queue_;
    
    boost::thread* thread_;
    boost::mutex   queue_mutex_;
    boost::mutex   image_mutex_;

    boost::condition_variable cond_var_;

  };
}

#endif // SEGMENTATION_INTERFACE
