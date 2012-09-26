/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include "object_segmentation_gui/object_segmenter.h"

#include "active_realtime_segmentation/pyra/tpimageutil.h"
#include <sensor_msgs/image_encodings.h>
#include "rviz_interaction_tools/image_tools.h"



namespace enc = sensor_msgs::image_encodings;

//Does not work right now, probably colliding with OGRE in Rviz
//#define USE_CUDA

namespace object_segmentation_gui {
  
  ObjectSegmenter::ObjectSegmenter( float grad_weight, int n_iter, 
				    bool use_gpu, float w_size,
				    float b_size) 
    : w_(640)
    , h_(480)
    , grad_weight_(grad_weight)
    , window_size_(w_size)
    , ball_size_(b_size)
    , use_gpu_(use_gpu)
    , n_iter_(n_iter)
    , init_(true)
    , num_fg_hypos_(0)
    , fgbgsegment(0)
    , gpu_image( w_, h_)
    , gpu_disparities( w_, h_)
    , init_mask( w_, h_)
    , segm_mask( w_, h_)
    , thread_(0)
  {}

  ObjectSegmenter::~ObjectSegmenter()
  {    
    if(thread_!=0){
      stopThread();
      thread_->join();
      delete thread_;
      thread_=0;
    }
    
    if(fgbgsegment!=0){
      delete fgbgsegment;
    }
    
  }
  
  void ObjectSegmenter::setNewData( const sensor_msgs::Image &image, 
				    const stereo_msgs::DisparityImage &disparity)
  {
    stereo_msgs::DisparityImage local_disp = disparity;
    
    // get the necessary parameters from the data 
    w_ = image.width;
    h_ = image.height;
    drange_ = disparity.max_disparity - disparity.min_disparity;
    ROS_DEBUG("Image Width %d, Image Height %d", w_, h_);
    ROS_DEBUG("Disparity Range %d", drange_);
    ROS_DEBUG("Disparity steps %f", disparity.delta_d);
    ROS_DEBUG("Min and Max Disparity %f %f", 
	      disparity.min_disparity, disparity.max_disparity);

    // make sure that invalid disparities are always below zero 
    // and in the range between 0 and max_disparity-min_disparity
    // since that is the assumption made in the segmentation
    for(int x=0; x<w_; ++x){
      for(int y=0; y<h_; ++y){
	if(!rviz_interaction_tools::hasDisparityValue(disparity, y, x)){
	  float val;
	  rviz_interaction_tools::getValue(disparity.image, y, x, val);
	  val =- disparity.min_disparity;
	  rviz_interaction_tools::setValue(local_disp.image, y, x, -1.0f);
	}
      }
    }
    
    // memory align the RGB and disparity image
    gpu_image.SetDataAlign(image, 3*w_, h_, true);
    gpu_disparities.SetDataAlign(local_disp.image, w_, h_);
    
    //    gpu_disparities.Store("/wg/stor2a/jbohg/KinectDepth.ppm", true, false);

    init_mask.SetSize(w_,h_);
    segm_mask.SetSize(w_,h_);
    
    Fill(init_mask, (uchar)0);
    Fill(segm_mask, (uchar)0);
    // Create the segmentation object
    if(fgbgsegment!=0)
      delete fgbgsegment;
    fgbgsegment = new FgBgSegment(w_, h_, drange_, grad_weight_, 
				  window_size_, ball_size_);
    
#ifdef USE_CUDA
    fgbgsegment->UseGPU(use_gpu_);
#else 
    if(use_gpu_)
      ROS_WARN("CUDA support not enabled! Running segmentation on CPU.");
    fgbgsegment->UseGPU(false);
	
#endif //USE_CUDA
    
    // After new data has been obtained, segmentation has to be initialised
    init_ = true;
    
    startThread( boost::bind( &ObjectSegmenter::doSegment, this) );
  }
  
  void ObjectSegmenter::queueAction( const Action &action)
  {
    boost::mutex::scoped_lock lock(queue_mutex_);
    if(action.type_==STOP)
      action_queue_.clear();
    action_queue_.push_back(action);
    cond_var_.notify_all();
    
  }

  
  bool ObjectSegmenter::dequeueAction( Action &action)
  {
    
    boost::mutex::scoped_lock lock(queue_mutex_);
    
    if(!action_queue_.empty()){
      action = action_queue_[0];
      action_queue_.pop_front();
      return true;
    } else {
      return false;
    }
    
  }

  void ObjectSegmenter::clearQueue()
  {
    boost::mutex::scoped_lock lock(queue_mutex_);
    action_queue_.clear();
  }

  void ObjectSegmenter::waitForAction(){
    
    boost::mutex::scoped_lock q_lock(queue_mutex_);
    
    while(init_ && action_queue_.empty()){
      cond_var_.wait(q_lock);
    }
  }
  
  void ObjectSegmenter::pause()
  {
    ROS_INFO("Entering pause");
    boost::mutex::scoped_lock q_lock(queue_mutex_);
    ROS_INFO("Before pause condition variable waiting");
    cond_var_.wait(q_lock);
    ROS_INFO("After pause condition variable waiting");
    
  }

  bool ObjectSegmenter::getCurrentResult( sensor_msgs::Image &image)
  {
    
    boost::mutex::scoped_lock lock(image_mutex_);
    fillImage(image, segm_mask);
    return true;
    
  }


  void ObjectSegmenter::setCurrentResult( )
  {
    boost::mutex::scoped_lock lock(image_mutex_);
    fgbgsegment->MakeSegmentImage(segm_mask);
    fgbgsegment->GetSurfaceParameters( alpha_, beta_, gamma_);
  }

  bool ObjectSegmenter::validResult( const Image<uint8_t> &segm_mask)
  {
    boost::mutex::scoped_lock lock(image_mutex_);
   
    int w = segm_mask.GetWidth();
    int h = segm_mask.GetHeight();
    
    uint8_t *segm_d = segm_mask.GetData();
    
    std::vector<int> segm_size;
    segm_size.resize(num_fg_hypos_+1,0);
    
    for(int i=0; i<w; ++i)
      for(int j=0; j<h; ++j){
	int adr = j*w+i;
	int label = segm_d[adr];
	if(label>1)
	  segm_size[label-1]++;
	else if (label==1)
	  segm_size[0]++;
      }
   
    int sum = 0;
    for(int i=1; i<(int)segm_size.size(); ++i)
      sum += segm_size[i];

    if((int)segm_size.size()>1 && sum==0)
      return false;
    else 
      return true;
  }
  
  void ObjectSegmenter::getCurrentSurface(float &alpha, float &beta, 
					  float &gamma)
  {
    boost::mutex::scoped_lock lock(image_mutex_);
    alpha = alpha_;
    beta  = beta_;
    gamma = gamma_;
  }
  
  void ObjectSegmenter::getCurrentSegmFlags( bool &with_colors, 
					     bool &with_color_holes,
					     bool &uniform,
					     bool &with_disparities,
					     bool &with_surface )
  {
    with_colors = fgbgsegment->GetWithColors();
    with_color_holes = fgbgsegment->GetWithColorHoles();
    uniform = fgbgsegment->GetUniform();
    with_disparities = fgbgsegment->GetWithDisparities();
    with_surface = fgbgsegment->GetWithSurface();
  }

  void ObjectSegmenter::setCurrentSegmFlags( bool with_colors, 
					     bool with_color_holes,
					     bool uniform,
					     bool with_disparities,
					     bool with_surface )
  {
    fgbgsegment->SetWithColors(with_colors);
    fgbgsegment->SetWithColorHoles(with_color_holes);
    fgbgsegment->SetUniform(uniform);
    fgbgsegment->SetWithDisparities(with_disparities);
    fgbgsegment->SetWithSurface(with_surface);
  }
  
  // execute by segmentation thread
  void ObjectSegmenter::doSegment()
  {

    bool stop = false;

    while(true){
      
      // while no seed points have been set, wait for control input
      waitForAction();
      
      fgbgsegment->Execute(gpu_image, gpu_disparities, init_, n_iter_);
      if(init_)
	init_ = false;
      
      setCurrentResult();
      
      Action ac;
      if( dequeueAction(ac) ){
	switch(ac.type_){
	case REGION:
	  fillInitMask(init_mask, ac.box_.p_tl_.x, ac.box_.p_tl_.y, 
		       ac.box_.p_br_.x, ac.box_.p_br_.y);
	  fgbgsegment->SetNewForeground(init_mask, gpu_disparities, drange_);
	  num_fg_hypos_++;
	  break;
	case CLICK:
	  if( !(ac.box_.p_tl_.x==-1 &&  ac.box_.p_tl_.y==-1)){
	    fgbgsegment->SetNewForeground( ac.box_.p_tl_.x, ac.box_.p_tl_.y, 
					   gpu_disparities, drange_);
	    num_fg_hypos_++;
	  }
	  break;
	case RESET:
	  setCurrentSegmFlags(ac.with_colors_,
			      ac.with_color_holes_,
			      ac.uniform_,
			      ac.with_disparities_,
			      ac.with_surface_);
	  init_ = true;
	  Fill(init_mask, (uchar)0);
	  Fill(segm_mask, (uchar)0);
	  break;
	case GRADCHANGE:
	  fgbgsegment->SetGradWeight( ac.grad_weight_);
	  break;
	case STOP:
	  stop = true;
	  clearQueue();
	  break;
	case PAUSE:
	  pause();
	  break;
	}
      }

      if(stop)
	break;
    }

  }

  template <typename Callable>
  void ObjectSegmenter::startThread( Callable f )
  {
    if ( thread_ != 0) {
      thread_->join();
      delete thread_;
    }
    thread_ = new boost::thread( f );
  }

  void ObjectSegmenter::stopThread(){
    
    ObjectSegmenter::Action action;
    action.type_ = ObjectSegmenter::STOP;
    queueAction(action);
  }

  // produce mask image for segment initialisation
  void ObjectSegmenter::fillInitMask( Image<uint8_t> &init_mask, 
				      int start_x, int start_y, 
				      int stop_x, int stop_y){

    uint8_t *init_d = init_mask.GetData();
    Fill(init_mask, (uchar)0);

    if(start_x>stop_x)
      swap(start_x, stop_x);
    
    if(start_y>stop_y)
      swap(start_y, stop_y);
    
    for(int y=start_y; y<stop_y; ++y){
      for(int x=start_x; x<stop_x; ++x){
	int i = y*w_ + x;
	init_d[i] = 1;
      }
    }
  }


void ObjectSegmenter::fillImage(sensor_msgs::Image &image, 
				const Image<uint8_t> &segm_mask)
  {
    uint8_t *segm_d = segm_mask.GetData();
    
    image.header.frame_id = "narrow_stereo_optical_frame";
    image.header.stamp = ros::Time::now();
    image.height = segm_mask.GetHeight();
    image.width  = segm_mask.GetWidth();
    image.encoding = enc::MONO8;
    image.is_bigendian = false;
    image.step = segm_mask.GetWidth();
    size_t size = image.step * image.height;
    image.data.resize(size);
    
    for(unsigned int x=0; x<image.width; ++x){
      for(unsigned int y=0; y<image.height; ++y){
	int i = y * image.width + x;
	image.data[i] = segm_d[i]; 
      }
    }
  }
  
  
}

