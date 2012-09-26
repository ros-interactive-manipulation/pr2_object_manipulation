#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include "pyra/tpimageutil.h"

#include "fgbgsegment.h"
#include "timercpu.h"


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

/** @file dosegment_node.cpp
 *  Image IO with ROS topics and execution of segmentation of data.
 *  NOTE: PR2 got only monochrome narrow stereo images, 
 *  RGB images therefore with gray level info on every channel
 */

/// ROS Node class
class ActiveRealTimeSegmenter
{
public:
  /// Constructor
  ActiveRealTimeSegmenter():
    nh_("")
  {
  }
  
  /// Destructor
  ~ ActiveRealTimeSegmenter()
  {
  }

  /// Execute Segmentation
  /** 
   * @param loops how many loops should segmentation run  
   */
  void execute( int loops );
  
private:

  /// Read sensor data from topics
  /** Read sensor data from topics and store
   * @param recent_image Storage for image sensor_msg
   * @param recent_disparity_image Storage for disparity image
   * @param time_out How long it is tried to get data 
   */
  bool assembleSensorData(sensor_msgs::Image::ConstPtr &recent_image,
			  stereo_msgs::DisparityImage::ConstPtr &recent_disparity_image,
			  ros::Duration time_out);

  /// The node handle
  ros::NodeHandle nh_;
};

bool ActiveRealTimeSegmenter::assembleSensorData(sensor_msgs::Image::ConstPtr &recent_image,
						 stereo_msgs::DisparityImage::ConstPtr &recent_disparity_image,
						 ros::Duration time_out)
{
  ROS_INFO("Segmentation through User Interaction: waiting for messages...");
  std::string image_topic("/narrow_stereo/left/image_rect");
  std::string disparity_image_topic("/narrow_stereo_textured/disparity");
  
  ros::Time start_time = ros::Time::now();
  while (!recent_image || !recent_disparity_image )
    {
      if (!recent_image)
	{
	  ROS_INFO_STREAM("  Waiting for message on topic " << image_topic);
	  recent_image = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic, nh_, ros::Duration(0.5));
	}
      if (!recent_disparity_image)
	{
	  ROS_INFO_STREAM("  Waiting for message on topic " << disparity_image_topic);
	  recent_disparity_image = ros::topic::waitForMessage<stereo_msgs::DisparityImage>
	    (disparity_image_topic, nh_, ros::Duration(0.5));
	}
      
      ros::Time current_time = ros::Time::now();
      if (time_out >= ros::Duration(0) && current_time - start_time >= time_out)
	{
	  ROS_INFO("Timed out");
	  return false;
	}
    }
  return true;
}

void ActiveRealTimeSegmenter::execute( int loops )
{
  // Read number of loops
  int fstidx = 0;
  int lstidx = loops;

  //Getting ROS messages 
  
  ros::NodeHandle nh_;
  
  // The image and disparity image
  sensor_msgs::Image::ConstPtr ros_image;
  stereo_msgs::DisparityImage::ConstPtr disparity_image;
  
  if(assembleSensorData(ros_image, disparity_image,ros::Duration(5.0)))
    ROS_INFO("All required messages received");
  else 
    ROS_INFO("Could not gather all required messages.");

  int w = ros_image->width;
  int h = ros_image->height;

  ROS_ASSERT(w==disparity_image->image.width && h==disparity_image->image.height);

  int drange = (int)(disparity_image->max_disparity-disparity_image->min_disparity);
  std::cout << "Disparity Range " << drange << std::endl;

  /* Initialise Segmenter with width and height of images, disparity range and 
   * gradient cost
   */
  FgBgSegment fgbgsegment(w, h, drange, 50.0);
#ifdef USE_CUDA
  fgbgsegment.UseGPU(true);
#else 
  fgbgsegment.UseGPU(false);
#endif //USE_CUDA
  Image<uint8_t> image(3*w, h);
  Image<float> disparities(w, h);
  
  /* Since PR2 got only monochrome narrow stereo images
   * make RGB image with all three channels having the same value.
   * Therefore, last argument set to true. For true RGB or single 
   * channel images, set to false.
   */ 
  image.SetDataAlign(*ros_image, 3*w, h, true);
  disparities.SetDataAlign(disparity_image->image, w, h);
  
  // Just for debugging
  // image.StoreRGB("CurrentImage.pgm");
  // disparities.Store("CurrentDisparityImage.pgm");
  
  for (int i=fstidx;i<=lstidx;i++) {
    char name[80];
    
    TimerCPU timer(2800);
    // number of iterations for belief propagation
    int n_iter = 5;
    /* Segmentation is executed on RGB image and disparity for n_iter iterations.
     * If third argument is true, segmentation is initialised.
     */
    fgbgsegment.Execute(image, disparities, (i==fstidx), n_iter);
    if (i==fstidx) {
      /* Default KTH: Initial Foreground in image center due to gaze shifts and fixation mechanism */
      //      fgbgsegment.SetNewForeground(w/2, h/2, disparities, drange);
      /* New foreground object is initialised around specific point (here (400,100)). */
      fgbgsegment.SetNewForeground(400, 100, disparities, drange);
    }
    std::cout << "Loop time: " << timer.read() << " ms" << std::endl;

    // Helper function to monitor segmentation and store intermediate results.
    Image<uint8_t> segm(w, h);
    fgbgsegment.MakeSegmentImage(segm);
    sprintf(name, "segm%04dx.pgm", i);
    segm.Store(name, true, false); 
    Image<uint8_t> border(3*w, h);
    border = image;
    fgbgsegment.MakeBorderImage(border);
    sprintf(name, "segm%04dx.ppm", i);
    border.StoreRGB(name);  
    Image<uint8_t> mask(w, h);
    fgbgsegment.MakeMaskImage(mask, 255, 1);
    sprintf(name, "mask%04dx.pgm", i);
    mask.Store(name);
  }

}

int main(int argc, char **argv)
{
  if (argc<2) {
    std::cerr << "Usage: " << argv[0] << " <loops>" << std::endl;
    std::cout << "   Ex:" << argv[0] << "  10" << std::endl;
    return 1;
  }

  int loops = atoi(argv[1]);

  ros::init(argc, argv, "active_realtime_segmentation_node");
  ActiveRealTimeSegmenter node;
  node.execute(loops);
  return 0;
}
