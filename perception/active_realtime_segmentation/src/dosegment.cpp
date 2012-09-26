#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <vector>
#ifdef USE_TBB
#include <tbb/tbb.h>
#endif // USE_TBB
#include "pyra/tpimageutil.h"

#include "fgbgsegment.h"
#include "timercpu.h"

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif // USE_OPENCV

/** @file dosegment.cpp
 *  Image IO and execution of segmentation on data.
 */


int main(int argc, char *argv[])
{
  if (argc<6) {
    std::cerr << "Usage: " << argv[0] << " <image format> <disparity format> <first index> <last index> <disparity range>" << std::endl;
    std::cout << "   Ex:" << argv[0] <<" ../demo/clim%04d.pgm ../demo/dimg%04d.pgm 1 9 64" << std::endl;
    return 1;
  }
  // Read arguments
  char *image_format = argv[1];
  char *disp_format = argv[2];
  int fstidx = atoi(argv[3]);
  int lstidx = atoi(argv[4]);
  int drange = atoi(argv[5]);
  if (lstidx<fstidx) {
    int tmpidx = fstidx; 
    fstidx = lstidx; 
    lstidx = tmpidx;  
  }
  int w = 640;
  int h = 480;
  /* Initialise Segmenter with width and height of images, disparity range and 
   * gradient cost
   */
  FgBgSegment fgbgsegment(w, h, drange, 30.0);
#ifdef USE_CUDA
  fgbgsegment.UseGPU(false);
#else 
  fgbgsegment.UseGPU(false);
#endif //USE_CUDA
  Image<uint8_t> image(3*w, h);
  Image<float> disparities(w, h);
  for (int i=fstidx;i<=lstidx;i++) {
    char name[80];
    sprintf(name, image_format, i); //%%%%
    /* Interfaces to read in images and convert them to 16 bit aligned images. */
#ifdef USE_OPENCV
    cv::Mat tmp_img = cv::imread(name, 1);
    if(tmp_img.empty()) {
      std::cout << "Image " << name << " does not exist " << std::endl;
      exit(-1);
    }
    cv::cvtColor(tmp_img, tmp_img, CV_BGR2RGB);
    image.SetDataAlign(tmp_img.ptr<uchar>(0), 3*w, h);
#else
    if(!image.LoadRGB(name)){
      std::cout << "Image " << name << " does not exist " << std::endl;
      exit(-1);
    }
#endif //USE_OPENCV
    sprintf(name, disp_format, i); //%%%%
#ifdef USE_OPENCV
    cv::Mat tmp_disp = cv::imread(name, 0);
    if(tmp_disp.empty()) {
      std::cout << "Image  " << name << " does not exist " << std::endl;
      exit(-1);
    }
    cv::Mat tmp_disp_fl;
    tmp_disp.convertTo(tmp_disp_fl,CV_32FC1);
    disparities.SetDataAlign(tmp_disp_fl.ptr<float>(0), w, h);
#else
    if(!disparities.Load(name)){
      std::cout << "Image " << name << " does not exist " << std::endl;
      exit(-1);
    }
#endif  //USE_OPENCV
    TimerCPU timer(2800);
    /* Segmentation is executed on RGB image and disparity for 10 iterations.
     * If third argument is true, segmentation is initialised.
     */
    fgbgsegment.Execute(image, disparities, (i==fstidx), 10);
    if (i==fstidx)
       /* Default KTH: Initial Foreground in image center due to gaze shifts and fixation mechanism */
      fgbgsegment.SetNewForeground(w/2, h/2, disparities, drange);
    
    // Helper function to monitor segmentation and store intermediate results.
    std::cout << "Loop time: " << timer.read() << " ms" << std::endl;
    Image<uint8_t> segm(w, h);
    fgbgsegment.MakeSegmentImage(segm);
    sprintf(name, "segm%04dx.pgm", i);
    segm.Store(name, true, false); 
    fgbgsegment.MakeBorderImage(image);
    sprintf(name, "segm%04dx.ppm", i);
    image.StoreRGB(name);  
    Image<uint8_t> mask(w, h);
    fgbgsegment.MakeMaskImage(mask, 255, 1);
    sprintf(name, "mask%04dx.pgm", i);
    mask.Store(name);
    //if (i==lstidx) i = fstidx + 1;
  }
  return 0;
}

