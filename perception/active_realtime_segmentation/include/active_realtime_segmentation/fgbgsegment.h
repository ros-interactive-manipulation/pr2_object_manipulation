/*
 * Copyright (c) 2011, Mårten Björkman (celle@csc.kth.se) 
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  1.Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  2.Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.  
 *  3.The name of Mårten Björkman may not be used to endorse or
 *    promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/////////////////////////////////////////////////////////////////////////
/*! @mainpage Documentation of the Active Real-Time Segmentation Project
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for the implementation of the real-time
 * active 3D scene segmentation presented and described in the following
 * publication:
 *
 * <CODE>
 * \@InProceedings{bjorkman10,<BR>
 *      author =   {M&aring;rten Bj&ouml;rkman and Danica Kragic},<BR>
 *      title =   {{Active 3D Scene Segmentation and Detection of Unknown Objects}},<BR>
 *      booktitle =   "IEEE International Conference on Robotics and Automation (ICRA)",<BR>
 *      year =   "2010",<BR>
 *      pages= {3114 - 3120}<BR>
 *}<BR>
 * </CODE>
 *
 * It is based on an implementation of Belief Propagtion on the
 * GPU. Therefore CUDA should preferably be installed. However, the
 * algorithm can also run on the CPU.
 *
 * @section install_sec Installation
 *
 * @subsection step1 Dependencies
 * The following dependencies are optional but recommended:
 *  - CUDA
 *  - TBB (Intel &reg; Threading Building Blocks)
 * 
 * The following library can be included for compatability, but is not required
 *  - OpenCV
 *  
 * @subsection step2 Compilation
 * <CODE>
 * cd [YOUR_SOURCE_DIRECTORY] <BR>
 * mkdir build <BR>
 * cd build <BR>
 * cmake .. <BR>
 * make <BR>
 * </CODE>
 *
 * @subsection step3 Compiling the Documentation
 * <CODE>
 *  cd [YOUR_SOURCE_DIRECTORY]/build <BR>
 *  make doc <BR>
 * </CODE>
 *
 * @subsection step4 Usage
 *  <CODE>fgbgtable \<image format\> \<disparity format\> \<first index\> \<last index\> \<disparity range\> </CODE>
 * 
 * Example: <BR>
 * <CODE>
 * cd [YOUR_SOURCE_DIRECTORY]/build <BR>
 * bin/fgbgsegment ../segpics/clim%04d.pgm ../segpics/dimg%04d.pgm 1 9 64 <BR>
 * </CODE>
 */
/////////////////////////////////////////////////////////////////////////////


#ifndef FGBGSEGMENT_H
#define FGBGSEGMENT_H

#include <vector>
#include <stdint.h>
#include "pyra/tpimage.h"
#include "matrix3.h"

/** @file fgbgsegment.h
    Contains top-level segmentation class*/

class CudaSegment;

/// Top-level segmentation class
class FgBgSegment {

  friend class CudaSegment;

  //enum Labeling { BACKGROUND = 1, FLATSURFACE = 2, FOREGROUND = 3 };
  static const float eps = 1e-6f;
public:
  /// number of histogram bins
  static const int hist_size = 12;

  /// Color model class
  class ColorModel { 
  protected:
    /// Segmentation object
    FgBgSegment &segm;
    
  public: 
    static const float weight = 0.01;
  public:
    float histogram[hist_size*hist_size];
    float greyhist[hist_size];
    float colorcost[hist_size*hist_size];
    float greycost[hist_size];
    /// Prior Color model
    ColorModel *prior;
    /// Constructor
    /** @param segm Segmentation*/
    ColorModel(FgBgSegment &segm);
    /// Copy Constructor
    /** @param model Color model*/
    ColorModel(const ColorModel &model);
    /// Copy color and cost histograms to destination color model.
    ColorModel &operator=(const ColorModel &model);
    ~ColorModel();

    /// Creates the disparity histogram
    /** @param probabilities Probabilities of each pixel to belong to that specific model.
     *  @param allPoints If true all pixels are included. If false only those pixels with a disparity value within the range are considered.
     *  @return Sum of the probabilities*/
    float CreateHistogram(Image<float> &probabilities, bool allPoints);
    /// Normalises histograms
    void SmoothAndNormalizeHist( float const* hist, float const* phist,
				 int size, 
				 float* const histogram, float* const cost);
    void NormalizeHist( float* const histogram,
			float* const cost,
			int size);
    /// Creates the colour histograms
    /** @param mask Binary segmentation mask of which all inside the segmentation is considered for the histogram*/
    void CreateHistogram(Image<uint8_t> &mask, bool allPoints);
    /// Updates the histograms
    virtual void Update() { };
  };  
  
protected:
  
  /// Class modelling flat surface hypothesis 
  class FlatSurface : public ColorModel {
  public:
    static const float weight_a  = 100.0f*100.0f; 
    static const float weight_b = 50.0f*50.0f; 
    static const float weight_d = 1.0f*1.0f;
    static const float strength = 10.0f;
  public:
    float alpha, beta, disp, spread_d;
    float min_x, min_y, max_x, max_y;
    /// Probabilities for each pixel to belong to the flat surface.
    Image<float> probabilities; 
  public:
    /// Constructor
    /** Flat Surface Hypothesis 
     *  @param segm Segmentation
     *  @param width Width of the Image
     *  @param height Height of the Image
     */
    FlatSurface(FgBgSegment &segm, int width, int height);
    /// Initialize the flat surface hypothesis by using RANSAC and m-estimators in x,y,disparity space
    void Initialize();
    /// Update flat surface hypothesis
    virtual void Update();
  };
  
  /// Class modelling foreground hypothesis 
  class Foreground : public ColorModel {
  public:
    static const float weight_p = 0.005f*0.005f;
    static const float weight_d = 0.5f*0.5f;
    static const float strength = 10.0f;
  public:

    /// rough size of objects in window (dependent on focal length of camera)
    float window_size;
    float ball_size;     

    /// Whether position and disparity are combined into one vector and therefore if correlations between them are taken into account
    bool combined; 
    Vector3 position3d; 
    Matrix3 spread3d;  
    /// Probabilities for each pixel to belong to the foreground.
    Image<float> probabilities; 
  public:
    /// Constructor
    /** Foreground Hypothesis 
     *  @param segm Segmentation
     *  @param width Width of the Image
     *  @param height Height of the Image
     */
    Foreground(FgBgSegment &segm, int width, int height);
    /// Initialize the foreground hypothesis 
    /** Initialize the foreground hypothesis by placing an ellipsis around initial point in x,y,disparty space
     * @param startx x-Coordinate of initial point
     * @param starty y-Coordinate of initial point
     */
    void Initialize(int startx, int starty);
    /// Update foreground hypothesis
    virtual void Update();
    
    /// Explicitely set size of initialisation region of rough object size 
    /// in image
    void SetInitParams( float l_window_size, float l_ball_size);
    
  };
  
  /// Class modelling background hypothesis 
  class Background : public ColorModel {
  public:
    static const float weight_d = 0.1f*0.1f;
    static const float strength = 10.0f;
  public:
    float disp, spread_d;
    /// Probabilities for each pixel to belong to the background.
    Image<float> probabilities;
  public:
     /// Constructor
    /** Background Hypothesis 
     *  @param segm Segmentation
     *  @param width Width of the Image
     *  @param height Height of the Image
     */
    Background(FgBgSegment &segm, int width, int height);
     /// Initialize the background hypothesis
    void Initialize();
    /// Update background hypothesis
    virtual void Update();
  };
  
  /// Whether to use surface model
  bool withSurface;
  /// Whether to use colour cue
  bool withColors;     
  /// Whether to use disparity cue
  bool withDisparities;
  /// Whether data has incomplete color information
  bool withColorHoles;
  /* If colour information has holes, use uniform probabilities or 
     histograms over luminance
  */
  bool uniform;
   /// Amount of status output
  int verbose;                
  
   /// Background model
  Background ground;   
  /// Flat surface model
  FlatSurface surface;          
  /// Foreground model
  std::vector<Foreground*> figures;  

  /// Hue image data
  Image<uint8_t> hue;
  /// Saturation image data
  Image<uint8_t> saturation;
  /// Grey image data
  Image<uint8_t> grey;  
  /// Current disparity data
  Image<float> *disparities;        
   /// Prior color models
  std::vector<ColorModel> colorPriors;
  
  /// Image width
  int width;
  /// Image height
  int height;
  /// Disparity Range
  int drange;        
  /// Gradient Importance Weight
  float gradWeight;

  /// For controlling rough size of initialisation region around point
  float windowSize;
  float ballSize;
  
   /// CUDA based segmentation object
  CudaSegment *cudaSegment;          
  /// Whether GPU optimization is used
  bool gpuopt;                        
    
  /// Create all histograms 
  /** Create all histograms for foreground, background and flat surface hypotheses
   * @param allPoints Whether to use all points to create histograms or only those in the disparity range */
  void CreateHistograms(bool allPoints);
  /// Initialise the segmentation
  void InitSegmentation();
  /// CPU based segmentation object
  /** Templated CPU based segmentation object given the number of foreground objects to segment
   * @param segm Segmentation object
   */
  template <int numFigures> static void PixSegment(FgBgSegment &segm);
  /// Conversion of images from RGB to HSV
  /** Conversion of images from RGB to HSV
   *  @param cimg RGB image to convert to HSV
   */
  void RGBToHSV(Image<uint8_t> &cimg);

public:
  /// Constructor
  /** Segmentation
   *  @param width Width of the Image
   *  @param height Height of the Image
   *  @param drange Disparity range
   *  @param gradWeight  Gradient Importance Weight
   */
  FgBgSegment(int width, int height, int drange, float gradWeight = 20.0, 
	      float w_size = 0.20f, float b_size = 0.20f);
  ~FgBgSegment();
  /// Whether or not to use the GPU based belief propagation
  void UseGPU(bool gpuopt = true);
  /// Execution of Segmentation
  /** Execution of Segmentation on one image for a number of iterations 
   *  @param image Image to segment
   *  @param disp  Disparity Image 
   *  @param initialize Whether or not to reinitialise the models 
   *  @param loops How many iterations per image
   *  @param startx not used 
   *  @param starty not used
   */
  void Execute(Image<uint8_t> &image, Image<float> &disp, 
	       bool initialize, int loops = 1, 
	       int startx = -1, int starty = -1);
  /// Set new foreground hypothesis
  /** Reinitialize the foreground disparity model based on pixel position
   *  @param startx X-coordinate of ellipse used for initialization
   *  @param starty Y-coordinate of ellipse used for initialization
   *  @param dimg Disparity image on which initialisation is based
   *  @param drange_ Disparity range
   */
  void SetNewForeground(int startx, int starty, Image<float> &dimg, int drange_);
  /// Set new foreground hypothesis
  /** Reinitialize the foreground disparity model based on mask
   *  @param mask Binary mask
   *  @param dimg Disparity image on which initialisation is based
   *  @param drange_ Disparity range
   *  @param reuseLast Whether or not to re-use already existing foreground hypothesis */
  void SetNewForeground(Image<uint8_t> &mask, Image<float> &dimg, 
			int drange, bool reuseLast = false);
  
  /// Get Surface Parameters from current estimation
  void GetSurfaceParameters( float &alpha, float &beta, float &disp);
    
  
  void GetSurfaceMinMax( float &min_x,
			 float &min_y,
			 float &max_x,
			 float &max_y );
			 

  /// Set withColors Flag
  void SetWithColors( bool val);
  /// Get withColors Flag
  bool GetWithColors();
  
  /// Set withColorHoles Flag
  void SetWithColorHoles( bool val);
  /// Get withColorHoles Flag
  bool GetWithColorHoles();
  
  /// Set uniform Flag
  void SetUniform( bool val);
  /// Get uniform Flag
  bool GetUniform();
  
  /// Set withSurface Flag
  void SetWithSurface( bool val);
  /// Get withSurface Flag
  bool GetWithSurface();
  
  /// Set withDisparities Flag
  void SetWithDisparities( bool val);
  /// Get withDisparities Flag
  bool GetWithDisparities();

  /// Set gradient weight Flag
  void SetGradWeight( float val);
  /// Get current gradient weight Flag
  float GetGradWeight();
  
  /// Dump Segmentation Image
  /** Dump Segmentation Image to a file segm*.pgm
   * @param image Gray-level image to be Dumped
   */
  void MakeSegmentImage(Image<uint8_t> &image);
  /// Dump Segmentation Mask Image
  /** Dump Segmentation Mask Image to a file mask*.pgm
   * @param image Binary image to be Dumped
   * @param val Intensity value of the segmentation mask
   * @param obj ID of foreground hypothesis
   */
  void MakeMaskImage(Image<uint8_t> &image, int val = 255, int obj = 0);
  /// Dump Segmentation Image
  /** Dump Bordered Segmentation Image to a file segm*.ppm
   * @param image RGB image to be overlayed with segmentation border
   */
  void MakeBorderImage(Image<uint8_t> &image);

};

#endif // FGBGSEGMENT_H
