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

#ifndef CUDASEGMENT_H
#define CUDASEGMENT_H

#include "pyra/tpimage.h"
/** @file cudasegment.h
    Contains CUDA segmentation class*/

typedef unsigned char uchar;
class FgBgSegment;

/// CUDA Segmentation class
class CudaSegment 
{
public:
  /// Maximum number of foreground objects
  static const int max_figures = 8;
private:
  /// Image width
  int width;
  /// Image height
  int height;     
  /// Memory allocation block pointer
  uchar *allocated_d;
  /// Colour image
  uchar *cimd_d;
  /// Gray-level image
  uchar *vimd_d;
  ///  Disparity map
  float *dimd_d;
  /// Colour histogram bin indices
  uint *bins_d;          
  /// Partial gradient magnitude sums
  int *sums_d;                  
  /// Horizontal Gradient costs
  float *costh_d;
  /// Vertical Gradient costs
  float *costv_d;        
  /// Prior label probabilities
  float *priors_d[max_figures+2]; 
  /// Posterior label beliefs
  float *beliefs_d[max_figures+2];
  /// Posterior label probabilities
  float *probs_d[max_figures+2]; 

  /// Create all histograms 
  /** Create all histograms for foreground, background and flat surface hypotheses. Currently not used because works only for smal histograms.
   * @param segment Segmentation object*/
  void CreateHistograms(FgBgSegment &segment);
  /// Compute the surface parameter beta
   /** Compute the surface parameter beta
   * @param image_d Disparity image
   * @return Beta
   */
  float ComputeBeta(uchar *image_d);
  /// Set gradient costs
  /** Set gradient costs
   *  @param image_d Disparity image
   *  @param beta surface parameter beta
   *  @param gamma
   */
  void SetGradientCosts(uchar *image_d, float beta, float gamma);
public:  
  /// Constructor
  /** Segmentation
   *  @param width Width of the Image
   *  @param height Height of the Image
   */
  CudaSegment(int width, int height);
  ~CudaSegment();
  /// Copy probabilities to GPU
  void CopyProbsToDevice(Image<float> **probs_h);
  /// Execute Segmentation
  /** Execute Segmentation on GPU
   *  @param segment Segmentation Object
   *  @param cimg RGB input Image
   *  @param dimg Disparity Image
   *  @param numFigures Number of foreground objects
   *  @param download Whether or not to copy new image data onto GPU
   */
  void Execute(FgBgSegment &segment, Image<uchar> &cimg, 
	       Image<float> &dimg, int numFigures, 
	       bool download = true);
};


#endif // CUDASEGMENT_H
 
