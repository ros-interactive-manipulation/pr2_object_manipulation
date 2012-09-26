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

#ifndef CUDABELIEF_H
#define CUDABELIEF_H
/** @file cudabelieft.h
    Contains CUDA belief propagation*/

/// Templated Belief Propagation
/** Templated Belief Propagation given a foreground object id
 *  @param dim ID of foreground object
 */
template<int dim>
class CudaBelief {
private:
  /// Whether or not to allocate space on GPU
  bool copyFromHost;
  /// Allocated space on GPU
  unsigned char *allocated_d;
  /// Messages to left and right
  float *msgl_d[2*dim], *msgr_d[2*dim];
  /// Messages to top and bottom
  float *msgu_d[2*dim], *msgd_d[2*dim];
  /// Space for priors and beliefs
  float *prior_d[dim], *belief_d[dim];
  /// Image size
  int width, height;
private:
  /// Execute Belief Propagation
  /** Execute Loopy Belief Propagation on GPU for some iterations
   *  @param loop Number of iterations
   */
  void ComputeBeliefs(int loop);
  /// Collect the messages send from node to node
  /** Collect the messages send from node to node
   *  @param costh_d Space for horizontal messages (left and right)
   *  @param costv_d Space for vertical messages (top and bottom)
   *  @param loop Loop id
   */
  void CollectMessages(float *costh_d, float *costv_d, int loop);
public:
  /// Constructor
  /** Segmentation
   *  @param w Width of the Image
   *  @param h Height of the Image
   *  @param copyFromHost Whether or not to allocate memory on the GPU for priors and beliefs
   */
  CudaBelief(int w, int h, bool copyFromHost = true);
  ~CudaBelief();
  /// Execute Belief propagation on GPU
  /** Execute Belief propagation on GPU
   *  @param priors Prior probabilities     
   *  @param beliefs Space for beliefs
   *  @param costh_d Horizontal costs
   *  @param costv_d Vertical costs
   */
  void Execute(float **priors, float **beliefs, float *costh_d, float *costv_d);
  /// Execute Belief propagation on GPU
  /** Execute Belief propagation on GPU
   *  @param priors Prior probabilities     
   *  @param beliefs Space for beliefs
   *  @param costh_d Horizontal costs
   *  @param costv_d Vertical costs
   *  @param loops Number of iterations
   *  @param depth Depth of propagation
   */
  void Execute(float **priors, float **beliefs, float *costh_d, float *costv_d, int loops, int depth);
};

#include "cudabelieft.cu"

#endif // CUDABELIEF_H
 
