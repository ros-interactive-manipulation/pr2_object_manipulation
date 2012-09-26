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

#ifndef BELIEF_H
#define BELIEF_H
/** @file belief.h
    Contains CPU belief propagation*/

/// Templated Belief Propagation Class
/** Performs loopy belief propagation on CPU
 *  @param dim Number of hypotheses
 */
template<int dim = 3>
class BeliefProp {
  /// Messages to left
  float *msgl[dim];
  /// Messages to right
  float *msgr[dim];
  /// Messages up
  float *msgu[dim];
  /// Messages down
  float *msgd[dim];
  /// Prior probabilities
  float *prior[dim];
  /// Labeling beliefs
  float *belief[dim];
  /// horizontal costs
  float *costh;
  /// vertical costs
  float *costv;
  /// Image width
  int width;
  /// Image height
  int height;
private:
  /// Initialise the Messages
  void InitMessages();
  /// Compute the Beliefs
  void ComputeBeliefs();
  /// Update the Messages
  void UpdateMessages();
public:
  /// Constructor
  /** @param w Image width
   * @param h Image height
   */
  BeliefProp(int w, int h);
  ~BeliefProp();
  /// Get  Prior probabilities
  /** @return prior probabilities
   */
  float **GetPriors();
  /// Get labelling beliefs 
  /** @return labelling beliefs 
   */
  float **GetBeliefs();
  /// Set the gradient costs
  /** 
   * @param img Gray-level image
   * @param gamma Gradient weight
   */
  void SetGradientCosts(Image<unsigned char> &img, float gamma);
  /// Perform Belief Propagation
  /** Perform loopy belief propagation on CPU
   *  @param loops Number of iterations
   */
  void Execute(int loops);
  /// Perform Belief Propagation
  /** Perform loopy belief propagation on CPU
   *  @param loops Number of iterations
   *  @param depth Depth of Propagation
   */
  void Execute(int loops, int depth);
  /// Compute Posterior with Graph Cuts
  /**  Compute Posterior with Graph Cuts (Not used right now)
   *  @param mask Not used right now
   */
  void ComputeMAP(Image<unsigned char> &mask);
};

#include "belief.cpp"

#endif // BELIEF_H

