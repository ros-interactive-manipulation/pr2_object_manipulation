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

#ifndef TIMERCPU_H
#define TIMERCPU_H
/** @file timercpu.h
    Contains CPU Timer Class*/

/// CPU Timer Class
class TimerCPU
{
  static const int bits = 10;
public:
  /// Start time
  long long beg_clock;
  ///  clock frequency in MHz
  float freq;
  /// Constructor
  /** Sets start time
   *  @param freq_  clock frequency in MHz
   */
  TimerCPU(float freq_) : freq(freq_) {   // freq = clock frequency in MHz
    beg_clock = getTSC(bits);
  }
  /// Get current CPU time stamp
  long long getTSC(int bits) {
    unsigned int low, high;
    __asm__(".byte 0x0f, 0x31" :"=a" (low), "=d" (high));
    return ((long long)high<<(32-bits)) | ((long long)low>>bits);
  }
  
  /// Read execution time in ms
  /** Sets end time and computes difference in ms
   *  @return execution time
   */
  float read() {
    long long end_clock = getTSC(bits);
    long long Kcycles = end_clock - beg_clock;
    float time = (float)(1<<bits)*Kcycles/freq/1e3;
    return time;
  }
};

#endif
