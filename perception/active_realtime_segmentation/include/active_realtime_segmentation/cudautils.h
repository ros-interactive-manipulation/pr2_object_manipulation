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

#ifndef CUDAUTILS_H
#define CUDAUTILS_H
/** @file cudautils.h
    CUDA utility functions*/

#include <cstdio>
#include <iostream>

#define safeCall(err)       __safeCall(err, __FILE__, __LINE__)
#define safeThreadSync()    __safeThreadSync(__FILE__, __LINE__)
#define checkMsg(msg)       __checkMsg(msg, __FILE__, __LINE__)

/// Safe call of CUDA function 
/** Allows safe call of CUDA function by catching thrown errors
 *  @param err CUDA Runtime API error
 *  @param file Name of source code file
 *  @param line Index of line in source code  
 */
inline void __safeCall(cudaError err, const char *file, const int line)
{
  if (cudaSuccess != err) {
    fprintf(stderr, "safeCall() Runtime API error in file <%s>, line %i : %s.\n", file, line, cudaGetErrorString(err));
    exit(-1);
  }
}

/// Safe CUDA thread synchronisation
/** Allows safe call of cudaThreadSynchronize() to catch threadSynchronize() Driver API error
 *  @param file Name of source code file
 *  @param line Index of line in source code  
 */
inline void __safeThreadSync(const char *file, const int line)
{
  cudaError err = cudaThreadSynchronize();
  if (cudaSuccess != err) {
    fprintf(stderr, "threadSynchronize() Driver API error in file '%s' in line %i : %s.\n", file, line, cudaGetErrorString(err));
    exit(-1);
  }
}

/// Check whether GPU threw an error
/** Check whether GPU threw an error
 *  @param errorMessage Space for CUDA error string
 *  @param file Name of source code file
 *  @param line Index of line in source code  
 */
inline void __checkMsg(const char *errorMessage, const char *file, const int line)
{
  cudaError_t err = cudaGetLastError();
  if (cudaSuccess != err) {
    fprintf(stderr, "checkMsg() CUDA error: %s in file <%s>, line %i : %s.\n", errorMessage, file, line, cudaGetErrorString(err));
    exit(-1);
  }
}

/// Initialise GPU
inline bool deviceInit(int dev)
{
  int deviceCount;
  safeCall(cudaGetDeviceCount(&deviceCount));
  if (deviceCount == 0) {
    fprintf(stderr, "CUDA error: no devices supporting CUDA.\n");
    return false;
  }
  if (dev < 0) dev = 0;						
  if (dev > deviceCount-1) dev = deviceCount - 1;
  cudaDeviceProp deviceProp;
  safeCall(cudaGetDeviceProperties(&deviceProp, dev));
  if (deviceProp.major < 1) {
    fprintf(stderr, "error: device does not support CUDA.\n");
    return false;					
  }
  safeCall(cudaSetDevice(dev));
  return true;
}


/// GPU timer class
class TimerGPU {
public:
  /// Start time
  cudaEvent_t start;
   /// Stop time
  cudaEvent_t stop;
  /// CUDA Stream
  cudaStream_t stream;
  /// Constructor
  /** Set start time
   * @param stream_ CUDA Stream
   */
  TimerGPU(cudaStream_t stream_ = 0) : stream(stream_) {
    cudaEventCreate(&start); 
    cudaEventCreate(&stop); 
    cudaEventRecord(start, stream); 
  }
  ~TimerGPU() {
    cudaEventDestroy(start); 
    cudaEventDestroy(stop);  
  }
  /// Stops time measuring 
  /** @return Time elapsed
   */
  float read() {
    cudaEventRecord(stop, stream); 
    cudaEventSynchronize(stop); 
    float time;
    cudaEventElapsedTime(&time, start, stop);
    return time;
  }
};

#endif

