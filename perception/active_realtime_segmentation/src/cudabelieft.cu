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

#include <iostream>
#include <math_constants.h>
#include "cudautils.h"
#include "timercpu.h" 

#define CBE_W 32  // ComputeBeliefs
#define CBE_H 16
#define CUD_W 32  // CollectUp & CollectDown
#define CUD_H 16
#define SDP_W 16  // ScaleDownPriors
#define SDP_H 16

__global__ void ComputeBeliefsD(float *prior, float *msgl, float *msgr, float *msgu, float *msgd, float *belief, int w, int h) 
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;
  int p = y*w + x;
  if (x<w && y<h) {
    belief[p] = prior[p] + msgl[p] + msgr[p] + msgu[p] + msgd[p];
  }   
}

template<int dim>
__global__ void CollectLeft(float *costh, float *beliefs, float *msgs, float *msgos, int w, int h)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;
  int p = y*w + x;
  int sz = w*h;
  float msg[dim];
  if (x<w-1 && y<h) {
    float minh = CUDART_MAX_NORMAL_F;
    for (int f=0;f<dim;f++) {
      msg[f] = beliefs[f*sz+p] - msgs[f*sz+p];
      minh = min(msg[f], minh);
    }
    float minc = minh + costh[p]; 
    for (int f=0;f<dim;f++) 
      msgos[f*sz+p+1] = min(msg[f], minc) - minh;
  }
}

template<int dim>
__global__ void CollectRight(float *costh, float *beliefs, float *msgs, float *msgos, int w, int h)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;
  int p = y*w + x;
  int sz = w*h;
  float msg[dim];
  if (x>0 && x<w && y<h) {
    float minh = CUDART_MAX_NORMAL_F;
    for (int f=0;f<dim;f++) {
      msg[f] = beliefs[f*sz+p] - msgs[f*sz+p];
      minh = min(msg[f], minh);
    }
    float minc = minh + costh[p-1]; 
    for (int f=0;f<dim;f++) 
      msgos[f*sz+p-1] = min(msg[f], minc) - minh;
  }
}

template<int dim>
__global__ void CollectUp(float *costv, float *beliefs, float *msgs, float *msgos, int w, int h)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;
  int p = y*w + x;
  int p0 = p - w;
  int sz = w*h;
  float msg[dim];
  if (x<w && y<h && y>0) {
    float minh = CUDART_MAX_NORMAL_F;
    for (int f=0;f<dim;f++) {
      msg[f] = beliefs[f*sz+p0] - msgs[f*sz+p0];
      minh = min(msg[f], minh);
    }
    float minc = minh + costv[p0]; 
    for (int f=0;f<dim;f++) 
      msgos[f*sz+p] = min(msg[f], minc) - minh;
  }
}

template<int dim>
__global__ void CollectDown(float *costv, float *beliefs, float *msgs, float *msgos, int w, int h)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;
  int p = y*w + x;
  int p0 = p + w;
  int sz = w*h;
  float msg[dim];
  if (x<w && y<h-1) {
    float minh = CUDART_MAX_NORMAL_F;
    for (int f=0;f<dim;f++) {
      msg[f] = beliefs[f*sz+p0] - msgs[f*sz+p0];
      minh = min(msg[f], minh);
    }
    float minc = minh + costv[p0]; 
    for (int f=0;f<dim;f++) 
      msgos[f*sz+p] = min(msg[f], minc) - minh;
  }
} 

__global__ void ScaleDownPriorsD(float *priors1, int width1, int height1, float *priors2, int width2, int height2, int dim)
{
  int x2 = blockIdx.x*blockDim.x + threadIdx.x;
  int y2 = blockIdx.y*blockDim.y + threadIdx.y;
  int d = y2/height2;
  int x1 = 2*x2;
  int y1 = 2*y2 + (height1 - 2*height2)*d;
  float *ptr2 = priors2 + (x2 + y2*width2);
  float *ptr1 = priors1 + (x1 + y1*width1);
  if (x2<width2 && d<dim)
    ptr2[0] = ptr1[0] + ptr1[1] + ptr1[width1] + ptr1[width1+1];
}
 
__global__ void ScaleDownGradCostsD(float *costh1, float *costv1, int width1, float *costh2, float *costv2, int width2, int height2)
{
  int x2 = blockIdx.x*blockDim.x + threadIdx.x;
  int y2 = blockIdx.y*blockDim.y + threadIdx.y;
  int x1 = 2*x2;
  int y1 = 2*y2;
  int p2 = x2 + y2*width2;
  int p1 = x1 + y1*width1;
  float *ptrh1 = costh1 + p1;
  float *ptrv1 = costv1 + p1;
  if (x2<width2 && y2<height2) {
    costh2[p2] = ptrh1[1] + ptrh1[width1+1];
    costv2[p2] = ptrv1[width1] + ptrv1[width1+1];
  }
}
 
__global__ void ScaleUpMessagesD(float *msgs1, int width1, int height1, float *msgs2, int width2, int height2, int dim)
{
  int x2 = blockIdx.x*blockDim.x + threadIdx.x;
  int y2 = blockIdx.y*blockDim.y + threadIdx.y;
  int d = y2/height2;
  int x1 = 2*x2;
  int y1 = 2*y2 + (height1 - 2*height2)*d;
  float *ptr2 = msgs2 + (x2 + y2*width2);
  float *ptr1 = msgs1 + (x1 + y1*width1);
  if (x2<width2 && d<dim)
    ptr1[0] = ptr1[1] = ptr1[width1] = ptr1[width1+1] = ptr2[0];
}
 
//====================================================================================//

template<int dim>
CudaBelief<dim>::CudaBelief(int w, int h, bool copyFromHost_) : width(w), height(h), copyFromHost(copyFromHost_)
{
  int sz = sizeof(float)*w*h;
  int totsz = 8*dim*sz;
  safeCall(cudaMalloc((void**)&allocated_d, totsz)); 
  for (int i=0;i<2*dim;i++) {
    msgl_d[i] = (float*)&allocated_d[(0*dim+i)*sz];
    msgr_d[i] = (float*)&allocated_d[(2*dim+i)*sz];
    msgu_d[i] = (float*)&allocated_d[(4*dim+i)*sz];
    msgd_d[i] = (float*)&allocated_d[(6*dim+i)*sz];
  }
  if (copyFromHost) {
    for (int i=0;i<dim;i++) {
      safeCall(cudaMalloc((void**)&prior_d[i], sz));
      safeCall(cudaMalloc((void**)&belief_d[i], sz));
    }
  }
}

template<int dim>
CudaBelief<dim>::~CudaBelief()
{
  safeCall(cudaFree(allocated_d));
  if (copyFromHost) {
    for (int i=0;i<dim;i++) {
      safeCall(cudaFree(prior_d[i]));
      safeCall(cudaFree(belief_d[i]));
    }
  }  
}

template<int dim>
void CudaBelief<dim>::ComputeBeliefs(int loop)
{
  int s = dim*(loop&1);
  dim3 block(CBE_W, CBE_H);
  dim3 grid((width+CBE_W-1)/CBE_W, (height+CBE_H-1)/CBE_H);
  for (int i=0;i<dim;i++) 
    ComputeBeliefsD<<<grid, block>>>(prior_d[i], msgl_d[s+i], msgr_d[s+i], msgu_d[s+i], 
      msgd_d[s+i], belief_d[i], width, height);
  checkMsg("Kernel execution failed"); 
  safeCall(cudaThreadSynchronize());
#if 0
  std::cout << "ComputeBeliefs" << std::endl;
  Image<float> dump(width, height);
  safeCall(cudaMemcpy(dump.GetData(),  prior_d[1], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  dump.Store("dump5.pgm", true, false);
  safeCall(cudaMemcpy(dump.GetData(),  msgr_d[s+1], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  dump.Store("dump2.pgm", true, false);
  safeCall(cudaMemcpy(dump.GetData(),  msgu_d[s+1], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  dump.Store("dump3.pgm", true, false);
  safeCall(cudaMemcpy(dump.GetData(),  msgd_d[s+1], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  dump.Store("dump4.pgm", true, false);
  safeCall(cudaMemcpy(dump.GetData(),  msgl_d[s+1], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  dump.Store("dump1.pgm", true, false);
  safeCall(cudaMemcpy(dump.GetData(),  belief_d[1], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  dump.Store("dump6.pgm", true, false);
#endif
}

template<int dim>
void CudaBelief<dim>::CollectMessages(float *costh_d, float *costv_d, int loop)
{
  int s1 = dim*(loop&1);
  int s2 = s1^dim;
  dim3 blocku(CUD_W, CUD_H);
  dim3 gridu((width+CUD_W-1)/CUD_W, (height+CUD_H-1)/CUD_H);
  CollectLeft<dim><<<gridu, blocku>>>(costh_d, belief_d[0], msgr_d[s1], msgl_d[s2], width, height); 
  CollectRight<dim><<<gridu, blocku>>>(costh_d, belief_d[0], msgl_d[s1], msgr_d[s2], width, height); 
  CollectUp<dim><<<gridu, blocku>>>(costv_d, belief_d[0], msgd_d[s1], msgu_d[s2], width, height); 
  CollectDown<dim><<<gridu, blocku>>>(costv_d, belief_d[0], msgu_d[s1], msgd_d[s2], width, height); 
  checkMsg("Kernel execution failed"); 
  safeCall(cudaThreadSynchronize());
}

template<int dim>
void CudaBelief<dim>::Execute(float **priors, float **beliefs, float *costh_d, float *costv_d)
{
  if (copyFromHost) {
    TimerCPU timer0(2800);
    for (int i=0;i<dim;i++) 
      safeCall(cudaMemcpy(prior_d[i], priors[i], sizeof(float)*width*height, cudaMemcpyHostToDevice));
    float delay0 = timer0.read();
    std::cout << "MemoryCopy       Time: " << delay0 << " ms " << width*height/delay0/1e3 << " Mpixels per second" << std::endl;
  } else {
    for (int i=0;i<dim;i++) {  
      prior_d[i] = priors[i]; 
      belief_d[i] = beliefs[i];
    }
  }
  safeCall(cudaMemset(msgl_d[0], 0, sizeof(float)*width*height*dim));
  safeCall(cudaMemset(msgr_d[0], 0, sizeof(float)*width*height*dim));
  safeCall(cudaMemset(msgu_d[0], 0, sizeof(float)*width*height*dim));
  safeCall(cudaMemset(msgd_d[0], 0, sizeof(float)*width*height*dim));
  float delayt = 0.0f;
  int loops = 10; 
  for (int i=0;i<loops;i++) {
    TimerCPU timer1(2800);
    ComputeBeliefs(i);
    float delay1 = timer1.read();
    delayt += delay1;
    //std::cout << "ComputeBeliefs   Time: " << delay1 << " ms " << width*height/delay1/1e3 << " Mpixels per second" << std::endl;
    TimerCPU timer2(2800);
    CollectMessages(costh_d, costv_d, i);
    float delay2 = timer2.read();
    delayt += delay2;
    //std::cout << "CollectMessages  Time: " << delay2 << " ms " << width*height/delay2/1e3 << " Mpixels per second" << std::endl;
  }
  TimerCPU timer3(2800);  
  ComputeBeliefs(loops);
  if (copyFromHost) 
    for (int i=0;i<dim;i++) 
      safeCall(cudaMemcpy(beliefs[i], belief_d[i], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  float delay3 = timer3.read();
  delayt += delay3;
  //std::cout << "ComputeBeliefs   Time: " << delay3 << " ms " << width*height/delay3/1e3 << " Mpixels per second" << std::endl;
  std::cout << "CudaBelief       Time: " << delayt << " ms " << width*height/delayt/1e3 << " Mpixels per second" << std::endl;
}

#include "pyra/tpimage.h" //%%%%

template<int dim>
void CudaBelief<dim>::Execute(float **priors, float **beliefs, float *costh_d, float *costv_d, int loops, int depth)
{
  if (copyFromHost) {
    TimerCPU timer0(2800);
    for (int i=0;i<dim;i++) 
      safeCall(cudaMemcpy(prior_d[i], priors[i], sizeof(float)*width*height, cudaMemcpyHostToDevice));
    float delay0 = timer0.read();
    std::cout << "MemoryCopy       Time: " << delay0 << " ms " << width*height/delay0/1e3 << " Mpixels per second" << std::endl;
  } else {
    for (int i=0;i<dim;i++) {  
      prior_d[i] = priors[i]; 
      belief_d[i] = beliefs[i];
    }
  }
  float delayt = 0.0f;
  if (depth>0) {
    TimerCPU timer4(2800);
    int width2 = width/2;
    int height2 = height/2;
    int sz = sizeof(float)*width2*height2;
    unsigned char *allocated2_d;
    float *prior2_d[dim], *belief2_d[dim];
    safeCall(cudaMalloc(&allocated2_d, (2*dim+2)*sz));
    float *costh2_d = (float*)&allocated2_d[0*sz];
    float *costv2_d = (float*)&allocated2_d[1*sz];
    for (int i=0;i<dim;i++) {
      prior2_d[i] = (float*)&allocated2_d[(0*dim+i+2)*sz];
      belief2_d[i] = (float*)&allocated2_d[(1*dim+i+2)*sz];
    }    
    dim3 block1(SDP_W, SDP_H);
    dim3 grid1((width2+SDP_W-1)/SDP_W, (height2*dim+SDP_H-1)/SDP_H);
    ScaleDownPriorsD<<<grid1, block1>>>(prior_d[0], width, height, prior2_d[0], width2, height2, dim);
    checkMsg("Kernel execution failed");  // NOTE: assumes priors to be stored sequentially
    safeCall(cudaThreadSynchronize());
    dim3 block2(SDP_W, SDP_H);
    dim3 grid2((width2+SDP_W-1)/SDP_W, (height2+SDP_H-1)/SDP_H);
    ScaleDownGradCostsD<<<grid2, block2>>>(costh_d, costv_d, width, costh2_d, costv2_d, width2, height2);
    checkMsg("Kernel execution failed");  
    safeCall(cudaThreadSynchronize());
    CudaBelief<dim> cudabelief(width2, height2, false);  
    float delay4 = timer4.read();
    delayt += delay4; 
    cudabelief.Execute(prior2_d, belief2_d, costh2_d, costv2_d, loops, depth-1);
    TimerCPU timer5(2800);
    safeCall(cudaFree(allocated2_d));
    float *mptrs1[4] = { msgl_d[0], msgr_d[0], msgu_d[0], msgd_d[0] };
    float *mptrs2[4] = { cudabelief.msgl_d[0], cudabelief.msgr_d[0], cudabelief.msgu_d[0], cudabelief.msgd_d[0] };
    for (int i=0;i<4;i++) {
      ScaleUpMessagesD<<<grid1, block1>>>(mptrs1[i], width, height, mptrs2[i], width2, height2, dim);
      checkMsg("Kernel execution failed");  
      safeCall(cudaThreadSynchronize());
    }
    float delay5 = timer5.read();
    delayt += delay5; 
  } else {
    safeCall(cudaMemset(msgl_d[0], 0, sizeof(float)*width*height*dim));
    safeCall(cudaMemset(msgr_d[0], 0, sizeof(float)*width*height*dim));
    safeCall(cudaMemset(msgu_d[0], 0, sizeof(float)*width*height*dim));
    safeCall(cudaMemset(msgd_d[0], 0, sizeof(float)*width*height*dim));
  }
  int l = loops; //(depth>0 ? loops : height/2);
  for (int i=0;i<l;i++) {
    TimerCPU timer1(2800);
    ComputeBeliefs(i);
    float delay1 = timer1.read();
    delayt += delay1;
    //std::cout << "ComputeBeliefs   Time: " << delay1 << " ms " << width*height/delay1/1e3 << " Mpixels per second" << std::endl;
    TimerCPU timer2(2800);
    CollectMessages(costh_d, costv_d, i);
    float delay2 = timer2.read();
    delayt += delay2;
    //std::cout << "CollectMessages  Time: " << delay2 << " ms " << width*height/delay2/1e3 << " Mpixels per second" << std::endl;
  }
  TimerCPU timer3(2800);  
  ComputeBeliefs(l);
  if (copyFromHost)  
    for (int i=0;i<dim;i++) 
      safeCall(cudaMemcpy(beliefs[i], belief_d[i], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  float delay3 = timer3.read();
  delayt += delay3; 

  //std::cout << "CudaBelief       Time: " << delayt << " ms " << width*height/delayt/1e3 << " Mpixels per second" << std::endl;
}
  
