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

#include "cudautils.h"
#include "fgbgsegment.h"
#include "cudasegment.h"
#include "cudabelieft.h"
#include "timercpu.h"
#include "matrix3.h"

#define HSV_W 32 // RGBToHSV  
#define HSV_H 8
#define PRI_W 16 // ComputePriors
#define PRI_H 16 
#define BTP_W 32 // BeliefsToProbs
#define BTP_H 16 
#define SUG_W 128 // SubUpGradients
#define SUG_H 2
#define SGC_W 32  // SetGradientCosts
#define SGC_H 8

//#undef __SSE2__

static const int hist_size = FgBgSegment::hist_size;
static const int max_figures = CudaSegment::max_figures;
 
__constant__ float fispread3d_d[max_figures*9];
__constant__ float fposition3d_d[max_figures*3];
__constant__ float sconstants_d[4];
__constant__ float fcolcost_d[max_figures*hist_size*hist_size];
__constant__ float scolcost_d[hist_size*hist_size];
__constant__ float gcolcost_d[hist_size*hist_size];
__constant__ float constd_d[max_figures+2];
__constant__ float constp_d[max_figures+2];
__constant__ float const0_d[max_figures+2];
__constant__ float constu_d[max_figures+2]; 

__global__ void RGBToHSV(uchar *srcd, uint *bins, uchar *vimd, int width, int height) 
{  
  __shared__ uchar rgb[3*HSV_W*HSV_H];
  int lx = threadIdx.x;
  int ly = threadIdx.y;
  int bx = blockIdx.x*blockDim.x;
  int y = blockIdx.y*blockDim.y + ly;
  int bw = __mul24(y, width) + bx;
  uchar *crgb = &rgb[3*__mul24(ly, HSV_W)];
  int *irgb = (int *)crgb;
  int *isrc = (int *)&srcd[3*bw];
  int minw = 3*min(HSV_W, width - bx); 
  if (4*lx<minw && y<height)   
    irgb[lx] = isrc[lx]; 
  __syncthreads();
  int x = bx + lx;
  if (x<width && y<height) {
    int p = bw + lx;
    int r = crgb[3*lx+0];
    int g = crgb[3*lx+1];
    int b = crgb[3*lx+2];  
    int minv = min(r, min(g, b));
    int maxv = max(r, max(g, b));
    vimd[p] = maxv;
    int diff = maxv - minv;
    int dif6 = diff*6;
    int hue = 0, sat = 0;
    if (diff>0) {
      if (maxv==r) 
	hue = (1536*diff + 256*(g - b))/dif6 & 255;
      else if (maxv==g)
	hue =  (512*diff + 256*(b - r))/dif6;
      else 
	hue = (1024*diff + 256*(r - g))/dif6;
    }
    if (maxv>0)
      sat = 255*(maxv - minv)/maxv;
    int ix = hist_size*hue/256;
    int iy = hist_size*sat/256;
    int idx = iy*hist_size + ix;
    bins[p] = idx;
  }
}

__global__ void ComputePriors(uint *bins, float *dimd, int drange, float *gpriors, float *spriors, float *fpriors, int numFigures, int width, int height)
{
  int lx = threadIdx.x;
  int ly = threadIdx.y;
  int x = blockIdx.x*blockDim.x + lx;
  int y = blockIdx.y*blockDim.y + ly;
  if (x<width && y<height) {
    int p = y*width + x;
    float d = dimd[p];
    bool ok = (d>=0 && d<drange);
    int idx = bins[p];
    int sz = width*height;
    for (int f=0;f<numFigures;f++) {
      float er_f = const0_d[f+2];
      float difx = x - fposition3d_d[f*3+0];
      float dify = y - fposition3d_d[f*3+1];
      float difp = difx*(difx*fispread3d_d[f*9+0] + 2.0f*dify*fispread3d_d[f*9+1]) + dify*dify*fispread3d_d[f*9+4];
      er_f += constp_d[f+2] + (difp<25.0f ? difp : 100.0f);
      if (ok) {
	float difd = d - fposition3d_d[f*3+2];
	er_f += difd*(2.0f*difx*fispread3d_d[f*9+2] + 2.0f*dify*fispread3d_d[f*9+5] + difd*fispread3d_d[f*9+8]) + constd_d[f+2];
      } else 
	er_f += constu_d[f+2];
      er_f += fcolcost_d[f*hist_size*hist_size+idx];
      fpriors[f*sz+p] = 0.5f*er_f;
    }
    float er_g = const0_d[0];
    float er_s = const0_d[1];
    er_g += constp_d[0]; 
    er_s += constp_d[1];
    if (ok) {
      er_g += constd_d[0];
      float diff = d - (sconstants_d[0]*x + sconstants_d[1]*y + sconstants_d[2]);
      er_s += diff*diff*sconstants_d[3] + constd_d[1];
    } else {
      er_g += constu_d[0];
      er_s += constu_d[1];
    }
    er_g += gcolcost_d[idx];
    er_s += scolcost_d[idx]; 
    gpriors[p] = 0.5f*er_g;
    spriors[p] = 0.5f*er_s;
  }
}

__global__ void BeliefsToProbs(float *gbeliefs, float *sbeliefs, float *fbeliefs, float *gprobs, float *sprobs, float *fprobs, int numFigures, int width, int height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;
  int sz = width*height;
  float fb[max_figures];
  if (x<width && y<height) {
    int p = y*width + x;
    float gb = gbeliefs[p];
    float sb = sbeliefs[p];
    float minbelief = fminf(gb, sb);
    for (int f=0;f<numFigures;f++) {
      fb[f] = fbeliefs[f*sz+p];
      minbelief = fminf(minbelief, fb[f]);
    }
    float prob_g = __expf(minbelief - gb) + 1e-6f;
    float prob_s = __expf(minbelief - sb) + 1e-6f;
    float sumprob = prob_g + prob_s;
    for (int f=0;f<numFigures;f++) {
      fb[f] = __expf(minbelief - fb[f]) + 1e-6f;
      sumprob += fb[f];
    }
    float isumprob = __frcp_rn(sumprob);
    gprobs[p] = prob_g*isumprob;
    sprobs[p] = prob_s*isumprob;
    for (int f=0;f<numFigures;f++) 
      fprobs[f*sz+p] = fb[f]*isumprob;
  }
}

#if 0  // Only possible if histograms are small

#define HIST_W 32
#define HIST_B (hist_size*hist_size)
#define HIST_S 200
#define MERG_W 256

__global__ void CollectHistograms(float *hists_d, uint *bins_d, float *wimd_d, uint dataCount)
{
  __shared__ float hist_s[HIST_W*HIST_B];
  int lx = threadIdx.x;
  float *base_s = hist_s + lx;
  #pragma unroll
  for (int i=0;i<HIST_B;i++)
    hist_s[lx + i*HIST_W] = 0;
  __syncthreads();
  uint step = (__umul24(blockDim.x, gridDim.x))*4;
  uint pos = (__umul24(blockIdx.x, blockDim.x) + lx)*4;
  uint4 *bins4_d = (uint4*)bins_d;
  float4 *wimd4_d = (float4*)wimd_d;
  for (;pos<dataCount;pos+=step) {
    uint4 bins = bins4_d[pos/4];
    float4 weig = wimd4_d[pos/4];
    base_s[__umul24(bins.x,HIST_W)] += weig.x;
    base_s[__umul24(bins.y,HIST_W)] += weig.y;
    base_s[__umul24(bins.z,HIST_W)] += weig.z;
    base_s[__umul24(bins.w,HIST_W)] += weig.w;
  } 
  __syncthreads();
  const uint mask = (HIST_W - 1);
  for (int j=0;j<(HIST_B+HIST_W-1)/HIST_W;j++) {
    if (lx<HIST_B) {
      float *base_s = hist_s + __umul24(lx, HIST_W);
      float sum = 0.0f;
      uint pos = lx & 15;
      #pragma unroll
      for (uint i=0;i<HIST_W;i+=4,pos+=4) 
	sum += base_s[(pos+0)&mask] + base_s[(pos+1)&mask] + base_s[(pos+2)&mask] + base_s[(pos+3)&mask];
      hists_d[blockIdx.x*HIST_B + lx] = sum;
    }
    lx += HIST_W;
  }
}

__global__ void MergeHistograms(float *hist_d, float *hists_d, uint histCount) 
{
  __shared__ float data[MERG_W];
  int lx = threadIdx.x;
  float sum = 0.0f;
  for (uint i=lx;i<histCount;i+=MERG_W)
    sum += hists_d[blockIdx.x + i*HIST_B];
  data[lx] = sum;
  for (uint stride = MERG_W/2;stride>0;stride>>=1) {
    __syncthreads();
    if (lx<stride)
      data[lx] += data[lx + stride];
  }
  if (lx==0)
    hist_d[blockIdx.x] = data[0];
}

#endif

__global__ void SumUpGradientsD(uchar *image_d, int *sums_d, int w, int h)
{
  __shared__ uchar rows[(SUG_W+16)*2];
  __shared__ int sums[SUG_W];
  int lx = threadIdx.x;
  int bx = blockIdx.x*blockDim.x;   
  int x = bx + lx;
  int y = (blockIdx.y*blockDim.y + threadIdx.y)*SUG_H;
  uchar *row1 = &rows[0];
  uchar *row2 = &rows[SUG_W+16];
  int bp = y*w + bx;
  int x4 = (bx + 4*lx);
  if (lx<SUG_W/4+1 && x4<w-3) { 
    int *irow = (int*)&image_d[bp];
    ((int*)row1)[lx] = irow[lx];
  }
  bp += w;
  __syncthreads();  
  int sumdiff2 = 0;
  for (int dy=1;dy<=SUG_H && (y+dy)<h;dy++) {
    if (lx<SUG_W/4+1 && x4<w-3) {
      int *irow = (int*)&image_d[bp]; 
      ((int*)row2)[lx] = irow[lx];
    }
    __syncthreads();
    if (x<w-1) {
      int val = row1[lx];
      int diff = val - row2[lx];
      sumdiff2 += diff*diff;
      diff = val - row1[lx+1]; 
      sumdiff2 += diff*diff;
    }
    bp += w;
    uchar *trow = row1;
    row1 = row2;
    row2 = trow;
    __syncthreads();
  }
  sums[lx] = sumdiff2;
  __syncthreads();
  int step = SUG_W/2;
  while (step>0) {
    if (lx<step)
      sums[lx] += sums[lx+step];
    step >>= 1;
    __syncthreads();
  }
  if (lx==0) 
    sums_d[blockIdx.y*gridDim.x + blockIdx.x] = sums[lx];
}

__global__ void SetGradientCostsD(uchar *image_d, float *costh_d, float *costv_d, float beta, float gamma, int w, int h)
{
  __shared__ uchar rows[(SGC_W+16)*(SGC_H+1)];
  const int lw = SGC_W + 16;
  int lx = threadIdx.x;
  int ly = threadIdx.y;
  int bx = blockIdx.x*blockDim.x;  
  int x = bx + lx;
  int y = blockIdx.y*blockDim.y + ly;
  int rp = y*w + bx;
  int p = rp + lx;
  int lp = ly*lw + lx;
  int x4 = (bx + 4*lx);
  if (lx<SGC_W/4+1 && x4<w-3 && y<h) {
    int *irow = (int*)&image_d[rp];
    int *lrow = (int*)&rows[ly*lw];
    lrow[lx] = irow[lx];
  }
  if (lx<SGC_W/4+1 && x4<w-3 && ly==0 && (y+SGC_H)<h) {
    int *irow = (int*)&image_d[rp + SGC_H*w];
    int *lrow = (int*)&rows[SGC_H*lw];
    lrow[lx] = irow[lx];
  } 
  __syncthreads();
  if (x<w && y<h) {
    int v0 = rows[lp];
    int vx = (x<w-1 ? rows[lp+1] : v0);
    int vy = (y<h-1 ? rows[lp+lw] : v0); 
    int dx = vx - v0;
    int dy = vy - v0;
    costh_d[p] = gamma*(0.1f + __expf(beta*dx*dx)); //%%%%
    costv_d[p] = gamma*(0.1f + __expf(beta*dy*dy)); //%%%%
  }
}

//======================================================================//

CudaSegment::CudaSegment(int w, int h) : 
  width(w), height(h)
{
  dim3 grid((width+SUG_W-1)/SUG_W, (height+SUG_H-1)/SUG_H);
  safeCall(cudaMalloc((void**)&sums_d, sizeof(int)*grid.x*grid.y));
  safeCall(cudaMalloc(&cimd_d, 3*width*height));
  safeCall(cudaMalloc(&dimd_d, sizeof(float)*width*height));
  safeCall(cudaMalloc(&vimd_d, width*height));
  safeCall(cudaMalloc(&bins_d, sizeof(int)*width*height));
  int sz = sizeof(float)*width*height;
  int n = max_figures+2;
  safeCall(cudaMalloc(&allocated_d, (3*n+2)*sz));
  costh_d = (float*)&allocated_d[0*sz];
  costv_d = (float*)&allocated_d[1*sz];
  for (int i=0;i<n;i++) {
    priors_d[i] = (float*)&allocated_d[(0*n+i+2)*sz];
    beliefs_d[i] = (float*)&allocated_d[(1*n+i+2)*sz];
    probs_d[i] = (float*)&allocated_d[(2*n+i+2)*sz];
  }
}

CudaSegment::~CudaSegment()
{
  safeCall(cudaFree(sums_d));
  safeCall(cudaFree(cimd_d));
  safeCall(cudaFree(dimd_d));
  safeCall(cudaFree(bins_d));
  safeCall(cudaFree(vimd_d));
  safeCall(cudaFree(allocated_d));
}

void CudaSegment::CopyProbsToDevice(Image<float> **probs_h)
{
  safeCall(cudaMemcpy(probs_d[0], probs_h[0]->GetData(), sizeof(float)*width*height, cudaMemcpyHostToDevice));
  safeCall(cudaMemcpy(probs_d[1], probs_h[1]->GetData(), sizeof(float)*width*height, cudaMemcpyHostToDevice));
  safeCall(cudaMemcpy(probs_d[2], probs_h[2]->GetData(), sizeof(float)*width*height, cudaMemcpyHostToDevice));
}

#if 0  // Only possible if histograms are small

void CudaSegment::CreateHistograms(FgBgSegment &segment)
{ 
  float *hist_d, *hists_d;
  uint count = width*height;
  const int blockSize = HIST_W*HIST_S;
  uint numBlocks = (count+blockSize-1)/blockSize;
  safeCall(cudaMalloc((void **)&hist_d, HIST_B*sizeof(float)));
  safeCall(cudaMalloc((void **)&hists_d, numBlocks*HIST_B*sizeof(float)));
  for (int j=0;j<3;j++) {
    CollectHistograms<<<numBlocks, HIST_W>>>(hists_d, bins_d, probs_d[j], count);
    checkMsg("Kernel execution failed");
    safeCall(cudaThreadSynchronize());
    MergeHistograms<<<HIST_B, MERG_W>>>(hist_d, hists_d, numBlocks);
    checkMsg("Kernel execution failed");
    safeCall(cudaThreadSynchronize());
    float hist_h[HIST_B];
    safeCall(cudaMemcpy(hist_h, hist_d, HIST_B*sizeof(float), cudaMemcpyDeviceToHost));
    float fac_old = 1.0f / (1.0f + FgBgSegment::ColorModel::weight);
    float fac_new = 1.0f - fac_old;
    float *hist = segment.ground.histogram;
    float *cost = segment.ground.colorcost;
    if (j==1) {
      hist = segment.surface.histogram;
      cost = segment.surface.colorcost;
    } else if (j==2) {
      hist = segment.figure.histogram;
      cost = segment.figure.colorcost;
    }
    float num = 0.0f;
    for (int i=0;i<HIST_B;i++) 
      num += hist_h[i];
    num = (num>0.0f ? num : 1e-6f);
    for (int i=0;i<HIST_B;i++) {
      hist[i] = fac_new*(hist_h[i]/num) + fac_old*hist[i];
      cost[i] = -2.0f*log(hist[i]);
    }
  }
  safeCall(cudaFree(hist_d));
  safeCall(cudaFree(hists_d));
}

#endif

float CudaSegment::ComputeBeta(uchar *image_d) 
{
  dim3 block(SUG_W, 1);
  dim3 grid((width+SUG_W-1)/SUG_W, (height+SUG_H-1)/SUG_H);
  SumUpGradientsD<<<grid, block>>>(image_d, sums_d, width, height);
  checkMsg("Kernel execution failed"); 
  safeCall(cudaThreadSynchronize());
  int *sums = new int[grid.x*grid.y];
  safeCall(cudaMemcpy(sums, sums_d, sizeof(int)*grid.x*grid.y, cudaMemcpyDeviceToHost));
  int sum = 0;
  for (int i=0;i<grid.x*grid.y;i++) 
    sum += sums[i];
  float avggrad = (float)sum/(width-1)/(height-1)/2;
  float beta = -1.0f/(2.0f*avggrad);
  delete [] sums;
  return beta;
}

void CudaSegment::SetGradientCosts(uchar *image_d, float beta, float gamma)
{
  dim3 block(SGC_W, SGC_H);
  dim3 grid((width+SGC_W-1)/SGC_W, (height+SGC_H-1)/SGC_H);
  SetGradientCostsD<<<grid, block>>>(image_d, costh_d, costv_d, beta, gamma, width, height); 
  checkMsg("Kernel execution failed"); 
  safeCall(cudaThreadSynchronize());
}
  
void CudaSegment::Execute(FgBgSegment &segment, 
			  Image<uchar> &cimg, 
			  Image<float> &dimg, int numFigures, bool download)
{
  TimerCPU timer0(2800);
  if (download) {
  
    //  cimg.StoreRGB("/u/jbohg/GPUCimg.pgm");
    //    dimg.Store("/u/jbohg/GPUDimg.pgm");
 
    // Upload RGB image and disparity data 
    safeCall(cudaMemcpy(cimd_d, cimg.GetData(), 3*width*height, cudaMemcpyHostToDevice));
    safeCall(cudaMemcpy(dimd_d, dimg.GetData(), sizeof(float)*width*height, cudaMemcpyHostToDevice));
    // Convert from RGB to HSV
    dim3 block1(HSV_W, HSV_H);
    dim3 grid1((width+HSV_W-1)/HSV_W, (height+HSV_H-1)/HSV_H);
    TimerCPU timerc(2800);
    ::RGBToHSV<<<grid1, block1>>>(cimd_d, bins_d, vimd_d, width, height); 
    checkMsg("Kernel execution failed");
    safeCall(cudaThreadSynchronize()); 
    float delayc = timerc.read();
  }  

  // Set prior probability constants 
  const int num = max_figures + 2;
  float constd[num] = { 2.0f*log((float)segment.drange/2.0f), log(segment.surface.spread_d) };
  float constp[num] = { 2.0f*log((float)width*height), 2.0f*log((float)width*height) };
  float const0[num] = { -2.0*log(0.45f), -2.0*log(0.45f) };
  float constu[num] = { -2.0*log(0.40f), -2.0*log(0.40f) };
  for (int f=0;f<numFigures;f++) {
    constd[f+2] = log(segment.figures[f]->spread3d(2,2)); 
    constp[f+2] = log(segment.figures[f]->spread3d.determinant()) - constd[f+2]; 
    const0[f+2] = -2.0*log(0.10f);
    constu[f+2] = -2.0*log(0.20f);
  }
  if (segment.surface.spread_d>2.0f && false) {  //%%%%
    const0[0] = -2.0*log(0.90f);                 //%%%% Ignore surface if it's too 'thick'
    const0[1] = -2.0*log(1e-12f);                //%%%%
  }
  safeCall(cudaMemcpyToSymbol(constd_d, constd, sizeof(float)*num));
  safeCall(cudaMemcpyToSymbol(constp_d, constp, sizeof(float)*num));
  safeCall(cudaMemcpyToSymbol(const0_d, const0, sizeof(float)*num));
  safeCall(cudaMemcpyToSymbol(constu_d, constu, sizeof(float)*num));
  
  // Copy model parameters
  float sconstants[] = { segment.surface.alpha, segment.surface.beta, segment.surface.disp, 1.0f/segment.surface.spread_d };
  safeCall(cudaMemcpyToSymbol(sconstants_d, sconstants, sizeof(float)*4));
  float fispread3d_h[max_figures*9];
  float fposition3d_h[max_figures*3];
  for (int f=0;f<numFigures;f++) {
    for (int i=0;i<3;i++) 
      fposition3d_h[f*3+i] = segment.figures[f]->position3d(i);
    Matrix3 ivar = segment.figures[f]->spread3d;
    if (ivar.determinant()!=0.0)
      ivar = ivar.invert();
    for (int j=0;j<3;j++) 
      for (int i=0;i<3;i++)  
	fispread3d_h[f*9+j*3+i] = ivar(j, i);
  }
  safeCall(cudaMemcpyToSymbol(fposition3d_d, fposition3d_h, sizeof(fposition3d_h)));
  safeCall(cudaMemcpyToSymbol(fispread3d_d, fispread3d_h, sizeof(fispread3d_h)));
  
  // Create colour histograms 
  safeCall(cudaMemcpyToSymbol(scolcost_d, segment.surface.colorcost, sizeof(float)*hist_size*hist_size));
  safeCall(cudaMemcpyToSymbol(gcolcost_d, segment.ground.colorcost, sizeof(float)*hist_size*hist_size));
  float fcolcost_h[max_figures*hist_size*hist_size];
  for (int f=0;f<numFigures;f++) 
    memcpy(&fcolcost_h[f*hist_size*hist_size], segment.figures[f]->colorcost, sizeof(float)*hist_size*hist_size);
  safeCall(cudaMemcpyToSymbol(fcolcost_d, fcolcost_h, sizeof(float)*hist_size*hist_size*numFigures));
 
  // Compute prior probabilities     
  dim3 block2(PRI_W, PRI_H); 
  dim3 grid2((width+PRI_W-1)/PRI_W, (height+PRI_H-1)/PRI_H);
  ComputePriors<<<grid2, block2>>>(bins_d, dimd_d, segment.drange, priors_d[0], priors_d[1], priors_d[2], numFigures, width, height);
  checkMsg("Kernel execution failed");
  safeCall(cudaThreadSynchronize());
#if 0
  Image<float> dumpimg(width, height);    
  safeCall(cudaMemcpy(dumpimg.GetData(), priors_d[0], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  dumpimg.Store("prior0.pgm", true, false);
  safeCall(cudaMemcpy(dumpimg.GetData(), priors_d[1], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  dumpimg.Store("prior1.pgm", true, false);
  safeCall(cudaMemcpy(dumpimg.GetData(), priors_d[2], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  dumpimg.Store("prior2.pgm", true, false);
  safeCall(cudaMemcpy(dumpimg.GetData(), priors_d[3], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  dumpimg.Store("prior3.pgm", true, false);
#endif 
    
  // Compute beta factor and set gradient costs
  float beta = ComputeBeta(vimd_d);
  SetGradientCosts(vimd_d, beta, segment.gradWeight); 
 
  // Perform belief propagation 
  int loops = 5;
  int depth = 4; 
  if (numFigures==0) { 
    CudaBelief<2> cudabelief(width, height, false);
    cudabelief.Execute(priors_d, beliefs_d, costh_d, costv_d, loops, depth);
  } else if (numFigures==1) { 
    CudaBelief<3> cudabelief(width, height, false);
    cudabelief.Execute(priors_d, beliefs_d, costh_d, costv_d, loops, depth);
  } else if (numFigures==2) {  
    CudaBelief<4> cudabelief(width, height, false);
    cudabelief.Execute(priors_d, beliefs_d, costh_d, costv_d, loops, depth);
  } else if (numFigures==3) {
    CudaBelief<5> cudabelief(width, height, false);
    cudabelief.Execute(priors_d, beliefs_d, costh_d, costv_d, loops, depth);
  } else if (numFigures==4) { 
    CudaBelief<6> cudabelief(width, height, false);
    cudabelief.Execute(priors_d, beliefs_d, costh_d, costv_d, loops, depth);
  } else if (numFigures==5) {
    CudaBelief<7> cudabelief(width, height, false);
    cudabelief.Execute(priors_d, beliefs_d, costh_d, costv_d, loops, depth); 
  } else if (numFigures==6) {
    CudaBelief<8> cudabelief(width, height, false);
    cudabelief.Execute(priors_d, beliefs_d, costh_d, costv_d, loops, depth);
  }

  // Convert from beliefs to probabilities
  dim3 block3(BTP_W, BTP_H); 
  dim3 grid3((width+BTP_W-1)/BTP_W, (height+BTP_H-1)/BTP_H);
  BeliefsToProbs<<<grid3, block3>>>(beliefs_d[0], beliefs_d[1], beliefs_d[2], probs_d[0], probs_d[1], probs_d[2], numFigures, width, height);
  checkMsg("Kernel execution failed");
  safeCall(cudaThreadSynchronize()); 

  // Copy back probabilities to host
  safeCall(cudaMemcpy(segment.ground.probabilities.GetData(),  probs_d[0], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  safeCall(cudaMemcpy(segment.surface.probabilities.GetData(), probs_d[1], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  for (int f=0;f<numFigures;f++) 
    safeCall(cudaMemcpy(segment.figures[f]->probabilities.GetData(),  probs_d[f+2], sizeof(float)*width*height, cudaMemcpyDeviceToHost));
  safeCall(cudaThreadSynchronize());
  float delayt = timer0.read();
  std::cout << "CudaSegment      Time: " << delayt << " ms " << width*height/delayt/1e3 << " Mpixels per second" << std::endl;
}

 
 
