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

#undef USE_TBB

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <vector>
#ifdef USE_TBB
#include <tbb/tbb.h>
#endif // USE_TBB
//#include <boost/thread/thread.hpp>
#include "pyra/tpimageutil.h"

#include "belief.h"
#include "largest.h"
typedef unsigned int uint;
#include "cudasegment.h" 
#include "fgbgsegment.h"
#include "timercpu.h"
#include <ros/assert.h>

FgBgSegment::FgBgSegment(int w, int h, int r, float b, 
			 float w_size, float b_size) 
  :  withSurface(true)
  ,  withColors(true)
  ,  withDisparities(true)
  ,  withColorHoles(true)
  ,  uniform(false)
  ,  verbose(0)
  ,  ground(*this, w, h)
  ,  surface(*this, w, h)
  ,  hue(w, h)
  ,  saturation(w, h)
  ,  grey(w, h)
  ,  width(w)
  ,  height(h)
  ,  drange(r)
  ,  gradWeight(b)
  ,  windowSize(w_size)
  ,  ballSize(b_size)
  ,  gpuopt(true)
{ 
#ifdef USE_CUDA
  cudaSegment = new CudaSegment(w, h);
#endif // USE_CUDA  
}

FgBgSegment::~FgBgSegment()
{
  for (unsigned int i=0;i<figures.size();i++)
    delete figures[i];
#ifdef USE_CUDA
  delete cudaSegment;
#endif // USE_CUDA  
}

void FgBgSegment::UseGPU(bool gpuopt_)
{
  gpuopt = gpuopt_;
}

FgBgSegment::ColorModel::ColorModel(FgBgSegment &segm_) : 
  segm(segm_), prior(NULL)
{
  for (int i=0;i<hist_size*hist_size;i++) 
    histogram[i] = 0.0f; 
}

FgBgSegment::ColorModel::ColorModel(const ColorModel &model) : 
  segm(model.segm), prior(NULL)
{
  // std::cout << "Copy colors" << std::endl;
  for (int i=0;i<hist_size*hist_size;i++) {
    histogram[i] = model.histogram[i]; 
    colorcost[i] = model.colorcost[i];
  }
}

FgBgSegment::ColorModel &FgBgSegment::ColorModel::operator=(const ColorModel &model)
{
  // std::cout << "Copy colors" << std::endl;
  for (int i=0;i<hist_size*hist_size;i++) {
    histogram[i] = model.histogram[i]; 
    colorcost[i] = model.colorcost[i];
  }
}

FgBgSegment::ColorModel::~ColorModel()
{

}

FgBgSegment::FlatSurface::FlatSurface(FgBgSegment &segm, 
				      int width, int height) 
  : ColorModel(segm)
  , alpha(0.0f)
  , beta(0.0f)
  , disp(1000.0f)
  , spread_d(1.0f)
  , probabilities(width, height)
{
  const int numFigures = segm.figures.size();
  Fill(probabilities, 1.0f/((float)numFigures+2.0f));
  //  Fill(probabilities, 1.0f/3.0f);
}

FgBgSegment::Foreground::Foreground(FgBgSegment &segm, 
				    int width, int height) 
  : ColorModel(segm)
  , window_size(0.20f)
  , ball_size(0.20f)
  , combined(false)
  , probabilities(width, height)
{

  const int numFigures = segm.figures.size();
  Fill(probabilities, 1.0f/((float)numFigures+2.0f));
  //  Fill(probabilities, 1.0f/3.0f);
}

FgBgSegment::Background::Background(FgBgSegment &segm, 
				    int width, int height) 
  : ColorModel(segm)
  , spread_d(16.0f)
  , probabilities(width, height)
{
  const int numFigures = segm.figures.size();
  Fill(probabilities, 1.0f/((float)numFigures+2.0f));
  //  Fill(probabilities, 1.0f/3.0f);
}

//=================== Initialization routines =====================//

void FgBgSegment::FlatSurface::Initialize()
{
  float *dimd = segm.disparities->GetData();
  int drange = segm.drange;
  int width = segm.width;
  int height = segm.height;

  // reset plane limits
  min_x = width;
  max_x = 0;
  min_y = height;
  max_y = 0;

  float minbeta = 0.08;
  // Find dominating disparity for each y-value
  int *hist = new int[height*drange];
  int *totals = new int[height];
  for (int y=0;y<height;y++) {
    int *histy = &hist[y*drange];
    for (int i=0;i<drange;i++) 
      histy[i] = 0;
    for (int x=0;x<width;x++) {
      int d = (int)dimd[y*width + x];
      if (d>=0 && d<drange)
	histy[d] ++;
    }
    for (int i=0;i<drange-2;i++)
      histy[i] = histy[i] + 2*histy[i+1] + histy[i+2];
    for (int i=drange-1;i>1;i--)
      histy[i] = histy[i] + 2*histy[i-1] + histy[i-2];
    totals[y] = 0;
    for (int i=0;i<drange;i++)
      totals[y] += histy[i];
  }
  // Find best line using random sampling
  float maxwei = 0.0f; 
  alpha = 0.0f; 
  beta = 0.0f; 
  disp = drange/2;
  for (int l=0;l<1000;l++) {
    int idx1 = rand()%height;
    int idx2 = rand()%height;
    while (idx1==idx2)
      idx2 = rand()%height;
    if (!totals[idx1] || !totals[idx2])
      continue;
    int cnt1 = rand()%totals[idx1];
    int cnt2 = rand()%totals[idx2];
    int disp1 = 0, disp2 = 0;
    for (int sum1=0;sum1<cnt1;disp1++) 
      sum1 += hist[idx1*drange+disp1];
    for (int sum2=0;sum2<cnt2;disp2++) 
      sum2 += hist[idx2*drange+disp2];
    disp1--;
    disp2--;
    float dgra = (float)(disp2 - disp1) / (idx2 - idx1);
    float dzer = disp2 - dgra*idx2;
    float sumwei = 0.0f;
    for (int y=0;y<height;y++) {
      for (int dd=-3;dd<=3;dd++) {
	int d = (int)(dgra*y + dzer + 0.5f) + dd;
	if (d<0 || d>=drange) 
	  continue;
	float er = d - (dgra*y + dzer);
	sumwei += hist[y*drange + d] / (4.0f + er*er);
      }
    }
    if (sumwei>maxwei && dgra>minbeta) {
      maxwei = sumwei;
      beta = dgra;
      disp = dzer;
    }
  }
  //std::cout << "### Init surface: " << maxwei/4 << " " << alpha << " " << beta << " " << disp << std::endl;
  // Improve line (depends only on y) using m-estimator
  for (int l=0;l<3;l++) {
    float syy = 0.0, sy1 = 0.0f, s11 = 0.0f;
    float sdy = 0.0, sd1 = 0.0f;
    for (int y=0;y<height;y++) {
      for (int dd=-3;dd<=3;dd++) {
	int d = (int)(beta*y + disp + 0.5f) + dd;
	if (d<0 || d>=drange) 
	  continue;
	float er = d - (beta*y + disp);
	float w = hist[y*drange + d] / (4.0f + er*er);
	syy += w*y*y;
	sy1 += w*y;
	s11 += w;
	sdy += w*d*y;
	sd1 += w*d;
      }
    }
    float det = syy*s11 - sy1*sy1;
    beta = s11*sdy - sy1*sd1;
    disp = syy*sd1 - sy1*sdy;
    if (det!=0.0f) {
      beta /= det;
      disp /= det;
    }
    //std::cout << "### Init surface: " << s11/4 << " " << alpha << " " << beta << " " << disp << std::endl;
  }
  disp += 0.5f;
  // Improve plane (depends on both x and y) using m-estimator
  for (int l=0;l<3;l++) {
    float sxx = 0.0, sx1 = 0.0f, s11 = 0.0f;
    float sdx = 0.0, sd1 = 0.0f;
    for (int y=0;y<height;y++) {
      for (int x=0;x<width;x++) {
	if (dimd[y*width+x]>0.0f) {
	  float d = dimd[y*width+x] - beta*y;
	  float er = d - (alpha*x + disp);
	  float w = 1.0f / (1.0f + er*er);
	  sxx += w*x*x;
	  sx1 += w*x;
	  s11 += w;
	  sdx += w*d*x;
	  sd1 += w*d;
	}
      }
    }
    float det = sxx*s11 - sx1*sx1;
    alpha = s11*sdx - sx1*sd1;
    disp = sxx*sd1 - sx1*sdx;
    if (det!=0.0f) {
      alpha /= det;
      disp /= det;
    }
  }
  int num = 0;
  float vard = 0.0f;
  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      int i = y*width + x;
      float d = dimd[i];
      if (d>=0.0f && d<drange) {
	float er = alpha*x + beta*y + disp - d;
	if (er*er<4*spread_d) {
	  
	  if(y>max_y)
	    max_y = y;;
	  if(y<min_y)
	    min_y = y;
	  
	  if(x>max_x)
	    max_x = x;
	  if(x<min_x)
	    min_x = x;
	  
	  vard += er*er;
	  num++;
	}
      }
    }
  }
  spread_d = (num ? vard/num : 1e-3);
  delete [] hist;
  delete [] totals;
  if (segm.verbose)
    std::cout << "### Flat surface: " << alpha << " " << beta << " " << disp << " " << spread_d << std::endl;
}  

void FgBgSegment::Foreground::Initialize(int startx, int starty)
{
  //  const float window_size = 0.10f; //%%%%
  //  const float ball_size = 0.10f;
  float *dimd = segm.disparities->GetData();
  int drange = segm.drange;
  int width = segm.width;
  int height = segm.height;
  // Find center disparity through histograms
  int *hist1 = new int[drange];
  int *hist2 = new int[drange];
  for (int i=0;i<drange;i++)
    hist1[i] = hist2[i] = 0;
  int size = (int)(window_size*height);
  int miny = std::max(starty - size/2, 0);
  int maxy = std::min(starty + size/2, height);
  int minx = std::max(startx - size/2, 0);
  int maxx = std::min(startx + size/2, width);
  for (int y=miny;y<maxy;y++) {
    for (int x=minx;x<maxx;x++) {
      int i = y*width + x;
      if (dimd[i]>0.0f && dimd[i]<drange) 
	hist1[(int)dimd[i]]++;
    }
  }
  for (int i=2;i<drange-2;i++)
    hist2[i] = (hist1[i-2] + hist1[i+2]) + 4*(hist1[i-1] + hist1[i+1]) + 6*hist1[i];
  for (int i=2;i<drange-2;i++)
    hist1[i] = (hist2[i-2] + hist2[i+2]) + 4*(hist2[i-1] + hist2[i+1]) + 6*hist2[i];
  int maxhist = 0;
  int d = 0;
  for (int i=2;i<drange-2;i++) {
    if (hist1[i]>maxhist) {
      maxhist = hist1[i]; 
      d = i;
    }
  }
  int meand = d;
  if (hist1[d+1]<hist1[d] && hist1[d-1]<hist1[d])
    meand += 0.5f*(hist1[d+1] - hist1[d-1]) / (2.0f*hist1[d] - hist1[d+1] - hist1[d-1]);
  // Find spread of disparity peak
  int maxd = d+1;
  while (maxd<drange-1 && hist1[maxd]<hist1[maxd-1]) 
    maxd++;
  int mind = d-1;
  while (mind>0 && hist1[mind]<hist1[mind+1]) 
    mind--;
  float d2 = 0.0f;
  int di = 0;
  for (int i=mind;i<=maxd;i++) {
    d2 += hist1[i]*(i-meand)*(i-meand);
    di += hist1[i];
  }
  float vard = d2/di + 1.0f;
  float stdd = sqrt(vard); 
  std::cout << "meand: " << meand << "  vard: " << vard << std::endl;
  delete [] hist1;
  delete [] hist2;
  // Find largest region within disparity drange and not on surface
  Image<uint8_t> region(width, height);
  uint8_t *regd = region.GetData();
#if 1
  segm.MakeSegmentImage(region);
  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      int i = y*width + x;
      if (!regd[i]) {
	int dx = x - startx;
	int dy = y - starty;
	if (segm.withDisparities) {
	  if ((dx*dx + dy*dy)>(height*height*ball_size*ball_size) || dimd[i]<(meand-stdd) || dimd[i]>(meand+stdd)) 
	    regd[i] = 1;
	} else {
	  if ((dx*dx + dy*dy)>(height*height*ball_size*ball_size)) 
	    regd[i] = 1;
	}
      }
    }
  }
#else
  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      int i = y*width + x;
      regd[i] = 1;
      if (segm.withDisparities) {
	int dx = x - startx;
	int dy = y - starty;
	if ((dx*dx + dy*dy)<(height*height*ball_size*ball_size)) {
	  if (dimd[i]>0.0f && dimd[i]<drange) {
	    float er = segm.surface.alpha*x + segm.surface.beta*y + segm.surface.disp - dimd[i];
	    if (er*er>2*segm.surface.spread_d && dimd[i]>(meand-stdd) && dimd[i]<(meand+stdd))
	      regd[i] = 0;
	  } 
	}
      } else {
	int dx = x - startx;
	int dy = y - starty;
	if ((dx*dx + dy*dy)<(height*height*ball_size*ball_size)) 
	  regd[i] = 0;
      }
    }
  }
#endif
  //region.Store("region.pgm", true, false);
  int num = 0;
  if (combined) {
    Matrix3 var3d;
    Vector3 pos3d;
    float var00 = 0.0f, var01 = 0.0f, var02 = 0.0f;
    float var11 = 0.0f, var12 = 0.0f, var22 = 0.0f;
    float pos0 = 0.0f, pos1 = 0.0f, pos2 = 0.0f;
    for (int y=0;y<height;y++) {
      for (int x=0;x<width;x++) {
	int i = y*width + x;
	if (regd[i]==0) {
	  float d = dimd[i];
	  var00 += x*x;
	  var01 += x*y;
	  var02 += x*d;
	  var11 += y*y;
	  var12 += y*d;
	  var22 += d*d;
	  pos0 += x;
	  pos1 += y;
	  pos2 += d;
	  num++;
	}
      }  
    }
    if (num>0) {
      var3d(0,0) = var00;
      var3d(0,1) = var01;
      var3d(0,2) = var02;
      var3d(1,1) = var11;
      var3d(1,2) = var12;
      var3d(2,2) = var22;
      pos3d(0) = pos0;
      pos3d(1) = pos1;
      pos3d(2) = pos2;
      var3d *= (num>0.0f ? 1.0/num : 1.0f);
      pos3d *= (num>0.0f ? 1.0/num : 1.0f);
      var3d(0,0) -= pos3d(0)*pos3d(0);
      var3d(0,1) -= pos3d(0)*pos3d(1);
      var3d(0,2) -= pos3d(0)*pos3d(2);
      var3d(1,1) -= pos3d(1)*pos3d(1);
      var3d(1,2) -= pos3d(1)*pos3d(2);
      var3d(2,2) -= pos3d(2)*pos3d(2);
      var3d(1,0) = var3d(0,1);
      var3d(2,0) = var3d(0,2);
      var3d(2,1) = var3d(1,2);
      position3d = pos3d;
      spread3d = var3d;
    }
  } else {
    Matrix2 var_p;
    Vector2 position;
    float disp = 0.0f;
    float var_d = 0.0f;
    float var00 = 0.0f, var01 = 0.0f, var11 = 0.0f;
    float pos0 = 0.0f, pos1 = 0.0f;
    for (int y=0;y<height;y++) {
      for (int x=0;x<width;x++) {
	int i = y*width + x;
	if (regd[i]==0) {
	  var00 += x*x;
	  var01 += x*y;
	  var11 += y*y;
	  pos0 += x;
	  pos1 += y;
	  disp += dimd[i];
	  var_d += dimd[i]*dimd[i];
	  num++;
	}
      }  
    }
    if (num>0) {
      double inum = (num>0.0 ? 1.0/num : 1.0f);
      var_p(0,0) = var00;
      var_p(0,1) = var01;
      var_p(1,1) = var11;
      position(0) = pos0;
      position(1) = pos1;
      var_p *= inum;
      position *= inum;
      var_p(0,0) -= position(0)*position(0);
      var_p(0,1) -= position(0)*position(1);
      var_p(1,1) -= position(1)*position(1);
      var_p(1,0) = var_p(0,1);
      var_d *= inum; 
      disp *= inum;
      var_d -= disp*disp;
      position3d(0) = position(0);
      position3d(1) = position(1);
      position3d(2) = disp;
      spread3d.identity();
      spread3d(0,0) = var_p(0,0);
      spread3d(0,1) = var_p(0,1);
      spread3d(1,0) = var_p(1,0);
      spread3d(1,1) = var_p(1,1);
      spread3d(2,2) = var_d;
    }
  }
  if (segm.verbose) {
    std::cout << "### Foreground position3d: " << position3d << " " << num << std::endl;
    std::cout << "### Foreground spread3d: " << spread3d << std::endl;
  }
}

void FgBgSegment::Foreground::SetInitParams( float l_window_size, 
					     float l_ball_size)
{
  window_size = l_window_size;
  ball_size   = l_ball_size;
}


void FgBgSegment::Background::Initialize()
{
  const int numFigures = segm.figures.size();
  float *dimd = segm.disparities->GetData();
  int drange = segm.drange;
  int width = segm.width;
  int height = segm.height;
  float mean = 0.0f;
  float vari = 0.0f;
  int num = 0;
  float ivar[numFigures][6]; 
  for (int f=0;f<numFigures;f++) {
    Matrix3 invs = segm.figures[f]->spread3d;
    invs = invs.invert();
    ivar[f][0] = invs(0,0), ivar[f][1] = invs(0,1), ivar[f][2] = invs(1,1);
    ivar[f][3] = invs(0,2), ivar[f][4] = invs(1,2), ivar[f][5] = invs(2,2);
  }
  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      int i = y*width + x;
      if (dimd[i]>0.0f && dimd[i]<drange) {
	float d = dimd[i];
	bool used = false;
	for (int f=0;f<numFigures;f++) {
	  float er_x = x - segm.figures[f]->position3d(0);
	  float er_y = y - segm.figures[f]->position3d(1);
	  float er_d = d - segm.figures[f]->position3d(2);
	  float er_f = er_x*er_x*ivar[f][0] + 2*er_x*er_y*ivar[f][1] + er_y*er_y*ivar[f][2];
	  er_f += er_d*(2*er_x*ivar[f][3] + 2*er_y*ivar[f][4] + er_d*ivar[f][5]);
	  used |= (er_f<16.0f);
	}
	float diff = (segm.surface.alpha*x + segm.surface.beta*y + segm.surface.disp - dimd[i]);
	float er_s = diff*diff/segm.surface.spread_d;
	used |= (er_s<16.0f);
	if (!used) {
	  mean += d;
	  vari += d*d;
	  num++;
	}
      }
    }
  }
  if (num>0) {
    mean /= num;
    vari /= num;
    disp = mean;
    spread_d = vari - mean*mean;
  } else {
    disp = drange/2;
    spread_d = disp*disp/4.0f;
  }    
  if (segm.verbose)
    std::cout << "### Clutter spread: " << disp << " " << spread_d << std::endl;
}

void FgBgSegment::InitSegmentation()
{
  const int numFigures = figures.size();
  float *dimd = disparities->GetData();
  float *reg_g = ground.probabilities.GetData();
  float *reg_s = surface.probabilities.GetData();
  /*
  Fill(ground.probabilities, 1.0f/3.0f);
  Fill(surface.probabilities, 1.0f/3.0f);
  */
  Fill(ground.probabilities, 1.0f/((float)numFigures+2.0f));
  Fill(surface.probabilities, 1.0f/((float)numFigures+2.0f));
  
  float er_g = 2.0f*log((float)width*height*drange);
  float const_s = log(surface.spread_d) + 2.0f*log((float)width*height);
  float *reg_f[numFigures], const_f[numFigures], prob_f[numFigures];
  float ivar[numFigures][6]; 
  for (int f=0;f<numFigures;f++) {

    //  Fill(figures[f]->probabilities, 1.0f/3.0f);
    Fill(figures[f]->probabilities, 1.0f/((float)numFigures+2.0f));

    reg_f[f] = figures[f]->probabilities.GetData();
    const_f[f] = log(figures[f]->spread3d.determinant());
    Matrix3 invs = figures[f]->spread3d;
    invs = invs.invert();
    ivar[f][0] = invs(0,0), ivar[f][1] = invs(0,1), ivar[f][2] = invs(1,1);
    ivar[f][3] = invs(0,2), ivar[f][4] = invs(1,2), ivar[f][5] = invs(2,2);
  }
  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      int i = y*width + x;
      float d = dimd[i];
      if (d>0.0f && d<drange) {
	float prob_g = (withSurface ? exp(-0.5f*er_g) : eps); 
	float diff = d - (surface.alpha*x + surface.beta*y + surface.disp);
	float er_s = diff*diff/surface.spread_d;
	er_s += const_s;
	float prob_s = exp(-0.5f*er_s); 
	float sumprob = prob_g + prob_s;
	for (int f=0;f<numFigures;f++) {
	  float difx = x - figures[f]->position3d(0);
	  float dify = y - figures[f]->position3d(1);
	  float difd = d - figures[f]->position3d(2);
	  float er_f = difx*difx*ivar[f][0] + 2*difx*dify*ivar[f][1] + dify*dify*ivar[f][2];
	  er_f += difd*(2*difx*ivar[f][3] + 2*dify*ivar[f][4] + difd*ivar[f][5]);
	  er_f += const_f[f];
	  prob_f[f] = exp(-0.5f*er_f); 
	  sumprob += prob_f[f];
	}
	reg_g[i] = prob_g / sumprob;
	reg_s[i] = prob_s / sumprob;
	for (int f=0;f<numFigures;f++) 
	  reg_f[f][i] = prob_f[f] / sumprob;
      }
    }
  }
} 

//================== Update routines =====================//

void FgBgSegment::FlatSurface::Update()
{ 
  float *dimd = segm.disparities->GetData();
  int drange = segm.drange;
  int width = segm.width;
  int height = segm.height;
  // reset plane limits
  min_x = width;
  max_x = 0;
  min_y = height;
  max_y = 0;
  float *reg_s = probabilities.GetData();
  float xx00 = 0.0f, xx01 = 0.0f, xx02 = 0.0f;
  float xx11 = 0.0f, xx12 = 0.0f, xx22 = 0.0f;
  float xd0 = 0.0f, xd1 = 0.0f, xd2 = 0.0f;
  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      int i = y*width + x;
      float d = dimd[i];
      if (d>0.0f && d<drange) {
	float w = reg_s[i];
	xx00 += x*x*w;
	xx01 += x*y*w;
	xx02 += x*w;
	xx11 += y*y*w;
	xx12 += y*w;
	xx22 += w;
	xd0 += x*d*w;
	xd1 += y*d*w;
	xd2 += d*w;
      }
    }
  }
  Matrix3 xx;
  Vector3 xd;
  xx(0,0) = xx00;
  xx(0,1) = xx01;
  xx(0,2) = xx02;
  xx(1,1) = xx11;
  xx(1,2) = xx12;
  xx(2,2) = xx22;
  xx(1,0) = xx(0,1);
  xx(2,0) = xx(0,2);
  xx(2,1) = xx(1,2);
  xd(0) = xd0;
  xd(1) = xd1;
  xd(2) = xd2;
  float num = xx(2,2);
  xx *= (num>0.0f ? 1.0/num : 1.0f);
  xd *= (num>0.0f ? 1.0/num : 1.0f);
  xx(0,0) += spread_d*weight_a;
  xx(1,1) += spread_d*weight_b;
  xx(2,2) += spread_d*weight_d;
  xd(0) += alpha * spread_d*weight_a;
  xd(1) += beta  * spread_d*weight_b;
  xd(2) += disp  * spread_d*weight_d;
  xx(0,0) += eps;
  xx(1,1) += eps;
  xx(2,2) += eps;
  Vector3 p = xx.invert()*xd;
  float sumer = 0.0f;
  num = 0.0f;
  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      int i = y*width + x;
      float d = dimd[i];
      if (d>0.0f && d<drange) {
	float er = alpha*x + beta*y + disp - d;
	float w = reg_s[i];
	sumer += w*er*er;
	num += w;

	if (er*er<4*spread_d) {
	  if(y>max_y)
	      max_y = y;;
	    if(y<min_y)
	      min_y = y;

	    if(x>max_x)
	      max_x = x;
	    if(x<min_x)
	      min_x = x;
	  }

      }
    }
  }
  alpha = p(0);
  beta  = p(1);
  disp = p(2);
  spread_d = ((num>0.0f ? sumer/num : 1.0f) + strength*spread_d) / (1 + strength);
  //if (spread_d>1.0f) //%%%%
  //  spread_d = 1.0f; //%%%%
  if (segm.verbose)
    std::cout << "### Flat surface: " << alpha << " " << beta << " " << disp << " " << spread_d << std::endl;
}

void FgBgSegment::Foreground::Update()
{
  float *dimd = segm.disparities->GetData();
  int drange = segm.drange;
  int width = segm.width;
  int height = segm.height;
  float *reg_f = probabilities.GetData();
  double num = 0.0f;
  if (combined) {
    Matrix3 var3d;
    Vector3 pos3d;
    var3d = 0.0;
    pos3d = 0.0;
    for (int y=0;y<height;y++) {
      for (int x=0;x<width;x++) {
	int i = y*width + x;
	if (dimd[i]>0.0f && dimd[i]<drange) {
	  float w = reg_f[i];
	  float d = dimd[i];
	  var3d(0,0) += x*x*w;
	  var3d(0,1) += x*y*w;
	  var3d(0,2) += x*d*w;
	  var3d(1,1) += y*y*w;
	  var3d(1,2) += y*d*w;
	  var3d(2,2) += d*d*w;
	  pos3d(0) += x*w;
	  pos3d(1) += y*w;
	  pos3d(2) += d*w;
	  num += w;
	}
      }
    }
    if (num>0.0f) {
      double inum = (num>0.0f ? 1.0f/num : 1.0f);
      var3d *= inum;
      pos3d *= inum;
      var3d(0,0) -= pos3d(0)*pos3d(0);
      var3d(0,1) -= pos3d(0)*pos3d(1);
      var3d(0,2) -= pos3d(0)*pos3d(2);
      var3d(1,1) -= pos3d(1)*pos3d(1);
      var3d(1,2) -= pos3d(1)*pos3d(2);
      var3d(2,2) -= pos3d(2)*pos3d(2);
      var3d(1,0) = var3d(0,1);
      var3d(2,0) = var3d(0,2);
      var3d(2,1) = var3d(1,2);
      Matrix3 weight;
      weight = 0.0;
      weight(0,0) = weight_p;
      weight(1,1) = weight_p;
      weight(2,2) = weight_d;
      Matrix3 scale = spread3d*weight; 
      scale(0,0) += 1.0;
      scale(1,1) += 1.0;
      scale(2,2) += 1.0;
      pos3d += spread3d*(weight*position3d);
      pos3d = scale.invert()*pos3d;
      var3d += spread3d*strength;
      var3d *= 1.0/(1.0 + strength) ;
      position3d = pos3d;
      spread3d = var3d;
    }
  } else {
    Matrix2 var_p;
    Vector2 position;
    double disp = 0.0;
    double var_d = 1.0;
    var_p = 0.0;
    position = 0.0;
    for (int y=0;y<height;y++) {
      for (int x=0;x<width;x++) {
	int i = y*width + x;
	if (dimd[i]>0.0f && dimd[i]<drange) {
	  float w = reg_f[i];
	  var_p(0,0) += x*x*w;
	  var_p(0,1) += x*y*w;
	  var_p(1,1) += y*y*w;
	  position(0) += x*w;
	  position(1) += y*w;
	  disp += dimd[i]*w;
	  var_d += dimd[i]*dimd[i]*w;
	  num += w;
	}
      }
    }
    if (num>0.0f) {
      double inum = (num>0.0 ? 1.0/num : 1.0);
      var_p *= inum;
      position *= inum;
      var_p(0,0) -= position(0)*position(0);
      var_p(0,1) -= position(0)*position(1);
      var_p(1,1) -= position(1)*position(1);
      var_p(1,0) = var_p(0,1); 
      Matrix2 spread_p = spread3d.submatrix(0,0);
      Vector2 figure_p;
      figure_p(0) = position3d(0); 
      figure_p(1) = position3d(1); 
      Matrix2 scale_p = spread_p*weight_p; 
      scale_p(0,0) += 1.0;
      scale_p(1,1) += 1.0;
      position += ((spread_p*figure_p)*weight_p);  
      position = scale_p.invert()*position;
      var_p += spread_p*strength;
      var_p *= 1.0/(1.0 + strength);
      double spread_d = spread3d(2,2);
      double figure_d = position3d(2);
      var_d *= inum; 
      disp *= inum;
      var_d -= disp*disp;
      var_d += 1.00; 
      disp += spread_d*weight_d*figure_d;
      disp /= (1.0 + spread_d*weight_d);
      var_d += strength*spread_d;
      var_d /= (1.0 + strength);
      position3d(0) = position(0);
      position3d(1) = position(1);
      position3d(2) = disp;
      spread3d.identity();
      spread3d(0,0) = var_p(0,0);
      spread3d(0,1) = var_p(0,1);
      spread3d(1,0) = var_p(1,0);
      spread3d(1,1) = var_p(1,1);
      spread3d(2,2) = var_d;
    }
  }
  if (segm.verbose) {
    std::cout << "### Foreground position3d: " << position3d << " " << num << std::endl;
    std::cout << "### Foreground spread3d: " << spread3d << std::endl;
  }
}

void FgBgSegment::Background::Update()
{
  float *reg_g = probabilities.GetData();
  float *dimd = segm.disparities->GetData();
  int drange = segm.drange;
  int width = segm.width;
  int height = segm.height;
  float mean = 0.0f;
  float vari = 0.0f;
  float num = 0.0f;
  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      int i = y*width + x;
      if (dimd[i]>0.0f && dimd[i]<drange) {
	float w = reg_g[i];
	mean += dimd[i]*w;
	vari += dimd[i]*dimd[i]*w;
	num += w;
      }
    }
  }
  if (num>0) {
    mean /= num;
    vari /= num;
    vari -= mean*mean;
    mean += spread_d*weight_d*disp;
    mean /= (1.0f + spread_d*weight_d);
    vari += strength*spread_d;
    vari /= (1.0f + strength);
    disp = mean;
    spread_d = vari;
  }
  if (segm.verbose)
    std::cout << "### Clutter spread: " << disp << " " << spread_d << std::endl;
}

float FgBgSegment::ColorModel::CreateHistogram( Image<float> &probabilities, 
						bool allPoints)
{
  int count = 0;

  float *reg_g = probabilities.GetData();
  float *dimd = segm.disparities->GetData();
  int drange = segm.drange;
  int width = segm.width;
  int height = segm.height;
  uint8_t *himd = segm.hue.GetData();
  uint8_t *simd = segm.saturation.GetData();
  uint8_t *gimd = segm.grey.GetData();
  float hist[hist_size*hist_size];
  float grey[hist_size];

  for (int i=0;i<hist_size*hist_size;i++) 
    hist[i] = 0.0f;

  for (int i=0;i<hist_size;i++) 
    grey[i] = 0.0f;

  float sumProb = 0.0f;
  for (int i=0;i<width*height;i++) {
    float prob = reg_g[i]; 
    sumProb += prob;
    if (allPoints || (dimd[i]>0.0f && dimd[i]<drange)) {
      if(!segm.withColorHoles || (dimd[i]>0.0f && dimd[i]<drange)){
	int ix = hist_size*himd[i]/256;
	int iy = hist_size*simd[i]/256;
	int idx = iy*hist_size + ix;
	hist[idx] += prob;
      } else if(!segm.uniform){
	ROS_ASSERT(!(dimd[i]>0.0f && dimd[i]<drange));
	int g = hist_size*gimd[i]/256;
	grey[g] += prob;
	count ++;
      }
    }
  }

  ROS_DEBUG("%d pixels with invalid colour information in histogram creation", count);

  float *phist = NULL;
  if (prior!=NULL)
    phist = prior->histogram;
  
  SmoothAndNormalizeHist( hist, phist, hist_size*hist_size, 
			  histogram, colorcost);
  if(!segm.uniform) {
    if (prior!=NULL)
      phist = prior->greyhist;
    else 
      phist = NULL;
    SmoothAndNormalizeHist( grey, phist, hist_size, greyhist, greycost);
  }
  /*
  float fac_old = 1.0f / ColorModel::weight;
  float fac_pri = 2.0f*fac_old;
  float num = 0.0f;
  for (int i=0;i<hist_size*hist_size;i++)
    num += hist[i];
  float inum = (num>0.0f ? 1.0/num : 1.0);
  if (prior!=NULL) {
    float *phist = prior->histogram;
    for (int i=0;i<hist_size*hist_size;i++) 
      histogram[i] = (hist[i]*inum + fac_old*histogram[i] + fac_pri*phist[i])/(1.0f + fac_old + fac_pri);
  } else {
    for (int i=0;i<hist_size*hist_size;i++) 
      histogram[i] = (hist[i]*inum + fac_old*histogram[i])/(1.0f + fac_old);
  }
  num = 0.0f;
  for (int i=0;i<hist_size*hist_size;i++) 
    num += histogram[i];
  inum = (num>0.0f ? 1.0/num : 1.0);
  for (int i=0;i<hist_size*hist_size;i++) {
    histogram[i] *= inum;
    colorcost[i] = -2.0f*log(std::max(histogram[i], eps/hist_size/hist_size));
  }
  */
  
  return sumProb;
}

void FgBgSegment::ColorModel::SmoothAndNormalizeHist( float const* hist,
						      float const* phist,
						      int size, 
						      float* const histogram,
						      float* const cost)
{
  
  float fac_old = 1.0f / ColorModel::weight;
  float fac_pri = 4.0f*fac_old;
  float num = 0.0f;
  for (int i=0;i<size;i++)
    num += hist[i];
  float inum = (num>0.0f ? 1.0/num : 1.0);
  if (phist!=NULL) {
    //  if (prior!=NULL) {
    //  float *phist = prior->histogram;
    for (int i=0;i<size;i++) 
      histogram[i] = (hist[i]*inum + fac_old*histogram[i] 
		      + fac_pri*phist[i])/(1.0f + fac_old + fac_pri);
  } else {
    for (int i=0;i<size;i++) 
      histogram[i] = (hist[i]*inum 
		      + fac_old*histogram[i])/(1.0f + fac_old);
  }

  NormalizeHist(histogram, cost, size);

  /*
    num = 0.0f;
    for (int i=0;i<hist_size*hist_size;i++) 
    num += histogram[i];
    inum = (num>0.0f ? 1.0/num : 1.0);
    for (int i=0;i<hist_size*hist_size;i++) {
    histogram[i] *= inum;
    cost[i] = -2.0f*log(std::max(histogram[i], eps/hist_size/hist_size));
    }
  */
}

void FgBgSegment::ColorModel::NormalizeHist( float* const histogram,
					     float* const cost,
					     int size)
{
  float num = 0.0f;
  for (int i=0;i<size;i++) 
    num += histogram[i];
  float inum = (num>0.0f ? 1.0/num : 1.0);
  for (int i=0;i<size;i++) {
    histogram[i] *= inum;
    cost[i] = -2.0f*log(std::max(histogram[i], eps/(float)size));
  }
}

void FgBgSegment::ColorModel::CreateHistogram(Image<uint8_t> &mask, 
					      bool allPoints)
{
  int count = 0;

  uint8_t *masd = mask.GetData();
  float   *dimd = segm.disparities->GetData();
  uint8_t *himd = segm.hue.GetData();
  uint8_t *simd = segm.saturation.GetData();
  uint8_t *gimd = segm.grey.GetData();
  int width = segm.width;
  int height = segm.height;

  for (int i=0;i<hist_size*hist_size;i++) 
    histogram[i] = 0.0f;

  if(!segm.uniform)
    for (int i=0;i<hist_size;i++) 
      greyhist[i] = 0.0f;

  int num  = 0;
  for (int i=0;i<width*height;i++) {
    if (masd[i]>0) {
      if (allPoints || (dimd[i]>0.0f && dimd[i]<segm.drange)) {
	if(!segm.withColorHoles || 
	   (dimd[i]>0.0f && dimd[i]<segm.drange)){
	  int ix = hist_size*himd[i]/256;
	  int iy = hist_size*simd[i]/256;
	  int idx = iy*hist_size + ix;
	  histogram[idx] ++;
	  num ++;
	} else if(!segm.uniform){
	  ROS_ASSERT(!(dimd[i]>0.0f && dimd[i]<segm.drange));
	  int g = hist_size*gimd[i]/256;
	  greyhist[g] ++;
	  count++;
	}
      }
    }
  }

  ROS_DEBUG("%d Pixels used for creating the histogram with mask", count);

  NormalizeHist(histogram, colorcost, hist_size*hist_size);
  if(!segm.uniform)
    NormalizeHist(greyhist, greycost, hist_size);
  /*
  double inum = (num>0 ? 1.0/num : 1.0);
  for (int i=0;i<hist_size*hist_size;i++) {
    histogram[i] *= inum;
    colorcost[i] = -2.0f*log(std::max(histogram[i], eps/hist_size/hist_size));
  }
  */
}

void FgBgSegment::CreateHistograms(bool allPoints)
{
  ground.CreateHistogram(ground.probabilities, allPoints);
  surface.CreateHistogram(surface.probabilities, allPoints);
  for (unsigned int f=0;f<figures.size();f++)
    figures[f]->CreateHistogram(figures[f]->probabilities, allPoints);
}

void FgBgSegment::SetNewForeground(int startx, int starty, 
				   Image<float> &dimg, int drange_)
{
  Foreground *model = new Foreground(*this, width, height);
  model->SetInitParams( windowSize, ballSize);
  figures.push_back(model);
  model->Initialize(startx, starty);
  Image<float> probs(width, height);
  float *prod = probs.GetData();
  float ivar[6];  
  Matrix3 invs = model->spread3d;
  if (invs.determinant()!=0.0)
    invs = invs.invert();
  ivar[0] = invs(0,0), ivar[1] = invs(0,1), ivar[2] = invs(1,1);
  ivar[3] = invs(0,2), ivar[4] = invs(1,2), ivar[5] = invs(2,2);
  Image<uint8_t> mask(width, height);
  MakeSegmentImage(mask);
  uint8_t *masd = mask.GetData();
  float *dimd = dimg.GetData();
  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      int i = y*width + x;
      if (!masd[i]) {
	float d = dimd[i];
	float er_x = x - model->position3d(0);  
	float er_y = y - model->position3d(1);
	float er_d = d - model->position3d(2);
	float er_f = er_x*er_x*ivar[0] + 2*er_x*er_y*ivar[1] + er_y*er_y*ivar[2];
	if (dimd[i]>0.0f && dimd[i]<drange_) 
	  er_f += er_d*(2*er_x*ivar[3] + 2*er_y*ivar[4] + er_d*ivar[5]);
	prod[y*width + x] = exp(-0.5f*er_f);
      } else
	prod[y*width + x] = 0.0f;
    }
  }
  model->CreateHistogram(probs, true);
}

void FgBgSegment::SetNewForeground(Image<uint8_t> &mask, 
				   Image<float> &dimg, 
				   int drange_, bool reuseLast)
{
  if (!reuseLast)
    figures.push_back(new Foreground(*this, width, height));
  Foreground &fig = *figures.back();
  uint8_t *masd = mask.GetData();
  float *dimd = dimg.GetData();
  Matrix2 var_p;
  Vector2 position;
  double disp = 0.0;
  double var_d = 1.0;
  double num = 0.0, numd = 0.0;
  var_p = 0.0;
  position = 0.0;
  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      int i = y*width + x;
      if (masd[i]>0) {
	var_p(0,0) += x*x;
	var_p(0,1) += x*y;
	var_p(1,1) += y*y;
	position(0) += x;
	position(1) += y;
	num += 1.0;
	float d = dimd[i];
	if (d>0.0 && d<drange_) {
	  disp += d;
	  var_d += d*d;
	  numd += 1.0;
	}
      }
    }
  }
  if (num>0.0) {
    double inum = (num>0.0 ? 1.0/num : 1.0);
    var_p *= inum;
    position *= inum;
    var_p(0,0) -= position(0)*position(0);
    var_p(0,1) -= position(0)*position(1);
    var_p(1,1) -= position(1)*position(1);
    var_p(1,0) = var_p(0,1); 
    inum = (numd>0.0 ? 1.0/numd : 1.0);
    var_d *= inum; 
    disp *= inum;
    var_d -= disp*disp;
    fig.position3d(0) = position(0);
    fig.position3d(1) = position(1);
    fig.position3d(2) = disp;
    fig.spread3d.identity();
    fig.spread3d(0,0) = var_p(0,0);
    fig.spread3d(0,1) = var_p(0,1);
    fig.spread3d(1,0) = var_p(1,0);
    fig.spread3d(1,1) = var_p(1,1);
    fig.spread3d(2,2) = var_d;
  }
  if (figures.size()<=colorPriors.size()) {
    ColorModel &prior = colorPriors[figures.size()-1];
    for (int i=0;i<hist_size*hist_size;i++) {
      fig.histogram[i] = prior.histogram[i];
      fig.colorcost[i] = prior.colorcost[i];
    }
    fig.prior = &prior;
  } else
    fig.CreateHistogram(mask, true);
  if (verbose){
    std::cout << "Figure position: " << fig.position3d << std::endl;
    std::cout << "Figure spread: " << std::endl << fig.spread3d << std::endl;
    std::cout << "Foregrounds: " << figures.size() << "  Priors: " << colorPriors.size() << std::endl;
  }
}

template <int numFigures>
void FgBgSegment::PixSegment(FgBgSegment &segm)
{
  int width = segm.width;
  int height = segm.height;
  int drange = segm.drange;
  float *dimd = segm.disparities->GetData();
  uint8_t *himd = segm.hue.GetData();
  uint8_t *simd = segm.saturation.GetData();
  uint8_t *gimd = segm.grey.GetData();
  
  Fill(segm.ground.probabilities, 1.0f/((float)numFigures+2.0f));
  Fill(segm.surface.probabilities, 1.0f/((float)numFigures+2.0f));
  /*
  Fill(segm.ground.probabilities, 1.0f/3.0f);
  Fill(segm.surface.probabilities, 1.0f/3.0f);
  */
  float const_sd = log(segm.surface.spread_d);
  float const_sp = 2.0f*log((float)width*height/1); 
  float const_s0 = -2.0*log(0.45f); 
  float const_su = -2.0*log(0.40f);
  float const_gd = 2.0f*log((float)drange/2); 
  float const_gp = 2.0f*log((float)width*height/1); 
  float const_g0 = -2.0*log(0.45f); 
  float const_gu = -2.0*log(0.40f);

  float const_fd[numFigures], const_fp[numFigures]; 
  float const_f0 = -2.0*log(0.10f);
  float const_fu = -2.0*log(0.20f);
  float ivar[numFigures][6]; 
  for (int f=0;f<numFigures;f++) {
    
    //Fill(segm.figures[f]->probabilities, 1.0f/3.0f);
    Fill(segm.figures[f]->probabilities, 1.0f/((float)numFigures+2.0f));

    const_fd[f] = log(segm.figures[f]->spread3d(2,2)); 
    const_fp[f] = log(segm.figures[f]->spread3d.determinant()) - const_fd[f];
    Matrix3 invs = segm.figures[f]->spread3d;
    invs = invs.invert();
    ivar[f][0] = invs(0,0), ivar[f][1] = invs(0,1), ivar[f][2] = invs(1,1);
    ivar[f][3] = invs(0,2), ivar[f][4] = invs(1,2), ivar[f][5] = invs(2,2);
  }

  BeliefProp<numFigures+2> belief(width, height);
  belief.SetGradientCosts(segm.grey, segm.gradWeight); 
  float **priors = belief.GetPriors();

  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      int i = y*width + x;
      int ix = hist_size*himd[i]/256;
      int iy = hist_size*simd[i]/256;
      int idx = iy*hist_size + ix;
      for (int f=0;f<numFigures;f++) {
	float er_f = const_f0;
	float difx = x - segm.figures[f]->position3d(0);
	float dify = y - segm.figures[f]->position3d(1);
	float difp = difx*difx*ivar[f][0] + 2*difx*dify*ivar[f][1] + dify*dify*ivar[f][2];
	er_f += const_fp[f] + (difp<9.0f ? difp : 100.0f);
	if (segm.withDisparities) {
	  float dcostf = const_fu;
	  if (dimd[i]>0.0f && dimd[i]<drange) {
	    float difd = dimd[i] - segm.figures[f]->position3d(2);
	    dcostf = difd*(2*difx*ivar[f][3] + 2*dify*ivar[f][4] + difd*ivar[f][5]) + const_fd[f];
	  }
	  er_f += dcostf;
	}
	if (segm.withColors && !segm.withColorHoles) {
	  er_f += segm.figures[f]->colorcost[idx];
	} else if(segm.withColors && segm.withColorHoles){
	  // in case colour of this pixel not defined 
	  // (hue & saturation ==0) equal probability for each label
	  if(!(dimd[i]>0.0f && dimd[i]<drange)){
	    if(segm.uniform){
	      
	      float norm = numFigures+2.0f;
	      er_f += -2.0f*log(1.0f/norm);
	      
	    } else {
	      
	      int idx = hist_size*gimd[i]/256;
	      er_f += segm.figures[f]->greycost[idx];

	    }
	  } else {
	    er_f += segm.figures[f]->colorcost[idx];
	  }
	}
	priors[2+f][i] = 0.5f*er_f;
      }
      float er_g = const_g0; 
      float er_s = const_s0;
      er_g += const_gp;
      er_s += const_sp;
      if (segm.withDisparities) {
	float dcosts = const_su; 
	float dcostg = const_gu;
	if (dimd[i]>0.0f && dimd[i]<drange) {
	  float diff = dimd[i] - (segm.surface.alpha*x 
				  + segm.surface.beta*y 
				  + segm.surface.disp);
	  dcosts = diff*diff/segm.surface.spread_d + const_sd;
	  dcostg = const_gd;
	} 
	er_s += dcosts;
	er_g += dcostg;
      } 
      if (segm.withColors && !segm.withColorHoles) { 
	er_g += segm.ground.colorcost[idx];
	er_s += segm.surface.colorcost[idx];
      } else if(segm.withColors && segm.withColorHoles){
	// in case colour of this pixel not defined 
	// (hue & saturation ==0) equal probability for each label
	if (!(dimd[i]>0.0f && dimd[i]<drange)){
	  if(segm.uniform){
	    
	    float norm = numFigures+2.0f;
	    er_g += -2.0f*log(1.0f/norm);
	    er_s += -2.0f*log(1.0f/norm);
	    
	  } else {
	    
	    int idx = hist_size*gimd[i]/256;
	    er_g += segm.ground.greycost[idx];
	    er_s += segm.surface.greycost[idx];
	    
	  }
	  
	} else {
	  er_g += segm.ground.colorcost[idx];
	  er_s += segm.surface.colorcost[idx];
	}
      }

      priors[0][i] = 0.5f*er_g;
      priors[1][i] = 0.5f*er_s;
    }
  }

  float **beliefs = belief.GetBeliefs();
  TimerCPU timer(2800);
  //belief.Execute(10);
  belief.Execute(5, 3);
  std::cout << "Belief: " <<  timer.read() << " ms" << std::endl;
  //belief.ComputeMAP(segm.grey); //%%%
  float *reg_g = segm.ground.probabilities.GetData();
  float *reg_s = segm.surface.probabilities.GetData();
  float *reg_f[numFigures]; 
  float prob_f[numFigures];
  for (int f=0;f<numFigures;f++) 
    reg_f[f] = segm.figures[f]->probabilities.GetData();
  for (int i=0;i<width*height;i++) {
    float minbelief = (beliefs[0][i]<beliefs[1][i] ? beliefs[0][i] : beliefs[1][i]);
    for (int f=0;f<numFigures;f++) 
      minbelief = (beliefs[2+f][i]<minbelief ? beliefs[2+f][i] : minbelief);
    float prob_g = exp(minbelief - beliefs[0][i]) + eps;
    float prob_s = exp(minbelief - beliefs[1][i]) + eps;
    float sumprob = prob_g + prob_s;
    for (int f=0;f<numFigures;f++) {
      prob_f[f] = exp(minbelief - beliefs[2+f][i]) + eps;
      sumprob += prob_f[f];
    }
    reg_g[i] = prob_g / sumprob;
    reg_s[i] = prob_s / sumprob;
    for (int f=0;f<numFigures;f++) 
      reg_f[f][i] = prob_f[f] / sumprob;
  }  
  if (segm.verbose>1) {
    Image<unsigned char> mask(width, height);
    segm.MakeSegmentImage(mask);
    mask.Store("mask.pgm", true, false);
    if (numFigures) {
      Image<float>(width, height, reg_g).Store("probsg.pgm", true, false);
      Image<float>(width, height, reg_s).Store("probss.pgm", true, false);
      Image<float>(width, height, reg_f[0]).Store("probsf1.pgm", true, false);
    }
  }
}

void FgBgSegment::RGBToHSV(Image<uint8_t> &cimg) 
{  
  uint8_t *srcd = cimg.GetData();
  uint8_t *himd = hue.GetData();
  uint8_t *simd = saturation.GetData();
  uint8_t *vimd = grey.GetData();
  for (int i=0;i<width*height;i++) {
    short r = srcd[3*i+0];
    short g = srcd[3*i+1];
    short b = srcd[3*i+2];
    int minv = (r<g ? r : g);
    minv = (b<minv ? b : minv);
    int maxv = (r>g ? r : g);
    maxv = (b>maxv ? b : maxv);
    int diff = maxv - minv;
    int dif6 = diff*6;
    if (diff==0) 
      himd[i] = 0;
    else if (maxv==r) {
      himd[i] = (1536*diff + 256*(g - b))/dif6 & 255;
    } else if (maxv==g)
      himd[i] =  (512*diff + 256*(b - r))/dif6;
    else 
      himd[i] = (1024*diff + 256*(r - g))/dif6;
    if (maxv==0)
      simd[i] = 0;
    else
      simd[i] = 255*(maxv - minv)/maxv;
    vimd[i] = maxv;
  }
}

void FgBgSegment::GetSurfaceParameters( float &alpha, 
					float &beta, 
					float &disp)
{
  alpha = surface.alpha;
  beta  = surface.beta;
  disp  = surface.disp;
  
}


void FgBgSegment::GetSurfaceMinMax( float &min_x,
				    float &min_y,
				    float &max_x,
				    float &max_y )
{
  min_x = surface.min_x;
  min_y = surface.min_y;
  max_x = surface.max_x;
  max_y = surface.max_y;
}


void FgBgSegment::SetWithColors( bool val)
{
  withColors = val;
}

bool FgBgSegment::GetWithColors(){ return withColors; }

void FgBgSegment::SetWithColorHoles( bool val)
{
  withColorHoles = val;
}

bool FgBgSegment::GetWithColorHoles(){ return withColorHoles;}

void FgBgSegment::SetUniform( bool val)
{
  uniform = val;
}

bool FgBgSegment::GetUniform(){ return uniform;}

void FgBgSegment::SetWithSurface( bool val)
{
  withSurface = val;
}

bool FgBgSegment::GetWithSurface(){ return withSurface;}

void FgBgSegment::SetWithDisparities( bool val)
{
  withDisparities = val;
}

bool FgBgSegment::GetWithDisparities(){return withDisparities;}

void FgBgSegment::SetGradWeight( float val) 
{
  gradWeight = val;
}

float FgBgSegment::GetGradWeight() {return gradWeight;}

void FgBgSegment::MakeSegmentImage(Image<uint8_t> &image)
{
  const int numFigures = figures.size();
  assert(image.GetWidth()==width && image.GetHeight()==height);
  float *reg_g = ground.probabilities.GetData();
  float *reg_s = surface.probabilities.GetData();
  float *reg_f[numFigures];
  uint8_t *imgd = image.GetData();
  for (int f=0;f<numFigures;f++)
    reg_f[f] = figures[f]->probabilities.GetData();
  for (int i=0;i<width*height;i++) {
    float maxprob = reg_g[i];
    uint8_t col = 0;
    if (reg_s[i]>maxprob) {
      maxprob = reg_s[i];
      col = 1;
    }
    for (int f=0;f<numFigures;f++) {
      if (reg_f[f][i]>maxprob) {
	maxprob = reg_f[f][i];
	col = f + 2;
      }
    }
    imgd[i] = col; 
  }
}
 
void FgBgSegment::MakeMaskImage(Image<uint8_t> &image, int val, int obj)
{
  const int numFigures = figures.size();
  assert(image.GetWidth()==width && image.GetHeight()==height);
  float *reg_g = ground.probabilities.GetData();
  float *reg_s = surface.probabilities.GetData();
  float *reg_f[numFigures]; 
  obj = std::min(numFigures-1, obj);
  uint8_t *imgd = image.GetData();
  for (int f=0;f<numFigures;f++)
    reg_f[f] = figures[f]->probabilities.GetData();
  for (int i=0;i<width*height;i++) {
    float maxprob = std::max(reg_g[i], reg_s[i]); 
    for (int f=0;f<numFigures;f++)
      maxprob = std::max(maxprob, reg_f[f][i]);
    imgd[i] = (reg_f[obj][i]==maxprob ? val : 0); 
  }
}
 
void FgBgSegment::MakeBorderImage(Image<uint8_t> &image)
{
  Image<uint8_t> segm(width, height);
  uint8_t *segd = segm.GetData();
  MakeMaskImage(segm, 1);
  FillHoles(segm); //%%%%
  KeepLargestSegment(segm, 1, 0, 1000); //%%%%
  uint8_t *imgd = image.GetData();
  for (int i=0;i<3*width*height;i++)  
    imgd[i] = 160*imgd[i]/256;
  for (int y=2;y<height-2;y++) {  
    for (int x=2;x<width-2;x++) {
      int p = y*width + x;
      int sum = segd[p] + segd[p+1] + segd[p-1] + segd[p+1+width] + segd[p-1+width] + 
	segd[p+width] + segd[p-width] + segd[p+1-width] + segd[p-1-width] + 
	segd[p+2*width] + segd[p-2*width] + segd[p+2] + segd[p-2]; 
      if (sum>0 && sum<13)
	imgd[3*p+0] = imgd[3*p+1] = imgd[3*p+2] = 255;
    }
  } 
}
 
/// Convenience class for multi-threading
class ModelWorker {
  /// Color model
  FgBgSegment::ColorModel &model;
  /// Probabilities of pixel belonging to that model
  Image<float> &probs;
  /// Sum of probabilities
  float &sumProb;
public:
  /// Constructor
  /** @param m model
   *  @param p Probabilities of pixel belonging to that model
   *  @param s Sum of probabilities
   */
  ModelWorker(FgBgSegment::ColorModel &m, 
	      Image<float> &p, float &s) : 
    model(m), probs(p), sumProb(s) { }
  /// Operator to Update model and create histograms based on updated model with multi-threading
  void operator()() {
    model.Update(); 
    sumProb = model.CreateHistogram(probs, true);
  }
  /// Operator to Update model and create histograms based on updated model with single-threading
  void runModel() {
    model.Update(); 
    sumProb = model.CreateHistogram(probs, true);
  }
};


void FgBgSegment::Execute(Image<uint8_t> &image, 
			  Image<float> &disp, 
			  bool initialize, int loops, int startx, int starty)
{
  bool cudaopt = gpuopt; 
  typedef void(*func)(FgBgSegment&);
  static const func PixSegment[7] = { &FgBgSegment::PixSegment<0>, 
				      &FgBgSegment::PixSegment<1>, 
				      &FgBgSegment::PixSegment<2>, 
				      &FgBgSegment::PixSegment<3>, 
				      &FgBgSegment::PixSegment<4>, 
				      &FgBgSegment::PixSegment<5>, 
				      &FgBgSegment::PixSegment<6>};
  disparities = &disp;
  RGBToHSV(image);

  for (int i=0;i<loops;i++) {
    
    if (initialize) { 
      colorPriors.clear();
      for (int i=0;i<(int)figures.size();i++) {
	colorPriors.push_back(*figures[i]);
	delete figures[i];
      }
      figures.clear();
      if (withSurface)
	surface.Initialize();
      ground.Initialize();
      InitSegmentation(); 
      CreateHistograms(false);
    }    

    int nFigures = figures.size();
    if (cudaopt) 
#ifdef USE_CUDA
      cudaSegment->Execute(*this, image, disp, nFigures, i==0);
#else 
    std::cout << "Cuda not available!"<< std::endl;
#endif //USE_CUDA
    else 
      PixSegment[nFigures](*this);

    //    ROS_INFO("How many foreground figures? %d", nFigures);
    
    // Boost equivalent to tbb task group
    /* 
      boost::thread_group grp;
      
      std::vector<ModelWorker*> workers;
      float sumProbs[8];
      if (withSurface)
      workers.push_back(new ModelWorker(surface, surface.probabilities, sumProbs[1]));
      for (int i=0;i<nFigures;i++) 
      workers.push_back(new ModelWorker(*figures[i], figures[i]->probabilities, sumProbs[i+2]));
      for (int i=0;i<workers.size();i++){
      grp.create_thread(*workers[i]);
      }
      
      grp.join_all( );
    */
    
#ifdef USE_TBB
    tbb::task_group g;
#endif // USE_TBB
    
    std::vector<ModelWorker*> workers;
    float sumProbs[8];
    if (withSurface)
      workers.push_back(new ModelWorker(surface, 
					surface.probabilities, 
					sumProbs[1]));

    for (int i=0;i<nFigures;i++) 
      workers.push_back(new ModelWorker(*figures[i], 
					figures[i]->probabilities, 
					sumProbs[i+2]));

    for (int i=0;i<(int)workers.size();i++){
#ifdef USE_TBB
      g.run(*workers[i]);
#else
      workers[i]->runModel();
#endif // USE_TBB
    }
    
    ground.Update();
    sumProbs[0] = ground.CreateHistogram(ground.probabilities, true);

#ifdef USE_TBB
    g.wait();
#endif // USE_TBB
    
    for (int i=0;i<(int)workers.size();i++)
      delete workers[i];
    
    if (verbose>1) {
      std::cout << "SumProbs: ";
      for (int i=0;i<(int)figures.size()+2;i++) 
	std::cout << (int)sumProbs[i] << " ";
      std::cout << std::endl;
    }
    initialize = false;
  }  

  if (verbose || true) 
    std::cout << "Surface spread: " << surface.spread_d << std::endl;
  //PixSegment[figures.size()](*this);
}
