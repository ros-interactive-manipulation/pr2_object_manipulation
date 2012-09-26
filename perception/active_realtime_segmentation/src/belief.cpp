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

#include <cmath>
#include <algorithm>
#include <float.h>
#include <assert.h>
#include "pyra/tpimage.h"
#include "belief.h"
#include "timercpu.h"

template<int dim> BeliefProp<dim>::BeliefProp(int w, int h) : width(w), height(h)
{
  int sz = w*h;
  for (int i=0;i<dim;i++) {
    msgl[i] = new float[sz];
    msgr[i] = new float[sz];
    msgu[i] = new float[sz];
    msgd[i] = new float[sz];
    prior[i] = new float[sz];
    belief[i] = new float[sz];
  }
  costh = new float[sz];
  costv = new float[sz];
}

template<int dim> BeliefProp<dim>::~BeliefProp()
{
  for (int i=0;i<dim;i++) {
    delete [] msgl[i];
    delete [] msgr[i];
    delete [] msgu[i];
    delete [] msgd[i];
    delete [] prior[i];
    delete [] belief[i];
  }
  delete [] costh;
  delete [] costv;
}

template<int dim> float **BeliefProp<dim>::GetPriors()
{
  return prior;
}

template<int dim> float **BeliefProp<dim>::GetBeliefs()
{
  return belief;
}

template<int dim> void BeliefProp<dim>::SetGradientCosts(Image<unsigned char> &img, float gamma)
{
  assert(img.GetWidth()==width && img.GetHeight()==height);
  unsigned char *imd = img.GetData();
  int sum = 0;
  for (int y=0;y<height-1;y++) {
    for (int x=0;x<width-1;x++) {
      int p = y*width + x;
      int diff = imd[p] - imd[p+width];
      sum += diff*diff;
      diff = imd[p] - imd[p+1];
      sum += diff*diff;
    }
  }  
  float sumdiff2 = (float)sum / (2*(width-1)*(height-1));
  float beta = -1.0f/(2.0f*sumdiff2); 
  for (int y=0;y<height;y++) {
    for (int x=0;x<width-1;x++) {
      int p = y*width + x;
      float diff = imd[p] - imd[p+1];
      costh[p] = gamma*exp(beta*diff*diff);
    }
    costh[y*width+width-1] = 0;
  }  
  for (int y=0;y<height-1;y++) {
    for (int x=0;x<width;x++) {
      int p = y*width + x;
      float diff = imd[p] - imd[p+width];
      costv[p] = gamma*exp(beta*diff*diff);
    }
  }  
  for (int x=0;x<width;x++)
    costv[(height-1)*width+x] = 0;
}

template<int dim> void BeliefProp<dim>::Execute(int loops)
{
  InitMessages();
  for (int i=0;i<loops;i++)
    UpdateMessages();
  ComputeBeliefs();
}

template<int dim> void BeliefProp<dim>::Execute(int loops, int depth)
{
  if (depth>0) {
    int w2 = width/2;
    int h2 = height/2;
    BeliefProp<dim> belief2(w2, h2);
    float **prior2 = belief2.GetPriors();
    for (int l=0;l<dim;l++) {
      for (int y2=0;y2<h2;y2++) {
	float *ptr = &prior[l][2*y2*width];
	float *ptr2 = &prior2[l][y2*w2];
	for (int x2=0;x2<w2;x2++,ptr+=2) 
	  ptr2[x2] = ptr[0] + ptr[1] + ptr[width] + ptr[width+1]; 
      }
    }
    for (int y2=0;y2<h2;y2++) {
      float *ptr = &costh[2*y2*width];
      float *ptr2 = &belief2.costh[y2*w2];
      for (int x2=0;x2<w2;x2++,ptr+=2) 
	ptr2[x2] = ptr[1] + ptr[width+1]; 
      ptr = &costv[2*y2*width];
      ptr2 = &belief2.costv[y2*w2];
      for (int x2=0;x2<w2;x2++,ptr+=2) 
	ptr2[x2] = ptr[width] + ptr[width+1]; 
    }
    belief2.Execute(loops, depth-1);
    for (int l=0;l<dim;l++) {
      for (int y2=0;y2<h2;y2++) {
	float *ptr = &msgl[l][2*y2*width];
	float *ptr2 = &belief2.msgl[l][y2*w2];
	for (int x2=0;x2<w2;x2++,ptr+=2) 
	  ptr[0] = ptr[1] = ptr[width] = ptr[width+1] = ptr2[x2]; 
	ptr = &msgr[l][2*y2*width];
	ptr2 = &belief2.msgr[l][y2*w2];
	for (int x2=0;x2<w2;x2++,ptr+=2) 
	  ptr[0] = ptr[1] = ptr[width] = ptr[width+1] = ptr2[x2]; 
	ptr = &msgu[l][2*y2*width];
	ptr2 = &belief2.msgu[l][y2*w2];
	for (int x2=0;x2<w2;x2++,ptr+=2) 
	  ptr[0] = ptr[1] = ptr[width] = ptr[width+1] = ptr2[x2]; 
	ptr = &msgd[l][2*y2*width];
	ptr2 = &belief2.msgd[l][y2*w2];
	for (int x2=0;x2<w2;x2++,ptr+=2) 
	  ptr[0] = ptr[1] = ptr[width] = ptr[width+1] = ptr2[x2]; 
      }
    }
  } else 
    InitMessages();
  TimerCPU timer(2800);
  for (int i=0;i<loops;i++) //(depth?loops:height);i++) 
    UpdateMessages();
  ComputeBeliefs();
  std::cout << "Belief       Time: " << timer.read() << std::endl;
}

template<int dim> void BeliefProp<dim>::InitMessages()
{
  int size = sizeof(float)*width*height;
  for (int i=0;i<dim;i++) {
    memset(msgl[i], 0, size);
    memset(msgr[i], 0, size);
    memset(msgu[i], 0, size);
    memset(msgd[i], 0, size);
  }
}

template<int dim> void BeliefProp<dim>::ComputeBeliefs()
{
  for (int i=0;i<dim;i++)
    for (int p=0;p<width*height;p++) 
      belief[i][p] = prior[i][p] + msgl[i][p] + msgr[i][p] + msgu[i][p] + msgd[i][p];
}

template<int dim> void BeliefProp<dim>::UpdateMessages()
{
  float h[dim];
  for (int phase=0;phase<2;phase++) {
    for (int y=0;y<height;y++) {
      int start = (y+phase) & 1;
      for (int x=start;x<width;x+=2) {
	int q = y*width + x;
	if (x>0) { // message from left
	  int p = q - 1; 
	  float minh = FLT_MAX;
	  for (int i=0;i<dim;i++) {
	    h[i] = prior[i][p] + msgl[i][p] + msgu[i][p] + msgd[i][p];
	    minh = (h[i]<minh ? h[i] : minh);
	  }
	  float minc = minh + costh[p]; 
	  for (int i=0;i<dim;i++)
	    msgl[i][q] = (h[i]<minc ? h[i] : minc) - minh;
	} 
	if (x<width-1) {
	  int p = q + 1; // message from right
	  float minh = FLT_MAX;
	  for (int i=0;i<dim;i++) {
	    h[i] = prior[i][p] + msgr[i][p] + msgu[i][p] + msgd[i][p];
	    minh = (h[i]<minh ? h[i] : minh);
	  }
	  float minc = minh + costh[q];
	  for (int i=0;i<dim;i++)
	    msgr[i][q] = (h[i]<minc ? h[i] : minc) - minh;
	}  
	if (y>0) { // message from up
	  int p = q - width; 
	  float minh = FLT_MAX;
	  for (int i=0;i<dim;i++) {
	    h[i] = prior[i][p] + msgl[i][p] + msgr[i][p] + msgu[i][p];
	    minh = (h[i]<minh ? h[i] : minh);
	  }
	  float minc = minh + costv[p];
	  for (int i=0;i<dim;i++)
	    msgu[i][q] = (h[i]<minc ? h[i] : minc) - minh;
	} 
	if (y<height-1) {
	  int p = q + width; // message from down
	  float minh = FLT_MAX;
	  for (int i=0;i<dim;i++) {
	    h[i] = prior[i][p] + msgl[i][p] + msgr[i][p] + msgd[i][p];
	    minh = (h[i]<minh ? h[i] : minh);
	  }
	  float minc = minh + costv[q];
	  for (int i=0;i<dim;i++)
	    msgd[i][q] = (h[i]<minc ? h[i] : minc) - minh;
	}
      }
    }
  }
} 

#if 0

#include "GCoptimization.h"

template<int dim> void BeliefProp<dim>::ComputeMAP(Image<unsigned char> &mask)
{
  MSR clk0, clk1;
  clk0.getTSC();

  int scale = 100;  
  // stores result of optimization
  int *result = new int[width*height];   
  // first set up the array for data costs
  int *data = new int[width*height*dim];
  for (int i=0;i<width*height;i++) 
    for (int l=0;l<dim;l++) 
      data[i*dim + l] = (int)(scale*prior[l][i]);

  // next set up the array for smooth costs
  int smooth[dim*dim];
  for (int l1=0;l1<dim;l1++)
    for (int l2=0;l2<dim;l2++)
      smooth[l2*dim + l1] = (l1 == l2 ? 0 : scale);
  
  // next set up spatially varying arrays V and H
  int *V = new int[width*height];
  int *H = new int[width*height];
  for (int y=0;y<height;y++) {
    for (int x=0;x<width;x++) {
      int p = y*width + x;
      H[p] = (int)costv[p];
      V[p] = (int)costh[p];
    }
  }
  
  try {
    GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(width, height, dim);
    gc->setDataCost(data);
    gc->setSmoothCostVH(smooth, V, H);
    std::cout << "Before optimization energy is " << (int)gc->compute_energy() << std::endl;
    gc->expansion(10);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
    std::cout << "After  optimization energy is " <<  (int)gc->compute_energy() << std::endl;
    
    for (int i=0;i<width*height;i++)
      result[i] = gc->whatLabel(i);
    delete gc;
  }
  catch (GCException e) {
    e.Report();
  }
  clk1.getTSC();
  std::cout << "ComputeMAP   Time: " << 256.0*clk1.diff(clk0, 8)/2670.0e3 << std::endl;
  
  static int cnt = 0;
  Image<int> maski(width, height, result);
  char filename[80];
  sprintf(filename, "mask%d.pgm", cnt++);
  maski.Store(filename, true, false);

  delete [] result;
  delete [] V;
  delete [] H;
  delete [] data;
}

#endif
