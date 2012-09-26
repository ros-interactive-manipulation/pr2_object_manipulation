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

#include <vector>
#include <xmmintrin.h>
#include "largest.h"
/** @file largest.cpp
    Implementations of utility functions to clean segment*/

/// Cluster class
struct Cluster {
  /// area
  int area;
  /// Minimum x of bounding box 
  short minx;
  /// Maximum x of bounding box
  short maxx;
  /// Minimum y of bounding box 
  short miny;
  /// Maximum y of bounding box
  short maxy;
};

/// Find connected components
/** Find connected components in segmentation which is later used to keep the largest component
 * @param equivTable Equivalence table for different clusters
 * @param maxClusters Maximum Number of Clusters
 * @param limg Segmentation mask to be clustered
 * @param comp Result image with components
 * @param label The label of the segmentation mask 
 * @return Number of Connected Components
 */
int FindConnectedComponents(std::vector<short> &equivTable, int maxClusters, 
			    Image<unsigned char> &limg, 
			    Image<short> &comp, int label)
{
  int w = limg.GetWidth();
  int h = limg.GetHeight();
  equivTable.resize(maxClusters);
  unsigned char *imgd = limg.GetData();
  short *comd = comp.GetData();
  int maxCompVal = maxClusters - 1;
  for (int i=0;i<maxClusters;i++)
    equivTable[i] = i;
  short currComp = 1;
  //======== very first for (0,0)
  if (imgd[0]==label) {
    equivTable[currComp] = currComp;
    comd[0] = currComp++;
    if (currComp>maxCompVal)
      currComp = maxCompVal;
  } else
    comd[0] = 0;
  //======== first line y=0
  for (int x=1;x<w;x++) {
    int indx = x - 1;
    bool ison = (imgd[x]==label);
    if (ison && comd[indx]!=0) {
      comd[x] = comd[indx];
    } else if (ison) {
      comd[x] = currComp++;
      if (currComp>maxCompVal)
	currComp = maxCompVal;
    } else
      comd[x] = 0;
  }
  //======== then lines y=1..height-1
  for (int y=1;y<h;y++) {
    int indy = y*w - w;
    bool ison = (imgd[y*w]==label);      // first x=0
    if (ison && comd[indy]!=0) {
      comd[y*w] = comd[indy];
    } else if (ison) {
      comd[y*w] = currComp++;
      if (currComp>maxCompVal)
	currComp = maxCompVal;
    } else
      comd[y*w] = 0;
    short last = comd[y*w];
    for (int x=1;x<8;x++) {  // then x=1...8
      int p = y*w + x;
      short compx = equivTable[last];
      short compy = equivTable[comd[p-w]];
      last = 0;
      if (imgd[p]==label) {  
	if (compx!=0) {
	  last = compx;
	  if (compy!=0 && compx!=compy) {
	    if (compx>compy)
	      equivTable[compx] = compy;
	    else
	      equivTable[compy] = compx;
	  }
	} else if (compy!=0)
	  last = compy;
	else {
	  last = currComp++;
	  if (currComp>maxCompVal)
	    currComp = maxCompVal;
	}
      } 
      comd[p] = last;
    }
    unsigned char *irow = &imgd[y*w];
    for (int x=8;x<w;x+=8) {  // then x=8...w-1
      int p = y*w + x;
#ifdef __SSE__
      __m64 zero = _mm_setzero_si64();
      __m64 *pos = (__m64*)&irow[x];
      if (_mm_movemask_pi8(_mm_cmpeq_pi8(pos[0], zero))==0xff) {
	__m64 *cpos = (__m64*)&comd[p];
	cpos[0] = zero;
	cpos[1] = zero;
	continue;
      }
#endif
      short last = comd[p-1];
      for (int i=0;i<8;i++,p++) {
	short compx = equivTable[last];
	short compy = equivTable[comd[p-w]];
	last = 0;
        if (irow[x+i]==label) {  
	  if (compx!=0) {
	    last = compx;
	    if (compy!=0) {
	      if (compx!=compy) {
		if (compx>compy)
		  equivTable[compx] = compy;
		else
		  equivTable[compy] = compx;
	      }
	    }
	  } else if (compy!=0)
	    last = compy;
	  else {
	    last = currComp++;
	    if (currComp>maxCompVal)
	      currComp = maxCompVal;
	  }
	} 
	comd[p] = last;
      }
    }
  }
  for (short i=0;i<currComp;i++) {
    int eq = equivTable[i];
    while (eq!=equivTable[eq])
      eq = equivTable[eq];
    equivTable[i] = eq;
  }
  int numClusters = 0;
  for (short i=0;i<currComp;i++) {
    int eq = equivTable[i];
    if (eq==i)
      equivTable[i] = numClusters++;
    else
      equivTable[i] = equivTable[eq];
  }
#if 0
  for (short i=0;i<currComp;i++)
    printf("equiv[%i] = %d\r\n", i, equivTable[i]);
#endif
#ifdef __SSE__
  _mm_empty();
#endif
  return numClusters;
}  

/// Relabel all clusters and fill their attributes
/** Fill the area and bounding box attributes of the clusters
 *  @param comp Connected component image
 *  @param equivTable Equivalence table
 *  @param numClusters Number of connected components
 *  @param clusters Vector with cluster objects
 */
void Relabel(Image<short> &comp, std::vector<short> &equivTable, int numClusters, std::vector<Cluster> &clusters)
{
  int width = comp.GetWidth();
  int height = comp.GetHeight();
  short *comd = comp.GetData();
  int w = width;
  int h = height;
  int num = numClusters;
  clusters.resize(numClusters);
  for (int i=0;i<num;i++) {
    clusters[i].area = 0;
    clusters[i].minx = 0x7fff;
    clusters[i].maxx = 0;
    clusters[i].miny = 0x7fff;
    clusters[i].maxy = 0;
  }
#ifdef __SSE__
  __m64 zero = _mm_setzero_si64();
  for (int y=0;y<h;y++) {       
    for (int x=0;x<w;x+=4) {
      int k = y*w + x;  
      __m64 *comp4 = (__m64*)&comd[k];
      const __m64 c4 = *comp4;
      if ((_mm_movemask_pi8(_m_pcmpeqw(zero, c4))&0xaa)!=0xaa) {
        const __m64 c1 = _mm_set1_pi16(comd[k]); 
        if ((_mm_movemask_pi8(_m_pcmpeqw(c1, c4))&0xaa)==0xaa) {
          short val = equivTable[comd[k]];
          int j = val - 1;
          *comp4 = _mm_set1_pi16(val);
	  Cluster &cl = clusters[j];
	  cl.area += 4;
	  cl.maxy = y;
	  if (y<cl.miny) cl.miny = y;
	  if (x<cl.minx) cl.minx = x;
	  if (x>cl.maxx-3) cl.maxx = x + 3;
        } else {
          for (int i=0;i<4;i++) {
            if (comd[i+k]!=0) {
              short val = equivTable[comd[i+k]];
              int j = val - 1;
              comd[i+k] = val;
	      Cluster &cl = clusters[j];
	      cl.area++;
	      cl.maxy = y;
	      if (y<cl.miny) cl.miny = y;
	      if (x<cl.minx) cl.minx = x;
	      if (x>cl.maxx) cl.maxx = x;
            }
          }
        }
      }
    }
  }
  _mm_empty();
#else
  for (int y=0;y<h;y++) {
    for (int x=0;x<w;x++) {
      int i = y*w + x;
      if (comd[i]!=0) {
        short val = equivTable[comd[i]];
        comd[i] = val;
	Cluster &cl = clusters[val-1];
	cl.area++;
	cl.maxy = y;
	if (y<cl.miny) cl.miny = y;
	if (x<cl.minx) cl.minx = x;
	if (x>cl.maxx) cl.maxx = x;
      }
    }
  }
#endif 
}


void KeepLargestSegment(Image<unsigned char> &segment, 
			int fromLabel, int toLabel, int minArea)
{
  int width = segment.GetWidth();
  int height = segment.GetHeight();
  std::vector<short> equivTable;
  Image<short> components(width, height);
  int numClusters = FindConnectedComponents(equivTable, 4096, 
					    segment, components, fromLabel);
  std::vector<Cluster> clusters;
  Relabel(components, equivTable, numClusters, clusters);
  if (!clusters.size())
    return;
  unsigned char *segd = segment.GetData();
  short *comd = components.GetData();
  if (minArea==0) {
    int maxSize = 0;
    int maxCluster = 0;
    for (unsigned int i=0;i<clusters.size();i++) {
      if (clusters[i].area>maxSize) {
	maxSize = clusters[i].area;
	maxCluster = i;
      }
    }
    assert(segment.GetHeight() == height);
    assert(components.GetHeight() == height);
    maxCluster ++;
    if (maxSize<100)
      maxCluster = -1;
    for (int i=0;i<width*height;i++) 
      if (segd[i]==fromLabel && comd[i]!=maxCluster) 
	segd[i] = toLabel;
  } else {
    for (int i=0;i<width*height;i++) 
      if (segd[i]==fromLabel && clusters[comd[i]-1].area<minArea) 
	segd[i] = toLabel;
  }
}


