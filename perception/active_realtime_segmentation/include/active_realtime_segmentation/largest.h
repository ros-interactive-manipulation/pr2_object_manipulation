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

#ifndef LARGEST_H
#define LARGEST_H
/** @file largest.h
    Utility functions to clean segment*/

#include "pyra/tpimage.h"

/// Keep the largest segment 
/** Keep the largest component of the segmentation 
 *  @param segment Image with segment
 *  @param fromLabel The label of the segment to clean
 *  @param toLabel The label to which all non large labels are set
 *  @param minArea Minimum area of components
 */
void KeepLargestSegment(Image<unsigned char> &segment, 
			int fromLabel, int toLabel, int minArea = 0);

/// Templated Function to fill holes
/** Templated Function to fill holes in the segmentation mask
 * @param segment Imge with segmentation mask
 */
template<class T> void FillHoles(Image<T> &segment)
{
  int width = segment.GetWidth();
  int height = segment.GetHeight();
  T *masd = segment.GetData();
  int w = width;
  int h = height;
  unsigned char *lastrow = new unsigned char[w];
  unsigned char *changes = new unsigned char[w];
  unsigned char *tmd = new unsigned char[w*h];
  for (int x=0;x<w;x++) {
    unsigned char bit = (masd[x]>0 ? 1 : 0);
    lastrow[x] = bit;
    changes[x] = bit;
  }
  for (int y=1;y<h;y++) {
    int l = 1;
    unsigned char *ptr = &masd[y*w];
    for (int x=0;x<w;x++) {
      unsigned char bit = (ptr[x]>0 ? 1 : 0);
      changes[x] += (bit^lastrow[x]);
      lastrow[x] = bit;
    }
    for (int x=0;x<w;x++) {
      unsigned char c = changes[x];
      c -= (c>l ? 2 : 0);
      l = c + 1; 
      changes[x] = c;  
    }
    l = 1;
    for (int x=w-1;x>=0;x--) {
      unsigned char c = changes[x];
      c -= (c>l ? 2 : 0);
      changes[x] = c;
      l = c + 1;
    }
    for (int x=0;x<w;x++) {
      unsigned char c = changes[x];
      tmd[y*w+x] = (c>0 ? 1 : 0);
    }
  }
  for (int x=0;x<w;x++) {
    unsigned char bit = (masd[(h-1)*w+x]>0 ? 1 : 0);
    lastrow[x] = bit;
    changes[x] = bit;
  }
  for (int y=h-2;y>=0;y--) {
    int l = 1;
    unsigned char *ptr = &masd[y*w];
    for (int x=0;x<w;x++) {
      unsigned char bit = (ptr[x]>0 ? 1 : 0);
      changes[x] += (bit^lastrow[x]);
      lastrow[x] = bit;
    }
    for (int x=0;x<w;x++) {
      unsigned char c = changes[x];
      c -= (c>l ? 2 : 0);
      l = c + 1; 
      changes[x] = c;  
    }
    l = 1;
    for (int x=w-1;x>=0;x--) {
      unsigned char c = changes[x];
      c -= (c>l ? 2 : 0);
      changes[x] = c;
      l = c + 1;
    }
    for (int x=0;x<w;x++) {
      unsigned char c = changes[x];
      ptr[x] = tmd[y*w+x] & (c>0 ? 1 : 0);
    }
  }
  delete [] tmd;
  delete [] changes;
  delete [] lastrow;
}


#endif
