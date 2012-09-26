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

#ifndef TPIMAGE_H
#define TPIMAGE_H

#include <fstream>
#include <iostream>
#include <string>
#include <cstring>
#include <cassert>
#include <typeinfo>
#include <stdint.h>


#include <sensor_msgs/Image.h>

typedef unsigned char uchar;

/** @file tpimage.h
    Templated image class */

/// Templated image class
template<class T>
class Image {
  int width, height;
  T *image, *img;
  bool localalloc;
public:
  /// Constructor
  /** @param w image width
      @param h image height
      @param ptr image data pointer (if NULL, allocated internally) */
  Image(int w, int h, T *ptr=NULL);
  //  ~Image() { if (localalloc) delete [] img; }
  ~Image() { if (localalloc) free(img); }
  /// Set new image size
  /** @param w image width
      @param h image height */
  void SetSize(int w, int h);
  /// Set image data position
  /** @param ptr image data pointer */
  void SetData(T *ptr) { image = ptr; }
  /// Set image data position and ensure correct alignment
  /** @param ptr image data pointer 
      @param w image width
      @param h image height*/
  void SetDataAlign(T *ptr, int w, int h);
  /// Set image data position and ensure correct alignment
  /** @param image ROS image sensor message
      @param w image width
      @param h image height*/
  void SetDataAlign(const sensor_msgs::Image &img_msg, int w, int h, bool withColor = true);
  /// Load grey-level image from PGM file
  /** @param filename image file name */
  bool Load(const char *filename);
  /// Load RGB image (three values per pixel) from PPM file  
  /** @param filename image file name */
  bool LoadRGB(const char *filename);
  /// Store grey-level image as PGM file
  /** @param filename image file name 
      @param norm whether to normalize image before storage
      @param ascii whether to store in ASCII or binary format */
  void Store(const char *filename, bool norm = false, bool ascii =false) const;
  /// Store RGB image (three values per pixel) as PPM file
  /** @param filename image file name */
  void StoreRGB(const char *filename) const;
  /// Convert from UYVY (two values per pixel) to RGB and store as PPM file 
  /** @param filename image file name */
  void StoreYUV(const char *filename) const;
  /// Get image data position 
  T *GetData() const { return image; }
  /// Get image width 
  int GetWidth() const { return width; }
  /// Get image height
  int GetHeight() const { return height; }
  /// Get pointer to pixel row \b i 
  T *operator[](int i) { return &image[i*width]; }
  /// Copy image data
  void operator=(Image<T> &src);
};

#endif // TPIMAGE_H
