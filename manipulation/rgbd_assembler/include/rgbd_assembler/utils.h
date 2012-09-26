/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sensor_msgs/image_encodings.h>
namespace enc = sensor_msgs::image_encodings;

namespace rgbd_assembler {

  void transformRGB(float val, float &r, float &g, float &b){
    union{int intp; float floatp; } a;
    a.floatp = val;
    int rgb = *reinterpret_cast<int*>(&a.intp);
  
    r = ((rgb >> 16) & 0xff) / 255.0f;
    g = ((rgb >> 8) & 0xff) / 255.0f;
    b = (rgb & 0xff) / 255.0f; 
  }

  float getRGB( float r, float g, float b){
    union{ int intp; float floatp; } a;
    int res = (int(r*255) << 16) | (int(g*255) << 8) | int(b*255);
    a.intp=res;
    float rgb = *(&a.floatp);
    return rgb;
  }

  
  enum Format { INVALID = -1, GRAY = 0, RGB, BGR, RGBA, BGRA };
  
  Format getFormat(const std::string& encoding)
  {
    if (encoding == enc::BGR8)   return BGR;
    if (encoding == enc::MONO8)  return GRAY;
    if (encoding == enc::RGB8)   return RGB;
    if (encoding == enc::MONO16) return GRAY;
    if (encoding == enc::BGRA8)  return BGRA;
    if (encoding == enc::RGBA8)  return RGBA;
    
    // We don't support conversions to/from other types
    return INVALID;
  }
 
  void RGBToHSV(float r, float g, float b,
	      float &h, float &s, float &v) 
{  
  
    float maxC = b;
    if (maxC < g) maxC = g;
    if (maxC < r) maxC = r;
    float minC = b;
    if (minC > g) minC = g;
    if (minC > r) minC = r;
    
    float delta = maxC - minC;
    
    v = maxC;
    s = 0;
    h = 0;
    
    if (delta == 0) {
      return;
    } else {
      s = delta / maxC;
      float dR = 60*(maxC - r)/delta + 180;
      float dG = 60*(maxC - g)/delta + 180;
      float dB = 60*(maxC - b)/delta + 180;
      if (r == maxC)
	h = dB - dG;
      else if (g == maxC)
	h = 120 + dR - dB;
      else
	h = 240 + dG - dR;
    }
    
    if (h<0)
      h+=360;
    if (h>=360)
      h-=360;
  }

void HSVToRGB(float h, float s, float v,
	      float &r, float &g, float &b) 
  {  
    if(s==0){
      r = v;
      g = v;
      b = v;
      return;
    }
    
    
    float h_tmp = h/60.0f;
    int i = floor(h_tmp);
    float f = h_tmp - i;
    //if(i%2) f = 1 - f;
    float p = v * (1-s);
    float q = v * (1-s*f);
    float t = v * (1-s * (1-f));
    

    switch(i){
    case 0:
      //   0: v, n, m); 
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      //   1: n, v, m); 
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      //    2: (m, v, n) 
      r = p;
      g = v;
      b = t;
      break;
    case 3:
      //       3: (m, n, v); 
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      //       4: (n, m, v)
      r = t;
      g = p;
      b = v;
      break;
    case 5:
      //      5: (v, m, n); 
      r = v;
      g = p;
      b = q;
      break;
    }
  }

}
