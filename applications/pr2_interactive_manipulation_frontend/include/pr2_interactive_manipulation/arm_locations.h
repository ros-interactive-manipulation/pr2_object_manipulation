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

namespace pr2_interactive_manipulation {

inline std::vector<double> getSidePosition(std::string arm_name)
{
  std::vector<double> sp(7,0.0);
  if (arm_name=="right_arm") 
  {
    //sp[0] = -2.135; sp[1] = 0.803; sp[2] = -1.732; sp[3] = -1.905; sp[4] = -2.369; sp[5] = -1.680; sp[6] = 1.398;
    //sp[0] = -2.135; sp[1] = -0.18; sp[2] = -1.732; sp[3] = -1.905; sp[4] = -2.369; sp[5] = -1.680; sp[6] = 1.398;
    sp[0] = -2.135; sp[1] = -0.02; sp[2] = -1.64; sp[3] = -2.07; sp[4] = -1.64; sp[5] = -1.680; sp[6] = 1.398;
  } 
  else 
  {
    //sp[0] = 2.135; sp[1] = 0.803; sp[2] = 1.732; sp[3] = -1.905; sp[4] = 2.369; sp[5] = -1.680; sp[6] = 1.398;
    //sp[0] = 2.135; sp[1] = -0.18; sp[2] = 1.732; sp[3] = -1.905; sp[4] = 2.369; sp[5] = -1.680; sp[6] = 1.398;
    sp[0] = 2.135; sp[1] = -0.02; sp[2] = 1.64; sp[3] = -2.07; sp[4] = 1.64; sp[5] = -1.680; sp[6] = 1.398;
  }
  return sp;
}

inline std::vector<double> getFrontPosition(std::string arm_name)
{
  std::vector<double> fp(7, 0.0);
  if (arm_name=="right_arm") 
  {
    fp[0] = 0.0; fp[1] = 1.203; fp[2] = 0.0; fp[3] = -2.105; fp[4] = -3.13; fp[5] = -1.680; fp[6] = 1.398;
  } 
  else 
  {
    fp[0] = 0.0; fp[1] = 1.203; fp[2] = 0.0; fp[3] = -2.105; fp[4] =  3.13; fp[5] = -1.680; fp[6] = 1.398;
  }
  return fp;
}

inline std::vector< std::vector<double> > getSideTrajectory(std::string arm_name)
{
  std::vector< std::vector<double> > traj;
  std::vector<double> p1(7, 0.0);
  std::vector<double> p2(7, 0.0);
  if (arm_name=="right_arm")
  {
    //p1[0]=-0.968; p1[1]=0.729; p1[2]=-0.554; p1[3]=-1.891; p1[4]=-1.786; p1[5]=-1.127; p1[6]=0.501;
    //p2[0]=-2.135; p2[1]=0.803; p2[2]=-1.732; p2[3]=-1.905; p2[4]=-2.369; p2[5]=-1.680; p2[6]=1.398;
    //p1[0]=-0.968; p1[1]=0.129; p1[2]=-0.554; p1[3]=-1.891; p1[4]=-1.786; p1[5]=-1.127; p1[6]=0.501;
    //p2[0]=-2.135; p2[1]=-0.18; p2[2]=-1.732; p2[3]=-1.905; p2[4]=-2.369; p2[5]=-1.680; p2[6]=1.398;
    p1[0]=-0.968; p1[1]=0.129; p1[2]=-0.554; p1[3]=-1.891; p1[4]=-1.786; p1[5]=-1.127; p1[6]=0.501;
    p2[0] = -2.135; p2[1] = -0.02; p2[2] = -1.64; p2[3] = -2.07; p2[4] = -1.64; p2[5] = -1.680; p2[6] = 1.398;
  }
  else
  {
    //p1[0]=0.968; p1[1]=0.729; p1[2]=0.554; p1[3]=-1.891; p1[4]=1.786; p1[5]=-1.127; p1[6]=0.501;
    //p2[0]=2.135; p2[1]=0.803; p2[2]=1.732; p2[3]=-1.905; p2[4]=2.369; p2[5]=-1.680; p2[6]=1.398;
    //p1[0]=0.968; p1[1]=0.129; p1[2]=0.554; p1[3]=-1.891; p1[4]=1.786; p1[5]=-1.127; p1[6]=0.501;
    //p2[0]=2.135; p2[1]=-0.18; p2[2]=1.732; p2[3]=-1.905; p2[4]=2.369; p2[5]=-1.680; p2[6]=1.398;
    p1[0]=0.968; p1[1]=0.129; p1[2]=0.554; p1[3]=-1.891; p1[4]=1.786; p1[5]=-1.127; p1[6]=0.501;
    p2[0] = 2.135; p2[1] = -0.02; p2[2] = 1.64; p2[3] = -2.07; p2[4] = 1.64; p2[5] = -1.680; p2[6] = 1.398;
  }
  traj.push_back(p1);
  traj.push_back(p2);
  return traj;
}

inline std::vector< std::vector<double> > getFrontTrajectory(std::string arm_name)
{
  std::vector< std::vector<double> > traj;
  std::vector<double> p1(7, 0.0);
  std::vector<double> p2(7, 0.0);
  if (arm_name=="right_arm")
  {
    p1[0]=-0.968; p1[1]=0.729; p1[2]=-0.554; p1[3]=-1.891; p1[4]=-1.786; p1[5]=-1.127; p1[6]=0.501;
    p2[0] = 0.0; p2[1] = 1.203; p2[2] = 0.0; p2[3] = -2.105; p2[4] = -3.13; p2[5] = -1.680; p2[6] = 1.398;
  }
  else
  {
    p1[0]=0.968; p1[1]=0.729; p1[2]=0.554; p1[3]=-1.891; p1[4]=1.786; p1[5]=-1.127; p1[6]=0.501;
    p2[0] = 0.0; p2[1] = 1.203; p2[2] = 0.0; p2[3] = -2.105; p2[4] = 3.13; p2[5] = -1.680; p2[6] = 1.398;
  }
  traj.push_back(p1);
  traj.push_back(p2);
  return traj;
}

inline std::vector<double> getHandoffPosition(std::string arm_name)
{
  std::vector<double> sp(7,0.0);
  if (arm_name=="right_arm")
  {
    //sp[0] = -2.135; sp[1] = 0.803; sp[2] = -1.732; sp[3] = -1.905; sp[4] = -2.369; sp[5] = -1.680; sp[6] = 1.398;
    sp[0] = -2.135; sp[1] = -0.18; sp[2] = -1.732; sp[3] = -1.905; sp[4] = -2.369; sp[5] = -1.680; sp[6] = 1.398;
  }
  else
  {
    //sp[0] = 2.135; sp[1] = 0.803; sp[2] = 1.732; sp[3] = -1.905; sp[4] = 2.369; sp[5] = -1.680; sp[6] = 1.398;
    sp[0] = 2.135; sp[1] = -0.18; sp[2] = 1.732; sp[3] = -1.905; sp[4] = 2.369; sp[5] = -1.680; sp[6] = 1.398;
  }
  return sp;
}

inline std::vector< std::vector<double> > getHandoffTrajectory(std::string arm_name)
{
  std::vector< std::vector<double> > traj;
  std::vector<double> p1(7, 0.0);
  std::vector<double> p2(7, 0.0);
  if (arm_name=="right_arm")
  {
    //p1[0]=-0.968; p1[1]=0.729; p1[2]=-0.554; p1[3]=-1.891; p1[4]=-1.786; p1[5]=-1.127; p1[6]=0.501;
    //p2[0]=-2.135; p2[1]=0.803; p2[2]=-1.732; p2[3]=-1.905; p2[4]=-2.369; p2[5]=-1.680; p2[6]=1.398;
    p1[0]=-0.968; p1[1]=0.129; p1[2]=-0.554; p1[3]=-1.891; p1[4]=-1.786; p1[5]=-1.127; p1[6]=0.501;
    p2[0]=-2.135; p2[1]=-0.18; p2[2]=-1.732; p2[3]=-1.905; p2[4]=-2.369; p2[5]=-1.680; p2[6]=1.398;
  }
  else
  {
    //p1[0]=0.968; p1[1]=0.729; p1[2]=0.554; p1[3]=-1.891; p1[4]=1.786; p1[5]=-1.127; p1[6]=0.501;
    //p2[0]=2.135; p2[1]=0.803; p2[2]=1.732; p2[3]=-1.905; p2[4]=2.369; p2[5]=-1.680; p2[6]=1.398;
    p1[0]=0.968; p1[1]=0.129; p1[2]=0.554; p1[3]=-1.891; p1[4]=1.786; p1[5]=-1.127; p1[6]=0.501;
    p2[0]=2.135; p2[1]=-0.18; p2[2]=1.732; p2[3]=-1.905; p2[4]=2.369; p2[5]=-1.680; p2[6]=1.398;
  }
  traj.push_back(p1);
  traj.push_back(p2);
  return traj;
}

inline std::vector<double> getSideHandoffPosition(std::string arm_name)
{
  std::vector<double> sp(7,0.0);
  if (arm_name=="right_arm")
  {
    sp[0] = -2.099; sp[1] = 0.041; sp[2] = -1.488; sp[3] = -0.694; sp[4] = -1.693; sp[5] = -1.596; sp[6] = 0.128;
  }
  else
  {
    sp[0] = 2.099; sp[1] = -0.041; sp[2] = 1.488; sp[3] = -0.694; sp[4] = 1.693; sp[5] = -1.596; sp[6] = 0.128;
  }
  return sp;
}

inline std::vector< std::vector<double> > getSideHandoffTrajectory(std::string arm_name)
{
  std::vector< std::vector<double> > traj;
  std::vector<double> p1(7, 0.0);
  std::vector<double> p2(7, 0.0);
  if (arm_name=="right_arm")
  {
    p1[0]=-0.968; p1[1]=0.129; p1[2]=-0.554; p1[3]=-1.891; p1[4]=-1.786; p1[5]=-1.127; p1[6]=0.501;
    p2[0] = -2.099; p2[1] = 0.041; p2[2] = -1.488; p2[3] = -0.694; p2[4] = -1.693; p2[5] = -1.596; p2[6] = 0.128;
  }
  else
  {
    p1[0]=0.968; p1[1]=0.129; p1[2]=0.554; p1[3]=-1.891; p1[4]=1.786; p1[5]=-1.127; p1[6]=0.501;
    p2[0] = 2.099; p2[1] = -0.041; p2[2] = 1.488; p2[3] = -0.694; p2[4] = 1.693; p2[5] = -1.596; p2[6] = 0.128;
  }
  traj.push_back(p1);
  traj.push_back(p2);
  return traj;
}

}
