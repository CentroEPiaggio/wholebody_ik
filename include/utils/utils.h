/* Copyright [2016] [Alessandro Settimi (ale.settimi@gmail.com), Mirko Ferrati, Danilo Caporale, Edoardo Farnioli]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#ifndef __UTILS_D_H
#define __UTILS_D_H

#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

// using namespace std;
// using  ns = chrono::nanoseconds;
// using get_time = chrono::steady_clock ;

class utils
{
  
private:
  std::chrono::steady_clock::time_point start, end;
  std::chrono::duration<double> elapsed_time;
  
  ros::Publisher vis_pub;
  visualization_msgs::Marker marker;

public:
  
  utils();
  
  ~utils();
  
  void inline tic();
  
  void inline toc();
  
  void publish_trajectory(std::map<double,KDL::Frame> points,std::string chain,double dt_viz,int steps=1000);
  
  void publish_marker();
};

#endif