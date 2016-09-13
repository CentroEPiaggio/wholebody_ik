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

#include <utils/utils.h>

  
  utils::utils() {
  ros::NodeHandle node_handle;  
  vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  }
  
  utils::~utils() {
  } 
  
  void inline utils::tic() {
    start = std::chrono::steady_clock::now();
  }
  
  void inline utils::toc() {
    end = std::chrono::steady_clock::now();
    elapsed_time = end-start;
//     std::cout << "\033[1;31m Elapsed time " << elapsed_time.count() << " \033[0m" << std::endl; // For colored output
    std::cout << "Elapsed time: " <<elapsed_time.count() << std::endl;
  }
  
  void utils::publish_trajectory(std::map<double,KDL::Frame> points,std::string chain,double dt_viz,int steps) {
    marker.header.frame_id = "l_sole";
    marker.header.stamp =  ros::Time();
    marker.ns = chain;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = .01;
    marker.scale.y = .01;
    marker.scale.z = .01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    
    
    marker.points.resize(steps);
    double time = 0;
    std::cout << chain<<std::endl;
    
    for(int i=0; i<steps;i++) {
        marker.points[i].x = points.at(time).p.x();
        marker.points[i].y = points.at(time).p.y();
        marker.points[i].z = points.at(time).p.z();
	  time +=dt_viz;
    
    }
    vis_pub.publish( marker );
//     ros::spinOnce();
    usleep(500000);
// getchar();
  }
  
  void utils::publish_marker() {
    
  }

