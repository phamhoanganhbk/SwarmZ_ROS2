// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>


#define SPEED_OF_LIGHT 299792458 // m/s
#define PI 3.1415
    
#define AMPLITUDE_SOURCE_R1_1 1
#define AMPLITUDE_SOURCE_R1_3 1
#define FREQUENCY_SOURCE_R1 100 // hz
#define PHASE_SOURCE_R1_1 0
#define PHASE_SOURCE_R1_3 0

#define AMPLITUDE_SOURCE_R2_1 1
#define AMPLITUDE_SOURCE_R2_3 1
#define FREQUENCY_SOURCE_R2 100 // hz
#define PHASE_SOURCE_R2_1 0
#define PHASE_SOURCE_R2_3 0

#define AMPLITUDE_SOURCE_R3_1 1
#define AMPLITUDE_SOURCE_R3_2 1
#define FREQUENCY_SOURCE_R3 100 // hz
#define PHASE_SOURCE_R3_1 0
#define PHASE_SOURCE_R3_3 0

#define AMPLITUDE_SOURCE_R4_1 1
#define AMPLITUDE_SOURCE_R4_3 1
#define FREQUENCY_SOURCE_R4_3 100 // hz
#define PHASE_SOURCE_R4_1 0
#define PHASE_SOURCE_R4_3 0

#define AMPLITUDE_SOURCE_R5_1 1
#define AMPLITUDE_SOURCE_R5_3 1
#define FREQUENCY_SOURCE_R5 100 // hz
#define PHASE_SOURCE_R5_1 0
#define PHASE_SOURCE_R5_3 0

#define AMPLITUDE_SOURCE_NOISE_WHITE_1 1
#define AMPLITUDE_SOURCE_NOISE_WHITE_3 1
#define FREQUENCY_SOURCE_NOISE_WHITE 100 // hz
#define PHASE_SOURCE_NOISE_WHITE_1 0
#define PHASE_SOURCE_NOISE_WHITE_3 0

#define AMPLITUDE_SOURCE_EXPLOSION_1 100
#define AMPLITUDE_SOURCE_EXPLOSION_3 0
#define FREQUENCY_SOURCE_EXPLOSION 20000 // 20 Khz
#define PHASE_SOURCE_EXPLOSION_1 0
#define PHASE_SOURCE_EXPLOSION_3 0

#define POS_SOURCE_R1_X  0
#define POS_SOURCE_R1_Y  0
#define POS_SOURCE_R1_Z  0 

#define POS_SOURCE_R2_X  0
#define POS_SOURCE_R2_Y  0
#define POS_SOURCE_R2_Z  0

#define POS_SOURCE_R3_X  0
#define POS_SOURCE_R3_Y  0
#define POS_SOURCE_R3_Z  0

#define POS_SOURCE_R4_X  0
#define POS_SOURCE_R4_Y  0
#define POS_SOURCE_R4_Z  0

#define POS_SOURCE_R5_X  0
#define POS_SOURCE_R5_Y  0
#define POS_SOURCE_R5_Z  0

#define POS_SOURCE_NOISE_WHITE_X  0
#define POS_SOURCE_NOISE_WHITE_Y  0
#define POS_SOURCE_NOISE_WHITE_Z  0

#define POS_SOURCE_EXPLOSITION_X 0
#define POS_SOURCE_EXPLOSITION_Y 0
#define POS_SOURCE_EXPLOSITION_Z 0  

#define POS_SOURCE_EXPLOSITION_LAT 43.13471   //deg
#define POS_SOURCE_EXPLOSITION_LONG 6.01507   //deg
#define POS_SOURCE_EXPLOSITION_ALT 6.0          //Height above mean sea (m)  


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_time(0)
  {
    
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
    /*
		sub_pose_local_r1 = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/px4_1/fmu/out/vehicle_local_position", qos,
		[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
			pose_local_r1_x = msg->x;
      pose_local_r1_y = msg->y;
      pose_local_r1_z = msg->z;
      
      std::cout << "\n\n\n\n";
			std::cout << "Distance between Drone 1 and Source S1"   << std::endl;
			std::cout << "x: "      << pose_local_r1_x    << std::endl;
      std::cout << "y: "      << pose_local_r1_y    << std::endl;
      std::cout << "z: "      << pose_local_r1_z    << std::endl;
      //dist_Ri_to_S = sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
      dist_Ri_to_S = haversine(43.1347108591022, 6.021229981808671, 43.13471, 6.01507);
      std::cout << "dist1: "      << dist_Ri_to_S   << std::endl;
		});
    */

    sub_pos_gps_r1 = this->create_subscription<px4_msgs::msg::SensorGps>("/px4_1/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg) {
			std::cout << "\n";
      pos_gps_r1_lat = msg->latitude_deg;
      pos_gps_r1_lon = msg->longitude_deg;
      pos_gps_r1_alt = msg->altitude_msl_m;
      
      //dist_Ri_to_S = calculateDistance(pos_gps_r1_lat, pos_gps_r1_lon, pos_gps_r1_alt, POS_SOURCE_EXPLOSITION_LAT, POS_SOURCE_EXPLOSITION_LONG, POS_SOURCE_EXPLOSITION_ALT);
      //std::cout << "distance_R1_to_Source (m): "      << dist_Ri_to_S << std::endl;


		});


    sub_pos_gps_r2 = this->create_subscription<px4_msgs::msg::SensorGps>("/px4_2/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg) {
			std::cout << "\n";
      pos_gps_r2_lat = msg->latitude_deg;
      pos_gps_r2_lon = msg->longitude_deg;
      pos_gps_r2_alt = msg->altitude_msl_m;
      
      //dist_Ri_to_S = calculateDistance(pos_gps_r2_lat, pos_gps_r2_lon, pos_gps_r2_alt, POS_SOURCE_EXPLOSITION_LAT, POS_SOURCE_EXPLOSITION_LONG, POS_SOURCE_EXPLOSITION_ALT);
      //std::cout << "distance_R2_to_Source (m): "      << dist_Ri_to_S << std::endl;
		});

    sub_pos_gps_r3 = this->create_subscription<px4_msgs::msg::SensorGps>("/px4_3/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg) {
			std::cout << "\n";
      pos_gps_r3_lat = msg->latitude_deg;
      pos_gps_r3_lon = msg->longitude_deg;
      pos_gps_r3_alt = msg->altitude_msl_m;
      
      dist_Ri_to_S = calculateDistance(pos_gps_r3_lat, pos_gps_r3_lon, pos_gps_r3_alt, POS_SOURCE_EXPLOSITION_LAT, POS_SOURCE_EXPLOSITION_LONG, POS_SOURCE_EXPLOSITION_ALT);
      std::cout << "distance_R3_to_Source (m): "      << dist_Ri_to_S << std::endl;
		});

    sub_pos_gps_r4 = this->create_subscription<px4_msgs::msg::SensorGps>("/px4_4/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg4) {
			std::cout << "\n";
      pos_gps_r4_lat = msg4->latitude_deg;
      pos_gps_r4_lon = msg4->longitude_deg;
      pos_gps_r4_alt = msg4->altitude_msl_m;
      
      dist_Ri_to_S = calculateDistance(pos_gps_r4_lat, pos_gps_r4_lon, pos_gps_r4_alt, POS_SOURCE_EXPLOSITION_LAT, POS_SOURCE_EXPLOSITION_LONG, POS_SOURCE_EXPLOSITION_ALT);
      std::cout << "distance_R4_to_Source (m): "      << dist_Ri_to_S << std::endl;
		});

    sub_pos_gps_r5 = this->create_subscription<px4_msgs::msg::SensorGps>("/px4_5/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg) {
			std::cout << "\n";
      pos_gps_r5_lat = msg->latitude_deg;
      pos_gps_r5_lon = msg->longitude_deg;
      pos_gps_r5_alt = msg->altitude_msl_m;
      
      dist_Ri_to_S = calculateDistance(pos_gps_r5_lat, pos_gps_r5_lon, pos_gps_r5_alt, POS_SOURCE_EXPLOSITION_LAT, POS_SOURCE_EXPLOSITION_LONG, POS_SOURCE_EXPLOSITION_ALT);
      std::cout << "distance_R5_to_Source (m): "      << dist_Ri_to_S << std::endl;
		});


    //publisher_            = this->create_publisher<std_msgs::msg::String>("topic", 10);
    source_01_publisher_  = this->create_publisher<std_msgs::msg::Float32>("/source_01",1);
    source_02_publisher_  = this->create_publisher<std_msgs::msg::Float32>("/source_02",1);
    source_03_publisher_  = this->create_publisher<std_msgs::msg::Float32>("/source_03",1);
    source_04_publisher_  = this->create_publisher<std_msgs::msg::Float32>("/source_04",1);
    source_05_publisher_  = this->create_publisher<std_msgs::msg::Float32>("/source_05",1);

    // Make and Publisher sources 
    //source_01_to_r01_pub_        = this->create_publisher<geometry_msgs::msg::Vector3>("/source_01/robot_01/pub",10);
    //source_02_to_r01_pub_        = this->create_publisher<geometry_msgs::msg::Vector3>("/source_02/robot_01/pub",10);



    timer_ = this->create_wall_timer(1ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:  

  void timer_callback()
  {
    //pose_source_explosition ->x = POS_SOURCE_EXPLOSITION_X;
    //pose_source_explosition ->y = POS_SOURCE_EXPLOSITION_Y;
    //pose_source_explosition ->z = POS_SOURCE_EXPLOSITION_Z;
    


    auto message = std_msgs::msg::String();
    message.data = "Creating Multiple sources " + std::to_string(count_time++);
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //publisher_          ->publish(message);
    
    
    data01_ = std::make_unique<geometry_msgs::msg::Vector3>();

    count_time_explosition ++;
    if (count_time_explosition >=10000){
      count_time_explosition = 0;
    }


    dist_s_explosition_to_r1 = calculateDistance(pos_gps_r1_lat, pos_gps_r1_lon, pos_gps_r1_alt, POS_SOURCE_EXPLOSITION_LAT, POS_SOURCE_EXPLOSITION_LONG, POS_SOURCE_EXPLOSITION_ALT);
    s_explosition_to_r1 = calcul_source_explosition(AMPLITUDE_SOURCE_EXPLOSION_1, AMPLITUDE_SOURCE_EXPLOSION_3, 
                                                    PHASE_SOURCE_EXPLOSION_1, PHASE_SOURCE_EXPLOSION_3, 
                                                    FREQUENCY_SOURCE_EXPLOSION,
                                                    dist_s_explosition_to_r1); 
    std::cout << "\n";
    std::cout << "distance_R1_to_Source (m): "      << s_explosition_to_r1 << std::endl;
                              
    dist_s_explosition_to_r2 = calculateDistance(pos_gps_r2_lat, pos_gps_r2_lon, pos_gps_r2_alt, POS_SOURCE_EXPLOSITION_LAT, POS_SOURCE_EXPLOSITION_LONG, POS_SOURCE_EXPLOSITION_ALT);
    s_explosition_to_r2 = calcul_source_explosition(AMPLITUDE_SOURCE_EXPLOSION_1, AMPLITUDE_SOURCE_EXPLOSION_3, 
                                                    PHASE_SOURCE_EXPLOSION_1, PHASE_SOURCE_EXPLOSION_3, 
                                                    FREQUENCY_SOURCE_EXPLOSION,
                                                    dist_s_explosition_to_r2); 
    std::cout << "\n";
    std::cout << "distance_R2_to_Source (m): "      << s_explosition_to_r2 << std::endl;

    /*
    dist_s_explosition_to_r3 = calculateDistance(pos_gps_r3_lat, pos_gps_r3_lon, pos_gps_r3_alt, POS_SOURCE_EXPLOSITION_LAT, POS_SOURCE_EXPLOSITION_LONG, POS_SOURCE_EXPLOSITION_ALT);
    s_explosition_to_r3 = calcul_source_explosition(AMPLITUDE_SOURCE_EXPLOSION_1, AMPLITUDE_SOURCE_EXPLOSION_3, 
                                                    PHASE_SOURCE_EXPLOSION_1, PHASE_SOURCE_EXPLOSION_3, 
                                                    FREQUENCY_SOURCE_EXPLOSION,
                                                    dist_s_explosition_to_r3); 
    dist_s_explosition_to_r4 = calculateDistance(pos_gps_r4_lat, pos_gps_r4_lon, pos_gps_r4_alt, POS_SOURCE_EXPLOSITION_LAT, POS_SOURCE_EXPLOSITION_LONG, POS_SOURCE_EXPLOSITION_ALT);
    s_explosition_to_r4 = calcul_source_explosition(AMPLITUDE_SOURCE_EXPLOSION_1, AMPLITUDE_SOURCE_EXPLOSION_3, 
                                                    PHASE_SOURCE_EXPLOSION_1, PHASE_SOURCE_EXPLOSION_3, 
                                                    FREQUENCY_SOURCE_EXPLOSION,
                                                    dist_s_explosition_to_r4); 
    dist_s_explosition_to_r5 = calculateDistance(pos_gps_r5_lat, pos_gps_r5_lon, pos_gps_r5_alt, POS_SOURCE_EXPLOSITION_LAT, POS_SOURCE_EXPLOSITION_LONG, POS_SOURCE_EXPLOSITION_ALT);
    s_explosition_to_r5 = calcul_source_explosition(AMPLITUDE_SOURCE_EXPLOSION_1, AMPLITUDE_SOURCE_EXPLOSION_3, 
                                                    PHASE_SOURCE_EXPLOSION_1, PHASE_SOURCE_EXPLOSION_3, 
                                                    FREQUENCY_SOURCE_EXPLOSION,
                                                    dist_s_explosition_to_r5);      
      
    */
    msg_ = std::make_unique<std_msgs::msg::Float32>();
    msg_->data = s_explosition_to_r1;
    source_01_publisher_->publish(std::move(msg_));
    
    msg2_ = std::make_unique<std_msgs::msg::Float32>();
    msg2_->data = s_explosition_to_r2;
    source_02_publisher_->publish(std::move(msg2_));

    msg3_ = std::make_unique<std_msgs::msg::Float32>();
    msg3_->data = s_explosition_to_r3;
    source_03_publisher_->publish(std::move(msg3_));

    msg4_ = std::make_unique<std_msgs::msg::Float32>();
    msg4_->data = s_explosition_to_r4;
    source_04_publisher_->publish(std::move(msg4_));

    msg5_ = std::make_unique<std_msgs::msg::Float32>();
    msg5_->data = s_explosition_to_r5;
    source_05_publisher_->publish(std::move(msg5_));
  }

  
  double calcul_source_receive(double pos_source_x, double pos_source_y, double pos_source_z, double pos_robot_x, double pos_robot_y, double pos_robot_z,
                                double amplitude_source_1, double amplitude_source_3, double phase_source_1, double phase_source_3, double frequency)
  {
    double dist_source2robot = sqrt((pos_source_x - pos_robot_x)*(pos_source_x - pos_robot_x) + (pos_source_y - pos_robot_y)*(pos_source_y - pos_robot_y) + (pos_source_z - pos_robot_z) * (pos_source_z - pos_robot_z)); 
    double delta_phase_1 = 2*PI*dist_source2robot*frequency/SPEED_OF_LIGHT;
    double delta_phase_3 = 2*PI*dist_source2robot*3*frequency/SPEED_OF_LIGHT; 
    //std::unique_ptr<geometry_msgs::msg::Vector3> data;
    return (1/(dist_source2robot*dist_source2robot)) * (amplitude_source_1 * sin(2*PI*frequency*count_time + phase_source_1 + delta_phase_1) + amplitude_source_3 * sin(2*PI*3*frequency*count_time + phase_source_3 + delta_phase_3));
    //data -> y = delta_phase_1;
    //data -> z = delta_phase_3;
  }

  double calcul_source_explosition(double amplitude_source_1, double amplitude_source_3, 
                                   double phase_source_1, double phase_source_3, 
                                   double frequency, double dist_source2robot)
  {
    //double dist_source2robot = sqrt((pos_source_x - pos_robot_x)*(pos_source_x - pos_robot_x) + (pos_source_y - pos_robot_y)*(pos_source_y - pos_robot_y) + (pos_source_z - pos_robot_z) * (pos_source_z - pos_robot_z)); 
    double delta_phase_1 = 2*PI*dist_source2robot*frequency/SPEED_OF_LIGHT;
    double delta_phase_3 = 2*PI*dist_source2robot*3*frequency/SPEED_OF_LIGHT; 
    
    std::setprecision(12);
    std::cout << "\n\n"; 
    std::cout << "diff phase 1(rad): "      << delta_phase_1 << std::endl;
    
    std::cout << std::fixed << std::setprecision(12);
    std::cout << "diff phase 2(rad): "      << count_time_explosition + delta_phase_1 << std::endl;
    //std::unique_ptr<geometry_msgs::msg::Vector3> data;
    std::cout << std::fixed << std::setprecision(12);
    return (1/(dist_source2robot*dist_source2robot))
              * (amplitude_source_1 * sin(2*PI*frequency*count_time_explosition + phase_source_1 + delta_phase_1) + amplitude_source_3 * sin(2*PI*3*frequency*count_time_explosition + phase_source_3 + delta_phase_3)) 
              * exp(-double (count_time_explosition/1000));
    
    //return (1/(dist_source2robot*dist_source2robot))
    //          * (amplitude_source_1 * sin(2*PI*frequency*count_time_explosition + phase_source_1 + delta_phase_1) ) 
    //          * exp(-count_time_explosition/1000);

    //return (amplitude_source_1 * sin(2*PI*frequency*count_time_explosition + phase_source_1 + delta_phase_1) );
    //return sin(2*PI*frequency*count_time_explosition + delta_phase_1)* exp(-count_time_explosition/1000);
    
    //return sin(count_time_explosition + delta_phase_1)* exp(-count_time_explosition/1000);
    
    //return exp(-count_time_explosition/1000);

    //return double ((1/(dist_source2robot*dist_source2robot))
      //        * (amplitude_source_1 * sin(2*PI*frequency*count_time + phase_source_1 + delta_phase_1) + amplitude_source_3 * sin(2*PI*3*frequency*count_time + phase_source_3 + delta_phase_3)));
  }


  // Define the radius of the Earth in meters
  const double R = 6371000; // Mean radius of the Earth
  // Function to calculate the distance using Vincenty formula
  double calculateDistance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2) {
    // Convert latitude and longitude from degrees to radians
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;

    // Calculate differences in latitude and longitude
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // Vincenty formula
    double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    // Calculate the distance
    double distance = R * c;

    // Correct for altitude
    distance = sqrt(distance * distance + pow(alt2 - alt1, 2));

    return distance;
  }


  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr sub_pos_gps_r1, sub_pos_gps_r2, sub_pos_gps_r3, sub_pos_gps_r4, sub_pos_gps_r5;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr source_01_publisher_, source_02_publisher_, 
                                                        source_03_publisher_, source_04_publisher_, source_05_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr source_01_to_r01_pub_, source_02_to_r01_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_pose_local_r1, sub_pose_local_r2, 
                                                              sub_pose_local_r3, sub_pose_local_r4, sub_pose_local_r5;

  std::unique_ptr<std_msgs::msg::Float32> msg_, msg2_, msg3_, msg4_, msg5_;
  std::unique_ptr<geometry_msgs::msg::Vector3> data01_, data02_, pose_local_r1, pose_local_r2, pose_local_r3, pose_local_r4, pose_local_r5,
                                                pose_source_explosition, pose_source_r1, pose_source_r2, pose_source_r3, pose_source_r4, pose_source_r5;

  size_t count_time;
  double count_time_explosition;
  double dist_s1_to_r1, dist_s1_to_r2, 
          dist_s_explosition_to_r1, dist_s_explosition_to_r2, dist_s_explosition_to_r3, dist_s_explosition_to_r4, dist_s_explosition_to_r5,
          dist_Ri_to_S;

  double s_explosition_to_r1, s_explosition_to_r2, s_explosition_to_r3, s_explosition_to_r4, s_explosition_to_r5;

  double pose_local_r1_x, pose_local_r1_y, pose_local_r1_z;
  double pose_local_r2_x, pose_local_r2_y, pose_local_r2_z;
  double pose_local_r3_x, pose_local_r3_y, pose_local_r3_z;
  double pose_local_r4_x, pose_local_r4_y, pose_local_r4_z;

  double pos_gps_r1_lat, pos_gps_r1_lon, pos_gps_r1_alt;
  double pos_gps_r2_lat, pos_gps_r2_lon, pos_gps_r2_alt;
  double pos_gps_r3_lat, pos_gps_r3_lon, pos_gps_r3_alt;
  double pos_gps_r4_lat, pos_gps_r4_lon, pos_gps_r4_alt;
  double pos_gps_r5_lat, pos_gps_r5_lon, pos_gps_r5_alt;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
