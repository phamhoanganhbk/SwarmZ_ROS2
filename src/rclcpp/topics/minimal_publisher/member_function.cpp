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


#define SPEED_OF_SOUND 343 // m/s
#define PI 3.1415
    
#define AMPLITUDE_SOURCE_MOTOR_1 5
#define AMPLITUDE_SOURCE_MOTOR_3 0
#define FREQUENCY_SOURCE_MOTOR 100 // hz
#define PHASE_SOURCE_MOTOR_1 0
#define PHASE_SOURCE_MOTOR_3 0


#define AMPLITUDE_SOURCE_NOISE_WHITE_1 1
#define AMPLITUDE_SOURCE_NOISE_WHITE_3 0
#define FREQUENCY_SOURCE_NOISE_WHITE 100 // hz
#define PHASE_SOURCE_NOISE_WHITE_1 0
#define PHASE_SOURCE_NOISE_WHITE_3 0

#define AMPLITUDE_SOURCE_EXPLOSION_1 100
#define AMPLITUDE_SOURCE_EXPLOSION_3 0
#define FREQUENCY_SOURCE_EXPLOSION 100 // 10 hz
#define PHASE_SOURCE_EXPLOSION_1 0
#define PHASE_SOURCE_EXPLOSION_3 0

/*
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

#define POS_SOURCE_EXPLOSION_X 0
#define POS_SOURCE_EXPLOSION_Y 0
#define POS_SOURCE_EXPLOSION_Z 0  
*/

#define POS_SOURCE_EXPLOSION_LAT 43.13471   //deg
#define POS_SOURCE_EXPLOSION_LONG 6.01507   //deg
#define POS_SOURCE_EXPLOSION_ALT 6.0        //Height above mean sea (m)  


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
		

    // Obtenez la localisation GPS du drone
    sub_pos_gps_r1 = this->create_subscription<px4_msgs::msg::SensorGps>("/px4_1/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg) {
			//std::cout << "\n";
      pos_gps_r1_lat = msg->latitude_deg;
      pos_gps_r1_lon = msg->longitude_deg;
      pos_gps_r1_alt = msg->altitude_msl_m;    
		});


    sub_pos_gps_r2 = this->create_subscription<px4_msgs::msg::SensorGps>("/px4_2/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg) {
			//std::cout << "\n";
      pos_gps_r2_lat = msg->latitude_deg;
      pos_gps_r2_lon = msg->longitude_deg;
      pos_gps_r2_alt = msg->altitude_msl_m;   
		});

    sub_pos_gps_r3 = this->create_subscription<px4_msgs::msg::SensorGps>("/px4_3/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg) {
			//std::cout << "\n";
      pos_gps_r3_lat = msg->latitude_deg;
      pos_gps_r3_lon = msg->longitude_deg;
      pos_gps_r3_alt = msg->altitude_msl_m;     
		});

    sub_pos_gps_r4 = this->create_subscription<px4_msgs::msg::SensorGps>("/px4_4/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg4) {
			//std::cout << "\n";
      pos_gps_r4_lat = msg4->latitude_deg;
      pos_gps_r4_lon = msg4->longitude_deg;
      pos_gps_r4_alt = msg4->altitude_msl_m;  
		});

    sub_pos_gps_r5 = this->create_subscription<px4_msgs::msg::SensorGps>("/px4_5/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg) {
			//std::cout << "\n";
      pos_gps_r5_lat = msg->latitude_deg;
      pos_gps_r5_lon = msg->longitude_deg;
      pos_gps_r5_alt = msg->altitude_msl_m;  
		});


    //publisher_            = this->create_publisher<std_msgs::msg::String>("topic", 10);
    source_01_publisher_  = this->create_publisher<std_msgs::msg::Float32>("/source_explosion/toDrone_1",10);
    source_02_publisher_  = this->create_publisher<std_msgs::msg::Float32>("/source_explosion/toDrone_2",10);
    source_03_publisher_  = this->create_publisher<std_msgs::msg::Float32>("/source_explosion/toDrone_3",10);
    source_04_publisher_  = this->create_publisher<std_msgs::msg::Float32>("/source_explosion/toDrone_4",10);
    source_05_publisher_  = this->create_publisher<std_msgs::msg::Float32>("/source_explosion/toDrone_5",10);

    // Make and Publisher sources 
    //source_01_to_r01_pub_        = this->create_publisher<geometry_msgs::msg::Vector3>("/source_01/robot_01/pub",10);
    //source_02_to_r01_pub_        = this->create_publisher<geometry_msgs::msg::Vector3>("/source_02/robot_01/pub",10);
    timer_ = this->create_wall_timer(1ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:  

  void timer_callback()
  {
    
    // Calculer la distance entre le drone et l'explosion
    dist_s_explosion_to_r1 = calculateDistance(pos_gps_r1_lat, pos_gps_r1_lon, pos_gps_r1_alt, 
                                                POS_SOURCE_EXPLOSION_LAT, POS_SOURCE_EXPLOSION_LONG, POS_SOURCE_EXPLOSION_ALT);
                              
    dist_s_explosion_to_r2 = calculateDistance(pos_gps_r2_lat, pos_gps_r2_lon, pos_gps_r2_alt, 
                                                POS_SOURCE_EXPLOSION_LAT, POS_SOURCE_EXPLOSION_LONG, POS_SOURCE_EXPLOSION_ALT);
    
    dist_s_explosion_to_r3 = calculateDistance(pos_gps_r3_lat, pos_gps_r3_lon, pos_gps_r3_alt, 
                                                POS_SOURCE_EXPLOSION_LAT, POS_SOURCE_EXPLOSION_LONG, POS_SOURCE_EXPLOSION_ALT);
                                 

    dist_s_explosion_to_r4 = calculateDistance(pos_gps_r4_lat, pos_gps_r4_lon, pos_gps_r4_alt, 
                                                POS_SOURCE_EXPLOSION_LAT, POS_SOURCE_EXPLOSION_LONG, POS_SOURCE_EXPLOSION_ALT);
                                       

    dist_s_explosion_to_r5 = calculateDistance(pos_gps_r5_lat, pos_gps_r5_lon, pos_gps_r5_alt, 
                                                POS_SOURCE_EXPLOSION_LAT, POS_SOURCE_EXPLOSION_LONG, POS_SOURCE_EXPLOSION_ALT);
    
    std::cout << "\n";
    std::cout << "dist_s_explosion_to_r1: "      << dist_s_explosion_to_r1   << std::endl;
    std::cout << "dist_s_explosion_to_r2: "      << dist_s_explosion_to_r2   << std::endl;
    std::cout << "dist_s_explosion_to_r3: "      << dist_s_explosion_to_r3   << std::endl;     
    std::cout << "dist_s_explosion_to_r4: "      << dist_s_explosion_to_r4   << std::endl;     
    std::cout << "dist_s_explosion_to_r5: "      << dist_s_explosion_to_r5   << std::endl;                                            

    // Calculer la distance entre les drones
    dist_r1_r2 = calculateDistance(pos_gps_r1_lat, pos_gps_r1_lon, pos_gps_r1_alt, pos_gps_r2_lat, pos_gps_r2_lon, pos_gps_r2_alt); 
    dist_r1_r3 = calculateDistance(pos_gps_r1_lat, pos_gps_r1_lon, pos_gps_r1_alt, pos_gps_r3_lat, pos_gps_r3_lon, pos_gps_r3_alt); 
    dist_r1_r4 = calculateDistance(pos_gps_r1_lat, pos_gps_r1_lon, pos_gps_r1_alt, pos_gps_r4_lat, pos_gps_r4_lon, pos_gps_r4_alt); 
    dist_r1_r5 = calculateDistance(pos_gps_r1_lat, pos_gps_r1_lon, pos_gps_r1_alt, pos_gps_r5_lat, pos_gps_r5_lon, pos_gps_r5_alt); 
    dist_r2_r3 = calculateDistance(pos_gps_r2_lat, pos_gps_r2_lon, pos_gps_r2_alt, pos_gps_r3_lat, pos_gps_r3_lon, pos_gps_r3_alt); 
    dist_r2_r4 = calculateDistance(pos_gps_r2_lat, pos_gps_r2_lon, pos_gps_r2_alt, pos_gps_r4_lat, pos_gps_r4_lon, pos_gps_r4_alt); 
    dist_r2_r5 = calculateDistance(pos_gps_r2_lat, pos_gps_r2_lon, pos_gps_r2_alt, pos_gps_r5_lat, pos_gps_r5_lon, pos_gps_r5_alt); 
    dist_r3_r4 = calculateDistance(pos_gps_r3_lat, pos_gps_r3_lon, pos_gps_r3_alt, pos_gps_r4_lat, pos_gps_r4_lon, pos_gps_r4_alt); 
    dist_r3_r5 = calculateDistance(pos_gps_r3_lat, pos_gps_r3_lon, pos_gps_r3_alt, pos_gps_r5_lat, pos_gps_r5_lon, pos_gps_r5_alt); 
    dist_r4_r5 = calculateDistance(pos_gps_r4_lat, pos_gps_r4_lon, pos_gps_r4_alt, pos_gps_r5_lat, pos_gps_r5_lon, pos_gps_r5_alt); 


    // Générer du bruit de moteur
    s_motor_to_r1 = calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r1_r2) 
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r1_r3)
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r1_r4) 
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r1_r5); 
    
    s_motor_to_r2 = calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r1_r2) 
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r2_r3)
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r2_r4) 
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r2_r5); 

    s_motor_to_r3 = calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r1_r3) 
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r2_r3)
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r3_r4) 
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r3_r5); 

    s_motor_to_r4 = calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r1_r4) 
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r2_r4)
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r3_r4) 
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r4_r5); 
    
    s_motor_to_r5 = calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r1_r5) 
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r2_r5)
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r3_r5) 
                  + calcul_source_motor(AMPLITUDE_SOURCE_MOTOR_1, AMPLITUDE_SOURCE_MOTOR_3, 
                                        PHASE_SOURCE_MOTOR_1, PHASE_SOURCE_MOTOR_3, 
                                        FREQUENCY_SOURCE_MOTOR,
                                        dist_r4_r5); 

    count_time_explosion ++;
    if (count_time_explosion >=10000){ //
      count_time_explosion = 0;
    }

    if (count_time_explosion/1000 >= dist_s_explosion_to_r1/SPEED_OF_SOUND)
    {
      s_explosion_to_r1 = calcul_source_explosion(AMPLITUDE_SOURCE_EXPLOSION_1, AMPLITUDE_SOURCE_EXPLOSION_3, 
                                                    PHASE_SOURCE_EXPLOSION_1, PHASE_SOURCE_EXPLOSION_3, 
                                                    FREQUENCY_SOURCE_EXPLOSION,
                                                    dist_s_explosion_to_r1); 
    }
    else{
      s_explosion_to_r1 = 0;
    }
    
    if (count_time_explosion/1000 >= dist_s_explosion_to_r2/SPEED_OF_SOUND){
      s_explosion_to_r2 = calcul_source_explosion(AMPLITUDE_SOURCE_EXPLOSION_1, AMPLITUDE_SOURCE_EXPLOSION_3, 
                                                    PHASE_SOURCE_EXPLOSION_1, PHASE_SOURCE_EXPLOSION_3, 
                                                    FREQUENCY_SOURCE_EXPLOSION,
                                                    dist_s_explosion_to_r2); 
    }
    else{
      s_explosion_to_r2 = 0;
    }

    if (count_time_explosion/1000 >= dist_s_explosion_to_r3/SPEED_OF_SOUND){
      s_explosion_to_r3 = calcul_source_explosion(AMPLITUDE_SOURCE_EXPLOSION_1, AMPLITUDE_SOURCE_EXPLOSION_3, 
                                                    PHASE_SOURCE_EXPLOSION_1, PHASE_SOURCE_EXPLOSION_3, 
                                                    FREQUENCY_SOURCE_EXPLOSION,
                                                    dist_s_explosion_to_r3); 
    }
    else{
      s_explosion_to_r3 = 0;
    }

    if (count_time_explosion/1000 >= dist_s_explosion_to_r4/SPEED_OF_SOUND){
      s_explosion_to_r4 = calcul_source_explosion(AMPLITUDE_SOURCE_EXPLOSION_1, AMPLITUDE_SOURCE_EXPLOSION_3, 
                                                    PHASE_SOURCE_EXPLOSION_1, PHASE_SOURCE_EXPLOSION_3, 
                                                    FREQUENCY_SOURCE_EXPLOSION,
                                                    dist_s_explosion_to_r4); 
    }
    else{
      s_explosion_to_r4 = 0;
    }

    if (count_time_explosion/1000 >= dist_s_explosion_to_r5/SPEED_OF_SOUND){
      s_explosion_to_r5 = calcul_source_explosion(AMPLITUDE_SOURCE_EXPLOSION_1, AMPLITUDE_SOURCE_EXPLOSION_3, 
                                                    PHASE_SOURCE_EXPLOSION_1, PHASE_SOURCE_EXPLOSION_3, 
                                                    FREQUENCY_SOURCE_EXPLOSION,
                                                    dist_s_explosion_to_r5); 
    }
    else{
      s_explosion_to_r5 = 0;
    }

    // Total des sources de signaux audio pour chaque drone
    s_total_to_r1 = s_explosion_to_r1 + s_motor_to_r1;
    s_total_to_r2 = s_explosion_to_r2 + s_motor_to_r2;
    s_total_to_r3 = s_explosion_to_r3 + s_motor_to_r3;
    s_total_to_r4 = s_explosion_to_r4 + s_motor_to_r4;
    s_total_to_r5 = s_explosion_to_r5 + s_motor_to_r5;

    
    msg_ = std::make_unique<std_msgs::msg::Float32>();
    msg_->data = s_total_to_r1;
    source_01_publisher_->publish(std::move(msg_));
    
    msg2_ = std::make_unique<std_msgs::msg::Float32>();
    msg2_->data = s_total_to_r2;
    source_02_publisher_->publish(std::move(msg2_));

    msg3_ = std::make_unique<std_msgs::msg::Float32>();
    msg3_->data = s_total_to_r3;
    source_03_publisher_->publish(std::move(msg3_));

    msg4_ = std::make_unique<std_msgs::msg::Float32>();
    msg4_->data = s_total_to_r4;
    source_04_publisher_->publish(std::move(msg4_));

    msg5_ = std::make_unique<std_msgs::msg::Float32>();
    msg5_->data = s_total_to_r5;
    source_05_publisher_->publish(std::move(msg5_));
  }


  double calcul_source_explosion(double amplitude_source_1, double amplitude_source_3, 
                                   double phase_source_1, double phase_source_3, 
                                   double frequency, double dist_source2robot)
  {
    double delta_phase_1 = 2*PI*dist_source2robot*frequency/SPEED_OF_SOUND;
    double delta_phase_3 = 2*PI*dist_source2robot*3*frequency/SPEED_OF_SOUND; 

    
    return ((1/(dist_source2robot*dist_source2robot))
              * (amplitude_source_1 * sin(2*PI*frequency*(count_time_explosion/1000) + phase_source_1 + delta_phase_1)) 
              * exp(-count_time_explosion/1000));
  }

  double calcul_source_motor(double amplitude_source_1, double amplitude_source_3, 
                                   double phase_source_1, double phase_source_3, 
                                   double frequency, double dist_source2robot)
  {
    double delta_phase_1 = 2*PI*dist_source2robot*frequency/SPEED_OF_SOUND;
    double delta_phase_3 = 2*PI*dist_source2robot*3*frequency/SPEED_OF_SOUND; 
    
    
    return ((1/(dist_source2robot*dist_source2robot))
              * (amplitude_source_1 * sin(2*PI*frequency*(count_time_explosion/1000) + phase_source_1 + delta_phase_1)));
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
                                                pose_source_explosion, pose_source_r1, pose_source_r2, pose_source_r3, pose_source_r4, pose_source_r5;

  size_t count_time;
  double count_time_explosion;
  double dist_s1_to_r1, dist_s1_to_r2, 
          dist_s_explosion_to_r1, dist_s_explosion_to_r2, dist_s_explosion_to_r3, dist_s_explosion_to_r4, dist_s_explosion_to_r5,
          dist_Ri_to_S;
  
  double dist_r1_r2, dist_r1_r3, dist_r1_r4, dist_r1_r5, dist_r2_r3, dist_r2_r4, dist_r2_r5, dist_r3_r4, dist_r3_r5, dist_r4_r5;

  double s_explosion_to_r1, s_explosion_to_r2, s_explosion_to_r3, s_explosion_to_r4, s_explosion_to_r5;
  double s_motor_to_r1, s_motor_to_r2, s_motor_to_r3, s_motor_to_r4, s_motor_to_r5;
  double s_total_to_r1, s_total_to_r2, s_total_to_r3, s_total_to_r4, s_total_to_r5;
  

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
