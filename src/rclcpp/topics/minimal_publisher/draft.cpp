//double calcul_source_receive(double pos_source_x, double pos_source_y, double pos_source_z, double pos_robot_x, double pos_robot_y, double pos_robot_z,
  //                              double amplitude_source_1, double amplitude_source_3, double phase_source_1, double phase_source_3, double frequency)
  double calcul_source_receive(double pos_source_x, double pos_source_y, double pos_source_z, double pos_robot_x, double pos_robot_y, double pos_robot_z,
                                double amplitude_source_1, double amplitude_source_3, double phase_source_1, double phase_source_3, double frequency)
  {
    double dist_source2robot = sqrt((pos_source_x - pos_robot_x)*(pos_source_x - pos_robot_x) + (pos_source_y - pos_robot_y)*(pos_source_y - pos_robot_y) + (pos_source_z - pos_robot->z) * (pos_source->z - pos_robot->z)); 
    double delta_phase_1 = 2*PI*dist_source2robot*frequency/SPEED_OF_LIGHT;
    double delta_phase_3 = 2*PI*dist_source2robot*3*frequency/SPEED_OF_LIGHT; 
    //std::unique_ptr<geometry_msgs::msg::Vector3> data;
    return (1/(dist_source2robot*dist_source2robot)) * (amplitude_source_1 * sin(2*PI*frequency + phase_source_1 + delta_phase_1) + amplitude_source_3 * sin(2*PI*3*frequency + phase_source_3 + delta_phase_3));
    //data -> y = delta_phase_1;
    //data -> z = delta_phase_3;
  }




  sub_pose_local_r2 = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/px4_2/fmu/out/vehicle_local_position", qos,
		[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
			pose_local_r2 -> x = msg->x;
      pose_local_r2 -> y = msg->y;
      pose_local_r2 -> z = msg->z;
      /*
      std::cout << "\n\n\n\n\n\n\n\n\n\n";
			std::cout << "Distance between Drone 1 and Source S1"   << std::endl;
			std::cout << "x: "      << msg->x    << std::endl;
      std::cout << "y: "      << msg->y    << std::endl;
      std::cout << "z: "      << msg->z    << std::endl;
      dist_s1_to_r1 = sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
      std::cout << "dist: "      << dist_s1_to_r1   << std::endl;
      */
		});

    sub_pose_local_r3 = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/px4_3/fmu/out/vehicle_local_position", qos,
		[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
			pose_local_r3 -> x = msg->x;
      pose_local_r3 -> y = msg->y;
      pose_local_r3 -> z = msg->z;
      /*
      std::cout << "\n\n\n\n\n\n\n\n\n\n";
			std::cout << "Distance between Drone 1 and Source S1"   << std::endl;
			std::cout << "x: "      << msg->x    << std::endl;
      std::cout << "y: "      << msg->y    << std::endl;
      std::cout << "z: "      << msg->z    << std::endl;
      dist_s1_to_r1 = sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
      std::cout << "dist: "      << dist_s1_to_r1   << std::endl;
      */
		});

    sub_pose_local_r4 = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/px4_4/fmu/out/vehicle_local_position", qos,
		[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
			pose_local_r4 -> x = msg->x;
      pose_local_r4 -> y = msg->y;
      pose_local_r4 -> z = msg->z;
      /*
      std::cout << "\n\n\n\n\n\n\n\n\n\n";
			std::cout << "Distance between Drone 1 and Source S1"   << std::endl;
			std::cout << "x: "      << msg->x    << std::endl;
      std::cout << "y: "      << msg->y    << std::endl;
      std::cout << "z: "      << msg->z    << std::endl;
      dist_s1_to_r1 = sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
      std::cout << "dist: "      << dist_s1_to_r1   << std::endl;
      */
		});

    sub_pose_local_r5 = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/px4_5/fmu/out/vehicle_local_position", qos,
		[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
			pose_local_r5 -> x = msg->x;
      pose_local_r5 -> y = msg->y;
      pose_local_r5 -> z = msg->z;
      /*
      std::cout << "\n\n\n\n\n\n\n\n\n\n";
			std::cout << "Distance between Drone 1 and Source S1"   << std::endl;
			std::cout << "x: "      << msg->x    << std::endl;
      std::cout << "y: "      << msg->y    << std::endl;
      std::cout << "z: "      << msg->z    << std::endl;
      dist_s1_to_r1 = sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
      std::cout << "dist: "      << dist_s1_to_r1   << std::endl;
      */
		});