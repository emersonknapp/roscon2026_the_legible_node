// https://github.com/KumarRobotics/ublox/blob/ros2/ublox_gps/src/node.cpp#L331C1-L365C4

  this->declare_parameter("dat.set", false);
  this->declare_parameter("dat.majA", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("dat.flat", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("dat.shift", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("dat.rot", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("dat.scale", rclcpp::PARAMETER_DOUBLE);
  if (getRosBoolean(this, "dat.set")) {
    std::vector<double> shift, rot;
    if (!this->get_parameter("dat.majA", cfg_dat_.maj_a)
        || !this->get_parameter("dat.flat", cfg_dat_.flat)
        || !this->get_parameter("dat.shift", shift)
        || !this->get_parameter("dat.rot", rot)
        || !this->get_parameter("dat.scale", cfg_dat_.scale)) {
      throw std::runtime_error(std::string("dat.set is true, therefore ") +
         "dat.majA, dat.flat, dat.shift, dat.rot, & dat.scale must be set");
    }
    if (shift.size() != 3 || rot.size() != 3) {
      throw std::runtime_error(std::string("size of dat.shift & dat.rot ") +
                               "must be 3");
    }
    checkRange(cfg_dat_.maj_a, 6300000.0, 6500000.0, "dat.majA");
    checkRange(cfg_dat_.flat, 0.0, 500.0, "dat.flat");

    checkRange(shift, 0.0, 500.0, "dat.shift");
    cfg_dat_.d_x = shift[0];
    cfg_dat_.d_y = shift[1];
    cfg_dat_.d_z = shift[2];

    checkRange(rot, -5000.0, 5000.0, "dat.rot");
    cfg_dat_.rot_x = rot[0];
    cfg_dat_.rot_y = rot[1];
    cfg_dat_.rot_z = rot[2];

    checkRange(cfg_dat_.scale, 0.0, 50.0, "scale");
  }
