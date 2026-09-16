if (param_listener_->is_old(params_)) {
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Frame: '%s'", params_.control.frame.id.c_str());
    RCLCPP_INFO(logger, "String: '%s'", std::string{params_.fixed_string}.c_str());
}