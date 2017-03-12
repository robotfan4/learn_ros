#include <pluginlib/class_list_macros.h>
#include <my_pluginlib_calculator/my_calculator_base.h>
#include <my_pluginlib_calculator/my_calculator_plugins.h>
PLUGINLIB_EXPORT_CLASS(my_calculator_plugins::Add, my_calculator_base::calc_functions); // the class name of plugin and base class.
PLUGINLIB_EXPORT_CLASS(my_calculator_plugins::Sub, my_calculator_base::calc_functions);
