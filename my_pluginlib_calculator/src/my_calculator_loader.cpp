#include<boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>
#include <my_pluginlib_calculator/my_calculator_base.h>
int main(int argc, char** argv)
{
 // load the plugins using ClassLoader class provided by pluginlib.
 // namefor the loaser and calculator_base
 pluginlib::ClassLoader<my_calculator_base::calc_functions> calc_loader("my_pluginlib_calculator", "my_calculator_base::calc_functions");
try
  {
    // create an instance of the add class using the classloader object
    boost::shared_ptr<my_calculator_base::calc_functions> add = calc_loader.createInstance("my_pluginlib_calculator/Add");
    // give the input and perform the operations
    add->get_numbers(10.0,10.0);
    double result = add->operation();

    ROS_INFO("Triangle area: %.2f", result);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  try
  {
    boost::shared_ptr<my_calculator_base::calc_functions> sub = calc_loader.createInstance("my_pluginlib_calculator/Sub");

    sub->get_numbers(10.0,10.0);
    double result = sub->operation();

    ROS_INFO("Substracted result: %.2f", result);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

}
