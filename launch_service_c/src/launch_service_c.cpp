#include <cstdio>
#include <string>
#include <iostream>
#include <pybind11/embed.h>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  pybind11::scoped_interpreter guard{}; // start the interpreter and keep it alive
  pybind11::exec("from ros2launch.api import launch_a_launch_file"); // use the Python API
  pybind11::exec("print(\"Python detected, lethal force engaged\")"); // use the Python API
  std::string pyline = "";
  pyline += "launch_a_launch_file(";
  pyline += "launch_file_path='/opt/ros/humble/share/dummy_robot_bringup/launch/dummy_robot_bringup.launch.py',";
  pyline += "launch_file_arguments=[]";
  pyline += ")";

  pybind11::exec(pyline); // use the Python API
  return 0;
}
