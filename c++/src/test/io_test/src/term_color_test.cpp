#include <ros/ros.h>

#include <io_lib/termcolor.hpp>

using namespace as64_::io_;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "term_color_test_node");

  // test foreground colors
  std::cout << grey    << "grey message"    << reset << std::endl;
  std::cout << red     << "red message"     << reset << std::endl;
  std::cout << green   << "green message"   << reset << std::endl;
  std::cout << yellow  << "yellow message"  << reset << std::endl;
  std::cout << blue    << "blue message"    << reset << std::endl;
  std::cout << magenta << "magenta message" << reset << std::endl;
  std::cout << cyan    << "cyan message"    << reset << std::endl;
  std::cout << white   << "white message"   << reset << std::endl;
  std::cout            << "default message" << std::endl;
  std::cout << std::endl;

  // test background colors
  std::cout << on_grey    << "message on grey"    << reset << std::endl;
  std::cout << on_red     << "message on red"     << reset << std::endl;
  std::cout << on_green   << "message on green"   << reset << std::endl;
  std::cout << on_yellow  << "message on yellow"  << reset << std::endl;
  std::cout << on_blue    << "message on blue"    << reset << std::endl;
  std::cout << on_magenta << "message on magenta" << reset << std::endl;
  std::cout << on_cyan    << "message on cyan"    << reset << std::endl;
  std::cout << on_white   << "message on white"   << reset << std::endl;
  std::cout               << "default message"    << std::endl;
  std::cout << std::endl;

  // test foreground and backgrounds colors
  std::cout << red  << on_white  << "red on white"   << reset << std::endl;
  std::cout << blue << on_yellow << "blue on yellow" << reset << std::endl;
  std::cout << std::endl;

  // test unsual attributes
  std::cout << bold      << red  << "bold red message"  << reset << std::endl;
  std::cout << dark      << blue << "dark blue message" << reset << std::endl;
  std::cout << underline << "underlined message" << reset << std::endl;
  std::cout << blink     << "blinked message"    << reset << std::endl;
  std::cout << reverse   << "reversed message"   << reset << std::endl;
  std::cout << concealed << "concealed message"  << reset << std::endl;
  std::cout              << "default message"    << std::endl;

  // test ansi escape characters are skipped for streams
  std::stringstream s1;
  s1 << red << "term" << blue << on_yellow << "color";

  if (s1.str() != "termcolor")
      return 1;

  // test ansi escape characters are preserved for streams if asked
  std::stringstream s2;
  s2 << colorize << red << "term" << nocolorize << blue << "color";

  if (s2.str() != "\033[31m" "termcolor")
      return 2;


  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
