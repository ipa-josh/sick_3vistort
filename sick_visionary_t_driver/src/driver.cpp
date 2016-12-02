#include <ros/ros.h>
#include <sick_visionary_t_driver/driver.h>

template<>
bool Driver_3DCS::read_param<uint8_t>(const std::string &name, uint8_t &var) {
	int num=var;
	if(ros::param::get("~"+name, num)) {
		var=num;
		return true;
	}
	return false;
}
