#ifndef READ_YAML_H
#define READ_YAML_H

#include <map>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <string>
#include <ros/package.h>

#define JOINT_PD_GAIN_FILEPATH "/config/pd_gain.yaml"

enum
{
    WST = 0, LHY, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR
};

class YAML_CONFIG_READER
{
private:
  double apply_for_all_Kp;
  double apply_for_all_Kd;

  std::map<std::string, double> jointP_gain; //name:P gain
  std::map<std::string, double> jointD_gain; //name:D gain

  const char *file_path = JOINT_PD_GAIN_FILEPATH;
  const std::string pkg_path = ros::package::getPath("kubot_study_pkg");


public:

 // YAML_CONFIG_READER();
  void getJoint_PD_gainFrom_yaml();
  double get_Kp(int joint_num);
  double get_Kd(int joint_num);




};

#endif // READ_YAML_H
