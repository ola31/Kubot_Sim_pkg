/*
#ifndef FOOTSTEP_PLANNER_H
#define FOOTSTEP_PLANNER_H

#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <vector>

#define PI 3.141593

using namespace std;

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;

struct XY
{
  double x;
  double y;
};


class FootstepPlanner
{
private:
  double fb_step_size;
  double step_time;
  double step_num;
  int start_foot;
  double dsp_ratio;
  double goal_turn_angle;
  double foot_height;
  double foot_distance;
  //vector<vector<double>> FootSteps;

  int last_foot;
  double last_side_foot[3];




public:
  vector<vector<double>> FootSteps;
  FootstepPlanner();
  FootstepPlanner(double fb_step_size_, double step_time_, double step_num_, int start_foot_, double dsp_ratio_, double goal_turn_angle_,double foot_height);

  void set_fb_step_size(double fb_step_size_);
  void set_step_time(double step_time_);
  void set_step_num(int step_num_);
  void set_start_foot(double start_foot);
  void set_dsp_ratio(double dsp_ratio);
  void set_goal_turn_angle(double goal_turn_angle_);

  double get_fb_step_size();
  double get_step_time();
  int get_step_num();
  double get_start_foot();
  double get_dsp_ratio();
  double get_goal_turn_angle();
  void Plan();
  struct XY get_zmp_ref(double t);
  MatrixXd get_Left_foot(double t);
  MatrixXd get_Right_foot(double t);
  double ellipse_traj(double present_time, double start_time, double T, double foot_Height);
  double ellipse_traj(double t, double T, double foot_Height);
  double Bezier_curve_8th(double time, double start_t, double T);
  int factorial(int n);


};



#endif // FOOTSTEP_PLANNER_H
*/
#ifndef FOOTSTEP_PLANNER_H
#define FOOTSTEP_PLANNER_H

#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include <deque>

#define PI 3.141593

using namespace std;

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;

struct XY
{
  double x;
  double y;
};


class FootstepPlanner
{
private:
  double walking_mode;
  double fb_step_size;
  double step_time;
  double step_num;
  int start_foot;
  double dsp_ratio;
  double goal_turn_angle;
  double foot_height;
  double foot_distance;
  //vector<vector<double>> FootSteps;

  int last_foot;
  double last_side_foot[3];

  double LHR_add_q;
  double RHR_add_q;
  double L_foot_y_adding;
  double R_foot_y_adding;

  double Hip_roll_add_q;
  double Foot_y_adding;
  double func_1_cos_param;
  bool two_feet_on_ground;
  double compensation_start_time_param;





public:
  vector<vector<double>> FootSteps;
  FootstepPlanner();
  FootstepPlanner(double fb_step_size_, double step_time_, double step_num_, int start_foot_, double dsp_ratio_, double goal_turn_angle_,double foot_height);

  void set_fb_step_size(double fb_step_size_);
  void set_step_time(double step_time_);
  void set_step_num(int step_num_);
  void set_start_foot(double start_foot);
  void set_dsp_ratio(double dsp_ratio);
  void set_goal_turn_angle(double goal_turn_angle_);

  double get_fb_step_size();
  double get_step_time();
  int get_step_num();
  double get_start_foot();
  double get_dsp_ratio();
  double get_goal_turn_angle();
  double get_LHR_add_q();
  double get_RHR_add_q();
  void Plan();
  struct XY get_zmp_ref(double t);
  double get_CoM_yaw(double t);
  MatrixXd get_Left_foot(double t);
  MatrixXd get_Right_foot(double t);
  double ellipse_traj(double present_time, double start_time, double T, double foot_Height);
  double ellipse_traj(double t, double T, double foot_Height);
  double Bezier_curve_8th(double time, double start_t, double T);
  double func_1_cos(double t, double start_t, double T, double max);
  int factorial(int n);


  /****************/

  double Fplanner_time = 0.00;
  double dt = 0.001;

  double max_fb_step;
  double max_rl_step;
  double max_rl_turn;

  double goal_fb_step;
  double goal_rl_step;
  double goal_rl_turn;

  double fb_step = -0.02;
  double rl_step = 0.03;//0.01;
  double rl_turn = 0.0;//PI/36;

  double unit_fb_step = 0.01;
  double unit_rl_step = 0.01;
  double unit_rl_turn = PI/36;

  double foot_index= 0; //left : 0, right : 1
  double t = 0.0;

  bool stop_flag = false;
  bool two_step_on_spot_to_go = false; //walk 2 step on spot to (stop -> go)

  MatrixXd foot_tf_global = MatrixXd::Identity(4,4);
  std::vector<double>pre_step;


  std::deque<std::vector<double>> footstep_deque;
  void update_footsteps(double t_sec);
  void init_deque(void);
  std::vector<double> get_step_vector(double del_x, double del_y, double turn_angle_radian, double foot_index);
  MatrixXd get_side_foot_tf(int present_foot_index, MatrixXd pre_step_tf);

  MatrixXd get_Left_foot2(double t);
  MatrixXd get_Right_foot2(double t);
  struct XY get_zmp_ref2(double t);
  double get_CoM_yaw2(double t);

  void update_step_size_param(void);


};



#endif // FOOTSTEP_PLANNER_H
