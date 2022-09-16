/*
 * RoK-3 Gazebo Simulation Code 
 * 
 * Robotics & Control Lab.
 * 
 * Master : BKCho
 * First developer : Yunho Han
 * Second developer : Minho Park
 * 
 * ======
 * Update date : 2022.03.16 by Yunho Han
 * ======
 */
//* Header file for C++
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <boost/bind.hpp>

#include <time.h>

#define X_ 0
#define Y_ 1
#define Z_ 2

//* Header file for Gazebo and Ros
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <functional>
#include <ignition/math/Vector3.hh>

//* Header file for RBDL and Eigen
#include <rbdl/rbdl.h> // Rigid Body Dynamics Library (RBDL)
#include <rbdl/addons/urdfreader/urdfreader.h> // urdf model read using RBDL
#include <Eigen/Dense> // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

#include "footstep_planner.h"
#include "read_yaml.h"

#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI

//Print color
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;


//RBDL//
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace std;

//* Angle Axis Struct ****
typedef struct _AngleAxis{
    Vector3d n;
    double th;
} AngleAxis; 


VectorXd q_cal2; //not using

//* Time Variables ****
double T = 3;        //sec
double t = T + 1.0;
double dt_ms = 1.0;  //ms

//* Phase Step ****
int phase = 0;

int go_straight_count = 0;
int turn_left_count = 0;

int walking_phase = 0;
bool is_running = true;

//* to check ik computing time
static struct timespec ik_start_time;
static struct timespec ik_end_time;

//* Position ****
Vector3d goal_posi_L;
Vector3d start_posi_L;
Vector3d command_posi_L;
Vector3d present_posi_L;

Vector3d goal_posi_R;
Vector3d start_posi_R;
Vector3d command_posi_R;
Vector3d present_posi_R;

//* Rotation ****
MatrixXd goal_rot_L(3,3);
MatrixXd start_rot_L(3,3);
MatrixXd present_rot_L(3,3);
MatrixXd command_rot_L(3,3);

MatrixXd goal_rot_R(3,3);
MatrixXd start_rot_R(3,3);
MatrixXd present_rot_R(3,3);
MatrixXd command_rot_R(3,3);

//* Joint Angle ****
VectorXd q_present_L(6);
VectorXd q_command_L(6);

VectorXd q_present_R(6);
VectorXd q_command_R(6);

//* Initial guess joint (IK) ****
VectorXd q0_L(6);
VectorXd q0_R(6);


//* etc ****
MatrixXd C_err_L; 
AngleAxis a_axis_L;

MatrixXd C_err_R; 
AngleAxis a_axis_R;

//* step
double fb_step = 0.1; //[m]
double rl_turn = 30;  //[deg]
double foot_height = 0.07; //[m]

double foot_distance = 0.105;  //distance from center


/*kinematic parameter*/
double L1 = 0.05;
double L2 = 0.0;
double L3 = 0.133;
double L4 = 0.138;
double L5 = 0.037;

/********************* Preview Control ************************/
/*
struct XY{
  double x;
  double y;
};
*/

double All_time_trajectory = 100;//15.0;  //(sec)
double dt = 0.001;  //sampling time
int N = 1000;  //preview NL
int n = (int)((double)All_time_trajectory/dt)+1;

double z_c = 0.22; //Height of CoM
double g = 9.81; //Gravity Acceleration

MatrixXd A(3,3);
MatrixXd B(3,1);
MatrixXd C(1,3);

int Qe = 1;
MatrixXd Qx(3,3);
Eigen::Matrix4d Q;
MatrixXd R(1,1); // 1*10^(-6)

Eigen::Vector4d BB;
MatrixXd II(4,1);
MatrixXd FF(4,3);
Eigen::Matrix4d AA;
MatrixXd KK;
MatrixXd Gi(1,1);
MatrixXd Gx;
VectorXd Gp(N);
MatrixXd XX;
MatrixXd AAc;

Eigen::EigenSolver<MatrixXd> eZMPSolver;

MatrixXd State_X(3,1);  // State(posi, vel, acc) for X_axis direction
MatrixXd State_Y(3,1);

struct XY input_u   = {.x=0, .y=0}; // control input jerk

struct XY zmp       = {.x=0, .y=0};
struct XY CoM       = {.x=0, .y=0};
struct XY zmp_error = {.x=0, .y=0}; // error or ZMP
struct XY sum_error = {.x=0, .y=0}; //sum of error
struct XY u_sum_p   = {.x=0, .y=0}; //sum or future Reference

struct XY get_zmp_ref(int step_time_index);
void set_system_model(void);
void set_Weight_Q(void);
void get_gain_G(void);
void initialize_CoM_State_Zero(void);
void initialize_starting_ZMP_Zero(void);
void Preview_Init_Setting(void);
MatrixXd ZMP_DARE(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, MatrixXd R);


int time_index = 0;

#include <vector>
std::vector<std::vector<double>> save;

/*****************************************************************/

//Foot step Planner
FootstepPlanner FootPlaner;
YAML_CONFIG_READER PD_gain_from_yaml;


namespace gazebo
{

    class rok3_plugin : public ModelPlugin
    {
        //*** Variables for RoK-3 Simulation in Gazebo ***//
        //* TIME variable
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        //* Model & Link & Joint Typedefs
        physics::ModelPtr model;

        physics::JointPtr L_Hip_yaw_joint;
        physics::JointPtr L_Hip_roll_joint;
        physics::JointPtr L_Hip_pitch_joint;
        physics::JointPtr L_Knee_joint;
        physics::JointPtr L_Ankle_pitch_joint;
        physics::JointPtr L_Ankle_roll_joint;

        physics::JointPtr R_Hip_yaw_joint;
        physics::JointPtr R_Hip_roll_joint;
        physics::JointPtr R_Hip_pitch_joint;
        physics::JointPtr R_Knee_joint;
        physics::JointPtr R_Ankle_pitch_joint;
        physics::JointPtr R_Ankle_roll_joint;
        physics::JointPtr torso_joint;

        physics::JointPtr LS, RS;

        //YAML_CONFIG_READER PD_gain_from_yaml;

        /* ROS */

        ros::NodeHandle nh;

        ros::Publisher LHY_pub;
        ros::Publisher LHR_pub;
        ros::Publisher LHP_pub;
        ros::Publisher LKN_pub;
        ros::Publisher LAP_pub;
        ros::Publisher LAR_pub;

        ros::Publisher test_pub;
        ros::Publisher test_pub2;
        ros::Publisher test_pub_L_z;
        ros::Publisher test_pub_R_z;

        ros::Publisher zmp_y_pub;
        ros::Publisher L_foot_z_pub;
        ros::Publisher R_foot_z_pub;
        ros::Publisher foot_x_pub;

        ros::Subscriber fb_step_sub;
        ros::Subscriber rl_step_sub;
        ros::Subscriber rl_turn_sub;
        ros::Subscriber is_stop_sub;

        std_msgs::Float64 LHY_msg;
        std_msgs::Float64 LHR_msg;
        std_msgs::Float64 LHP_msg;
        std_msgs::Float64 LKN_msg;
        std_msgs::Float64 LAP_msg;
        std_msgs::Float64 LAR_msg;

        std_msgs::Float64 test_msg;
        std_msgs::Float64 test_msg2;
        std_msgs::Float64 test_msg_L_z;
        std_msgs::Float64 test_msg_R_z;

        std_msgs::Float64 zmp_y_msg;
        std_msgs::Float64 L_foot_z_msg;
        std_msgs::Float64 R_foot_z_msg;
        std_msgs::Float64 foot_x_msg;



        //* Index setting for each joint
        
        enum
        {
            WST = 0, LHY, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR
        };

        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot

        typedef struct RobotJoint //Joint variable struct for joint control 
        {
            double targetDegree; //The target deg, [deg]
            double targetRadian; //The target rad, [rad]

            double targetVelocity; //The target vel, [rad/s]
            double targetTorque; //The target torque, [N·m]

            double actualDegree; //The actual deg, [deg]
            double actualRadian; //The actual rad, [rad]
            double actualVelocity; //The actual vel, [rad/s]
            double actualRPM; //The actual rpm of input stage, [rpm]
            double actualTorque; //The actual torque, [N·m]

            double Kp;
            double Ki;
            double Kd;

        } ROBO_JOINT;
        ROBO_JOINT* joint;

    public:
        //*** Functions for RoK-3 Simulation in Gazebo ***//
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
        void UpdateAlgorithm(); // Algorithm update while simulation
        void UpdateAlgorithm2();
        void UpdateAlgorithm3();

        void jointController(); // Joint Controller for each joint

        void GetJoints(); // Get each joint data from [physics::ModelPtr _model]
        void GetjointData(); // Get encoder data of each joint

        void initializeJoint(); // Initialize joint variables for joint control
        void SetJointPIDgain(); // Set each joint PID gain for joint control

        void fb_step_callback(const std_msgs::Float64::ConstPtr& msg);
        void rl_step_callback(const std_msgs::Float64::ConstPtr& msg);
        void rl_turn_callback(const std_msgs::Float64::ConstPtr& msg);
        void is_stop_callback(const std_msgs::Bool::ConstPtr& msg);
    };
    GZ_REGISTER_MODEL_PLUGIN(rok3_plugin);
}


/* get transform I0*/
//MatrixXd getTransformI0(){
//    
//   MatrixXd tmp_m(4,4);
//    
//    
//   tmp_m<<1,0,0,0,\
//         0,1,0,0,\
//         0,0,1,0,\
//         0,0,0,1;
//   
//   //tmp_m(0,0) = 1; tmp_m(0,1) = 0; tmp_m(0,2) = 0; tmp_m(0,3) = 0;
//   //tmp_m(0,0) = 1; tmp_m(0,1) = 0; tmp_m(0,2) = 0; tmp_m(0,3) = 0;
//   //tmp_m(0,0) = 1; tmp_m(0,1) = 0; tmp_m(0,2) = 0; tmp_m(0,3) = 0;
//   //tmp_m(0,0) = 1; tmp_m(0,1) = 0; tmp_m(0,2) = 0; tmp_m(0,3) = 0;
//    
//   return tmp_m;
//    
//}
//
//MatrixXd getTransform3E(){
//    
//    MatrixXd tmp_m(4,4);
//    int L3 = 1; //m
//    
//   tmp_m<<1,0,0,0,\
//         0,1,0,0,\
//         0,0,1,L3,\
//         0,0,0,1;
//    
//   return tmp_m; 
//    
//}
//
//MatrixXd jointToTransform01(VectorXd q){
//    
//    //q = generalized coordinates, a = [q1;q2;q3];
//    
//    MatrixXd tmp_m(4,4);
//    double qq = q(0);
//    int L0 = 1;
//    
//    double sq = sin(qq);
//    double cq = cos(qq);
//    
//   tmp_m<<cq, 0, sq, 0,\
//         0, 1,  0, 0,\
//        -sq, 0, cq, L0,\
//         0, 0, 0,  1;   
//    
//    
//    return tmp_m;
//}
//
//MatrixXd jointToTransform12(VectorXd q){
//    
//    //q = generalized coordinates, a = [q1;q2;q3];
//    
//    MatrixXd tmp_m(4,4);
//    double qq = q(1);
//    int L0 = 1;
//    
//    double sq = sin(qq);
//    double cq = cos(qq);
//    
//   tmp_m<<cq, 0, sq, 0,\
//         0, 1,  0, 0,\
//        -sq, 0, cq, L0,\
//         0, 0, 0,  1;   
//    
//    
//    return tmp_m;
//}
//
//MatrixXd jointToTransform23(VectorXd q){
//    
//    //q = generalized coordinates, a = [q1;q2;q3];
//    
//    MatrixXd tmp_m(4,4);
//    
//    double qq = q(2);
//    int L0 = 1;
//    
//    double sq = sin(qq);
//    double cq = cos(qq);
//    
//   tmp_m<<cq, 0, sq, 0,\
//         0, 1,  0, 0,\
//        -sq, 0, cq, L0,\
//         0, 0, 0,  1;   
//    
//    
//    return tmp_m;
//}
//
//VectorXd jointToPosition(VectorXd q){
//    MatrixXd TI0(4,4),T3E(4,4),T01(4,4),T12(4,4),T23(4,4),TIE(4,4);
//    TI0 = getTransformI0();
//    T3E = getTransform3E();
//    T01 = jointToTransform01(q);
//    T12 = jointToTransform12(q);
//    T23 = jointToTransform23(q);   
//    TIE = TI0*T01*T12*T23*T3E;
//    
//    Vector3d position;
//    //position(0) = TIE(3,0);
//   // position(1) = TIE(3,1);  
//    //position(2) = TIE(3,2);
//    position = TIE.block(0,3,3,1);
//    
//    return position;
//}
//
//MatrixXd jointToRotMat(VectorXd q){
//    MatrixXd TI0(4,4),T3E(4,4),T01(4,4),T12(4,4),T23(4,4),TIE(4,4);
//    TI0 = getTransformI0();
//    T3E = getTransform3E();
//    T01 = jointToTransform01(q);
//    T12 = jointToTransform12(q);
//    T23 = jointToTransform23(q);   
//    TIE = TI0*T01*T12*T23*T3E;
//    
//    MatrixXd rot_m(3,3);
//    rot_m<<TIE(0,0),TIE(0,1),TIE(0,2),\
//          TIE(1,0),TIE(1,1),TIE(1,2),\
//          TIE(2,0),TIE(2,1),TIE(2,2);
//    
//    return rot_m;
//    
//}
//
//VectorXd rotToEuler(MatrixXd rot_Mat){   //Euler ZYX
//    Vector3d euler_zyx = {0,0,0};
//    
//    euler_zyx(0) = atan2(rot_Mat(1,0),rot_Mat(0,0));
//    euler_zyx(1) = atan2(-rot_Mat(2,0),sqrt(pow(rot_Mat(2,1),2)+pow(rot_Mat(2,2),2)));
//    euler_zyx(2) = atan2(rot_Mat(2,1),rot_Mat(2,2));
//    
//    return euler_zyx;
//}
//




MatrixXd getTransformI0(){
    
   MatrixXd tmp_m(4,4);
    
   tmp_m<<1,0,0,0,\
          0,1,0,0,\
          0,0,1,0,\
          0,0,0,1;
   
   //tmp_m(0,0) = 1; tmp_m(0,1) = 0; tmp_m(0,2) = 0; tmp_m(0,3) = 0;
   //tmp_m(0,0) = 1; tmp_m(0,1) = 0; tmp_m(0,2) = 0; tmp_m(0,3) = 0;
   //tmp_m(0,0) = 1; tmp_m(0,1) = 0; tmp_m(0,2) = 0; tmp_m(0,3) = 0;
   //tmp_m(0,0) = 1; tmp_m(0,1) = 0; tmp_m(0,2) = 0; tmp_m(0,3) = 0;
    
   return tmp_m;
    
}

MatrixXd getTransform6E(){
    
    MatrixXd tmp_m(4,4);
    int L3 = 1; //m
    
    tmp_m<< 1,   0,   0,   0,\
            0,   1,   0,   0,\
            0,   0,   1,  -L5,\
            0,   0,   0,   1;   
    
   return tmp_m; 
}

MatrixXd jointToTransform01(VectorXd q){
    
    //q = generalized coordinates, a = [q1;q2;q3];
    
    MatrixXd tmp_m(4,4);
    double qq = q(0);
    //int L0 = 1;
    
    double sq = sin(qq);
    double cq = cos(qq);
    
   tmp_m<< cq, -sq,  0,  0,\
           sq,  cq,  0,  L1,\
            0,   0,  1, -L2,\
            0,   0,  0,  1;   

    return tmp_m;
}

MatrixXd jointToTransform01_R(VectorXd q){
    
    //q = generalized coordinates, a = [q1;q2;q3];
    
    MatrixXd tmp_m(4,4);
    double qq = q(0);
    //int L0 = 1;
    
    double sq = sin(qq);
    double cq = cos(qq);
    
   tmp_m<< cq, -sq,  0,  0,\
           sq,  cq,  0, -L1,\
            0,   0,  1, -L2,\
            0,   0,  0,  1;   
    
    return tmp_m;
}

MatrixXd jointToTransform12(VectorXd q){
    
    //q = generalized coordinates, a = [q1;q2;q3];
    
    MatrixXd tmp_m(4,4);
    double qq = q(1);
    //int L0 = 1;
    
    double sq = sin(qq);
    double cq = cos(qq);
    
   tmp_m<< 1,  0,   0, 0,\
           0, cq, -sq, 0,\
           0, sq,  cq, 0,\
           0,  0,   0, 1;   
    
    return tmp_m;
}

MatrixXd jointToTransform23(VectorXd q){
    
    //q = generalized coordinates, a = [q1;q2;q3];
    
    MatrixXd tmp_m(4,4);
    double qq = q(2);
    //int L0 = 1;
    
    double sq = sin(qq);
    double cq = cos(qq);
    
   tmp_m<< cq,  0,  sq, 0,\
            0,  1,   0, 0,\
          -sq,  0,  cq, 0,\
            0,  0,   0, 1;   
    
    return tmp_m;
}

MatrixXd jointToTransform34(VectorXd q){
    
    //q = generalized coordinates, a = [q1;q2;q3];
    
    MatrixXd tmp_m(4,4);
    double qq = q(3);
    //int L0 = 1;
    
    double sq = sin(qq);
    double cq = cos(qq);
    
   tmp_m<< cq,  0,  sq, 0,\
            0,  1,   0, 0,\
          -sq,  0,  cq, -L3,\
            0,  0,   0, 1;   
    
    return tmp_m;
}

MatrixXd jointToTransform45(VectorXd q){
    
    //q = generalized coordinates, a = [q1;q2;q3];
    
    MatrixXd tmp_m(4,4);
    double qq = q(4);
    //int L0 = 1;
    
    double sq = sin(qq);
    double cq = cos(qq);
    
   tmp_m<< cq,  0,  sq, 0,\
            0,  1,   0, 0,\
          -sq,  0,  cq, -L4,\
            0,  0,   0, 1;   
    
    return tmp_m;
}

MatrixXd jointToTransform56(VectorXd q){
    
    //q = generalized coordinates, a = [q1;q2;q3];
    
    MatrixXd tmp_m(4,4);
    double qq = q(5);
    //int L0 = 1;
    
    double sq = sin(qq);
    double cq = cos(qq);
    
   tmp_m<<   1,    0,   0,  0,\
             0,   cq, -sq,  0,\
             0,   sq,  cq,  0,\
             0,    0,   0,  1;   
    
    return tmp_m;
}

VectorXd jointToPosition_L(VectorXd q){
    MatrixXd TI0(4,4),T6E(4,4),T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4),TIE(4,4);
    TI0 = getTransformI0();
    T6E = getTransform6E();
    T01 = jointToTransform01(q);
    T12 = jointToTransform12(q);
    T23 = jointToTransform23(q);   
    T34 = jointToTransform34(q);  
    T45 = jointToTransform45(q);  
    T56 = jointToTransform56(q);  
    TIE = TI0*T01*T12*T23*T34*T45*T56*T6E;
    
    Vector3d position;

    position = TIE.block(0,3,3,1);
    
    return position;
}

VectorXd jointToPosition_R(VectorXd q){
    MatrixXd TI0(4,4),T6E(4,4),T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4),TIE(4,4);
    TI0 = getTransformI0();
    T6E = getTransform6E();
    T01 = jointToTransform01_R(q);
    T12 = jointToTransform12(q);
    T23 = jointToTransform23(q);   
    T34 = jointToTransform34(q);  
    T45 = jointToTransform45(q);  
    T56 = jointToTransform56(q);  
    TIE = TI0*T01*T12*T23*T34*T45*T56*T6E;
    
    Vector3d position;

    position = TIE.block(0,3,3,1);
    
    return position;
}

//
MatrixXd jointToRotMat_L(VectorXd q){
    MatrixXd TI0(4,4),T6E(4,4),T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4),TIE(4,4);
    TI0 = getTransformI0();
    T6E = getTransform6E();
    T01 = jointToTransform01(q);
    T12 = jointToTransform12(q);
    T23 = jointToTransform23(q);   
    T34 = jointToTransform34(q);  
    T45 = jointToTransform45(q);  
    T56 = jointToTransform56(q);  
    TIE = TI0*T01*T12*T23*T34*T45*T56*T6E;
    
    MatrixXd rot_m(3,3);
    rot_m<<TIE(0,0),TIE(0,1),TIE(0,2),\
          TIE(1,0),TIE(1,1),TIE(1,2),\
          TIE(2,0),TIE(2,1),TIE(2,2);
    
    return rot_m;
    
}

MatrixXd jointToRotMat_R(VectorXd q){
    MatrixXd TI0(4,4),T6E(4,4),T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4),TIE(4,4);
    TI0 = getTransformI0();
    T6E = getTransform6E();
    T01 = jointToTransform01_R(q);
    T12 = jointToTransform12(q);
    T23 = jointToTransform23(q);   
    T34 = jointToTransform34(q);  
    T45 = jointToTransform45(q);  
    T56 = jointToTransform56(q);  
    TIE = TI0*T01*T12*T23*T34*T45*T56*T6E;
    
    MatrixXd rot_m(3,3);
    rot_m<<TIE(0,0),TIE(0,1),TIE(0,2),\
          TIE(1,0),TIE(1,1),TIE(1,2),\
          TIE(2,0),TIE(2,1),TIE(2,2);
    
    return rot_m;
    
}
//
VectorXd rotToEuler(MatrixXd rot_Mat){   //Euler ZYX
    Vector3d euler_zyx = {0,0,0};
    
    euler_zyx(0) = atan2(rot_Mat(1,0),rot_Mat(0,0));
    euler_zyx(1) = atan2(-rot_Mat(2,0),sqrt(pow(rot_Mat(2,1),2)+pow(rot_Mat(2,2),2)));
    euler_zyx(2) = atan2(rot_Mat(2,1),rot_Mat(2,2));
    
    return euler_zyx;
}

MatrixXd jointToPosJac(VectorXd q)
{
    // Input: vector of generalized coordinates (joint angles)
    // Output: J_P, Jacobian of the end-effector translation which maps joint velocities to end-effector linear velocities in I frame.
    MatrixXd J_P = MatrixXd::Zero(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d r_I_I1, r_I_I2, r_I_I3, r_I_I4, r_I_I5, r_I_I6;
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1,n_I_2,n_I_3,n_I_4,n_I_5,n_I_6;
    Vector3d r_I_IE;


    //* Compute the relative homogeneous transformation matrices.
    T_I0 =  getTransformI0();
    T_01 = jointToTransform01(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);   
    T_34 = jointToTransform34(q);  
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();


    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0*T_01;
    T_I2 = T_I0*T_01*T_12;
    T_I3 = T_I0*T_01*T_12*T_23;
    T_I4 = T_I0*T_01*T_12*T_23*T_34;
    T_I5 = T_I0*T_01*T_12*T_23*T_34*T_45;
    T_I6 = T_I0*T_01*T_12*T_23*T_34*T_45*T_56;

    //* Extract the rotation matrices from each homogeneous transformation matrix. Use sub-matrix of EIGEN. https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);

    //* Extract the position vectors from each homogeneous transformation matrix. Use sub-matrix of EIGEN.
    r_I_I1 = T_I1.block(0,3,3,1);
    r_I_I2 = T_I2.block(0,3,3,1);
    r_I_I3 = T_I3.block(0,3,3,1);
    r_I_I4 = T_I4.block(0,3,3,1);
    r_I_I5 = T_I5.block(0,3,3,1);
    r_I_I6 = T_I6.block(0,3,3,1);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    //* Compute the unit vectors for the inertial frame I.
    n_I_1 = R_I1*n_1;
    n_I_2 = R_I2*n_2;
    n_I_3 = R_I3*n_3;
    n_I_4 = R_I4*n_4;
    n_I_5 = R_I5*n_5;
    n_I_6 = R_I6*n_6;

    //* Compute the end-effector position vector.

    MatrixXd T_IE(4,4);
    T_IE = T_I0*T_01*T_12*T_23*T_34*T_45*T_56*T_6E;

    r_I_IE = T_IE.block(0,3,3,1);

    //* Compute the translational Jacobian. Use cross of EIGEN.
    J_P.col(0) << n_I_1.cross(r_I_IE-r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE-r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE-r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE-r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE-r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE-r_I_I6);

    //std::cout << "Test, JP:" << std::endl << J_P << std::endl;

    return J_P;
}

MatrixXd jointToPosJac_R(VectorXd q)
{
    // Input: vector of generalized coordinates (joint angles)
    // Output: J_P, Jacobian of the end-effector translation which maps joint velocities to end-effector linear velocities in I frame.
    MatrixXd J_P = MatrixXd::Zero(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d r_I_I1, r_I_I2, r_I_I3, r_I_I4, r_I_I5, r_I_I6;
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1,n_I_2,n_I_3,n_I_4,n_I_5,n_I_6;
    Vector3d r_I_IE;


    //* Compute the relative homogeneous transformation matrices.
    T_I0 =  getTransformI0();
    T_01 = jointToTransform01_R(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);   
    T_34 = jointToTransform34(q);  
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();


    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0*T_01;
    T_I2 = T_I0*T_01*T_12;
    T_I3 = T_I0*T_01*T_12*T_23;
    T_I4 = T_I0*T_01*T_12*T_23*T_34;
    T_I5 = T_I0*T_01*T_12*T_23*T_34*T_45;
    T_I6 = T_I0*T_01*T_12*T_23*T_34*T_45*T_56;

    //* Extract the rotation matrices from each homogeneous transformation matrix. Use sub-matrix of EIGEN. https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);

    //* Extract the position vectors from each homogeneous transformation matrix. Use sub-matrix of EIGEN.
    r_I_I1 = T_I1.block(0,3,3,1);
    r_I_I2 = T_I2.block(0,3,3,1);
    r_I_I3 = T_I3.block(0,3,3,1);
    r_I_I4 = T_I4.block(0,3,3,1);
    r_I_I5 = T_I5.block(0,3,3,1);
    r_I_I6 = T_I6.block(0,3,3,1);

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    //* Compute the unit vectors for the inertial frame I.
    n_I_1 = R_I1*n_1;
    n_I_2 = R_I2*n_2;
    n_I_3 = R_I3*n_3;
    n_I_4 = R_I4*n_4;
    n_I_5 = R_I5*n_5;
    n_I_6 = R_I6*n_6;

    //* Compute the end-effector position vector.

    MatrixXd T_IE(4,4);
    T_IE = T_I0*T_01*T_12*T_23*T_34*T_45*T_56*T_6E;

    r_I_IE = T_IE.block(0,3,3,1);

    //* Compute the translational Jacobian. Use cross of EIGEN.
    J_P.col(0) << n_I_1.cross(r_I_IE-r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE-r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE-r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE-r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE-r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE-r_I_I6);

    //std::cout << "Test, JP:" << std::endl << J_P << std::endl;

    return J_P;
}


MatrixXd jointToRotJac(VectorXd q)
{
   // Input: vector of generalized coordinates (joint angles)
    // Output: J_R, Jacobian of the end-effector orientation which maps joint velocities to end-effector angular velocities in I frame.
    MatrixXd J_R(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1,n_I_2,n_I_3,n_I_4,n_I_5,n_I_6;

    //* Compute the relative homogeneous transformation matrices.
    T_I0 =  getTransformI0();
    T_01 = jointToTransform01(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);   
    T_34 = jointToTransform34(q);  
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();


    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0*T_01;
    T_I2 = T_I0*T_01*T_12;
    T_I3 = T_I0*T_01*T_12*T_23;
    T_I4 = T_I0*T_01*T_12*T_23*T_34;
    T_I5 = T_I0*T_01*T_12*T_23*T_34*T_45;
    T_I6 = T_I0*T_01*T_12*T_23*T_34*T_45*T_56;


    //* Extract the rotation matrices from each homogeneous transformation matrix.
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);


    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    n_I_1 = R_I1*n_1;
    n_I_2 = R_I2*n_2;
    n_I_3 = R_I3*n_3;
    n_I_4 = R_I4*n_4;
    n_I_5 = R_I5*n_5;
    n_I_6 = R_I6*n_6;

    //* Compute the translational Jacobian.
    J_R.col(0) << n_I_1;
    J_R.col(1) << n_I_2;
    J_R.col(2) << n_I_3;
    J_R.col(3) << n_I_4;
    J_R.col(4) << n_I_5;
    J_R.col(5) << n_I_6;


    //std::cout << "Test, J_R:" << std::endl << J_R << std::endl;

    return J_R;
}

MatrixXd jointToRotJac_R(VectorXd q)
{
   // Input: vector of generalized coordinates (joint angles)
    // Output: J_R, Jacobian of the end-effector orientation which maps joint velocities to end-effector angular velocities in I frame.
    MatrixXd J_R(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1,n_I_2,n_I_3,n_I_4,n_I_5,n_I_6;

    //* Compute the relative homogeneous transformation matrices.
    T_I0 =  getTransformI0();
    T_01 = jointToTransform01_R(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);   
    T_34 = jointToTransform34(q);  
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();


    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0*T_01;
    T_I2 = T_I0*T_01*T_12;
    T_I3 = T_I0*T_01*T_12*T_23;
    T_I4 = T_I0*T_01*T_12*T_23*T_34;
    T_I5 = T_I0*T_01*T_12*T_23*T_34*T_45;
    T_I6 = T_I0*T_01*T_12*T_23*T_34*T_45*T_56;


    //* Extract the rotation matrices from each homogeneous transformation matrix.
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);


    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;

    n_I_1 = R_I1*n_1;
    n_I_2 = R_I2*n_2;
    n_I_3 = R_I3*n_3;
    n_I_4 = R_I4*n_4;
    n_I_5 = R_I5*n_5;
    n_I_6 = R_I6*n_6;

    //* Compute the translational Jacobian.
    J_R.col(0) << n_I_1;
    J_R.col(1) << n_I_2;
    J_R.col(2) << n_I_3;
    J_R.col(3) << n_I_4;
    J_R.col(4) << n_I_5;
    J_R.col(5) << n_I_6;


    //std::cout << "Test, J_R:" << std::endl << J_R << std::endl;

    return J_R;
}

MatrixXd pseudoInverseMat(MatrixXd A, double lambda)
{
    // Input: Any m-by-n matrix
    // Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula
    MatrixXd pinvA;
    MatrixXd I;

    int m = A.rows();
    int n = A.cols();
    if(m>=n){
        I = MatrixXd::Identity(n,n);
        pinvA = ((A.transpose() * A + lambda*lambda*I).inverse())*A.transpose();
    }
    else if(m<n){
        I = MatrixXd::Identity(m,m);
        pinvA = A.transpose()*((A * A.transpose() + lambda*lambda*I).inverse());
    }

    return pinvA;
}

VectorXd rotMatToRotVec(MatrixXd C)
{
    // Input: a rotation matrix C
    // Output: the rotational vector which describes the rotation C
    Vector3d phi,n;
    double th;
    
    th = acos( (C(0,0) + C(1,1) + C(2,2) -1.0) / 2.0 );

    if(fabs(th)<0.001){
         n << 0,0,0;
    }
    else{
        n << (C(2,1) - C(1,2)), (C(0,2) - C(2,0)) , (C(1,0) - C(0,1)) ;
        n = (1.0 / (2.0*sin(th))) * n;
    }
        
    phi = th*n;
    
    return phi;
}

AngleAxis rotMatToAngleAxis(MatrixXd C)
{
    // Input: a rotation matrix C
    // Output: the rotational vector which describes the rotation C

    AngleAxis a_axis;

    Vector3d n;
    double th;
    
    th = acos( (C(0,0) + C(1,1) + C(2,2) -1.0) / 2.0 );

    if(fabs(th)<0.001){
         n << 0,0,0;
    }
    else{
        n << (C(2,1) - C(1,2)), (C(0,2) - C(2,0)) , (C(1,0) - C(0,1)) ;
        n = (1.0 / (2.0*sin(th))) * n;
    }
        
    //phi = th*n;
    a_axis.n = n;
    a_axis.th = th;
    
    return a_axis;
}

MatrixXd angleAxisToRotMat(AngleAxis angle_axis){
    Vector3d n = angle_axis.n;
    double th = angle_axis.th;
    MatrixXd C(3,3);

    if(fabs(th)<0.001){
        C<<1,0,0,\
           0,1,0,\
           0,0,1;
    }
    else{

        double nx = n(0);
        double ny = n(1);
        double nz = n(2);
        double s = sin(th);
        double c = cos(th);
        C<< nx*nx*(1.0-c)+c,     nx*ny*(1.0-c)-nz*s,   nx*nz*(1.0-c)+ny*s,\
            nx*ny*(1.0-c)+nz*s,  ny*ny*(1.0-c)+c,      ny*nz*(1.0-c)-nx*s, \
            nx*nz*(1.0-c)-ny*s,  ny*nz*(1.0-c)+nx*s,   nz*nz*(1.0-c)+c;
    }
    return C;
}              

MatrixXd EulerZyxToRotMat(double z_rot, double y_rot, double x_rot){
    MatrixXd Z_rot(3,3);
    MatrixXd Y_rot(3,3);
    MatrixXd X_rot(3,3);

    double cz = cos(z_rot);
    double cy = cos(y_rot);
    double cx = cos(x_rot);

    double sz = sin(z_rot);
    double sy = sin(y_rot);
    double sx = sin(x_rot);

    Z_rot << cz, -sz, 0,\
             sz, cz, 0,\
              0,  0, 1;
    Y_rot << cy, 0, sy, \
              0, 1,  0, \
             -sy, 0, cy;
    X_rot << 1, 0, 0,\
             0, cx, -sx,\
             0,  sx ,cx  ;
             
    return Z_rot*Y_rot*X_rot;

}

VectorXd inverseKinematics_L(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol)
{
    // Input: desired end-effector position, desired end-effector orientation, initial guess for joint angles, threshold for the stopping-criterion
    // Output: joint angles which match desired end-effector position and orientation

    clock_gettime(CLOCK_MONOTONIC, &ik_start_time);
    
    int num_it=0;
    MatrixXd J_P(3,6), J_R(3,6), J(6,6), pinvJ(6,6), C_err(3,3), C_IE(3,3);
    VectorXd q(6),dq(6),dXe(6);
    Vector3d dr, dph;
    double lambda;
    
    //* Set maximum number of iterations
    double max_it = 200;
    
    //* Initialize the solution with the initial guess
    q=q0;
    C_IE = jointToRotMat_L(q);
    C_err = C_des * C_IE.transpose();
    
    //* Damping factor
    lambda = 0.001;
    
    //* Initialize error
    dr = r_des - jointToPosition_L(q);
    dph =  rotMatToRotVec(C_err);
    dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
    
    ////////////////////////////////////////////////
    //** Iterative inverse kinematics
    ////////////////////////////////////////////////
    
    //* Iterate until terminating condition
    while (num_it<max_it && dXe.norm()>tol)
    {
        
        //Compute Inverse Jacobian
        J_P = jointToPosJac(q);
        J_R = jointToRotJac(q);

        J.block(0,0,3,6) = J_P;
        J.block(3,0,3,6) = J_R; // Geometric Jacobian
        
        // Convert to Geometric Jacobian to Analytic Jacobian
        dq = pseudoInverseMat(J,lambda)*dXe;
        
        // Update law
        q += 0.5*dq;
        
        // Update error
        C_IE = jointToRotMat_L(q);
        C_err = C_des * C_IE.transpose();
        
        dr = r_des - jointToPosition_L(q);
        dph = rotMatToRotVec(C_err);
        dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
                   
        num_it++;
    }
    clock_gettime(CLOCK_MONOTONIC, &ik_end_time);

    long nano_sec_dt = ik_end_time.tv_nsec - ik_start_time.tv_nsec;
    if(nano_sec_dt<0) nano_sec_dt += 1000000000;
    std::cout << "iteration: " << num_it << ", value: " << q << std::endl;
    std::cout<<"IK Dt(us) : "<<nano_sec_dt/1000<<std::endl;
    
    return q;
}

VectorXd inverseKinematics_R(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol)
{
    // Input: desired end-effector position, desired end-effector orientation, initial guess for joint angles, threshold for the stopping-criterion
    // Output: joint angles which match desired end-effector position and orientation

    clock_gettime(CLOCK_MONOTONIC, &ik_start_time);
    
    int num_it=0;
    MatrixXd J_P(3,6), J_R(3,6), J(6,6), pinvJ(6,6), C_err(3,3), C_IE(3,3);
    VectorXd q(6),dq(6),dXe(6);
    Vector3d dr, dph;
    double lambda;
    
    //* Set maximum number of iterations
    double max_it = 200;
    
    //* Initialize the solution with the initial guess
    q=q0;
    C_IE = jointToRotMat_R(q);
    C_err = C_des * C_IE.transpose();
    
    //* Damping factor
    lambda = 0.001;
    
    //* Initialize error
    dr = r_des - jointToPosition_R(q);
    dph =  rotMatToRotVec(C_err);
    dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
    
    ////////////////////////////////////////////////
    //** Iterative inverse kinematics
    ////////////////////////////////////////////////
    
    //* Iterate until terminating condition
    while (num_it<max_it && dXe.norm()>tol)
    {
        
        //Compute Inverse Jacobian
        J_P = jointToPosJac_R(q);
        J_R = jointToRotJac_R(q);

        J.block(0,0,3,6) = J_P;
        J.block(3,0,3,6) = J_R; // Geometric Jacobian
        
        // Convert to Geometric Jacobian to Analytic Jacobian
        dq = pseudoInverseMat(J,lambda)*dXe;
        
        // Update law
        q += 0.5*dq;
        
        // Update error
        C_IE = jointToRotMat_R(q);
        C_err = C_des * C_IE.transpose();
        
        dr = r_des - jointToPosition_R(q);
        dph = rotMatToRotVec(C_err);
        dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
                   
        num_it++;
    }
    clock_gettime(CLOCK_MONOTONIC, &ik_end_time);

    long nano_sec_dt = ik_end_time.tv_nsec - ik_start_time.tv_nsec;
    if(nano_sec_dt<0) nano_sec_dt += 1000000000;
    std::cout << "iteration: " << num_it << ", value: " << q << std::endl;
    std::cout<<"IK Dt(us) : "<<nano_sec_dt/1000<<std::endl;
    
    return q;
}


/* Preparing Robot control Practice*/
void Practice(void){
    MatrixXd TI0(4,4),T6E(4,4),T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4),TIE(4,4);
    Vector3d pos,euler;
    MatrixXd CIE(3,3);
    VectorXd q(6);
   // q = {10,20,30,40,50,60};
   // VectorXd q;
    q(0) = 10;
    q(1) = 20;
    q(2) = 30;
    q(3) = 40;
    q(4) = 50;
    q(5) = 60; 
   //q(0) = 0;
   //q(1) = 0;
   //q(2) = -30;
   //q(3) = 60;
   //q(4) = -30;
   //q(5) = 0; 

    /*
    Practice2 is finished. 
    */

    
    q = q*PI/180;

    //TI0 = getTransformI0();
    //T6E = getTransform6E();
    //T01 = jointToTransform01(q);
    //T12 = jointToTransform12(q);
    //T23 = jointToTransform23(q);   
    //T34 = jointToTransform34(q);  
    //T45 = jointToTransform45(q);  
    //T56 = jointToTransform56(q);  
//
    //TIE = TI0*T01*T12*T23*T34*T45*T56*T6E;
    //
    //pos = jointToPosition(q);
    //CIE = jointToRotMat(q);
    //euler = rotToEuler(CIE);
    //
    std::cout<<"hello_world"<<std::endl<<std::endl;
    //
    //
    //std::cout<<"TIE = "<<std::endl<<TIE<<std::endl;
    //
    //std::cout<<"Position = "<<std::endl<<pos<<std::endl;
    //std::cout<<"CIE = "<<std::endl<<CIE<<std::endl;
    //std::cout<<"Euler = "<<std::endl<<euler<<std::endl;
    //MatrixXd J_P = MatrixXd::Zero(3,6);
   // MatrixXd J_R(3,6);
   // J_P = jointToPosJac(q);
   // J_R = jointToRotJac(q);


  //  std::cout << "Test, JP:" << std::endl << J_P << std::endl;
   // std::cout << "Test, JR:" << std::endl << J_R << std::endl;
    //Practice3 was completed. 


    


   // MatrixXd J(6,6);
   // J << jointToPosJac(q),\
   //      jointToRotJac(q);
   //                
   // MatrixXd pinvj;
   // pinvj = pseudoInverseMat(J, 0.0);
//
   // MatrixXd invj;
   // invj = J.inverse();
//
   // std::cout<<" Test, Inverse"<<std::endl;
   // std::cout<< invj <<std::endl;
   // std::cout<<std::endl;
   // 
//
   // std::cout<<" Test, PseudoInverse"<<std::endl;
   // std::cout<< pinvj <<std::endl;
   // std::cout<<std::endl;
   // 
    VectorXd q_des(6),q_init(6);
    q_des = q;
    MatrixXd C_err(3,3), C_des(3,3), C_init(3,3);
//
    q_init = 0.5*q_des;
    C_des = jointToRotMat_L(q_des);
    C_init = jointToRotMat_L(q_init);
    C_err = C_des * C_init.transpose();
//
    VectorXd dph(3);
//
    dph = rotMatToRotVec(C_err);
   // 
   // std::cout<<" Test, Rotational Vector"<<std::endl;
    //std::cout<< dph <<std::endl;
    //std::cout<<std::endl;

    //Practice 4 was completed

/*
    Vector3d r_des = jointToPosition(q);
    MatrixXd C_des = jointToRotMat(q);
        
    VectorXd q_cal = inverseKinematics(r_des, C_des, q*0.5, 0.001);

    std::cout<<"IK result"<<std::endl;
    std::cout<<q_cal*57.2958<<std::endl;
*/

 //   Vector3d r_des = {0,0.105,-0.55};
 //  // r_des(0) = 0;
 //  // r_des(1) = 0.105;
 //  // r_des(2) = -0.55;
 //   //r_des<<0,0.105,-0.55;
//
 //   MatrixXd C_des(3,3);
 //   C_des<<   1,    0,   0,\
 //             0,    1,   0,\
 //             0,    0,   1;  
 //    
 //    VectorXd q_cal = inverseKinematics(r_des, C_des, q, 0.001);
 //    std::cout<<"IK result"<<std::endl;
 //   std::cout<<q_cal*57.2958<<std::endl;
//
 //   q_cal2 = q_cal;
//
 //   Vector3d rrr;
 //   rrr = jointToPosition(q_cal);
 //   



    //practice 5 was completed

  //q0 <<0, 0, -30, 60, -30, 0;
  // q0 = q0*D2R; 
}
double func_1_cos(double t, double init, double final, double T){

    // t : current time
    
    double des;
    des = (final - init)*0.5*(1.0 - cos(PI*(t/T))) + init;
    return des;
}

Vector3d func_1_cos(double t, Vector3d init, Vector3d final, double T){
    Vector3d des;
    des(0) = (final(0) - init(0))*0.5*(1.0 - cos(PI*(t/T))) + init(0);
    des(1) = (final(1) - init(1))*0.5*(1.0 - cos(PI*(t/T))) + init(1);
    des(2) = (final(2) - init(2))*0.5*(1.0 - cos(PI*(t/T))) + init(2);
    return des;
}
VectorXd func_1_cos(double t, VectorXd init, VectorXd final, double T){
    VectorXd des(6);
    des(0) = (final(0) - init(0))*0.5*(1.0 - cos(PI*(t/T))) + init(0);
    des(1) = (final(1) - init(1))*0.5*(1.0 - cos(PI*(t/T))) + init(1);
    des(2) = (final(2) - init(2))*0.5*(1.0 - cos(PI*(t/T))) + init(2);
    des(3) = (final(3) - init(3))*0.5*(1.0 - cos(PI*(t/T))) + init(3);
    des(4) = (final(4) - init(4))*0.5*(1.0 - cos(PI*(t/T))) + init(4);
    des(5) = (final(5) - init(5))*0.5*(1.0 - cos(PI*(t/T))) + init(5);
    return des;
}


void gazebo::rok3_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    
    /*
     * Loading model data and initializing the system before simulation 
     */

    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    //* [physics::ModelPtr model] based model update
    GetJoints();



    //* RBDL API Version Check
    int version_test;
    version_test = rbdl_get_api_version();
    printf(C_GREEN "RBDL API version = %d\n" C_RESET, version_test);

    //* model.urdf file based model data input to [Model* rok3_model] for using RBDL
    Model* rok3_model = new Model();
    //Addons::URDFReadFromFile("/home/ola/.gazebo/models/rok3_model/urdf/rok3_model.urdf", rok3_model, true, true);
    Addons::URDFReadFromFile("/home/ola/.gazebo/models/kubot22/urdf/kubot22.urdf", rok3_model, true, true);
    //↑↑↑ Check File Path ↑↑↑
    nDoF = rok3_model->dof_count - 6; // Get degrees of freedom, except position and orientation of the robot
    ROS_WARN("DOF : %d",nDoF);
    joint = new ROBO_JOINT[nDoF+1]; // Generation joint variables struct

    //* initialize and setting for robot control in gazebo simulation
    initializeJoint();
    SetJointPIDgain();


    //ROS Publishers
    LHY_pub = nh.advertise<std_msgs::Float64>("command_joint/LHY", 1000);
    LHR_pub = nh.advertise<std_msgs::Float64>("command_joint/LHR", 1000);
    LHP_pub = nh.advertise<std_msgs::Float64>("command_joint/LHP", 1000);
    LKN_pub = nh.advertise<std_msgs::Float64>("command_joint/LKN", 1000);
    LAP_pub = nh.advertise<std_msgs::Float64>("command_joint/LAP", 1000);
    LAR_pub = nh.advertise<std_msgs::Float64>("command_joint/LHR", 1000);

    test_pub = nh.advertise<std_msgs::Float64>("kubotsim/test", 1000);
    test_pub2 = nh.advertise<std_msgs::Float64>("kubotsim/test2", 1000);
    test_pub_L_z = nh.advertise<std_msgs::Float64>("kubotsim/CoM_y", 1000);
    test_pub_R_z = nh.advertise<std_msgs::Float64>("kubotsim/zmp_y", 1000);

    zmp_y_pub = nh.advertise<std_msgs::Float64>("kubotsim/zmp_y_ref", 1000);
    L_foot_z_pub = nh.advertise<std_msgs::Float64>("kubotsim/L_foot_z", 1000);
    R_foot_z_pub = nh.advertise<std_msgs::Float64>("kubotsim/R_foot_z", 1000);

    fb_step_sub = nh.subscribe("kubotsim/walk_param/fb_step", 1000, &gazebo::rok3_plugin::fb_step_callback,this);
    rl_step_sub = nh.subscribe("kubotsim/walk_param/rl_step", 1000, &gazebo::rok3_plugin::rl_step_callback,this);
    rl_turn_sub = nh.subscribe("kubotsim/walk_param/rl_turn", 1000, &gazebo::rok3_plugin::rl_turn_callback,this);
    is_stop_sub = nh.subscribe("kubotsim/walk_param/is_stop", 1000, &gazebo::rok3_plugin::is_stop_callback,this);

    /*** Initial setting for Preview Control ***/
    Preview_Init_Setting();



    //* setting for getting dt
    
    //last_update_time = model->GetWorld()->GetSimTime();
    #if GAZEBO_MAJOR_VERSION >= 8
        last_update_time = model->GetWorld()->SimTime();
    #else
        last_update_time = model->GetWorld()->GetSimTime();
    #endif

    //update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&rok3_plugin::UpdateAlgorithm, this));
    //update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&rok3_plugin::UpdateAlgorithm2, this));
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&rok3_plugin::UpdateAlgorithm3, this));
    
    Practice();

    FootPlaner.Plan();
    FootPlaner.init_deque();
/*
    FootstepPlanner f;
    f.Plan();
    int step_num = f.get_step_num();
    for(int i=0;i<f.FootSteps.size();i++){
      double x =   f.FootSteps[i][0];
      double y =   f.FootSteps[i][1];
      double yaw = f.FootSteps[i][2];
      printf("(%.4lf, %.4lf, %.4lf)\n",x,y,yaw);
    }
*/

    t = 0.0;

}

void gazebo::rok3_plugin::UpdateAlgorithm()
{
    /*
     * Algorithm update while simulation
     */

    //* UPDATE TIME : 1ms
    ///common::Time current_time = model->GetWorld()->GetSimTime();
    #if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = model->GetWorld()->SimTime();
    #else
        common::Time current_time = model->GetWorld()->GetSimTime();
    #endif

    dt = current_time.Double() - last_update_time.Double();
     //   cout << "dt:" << dt << endl;
    time = time + dt;
        cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;


    //* Read Sensors data
    GetjointData();


    //Practice 6-0
        /* [0;0;0;0;0;0] -> [0;0;-63.756;127.512;-63.756] */
    /*
        if(phase == 0 and time<T){
            joint[LHY].targetRadian = func_1_cos(time,0,D2R*0,T);
            joint[LHR].targetRadian = func_1_cos(time,0,D2R*0,T);
            joint[LHP].targetRadian = func_1_cos(time,0,D2R*-63.756,T);
            joint[LKN].targetRadian = func_1_cos(time,0,D2R*127.512,T);
            joint[LAP].targetRadian = func_1_cos(time,0,D2R*-63.756,T);
            joint[LAR].targetRadian = func_1_cos(time,0,D2R*0,T);
        }
        else if(phase == 0){
            phase ++;
            time = 0;
        }   
        else if(phase == 1 and time<T){
            joint[LHY].targetRadian = func_1_cos(time,0,D2R*0,T);
            joint[LHR].targetRadian = func_1_cos(time,0,D2R*0,T);
            joint[LHP].targetRadian = func_1_cos(time,D2R*-63.756,D2R*0,T);
            joint[LKN].targetRadian = func_1_cos(time,D2R*127.512,D2R*0,T);
            joint[LAP].targetRadian = func_1_cos(time,D2R*-63.756,D2R*0,T);
            joint[LAR].targetRadian = func_1_cos(time,0,D2R*0,T);
        }
        else if(phase == 1){
            phase --;
            time = 0;
        }
    */
    
    //Practice 6-1
    
    //
    //VectorXd q_zero(6);
    //q_zero<<0,0,0,0,0,0;
    //
    //
    //start_posi = jointToPosition(q_zero);
    //goal_posi<<0,0.105,-0.55;
    //command_rot<<1,0,0,\
    //             0,1,0,\
    //             0,0,1;
    //
    //if(time<T){
    //    command_posi = func_1_cos(time,start_posi, goal_posi, T);
    //    q_command = inverseKinematics(command_posi,command_rot,,0.001);
    //    q0 = q_command;
    //    joint[LHY].targetRadian = q_command(0);
    //    joint[LHR].targetRadian = q_command(1);
    //    joint[LHP].targetRadian = q_command(2);
    //    joint[LKN].targetRadian = q_command(3);
    //    joint[LAP].targetRadian = q_command(4);
    //    joint[LAR].targetRadian = q_command(5);
    //}
    //
    //
    
     



    if(phase == 0 and time>2){
        phase++;
        time = 0;
    }
    if(phase == 0){
        q_command_R<<0,0,0,0,0,0;
        q_command_L<<0,0,0,0,0,0;
        q_present_R<<0,0,0,0,0,0;
        q_present_L<<0,0,0,0,0,0;
        q0_R <<0, 0, -30, 60, -30, 0;
        q0_L <<0, 0, -30, 60, -30, 0;
        q0_R = q0_R*D2R;
        q0_L = q0_L*D2R;
    }
    
    /*
    
    if(phase == 1){
        //cout<<"phase 1"<<endl;
        q_present(0) = 0.0; //joint[LHY].actualRadian;
        q_present(1) = 0.0; //joint[LHR].actualRadian;
        q_present(2) = 0.0; //joint[LHP].actualRadian;
        q_present(3) = 0.0; //joint[LKN].actualRadian;
        q_present(4) = 0.0; //joint[LAP].actualRadian;
        q_present(5) = 0.0; //joint[LAR].actualRadian;
    
        present_posi = jointToPosition(q_present);
        present_rot = jointToRotMat(q_present);
    
        start_posi = goal_posi  = present_posi;  //초기화
        start_rot = goal_rot = present_rot;     //초기화
    
        q_command = q_present;
    
        q0 <<0, 0, -30, 60, -30, 0;
        q0 = q0*D2R;
    
        t = 0.0;
    
        phase++;
    
    }
    */
    else if(phase == 1){
        q_command_R(0) = func_1_cos(time,0,D2R*0,T);
        q_command_R(1) = func_1_cos(time,0,D2R*0,T);
        q_command_R(2) = func_1_cos(time,0,D2R*-63.756,T);
        q_command_R(3) = func_1_cos(time,0,D2R*127.512,T);
        q_command_R(4) = func_1_cos(time,0,D2R*-63.756,T);
        q_command_R(5) = func_1_cos(time,0,D2R*0,T);    
    
        q_command_L(0) = func_1_cos(time,0,D2R*0,T);
        q_command_L(1) = func_1_cos(time,0,D2R*0,T);
        q_command_L(2) = func_1_cos(time,0,D2R*-63.756,T);
        q_command_L(3) = func_1_cos(time,0,D2R*127.512,T);
        q_command_L(4) = func_1_cos(time,0,D2R*-63.756,T);
        q_command_L(5) = func_1_cos(time,0,D2R*0,T);  
    
        if(time>T){
           phase ++;
           //phase = 12;
            time = 0;
            //RIGHT LEG
            present_posi_R = jointToPosition_R(q_command_R);
            present_rot_R = jointToRotMat_R(q_command_R);
            start_posi_R = goal_posi_R = present_posi_R;
            start_rot_R = goal_rot_R = present_rot_R;
    
            goal_posi_R(Z_) -= 0.1;
            goal_rot_R = EulerZyxToRotMat(0*D2R, 0*D2R, 0*D2R);
    
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);
    
            //LEFT_LEG
            present_posi_L = jointToPosition_L(q_command_L);
            present_rot_L = jointToRotMat_L(q_command_L);
            start_posi_L = goal_posi_L = present_posi_L;
            start_rot_L = goal_rot_L = present_rot_L;
    
            goal_posi_L(Z_) -= 0.1;
            goal_rot_L = EulerZyxToRotMat(0*D2R, 0*D2R, 0*D2R);
    
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
    
        }
    }
    

   // C_err = goal_rot*start_rot.transpose();
   // a_axis = rotMatToAngleAxis(C_err);

    AngleAxis del_a_axis_L = a_axis_L; //초기화
    MatrixXd del_C_L;

    AngleAxis del_a_axis_R = a_axis_R; //초기화
    MatrixXd del_C_R;

    command_rot_L = start_rot_L;
    command_rot_R = start_rot_R;

    //EigenXd

    if(phase >= 2 and time <=T){
        //RIGHT LEG
        command_posi_R = func_1_cos(time,start_posi_R, goal_posi_R,T);
        del_a_axis_R.th = func_1_cos(time, 0.0 , a_axis_R.th, T);
        del_C_R = angleAxisToRotMat(del_a_axis_R);
        command_rot_R = start_rot_R*del_C_R;//goal_rot;//start_rot*del_C;
        q_command_R = inverseKinematics_R(command_posi_R, command_rot_R, q0_R, 0.001);       
        q0_R = q_command_R;

        //LEFT LEG
        command_posi_L = func_1_cos(time,start_posi_L, goal_posi_L,T);
        del_a_axis_L.th = func_1_cos(time, 0.0 , a_axis_L.th, T);
        del_C_L = angleAxisToRotMat(del_a_axis_L);
        command_rot_L = start_rot_L*del_C_L;//goal_rot;//start_rot*del_C;
        q_command_L = inverseKinematics_L(command_posi_L, command_rot_L, q0_L, 0.001);       
        q0_L = q_command_L;

    }
    else if(phase>=2 and time>T){
        if(phase == 2){   
            phase++;

            //RIGHT LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;
            //goal_posi<<0,0.105,-0.55;
            
            goal_posi_R(Y_) += (L1+0.02);
            goal_rot_R = EulerZyxToRotMat(0*D2R, 0*D2R, 0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);

            //LEFT LEG
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;
            //goal_posi<<0,0.105,-0.55;
            goal_posi_L(Y_) += (L1+0.02);
            goal_rot_L = EulerZyxToRotMat(0*D2R, 0*D2R, 0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
        }
        else if(phase == 3){  //3->4 left foot up
            phase ++;

            //RIGHT_LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;

            //goal_posi_R(Z_) -= 0.2;
            goal_rot_R = EulerZyxToRotMat(0, 0*D2R, 0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);

            //LEFT_LEG
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;
            //goal_posi<<0,0.105,-0.55;
            goal_posi_L(Z_) += foot_height;
            goal_posi_L(X_) += 0.5*fb_step;
            goal_rot_L = EulerZyxToRotMat(0, 0*D2R, 0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
        }
        else if(phase == 4){  //4->5 left foot down
            phase ++;

            //RIGHT_LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;
            //goal_posi(Z_) -= 0.2;
            goal_rot_R = EulerZyxToRotMat(0, 0*D2R, 0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);

            //LEFT_LEG
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;
            goal_posi_L(Z_) -= foot_height;
            goal_posi_L(X_) += 0.5*fb_step;
            goal_rot_L = EulerZyxToRotMat(0, 0*D2R, 0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
        }
        else if(phase == 5){
            phase ++;

            //RIGHT_LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;
            goal_posi_R(Y_) -= 2*(L1+0.02);
            goal_posi_R(X_) -= fb_step;
            goal_rot_R = EulerZyxToRotMat(0, 0*D2R, 0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);

            //LEFT_LEG
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;

            goal_posi_L(Y_) -= 2*(L1+0.02);
            goal_posi_L(X_) -= fb_step;
            goal_rot_L = EulerZyxToRotMat(0, 0*D2R, 0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
        }
        else if(phase == 6){  // 6->7 right foot up
            phase ++;

            //RIGHT_LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;

            goal_posi_R(Z_) += foot_height;
            goal_posi_R(X_) += 0.5*fb_step;

            goal_rot_R = EulerZyxToRotMat(0, 0*D2R, 0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);

            //LEFT_LEG
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;

            goal_rot_L = EulerZyxToRotMat(0, 0*D2R, 0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
        }
        else if(phase == 7){  // 7->8 right foot down
            phase ++;

            //RIGHT_LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;

            goal_posi_R(Z_) -= foot_height;
            goal_posi_R(X_) += 0.5*fb_step;

            goal_rot_R = EulerZyxToRotMat(0, 0*D2R, 0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);

            //LEFT_LEG
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;

            goal_rot_L = EulerZyxToRotMat(0, 0*D2R, 0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
        }
        else if(phase == 8){ 
            go_straight_count ++;
            if(walking_phase==0){
                if(go_straight_count>=3){
                    walking_phase++;
                    go_straight_count = 0;
                    phase = 12;
                }
                else{
                    phase = 2;
                }
            }
            else if(walking_phase == 2){
                if(go_straight_count>=2){
                    //walking_phase++;
                    //go_straight_count = 0;
                    //phase = 12;
                    //stop
                    
                    phase = 100;
                }
                else{
                    phase = 2;
                }                
            }

            

            //RIGHT_LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;    
            goal_posi_R(Y_) += (L1+0.02);
            goal_rot_R = EulerZyxToRotMat(0*D2R, 0*D2R, 0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);    
            //LEFT_LEG
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;    
            goal_posi_L(Y_) += (L1+0.02);
            goal_rot_L = EulerZyxToRotMat(0*D2R, 0*D2R, 0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
            
        }



// ************ Turn **************************

        else if(phase == 12){   
            phase++;

            //RIGHT LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;
            //goal_posi<<0,0.105,-0.55;
            
            goal_posi_R(Y_) += (L1+0.02);
            goal_rot_R = EulerZyxToRotMat(0*D2R, 0*D2R, 0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);

            //LEFT LEG
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;
            //goal_posi<<0,0.105,-0.55;
            goal_posi_L(Y_) += (L1+0.02);
            goal_rot_L = EulerZyxToRotMat(0*D2R, 0*D2R, 0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
        }
        else if(phase == 13){  //3->4 left foot up
            phase ++;

            //RIGHT_LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;

            //goal_posi_R(Z_) -= 0.2;
            goal_rot_R = EulerZyxToRotMat(-0.25*D2R*rl_turn, 0*D2R, 0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);

            //LEFT_LEG
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;
            //goal_posi<<0,0.105,-0.55;
            goal_posi_L(Z_) += foot_height;
            goal_rot_L = EulerZyxToRotMat(+1*0.25*rl_turn*D2R, 0*D2R,0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
        }
        else if(phase == 14){  //4->5 left foot down
            phase ++;

            //RIGHT_LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;
            //goal_posi(Z_) -= 0.2;
            goal_rot_R = EulerZyxToRotMat(-0.5*D2R*rl_turn, 0*D2R, 0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);

            //LEFT_LEG
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;
            goal_posi_L(Z_) -= foot_height;
            goal_rot_L = EulerZyxToRotMat(+1*0.5*rl_turn*D2R, 0*D2R,0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
        }
        else if(phase == 15){
            phase ++;

            //RIGHT_LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;

            goal_posi_R(Y_) += -1.0*2.0*(L1+0.02);
            
            //goal_rot_R = EulerZyxToRotMat(0, 0*D2R, 0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);

            //LEFT_LEG
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;

            goal_posi_L(Y_) += -1.0*2.0*(L1+0.02);

            //goal_rot_L = EulerZyxToRotMat(+1*rl_turn*D2R, 0*D2R,0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
        }
        else if(phase == 16){  // 6->7 right foot up
            phase ++;

            //RIGHT_LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;

            goal_posi_R(Z_) += foot_height;
            
            goal_rot_R = EulerZyxToRotMat(-0.25*D2R, 0*D2R,0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);

            //LEFT_LEG 
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;

            goal_rot_L = EulerZyxToRotMat(0.25*D2R, 0*D2R, 0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
        }
        else if(phase == 17){  // 7->8 right foot down
            phase ++;

            //RIGHT_LEG
            start_posi_R = goal_posi_R;
            start_rot_R = goal_rot_R;

            goal_posi_R(Z_) -= foot_height;

            goal_rot_R = EulerZyxToRotMat(0*D2R, 0*D2R,0*D2R);
            C_err_R = goal_rot_R*start_rot_R.transpose();
            a_axis_R = rotMatToAngleAxis(C_err_R);

            //LEFT_LEG
            start_posi_L = goal_posi_L;
            start_rot_L = goal_rot_L;

            goal_rot_L = EulerZyxToRotMat(0*D2R, 0*D2R, 0*D2R);
            C_err_L = goal_rot_L*start_rot_L.transpose();
            a_axis_L = rotMatToAngleAxis(C_err_L);
        }
        else if(phase == 18){  
            turn_left_count++;
            if(walking_phase == 1){
                if(turn_left_count ==3){
                    turn_left_count = 0;
                    walking_phase++;
                    phase = 2;
                }
                else
                    phase = 12;
            }
            if(is_running){
                //RIGHT_LEG
                start_posi_R = goal_posi_R;
                start_rot_R = goal_rot_R;
    
                goal_posi_R(Y_) += (L1+0.02);
    
                goal_rot_R = EulerZyxToRotMat(0*D2R, 0*D2R, 0*D2R);
                C_err_R = goal_rot_R*start_rot_R.transpose();
                a_axis_R = rotMatToAngleAxis(C_err_R);
    
                //LEFT_LEG
                start_posi_L = goal_posi_L;
                start_rot_L = goal_rot_L;
    
                goal_posi_L(Y_)  += (L1+0.02);
    
                goal_rot_L = EulerZyxToRotMat(0*D2R, 0*D2R, 0*D2R);
                C_err_L = goal_rot_L*start_rot_L.transpose();
                a_axis_L = rotMatToAngleAxis(C_err_L);
            }
        }
        else if(phase == 100)
            phase++;

  



        //start_posi = goal_posi;
        //start_rot = goal_rot;
        time = 0.0;
        //Stop Walking
        if(phase == 101)
            time = T;
    }

    //t+=dt_ms;

   static double max_yaw =0.0;

   cout<<"===== Phase ====="<<endl;
   cout<<phase<<endl;
   cout<<"==== Goal Position ===="<<endl;
   cout<<goal_posi_R<<endl;
   cout<<"===== q desired ====="<<endl;
   cout<<q_command_R*R2D<<endl;
   cout<<"====================="<<endl;
   cout<< "Max Yaw : "<<max_yaw<<endl;
   cout<<"dt : "<<dt<<endl;
   cout<<"====================="<<endl;

   if(abs(q_command_R(0)) > abs(max_yaw)){
       max_yaw = q_command_R(0);
   }

    
    //* Target Angles

    joint[LHY].targetRadian = q_command_L(0);//*D2R;
    joint[LHR].targetRadian = q_command_L(1);//*D2R;
    joint[LHP].targetRadian = q_command_L(2);//*D2R;
    joint[LKN].targetRadian = q_command_L(3);//*D2R;
    joint[LAP].targetRadian = q_command_L(4);//*D2R;
    joint[LAR].targetRadian = q_command_L(5);//*D2R;

    joint[RHY].targetRadian = q_command_R(0);//*D2R;
    joint[RHR].targetRadian = q_command_R(1);//*D2R;
    joint[RHP].targetRadian = q_command_R(2);//*D2R;
    joint[RKN].targetRadian = q_command_R(3);//*D2R;
    joint[RAP].targetRadian = q_command_R(4);//*D2R;
    joint[RAR].targetRadian = q_command_R(5);//*D2R;


    //* Publish topics
    LHY_msg.data = q_command_L(0);
    LHR_msg.data = q_command_L(1);
    LHP_msg.data = q_command_L(2);
    LKN_msg.data = q_command_L(3);
    LAP_msg.data = q_command_L(4);
    LAR_msg.data = q_command_L(5);

    LHY_pub.publish(LHY_msg);
    LHR_pub.publish(LHR_msg);
    LHP_pub.publish(LHP_msg);
    LKN_pub.publish(LKN_msg);
    LAP_pub.publish(LAP_msg);
    LAR_pub.publish(LAR_msg);
    



  /*First motion Complete.*/



    //* Joint Controller
    jointController();
}


void gazebo::rok3_plugin::jointController()
{
    /*
     * Joint Controller for each joint
     */

    // Update target torque by control
    for (int j = 0; j < nDoF+1; j++) {
        joint[j].targetTorque = joint[j].Kp * (joint[j].targetRadian-joint[j].actualRadian)\
                              + joint[j].Kd * (joint[j].targetVelocity-joint[j].actualVelocity);
    }

    // Update target torque in gazebo simulation     
   L_Hip_yaw_joint->SetForce(1, joint[LHY].targetTorque);
   L_Hip_roll_joint->SetForce(1, joint[LHR].targetTorque);
   L_Hip_pitch_joint->SetForce(1, joint[LHP].targetTorque);
   L_Knee_joint->SetForce(1, joint[LKN].targetTorque);
   L_Ankle_pitch_joint->SetForce(1, joint[LAP].targetTorque);
   L_Ankle_roll_joint->SetForce(1, joint[LAR].targetTorque);
   ///
   /// R_Hip_yaw_joint->SetForce(0, joint[RHY].targetTorque);
   /// R_Hip_roll_joint->SetForce(0, joint[RHR].targetTorque);
   /// R_Hip_pitch_joint->SetForce(0, joint[RHP].targetTorque);
   /// R_Knee_joint->SetForce(0, joint[RKN].targetTorque);
   /// R_Ankle_pitch_joint->SetForce(0, joint[RAP].targetTorque);
   /// R_Ankle_roll_joint->SetForce(0, joint[RAR].targetTorque);


    R_Hip_yaw_joint->SetForce(1, joint[RHY].targetTorque);
    R_Hip_roll_joint->SetForce(1, joint[RHR].targetTorque);
    R_Hip_pitch_joint->SetForce(1, joint[RHP].targetTorque);
    R_Knee_joint->SetForce(1, joint[RKN].targetTorque);
    R_Ankle_pitch_joint->SetForce(1, joint[RAP].targetTorque);
    R_Ankle_roll_joint->SetForce(1, joint[RAR].targetTorque);
    //torso_joint->SetForce(0, joint[WST].targetTorque);
    //ROS_WARN("torque: %lf",joint[RAR].targetTorque);
}

void gazebo::rok3_plugin::GetJoints()
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */

    //* Joint specified in model.sdf
  /*
    L_Hip_yaw_joint = this->model->GetJoint("L_Hip_yaw_joint");
    L_Hip_roll_joint = this->model->GetJoint("L_Hip_roll_joint");
    L_Hip_pitch_joint = this->model->GetJoint("L_Hip_pitch_joint");
    L_Knee_joint = this->model->GetJoint("L_Knee_joint");
    L_Ankle_pitch_joint = this->model->GetJoint("L_Ankle_pitch_joint");
    L_Ankle_roll_joint = this->model->GetJoint("L_Ankle_roll_joint");
    R_Hip_yaw_joint = this->model->GetJoint("R_Hip_yaw_joint");
    R_Hip_roll_joint = this->model->GetJoint("R_Hip_roll_joint");
    R_Hip_pitch_joint = this->model->GetJoint("R_Hip_pitch_joint");
    R_Knee_joint = this->model->GetJoint("R_Knee_joint");
    R_Ankle_pitch_joint = this->model->GetJoint("R_Ankle_pitch_joint");
    R_Ankle_roll_joint = this->model->GetJoint("R_Ankle_roll_joint");
    torso_joint = this->model->GetJoint("torso_joint");
*/
    L_Hip_yaw_joint = this->model->GetJoint("LP");
    L_Hip_roll_joint = this->model->GetJoint("LPm");
    L_Hip_pitch_joint = this->model->GetJoint("LPd");
    L_Knee_joint = this->model->GetJoint("LK");
    L_Ankle_pitch_joint = this->model->GetJoint("LA");
    L_Ankle_roll_joint = this->model->GetJoint("LF");
    R_Hip_yaw_joint = this->model->GetJoint("RP");
    R_Hip_roll_joint = this->model->GetJoint("RPm");
    R_Hip_pitch_joint = this->model->GetJoint("RPd");
    R_Knee_joint = this->model->GetJoint("RK");
    R_Ankle_pitch_joint = this->model->GetJoint("RA");
    R_Ankle_roll_joint = this->model->GetJoint("RF");
    //* FTsensor joint
   // LS = this->model->GetJoint("LS");
    //RS = this->model->GetJoint("RS");
}

void gazebo::rok3_plugin::GetjointData()
{
    /*
     * Get encoder and velocity data of each joint
     * encoder unit : [rad] and unit conversion to [deg]
     * velocity unit : [rad/s] and unit conversion to [rpm]
     */
    
  #if GAZEBO_MAJOR_VERSION >= 8

    
    joint[LHY].actualRadian = L_Hip_yaw_joint->Position(0);
    joint[LHR].actualRadian = L_Hip_roll_joint->Position(0);
    joint[LHP].actualRadian = L_Hip_pitch_joint->Position(0);
    joint[LKN].actualRadian = L_Knee_joint->Position(0);
    joint[LAP].actualRadian = L_Ankle_pitch_joint->Position(0);
    joint[LAR].actualRadian = L_Ankle_roll_joint->Position(0);

    joint[RHY].actualRadian = R_Hip_yaw_joint->Position(0);
    joint[RHR].actualRadian = R_Hip_roll_joint->Position(0);
    joint[RHP].actualRadian = R_Hip_pitch_joint->Position(0);
    joint[RKN].actualRadian = R_Knee_joint->Position(0);
    joint[RAP].actualRadian = R_Ankle_pitch_joint->Position(0);
    joint[RAR].actualRadian = R_Ankle_roll_joint->Position(0);

    //std::cout<<(joint[RKN].actualRadian)*R2D<<std::endl;

    //joint[WST].actualRadian = torso_joint->Position(0);

    
  #else
    joint[LHY].actualRadian = L_Hip_yaw_joint->GetAngle(0).Radian();
    joint[LHR].actualRadian = L_Hip_roll_joint->GetAngle(0).Radian();
    joint[LHP].actualRadian = L_Hip_pitch_joint->GetAngle(0).Radian();
    joint[LKN].actualRadian = L_Knee_joint->GetAngle(0).Radian();
    joint[LAP].actualRadian = L_Ankle_pitch_joint->GetAngle(0).Radian();
    joint[LAR].actualRadian = L_Ankle_roll_joint->GetAngle(0).Radian();

    joint[RHY].actualRadian = R_Hip_yaw_joint->GetAngle(0).Radian();
    joint[RHR].actualRadian = R_Hip_roll_joint->GetAngle(0).Radian();
    joint[RHP].actualRadian = R_Hip_pitch_joint->GetAngle(0).Radian();
    joint[RKN].actualRadian = R_Knee_joint->GetAngle(0).Radian();
    joint[RAP].actualRadian = R_Ankle_pitch_joint->GetAngle(0).Radian();
    joint[RAR].actualRadian = R_Ankle_roll_joint->GetAngle(0).Radian();

     ROS_WARN("ee");

    //joint[WST].actualRadian = torso_joint->GetAngle(0).Radian();
  #endif


    for (int j = 0; j < nDoF+1; j++) {
        joint[j].actualDegree = joint[j].actualRadian*R2D;
    }


    joint[LHY].actualVelocity = L_Hip_yaw_joint->GetVelocity(0);
    joint[LHR].actualVelocity = L_Hip_roll_joint->GetVelocity(0);
    joint[LHP].actualVelocity = L_Hip_pitch_joint->GetVelocity(0);
    joint[LKN].actualVelocity = L_Knee_joint->GetVelocity(0);
    joint[LAP].actualVelocity = L_Ankle_pitch_joint->GetVelocity(0);
    joint[LAR].actualVelocity = L_Ankle_roll_joint->GetVelocity(0);

    joint[RHY].actualVelocity = R_Hip_yaw_joint->GetVelocity(0);
    joint[RHR].actualVelocity = R_Hip_roll_joint->GetVelocity(0);
    joint[RHP].actualVelocity = R_Hip_pitch_joint->GetVelocity(0);
    joint[RKN].actualVelocity = R_Knee_joint->GetVelocity(0);
    joint[RAP].actualVelocity = R_Ankle_pitch_joint->GetVelocity(0);
    joint[RAR].actualVelocity = R_Ankle_roll_joint->GetVelocity(0);

    //joint[WST].actualVelocity = torso_joint->GetVelocity(0);


    //    for (int j = 0; j < nDoF; j++) {
    //        cout << "joint[" << j <<"]="<<joint[j].actualDegree<< endl;
    //    }

}

void gazebo::rok3_plugin::initializeJoint()
{
    /*
     * Initialize joint variables for joint control
     */
    
    for (int j = 0; j < nDoF+1; j++) {
        joint[j].targetDegree = 0;
        joint[j].targetRadian = 0;
        joint[j].targetVelocity = 0;
        joint[j].targetTorque = 0;
        
        joint[j].actualDegree = 0;
        joint[j].actualRadian = 0;
        joint[j].actualVelocity = 0;
        joint[j].actualRPM = 0;
        joint[j].actualTorque = 0;
    }
}

void gazebo::rok3_plugin::SetJointPIDgain()
{
   //YAML_CONFIG_READER PD_gain_from_yaml;
    PD_gain_from_yaml.getJoint_PD_gainFrom_yaml();
    /*
     * Set each joint PID gain for joint control
     */
/*
    joint[LHY].Kp = 130.0;
    joint[LHR].Kp = 270.0;
    joint[LHP].Kp = 260.0;
    joint[LKN].Kp = 170.0;
    joint[LAP].Kp = 180.0;
    joint[LAR].Kp = 170.0;

    joint[RHY].Kp = joint[LHY].Kp;
    joint[RHR].Kp = joint[LHR].Kp;
    joint[RHP].Kp = joint[LHP].Kp;
    joint[RKN].Kp = joint[LKN].Kp;
    joint[RAP].Kp = joint[LAP].Kp;
    joint[RAR].Kp = joint[LAR].Kp;
*/

    //joint[WST].Kp = 2* 2.;

/*
    joint[LHY].Kd =   0.8;
    joint[LHR].Kd =   1.8;
    joint[LHP].Kd =   1.7;
    joint[LKN].Kd =   1.0;
    joint[LAP].Kd =   1.0;
    joint[LAR].Kd =   0.5;

    joint[RHY].Kd = joint[LHY].Kd;
    joint[RHR].Kd = joint[LHR].Kd;
    joint[RHP].Kd = joint[LHP].Kd;
    joint[RKN].Kd = joint[LKN].Kd;
    joint[RAP].Kd = joint[LAP].Kd;
    joint[RAR].Kd = joint[LAR].Kd;
*/

   // joint[WST].Kd = 2.;

  joint[LHY].Kp = PD_gain_from_yaml.get_Kp(LHY);
  joint[LHR].Kp = PD_gain_from_yaml.get_Kp(LHR);
  joint[LHP].Kp = PD_gain_from_yaml.get_Kp(LHP);
  joint[LKN].Kp = PD_gain_from_yaml.get_Kp(LKN);
  joint[LAP].Kp = PD_gain_from_yaml.get_Kp(LAP);
  joint[LAR].Kp = PD_gain_from_yaml.get_Kp(LAR);

  joint[RHY].Kp = joint[LHY].Kp;
  joint[RHR].Kp = joint[LHR].Kp;
  joint[RHP].Kp = joint[LHP].Kp;
  joint[RKN].Kp = joint[LKN].Kp;
  joint[RAP].Kp = joint[LAP].Kp;
  joint[RAR].Kp = joint[LAR].Kp;

  joint[LHY].Kd = PD_gain_from_yaml.get_Kd(LHY);
  joint[LHR].Kd = PD_gain_from_yaml.get_Kd(LHR);
  joint[LHP].Kd = PD_gain_from_yaml.get_Kd(LHP);
  joint[LKN].Kd = PD_gain_from_yaml.get_Kd(LKN);
  joint[LAP].Kd = PD_gain_from_yaml.get_Kd(LAP);
  joint[LAR].Kd = PD_gain_from_yaml.get_Kd(LAR);

  joint[RHY].Kd = joint[LHY].Kd;
  joint[RHR].Kd = joint[LHR].Kd;
  joint[RHP].Kd = joint[LHP].Kd;
  joint[RKN].Kd = joint[LKN].Kd;
  joint[RAP].Kd = joint[LAP].Kd;
  joint[RAR].Kd = joint[LAR].Kd;

}


MatrixXd rotMat_X(double pitch){
  MatrixXd tmp(3,3);
  double s = sin(pitch);
  double c = cos(pitch);
  tmp<<1,  0,  0,
       0,  c, -s,
       0,  s,  s;
  return tmp;
}
MatrixXd rotMat_Y(double pitch){
  MatrixXd tmp(3,3);
  double s = sin(pitch);
  double c = cos(pitch);
  tmp<< c,  0,  s,
        0,  1,  0,
       -s,  0,  c;
  return tmp;
}
MatrixXd rotMat_Z(double pitch){
  MatrixXd tmp(3,3);
  double s = sin(pitch);
  double c = cos(pitch);
  tmp<< c, -s,  0,
        s,  c,  0,
        0,  0,  1;
  return tmp;
}


VectorXd IK_Geometric(MatrixXd Body,double D, double A,double B, double AH, MatrixXd Foot){
  Matrix3d Body_R, Foot_R;
  Vector3d Body_P, Foot_P;

  VectorXd q(6);

  Body_R = Body.block(0,0,3,3);
  Foot_R = Foot.block(0,0,3,3);

  Body_P = Body.block(0,3,3,1);
  Foot_P = Foot.block(0,3,3,1);

  Vector3d D_ = {0,D,0};
  Vector3d AH_ = {0,0,AH};

  Vector3d r = Foot_R.transpose() * ((Body_P + Body_R * D_) - (Foot_P + Foot_R*AH_));

  double C = r.norm();

  double cos_alpha = (A*A + B*B - C*C)/(2.0*A*B);
  double alpha;

  if(cos_alpha>=1) alpha = 0.0;
  else if(cos_alpha<=0.0) alpha = PI;
  else alpha = acos((A*A + B*B - C*C)/(2.0*A*B));

  double alpha2 = acos((C*C + B*B - A*A)/(2.0*A*C));

  double beta = atan2(r(Z_),r(Y_));
  q(3) = abs(PI - alpha); //Knee
  q(4) = atan2(r(Z_), r(X_)) - alpha2 - (PI/2.0); //ankle pitch
  q(5) = (PI/2.0) - beta; //ankle roll


  MatrixXd R(3,3);
  R = Body_R.transpose()*Foot_R*rotMat_X(-q(5))*rotMat_Y(-q(4))*rotMat_Y(-q(3));

  q(0) = atan2(-R(0,1), R(1,1)); //hip yaw
  q(1) = atan2(R(2,1), -R(0,1)*sin(q(0)) + R(1,1)*cos(q(0)) ); //hip roll
  q(2) = atan2(-R(2,0), R(2,2));  //hip pitch
  if(q(2)>PI/2) q(2) = q(2)-PI; //(ola)

 //cout<<"-----------"<<endl;
 //cout<<q*R2D<<endl;
 //cout<<"-----------"<<endl;
  return q;
}

void gazebo::rok3_plugin::UpdateAlgorithm2()
{
    /*
     * Algorithm update while simulation
     */

    //* UPDATE TIME : 1ms
    ///common::Time current_time = model->GetWorld()->GetSimTime();
    #if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = model->GetWorld()->SimTime();
    #else
        common::Time current_time = model->GetWorld()->GetSimTime();
    #endif

    dt = current_time.Double() - last_update_time.Double();
     //   cout << "dt:" << dt << endl;
    time = time + dt;
    //cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;


    //* Read Sensors data
   GetjointData();

    MatrixXd Body(4,4);
    MatrixXd Foot_L(4,4);
    MatrixXd Foot_R(4,4);

    double body_z = 0.5;//m
    double foot_z = 0.0; //m
    double foot_y = 0.105;//m
    double foot_x = 0.1;//m

    double step_length = 0.05;
    double step_height = 0.1;

    Body <<  1,0,0,      0,
             0,1,0,      0,
             0,0,1, body_z,
             0,0,0,      1;

    Foot_L <<1,0,0,       0,
             0,1,0, -foot_y,
             0,0,1,  foot_z,
             0,0,0,       1;

    Foot_R <<1,0,0,      0,
             0,1,0, foot_y,
             0,0,1, foot_z,
             0,0,0,      1;

    VectorXd q_L(6);
    VectorXd q_R(6);
    VectorXd init(6);
    init<<0,0,0,0,0,0;

    q_L = IK_Geometric(Body, -foot_y, 0.35, 0.35, 0.09, Foot_L);
    q_R = IK_Geometric(Body, foot_y, 0.35, 0.35, 0.09, Foot_R);

    //t = 0.0;

    if(t<T){
      if(phase == 0){
        q_command_L = func_1_cos(t,init,q_L,T);
        q_command_R = func_1_cos(t,init,q_R,T);
        t += dt;
      }
      else if(phase == 1){

        double Lfoot_x_traj = step_length*(cos(PI*(1.0-(t/T)))+1);
        double Lfoot_z_traj = step_height*sin(PI*(1.0-(t/T)));
        double Lfoot_yaw_traj = func_1_cos(t,0,PI/4,T);
        double Rfoot_yaw_traj = func_1_cos(t,0,-PI/2,T);
        double s = sin(Lfoot_yaw_traj);
        double c = cos(Lfoot_yaw_traj);
        double ss = sin(Rfoot_yaw_traj);
        double cc = cos(Rfoot_yaw_traj);
        Body <<  1,0,0,0,
                 0,1,0,0,
                 0,0,1,body_z,
                 0,0,0,1;

        Foot_L <<1, 0, 0, Lfoot_x_traj,
                 0,  c, -s, -foot_y,
                 0,  s,  c, Lfoot_z_traj,
                 0,0,0,1;
        Foot_R <<cc,  -ss, 0,-Lfoot_x_traj,
                 ss,  cc, 0,foot_y,
                 0,  0, 1,Lfoot_z_traj,
                 0,  0, 0,1;

        q_command_L = IK_Geometric(Body, -foot_y, 0.35, 0.35, 0.09, Foot_L);
        q_command_R = IK_Geometric(Body, foot_y, 0.35, 0.35, 0.09, Foot_R);
        t += dt;
     }
    }
     else{
      t = 0;
      phase++;
     }



    //q_command_L = q_L;
    //q_command_R = q_R;


    //* Target Angles

    joint[LHY].targetRadian = q_command_L(0);//*D2R;
    joint[LHR].targetRadian = q_command_L(1);//*D2R;
    joint[LHP].targetRadian = q_command_L(2);//*D2R;
    joint[LKN].targetRadian = q_command_L(3);//*D2R;
    joint[LAP].targetRadian = q_command_L(4);//*D2R;
    joint[LAR].targetRadian = q_command_L(5);//*D2R;

    joint[RHY].targetRadian = q_command_R(0);//*D2R;
    joint[RHR].targetRadian = q_command_R(1);//*D2R;
    joint[RHP].targetRadian = q_command_R(2);//*D2R;
    joint[RKN].targetRadian = q_command_R(3);//*D2R;
    joint[RAP].targetRadian = q_command_R(4);//*D2R;
    joint[RAR].targetRadian = q_command_R(5);//*D2R;


    //* Publish topics
    LHY_msg.data = q_command_L(0);
    LHR_msg.data = q_command_L(1);
    LHP_msg.data = q_command_L(2);
    LKN_msg.data = q_command_L(3);
    LAP_msg.data = q_command_L(4);
    LAR_msg.data = q_command_L(5);

    LHY_pub.publish(LHY_msg);
    LHR_pub.publish(LHR_msg);
    LHP_pub.publish(LHP_msg);
    LKN_pub.publish(LKN_msg);
    LAP_pub.publish(LAP_msg);
    LAR_pub.publish(LAR_msg);




  /*First motion Complete.*/



    //* Joint Controller
    jointController();
}


void gazebo::rok3_plugin::UpdateAlgorithm3()
{
    /*
     * Algorithm update while simulation
     */

    //* UPDATE TIME : 1ms
    ///common::Time current_time = model->GetWorld()->GetSimTime();
    #if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = model->GetWorld()->SimTime();
    #else
        common::Time current_time = model->GetWorld()->GetSimTime();
    #endif

    dt = current_time.Double() - last_update_time.Double();
     //   cout << "dt:" << dt << endl;
    time = time + dt;
    //cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;

    //MatrixXd leftfoot(4,4);
    //MatrixXd rightfoot(4,4);
    //leftfoot = FootPlaner.get_Left_foot(time);
    //rightfoot = FootPlaner.get_Right_foot(time);

    ///test_msg.data = leftfoot(0,3);
    ///test_msg2.data = rightfoot(0,3);
    ///test_msg_L_z.data = leftfoot(2,3);
    ///test_msg_R_z.data = rightfoot(2,3);
    ///test_pub.publish(test_msg);
    ///test_pub2.publish(test_msg2);
    ///test_pub_L_z.publish(test_msg_L_z);
    ///test_pub_R_z.publish(test_msg_R_z);
    ///


    //* Read Sensors data
   GetjointData();

    MatrixXd Body(4,4);
    MatrixXd Foot_L(4,4);
    MatrixXd Foot_R(4,4);

    double body_z = z_c;//0.5;//m
    double L_foot_z = 0.0; //m
    double R_foot_z = 0.0; //m
    double foot_y = L1;//m
    double foot_x = 0;//m

    double step_length = 0.05;
    double step_height = 0.1;

    if((phase == 1) /*&& time_index < n-N*/){ //use Preview Control

      FootPlaner.update_footsteps(time_index*dt);

      //zmp_error.x = zmp.x - FootPlaner.get_zmp_ref(time_index*dt).x;
      //zmp_error.y = zmp.y - FootPlaner.get_zmp_ref(time_index*dt).y;
      zmp_error.x = zmp.x - FootPlaner.get_zmp_ref2(time_index*dt).x;
      zmp_error.y = zmp.y - FootPlaner.get_zmp_ref2(time_index*dt).y;
      //printf("%lf\n",FootPlaner.get_zmp_ref2(time_index*dt).y);

      sum_error.x = sum_error.x + zmp_error.x;
      sum_error.y = sum_error.y + zmp_error.y;

      //printf("*****************\n");
      for(int j=0;j<N;j++){
        //u_sum_p.x = u_sum_p.x + Gp(j)*FootPlaner.get_zmp_ref(dt*(time_index + j + 1)).x;
        //u_sum_p.y = u_sum_p.y + Gp(j)*FootPlaner.get_zmp_ref(dt*(time_index + j + 1)).y;
        u_sum_p.x = u_sum_p.x + Gp(j)*FootPlaner.get_zmp_ref2(dt*(time_index + j + 1)).x;
        u_sum_p.y = u_sum_p.y + Gp(j)*FootPlaner.get_zmp_ref2(dt*(time_index + j + 1)).y;
        //printf("%lf\n",FootPlaner.get_zmp_ref2(dt*(time_index + j + 1)).y);
      }
      //printf("*****************\n");

      input_u.x = -Gi(0,0)*sum_error.x - (Gx*State_X)(0,0) - u_sum_p.x;
      input_u.y = -Gi(0,0)*sum_error.y - (Gx*State_Y)(0,0) - u_sum_p.y;

      //get CoM
      CoM.x = State_X(0,0);
      CoM.y = State_Y(0,0);

      //get new System State based on System state-space Eq
      State_X = A*State_X + B*input_u.x;
      State_Y = A*State_Y + B*input_u.y;

      //get ZMP based on System state-space Eq
      zmp.x = (C*State_X)(0,0);
      zmp.y = (C*State_Y)(0,0);

      u_sum_p.x = 0;
      u_sum_p.y = 0;

     }

    //std::cout<<"com_y : "<<CoM.y<<std::endl;




    //t = 0.0;
    if(phase == 0){
      if(t<T){
        Body <<  1,0,0,  CoM.x,
                 0,1,0,  CoM.y,
                 0,0,1, body_z,
                 0,0,0,      1;

        Foot_L <<1,0,0,         0,
                 0,1,0,    foot_y,
                 0,0,1,  L_foot_z,
                 0,0,0,         1;

        Foot_R <<1,0,0,      0,
                 0,1,0, -foot_y,
                 0,0,1, R_foot_z,
                 0,0,0,      1;

        VectorXd q_L(6);
        VectorXd q_R(6);
        VectorXd init(6);
        init<<0,0,0,0,0,0;

        q_L = IK_Geometric(Body, L1, L3, L4, L5, Foot_L);
        q_R = IK_Geometric(Body, -L1, L3, L4, L5, Foot_R);
        q_command_L = func_1_cos(t,init,q_L,T);
        q_command_R = func_1_cos(t,init,q_R,T);
        t += dt;
      }
      else {
        phase++;
        t = 0;

      }
    }
    else if(phase == 1){
      //std::cout<<"*******************"<<std::endl;
       //FootPlaner.update_footsteps(time_index*dt);
       //std::vector<double>a = FootPlaner.footstep_deque[0];
       ///std::vector<double>b = FootPlaner.footstep_deque[1];
       ///std::vector<double>c = FootPlaner.footstep_deque[2];
       //printf("deque : %lf %lf %lf %lf\n",a[0],a[1],a[2],a[3]);
       ///printf("deque : %lf %lf %lf %lf\n",b[0],b[1],b[2],b[3]);
       ///printf("deque : %lf %lf %lf %lf\n",c[0],c[1],c[2],c[3]);
       //XY zmp_ref2= FootPlaner.get_zmp_ref2(time_index*dt);
      /// printf("zmp_ref_2 (x): %lf (y):%lf \n",zmp_ref2.x, zmp_ref2.y);
       ///printf("deque_size : %d\n",FootPlaner.footstep_deque.size());
/*
      Body <<  1,0,0,  CoM.x,
               0,1,0,  CoM.y,
               0,0,1, body_z,
               0,0,0,      1;
*/

      //double CoM_yaw_= FootPlaner.get_CoM_yaw(time_index*dt);
      double CoM_yaw_= FootPlaner.get_CoM_yaw2(time_index*dt);
      //printf("com_yaw : %lf\n",CoM_yaw_*180/PI);
      double com_sin = sin(CoM_yaw_);
      double com_cos = cos(CoM_yaw_);

      Body <<  com_cos, -com_sin, 0,  CoM.x,
               com_sin,  com_cos, 0,  CoM.y,
                     0,        0, 1, body_z,
                     0,        0, 0,      1;

    // Foot_L <<1,0,0,         0,
    //          0,1,0,    foot_y,
    //          0,0,1,  L_foot_z,
    //          0,0,0,         1;
    //
    // Foot_R <<1,0,0,      0,
    //          0,1,0, -foot_y,
    //          0,0,1, R_foot_z,
    //          0,0,0,      1;

      //Foot_L = FootPlaner.get_Left_foot(time_index*dt);
      //Foot_R = FootPlaner.get_Right_foot(time_index*dt);
      Foot_L = FootPlaner.get_Left_foot2(time_index*dt);
      Foot_R = FootPlaner.get_Right_foot2(time_index*dt);

      //std::cout<<"Foot_L"<<std::endl;
      //std:cout<<Foot_L<<std::endl;

      test_msg_L_z.data = CoM.y; //Foot_R(0,3);
      test_msg_R_z.data = zmp.y;//Foot_R(1,3);

      test_pub_L_z.publish(test_msg_L_z);
      test_pub_R_z.publish(test_msg_R_z);

      zmp_y_msg.data = FootPlaner.get_zmp_ref2(time_index*dt).y;
      //zmp_y_msg.data = FootPlaner.get_zmp_ref(time_index*dt).y;
      L_foot_z_msg.data = Foot_L(2,3);
      R_foot_z_msg.data = Foot_R(2,3);

      L_foot_z_pub.publish(L_foot_z_msg);
      R_foot_z_pub.publish(R_foot_z_msg);
      zmp_y_pub.publish(zmp_y_msg);


      VectorXd q_L(6);
      VectorXd q_R(6);
      VectorXd init(6);
      init<<0,0,0,0,0,0;

      q_L = IK_Geometric(Body, L1, L3, L4, L5, Foot_L);
      q_R = IK_Geometric(Body, -L1, L3, L4, L5, Foot_R);

      if(time_index >= n-N){

        /*
        FILE *fp; // DH : for save the data
        fp = fopen("/home/ola/catkin_test_ws/src/Kubot_Sim_Pkg/src/data_save_ola/zmp_com_l_footup.txt", "w");
        for(int i=0;i<save.size();i++){
          if(i == 0){
            fprintf(fp,"time\t");
            fprintf(fp,"zmp_ref_y\t");
            fprintf(fp,"zmp_y\t");
            fprintf(fp,"COM_y\t");
            fprintf(fp,"r_Foot_z\n");
          }
          fprintf(fp, "%.4f\t", save[i][0]); //time
          fprintf(fp, "%.4f\t", save[i][1]); //zmp_ref_y
          fprintf(fp, "%.4f\t", save[i][2]); //zmp y
          fprintf(fp, "%.4f\t", save[i][3]); //CoM y
          fprintf(fp, "%.4f\n", save[i][4]); //r_foot_z
        }
        fclose(fp);
        */

        phase++;

      }
/*
      if(time_index*dt > 5){
        R_foot_z = func_1_cos(t,0,foot_height,T);
        Foot_R <<1,0,0,      0,
                 0,1,0, -foot_y,
                 0,0,1,  R_foot_z,
                 0,0,0,      1;
        q_R = IK_Geometric(Body, -L1, L3, L4, L5, Foot_R);
        if(t < T) t+=dt;
      }
*/

      std::vector<double> data;
      data.push_back(time_index*dt);
      data.push_back(get_zmp_ref(time_index).y);
      data.push_back(zmp.y);
      data.push_back(CoM.y);
      data.push_back(R_foot_z);
      save.push_back(data);
      /***** Time update *****/

      //printf("time index * dt : %lf\n",time_index*dt);
      //printf("fplanner time   : %lf\n",FootPlaner.Fplanner_time);
      //printf("error  r time   : %lf\n",FootPlaner.Fplanner_time - (time_index*dt));
      //printf("*************\n");

      time_index++;

      //FootPlaner.Fplanner_time+=FootPlaner.dt;
      q_command_L = q_L;
      q_command_R = q_R;
    }
    else if(phase = 2){

    }
    //FootPlaner.update_footsteps(time_index*dt);



    //q_command_L = q_L;
    //q_command_R = q_R;


    //* Target Angles

    joint[LHY].targetRadian = q_command_L(0);//*D2R;
    joint[LHR].targetRadian = q_command_L(1);//*D2R;
    joint[LHP].targetRadian = q_command_L(2);//*D2R;
    joint[LKN].targetRadian = q_command_L(3);//*D2R;
    joint[LAP].targetRadian = q_command_L(4);//*D2R;
    joint[LAR].targetRadian = q_command_L(5);//*D2R;

    joint[RHY].targetRadian = q_command_R(0);//*D2R;
    joint[RHR].targetRadian = q_command_R(1);//*D2R;
    joint[RHP].targetRadian = q_command_R(2);//*D2R;
    joint[RKN].targetRadian = q_command_R(3);//*D2R;
    joint[RAP].targetRadian = q_command_R(4);//*D2R;
    joint[RAR].targetRadian = q_command_R(5);//*D2R;


    //* Publish topics
    LHY_msg.data = q_command_L(0);
    LHR_msg.data = q_command_L(1);
    LHP_msg.data = q_command_L(2);
    LKN_msg.data = q_command_L(3);
    LAP_msg.data = q_command_L(4);
    LAR_msg.data = q_command_L(5);

    LHY_pub.publish(LHY_msg);
    LHR_pub.publish(LHR_msg);
    LHP_pub.publish(LHP_msg);
    LKN_pub.publish(LKN_msg);
    LAP_pub.publish(LAP_msg);
    LAR_pub.publish(LAR_msg);




  /*First motion Complete.*/



    //* Joint Controller
    jointController();
}

struct XY get_zmp_ref(int step_time_index){
  struct XY zmp_ref;

  double time = dt*step_time_index;
  /*
  if(time<3){
    zmp_ref.x = 0;
    zmp_ref.y = 0;
  }
  else if(time<3.5){
    zmp_ref.x  = 0;
    zmp_ref.y = L1;
  }
  else if(time<4){
    zmp_ref.x  = 0;
    zmp_ref.y = -L1;
  }
  else if(time<4.5){
    zmp_ref.x  = 0;
    zmp_ref.y = L1;
  }
  else if(time<5){
    zmp_ref.x  = 0;
    zmp_ref.y = -L1;
  }
  else if(time<5.5){
    zmp_ref.x  = 0;
    zmp_ref.y = L1;
  }
  else if(time<6){
    zmp_ref.x  = 0;
    zmp_ref.y = -L1;
  }
  else if(time<6.5){
    zmp_ref.x  = 0;
    zmp_ref.y = L1;
  }
  else if(time<7){
    zmp_ref.x  = 0;
    zmp_ref.y = -L1;
  }
  else if(time<7.5){
    zmp_ref.x  = 0;
    zmp_ref.y = L1;
  }
  else if(time<8){
    zmp_ref.x  = 0;
    zmp_ref.y = -L1;
  }
  else{
    zmp_ref.x = 0;
    zmp_ref.y = 0;
  }
  */

  zmp_ref.x = 0;
  zmp_ref.y = 0;;
  if(time>5){
    zmp_ref.x = 0;
    zmp_ref.y = L1;
  }

  return zmp_ref;

}


void set_system_model(void){
  A << 1, dt, dt*dt/2,\
       0, 1,      dt,\
       0, 0,       1;

  B << dt*dt*dt/6,\
       dt*dt/2,\
       dt;
  C << 1, 0, -z_c/g;

}

void set_Weight_Q(void){
  Qe = 1;

  Qx << 0,0,0,\
        0,0,0,\
        0,0,0;

  Q(0,0) = Qe; Q.block(0,1,1,3) = MatrixXd::Zero(1,3);
  Q.block(1,0,3,1) = MatrixXd::Zero(3,1); Q.block(1,1,3,3)= Qx;

  R << 0.000001;
}

void initialize_CoM_State_Zero(void){
   State_X<<0,0,0;
   State_Y<<0,0,0;
}
void initialize_starting_ZMP_Zero(void){
   zmp.x = 0;
   zmp.y = 0;
}

void get_gain_G(void){

  BB << (C*B)(0,0),\
        B(0,0),\
        B(1,0),\
        B(2,0);

  II <<1,\
       0,\
       0,\
       0;

  FF.block(0,0,1,3) = C*A;
  FF.block(1,0,3,3) = A;

  AA.block(0,0,4,1) = II;
  AA.block(0,1,4,3) = FF;

  KK = ZMP_DARE(AA,BB,Q,R);

  Gi = (R + (BB.transpose()*KK*BB)).inverse()*(BB.transpose()*KK*II);
  Gx = (R + (BB.transpose()*KK*BB)).inverse()*(BB.transpose()*KK*FF);
  AAc = AA - BB*((R + (BB.transpose()*KK*BB)).inverse())*BB.transpose()*KK*AA;


  for(int i=0;i<N;i++){
    if(i==0){
      XX = -AAc.transpose()*KK*II;
      Gp(i) = -Gi(0,0);
    }
    else{
      Gp(i) = (((R + (BB.transpose()*KK*BB)).inverse())*(BB.transpose()*XX))(0,0);
      XX = AAc.transpose()*XX;
    }


    /** this is for data save **/
    FILE *fp2; // DH : for save the data
    fp2 = fopen("/home/ola/catkin_test_ws/src/Kubot_Sim_Pkg/src/data_save_ola/Gp.txt", "w");
    fprintf(fp2, "index\t");
    fprintf(fp2, "Gp\n");
    for(int i=0;i<N;i++){
      fprintf(fp2,"%d\t",i);
      fprintf(fp2, "%.4f\n", Gp(i)); //time
    }
    fclose(fp2);

  }

}

void Preview_Init_Setting(void){
  set_system_model();
  set_Weight_Q();;
  get_gain_G();
  initialize_CoM_State_Zero();
  initialize_starting_ZMP_Zero();
}

MatrixXd ZMP_DARE(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, MatrixXd R)//Kookmin.Univ Preview
{
    unsigned int nSize = A.rows();
    MatrixXd Z(nSize* 2, nSize* 2);

    Z.block(0, 0, nSize, nSize) = A+ B* R.inverse()* B.transpose()* (A.inverse()).transpose() * Q;
    Z.block(0, nSize, nSize, nSize) = -B* R.inverse()* B.transpose()* (A.inverse()).transpose();
    Z.block(nSize, 0, nSize, nSize) = -(A.inverse()).transpose()* Q;
    Z.block(nSize, nSize, nSize, nSize) = (A.inverse()).transpose();

    eZMPSolver.compute(Z, true);

    Eigen::MatrixXcd U(nSize* 2, nSize);
    unsigned int j=0;
    for (unsigned int i=0; i<nSize* 2; i++)
    {
        std::complex<double> eigenvalue = eZMPSolver.eigenvalues()[i];
        double dReal = eigenvalue.real();
        double dImag = eigenvalue.imag();

        if( std::sqrt((dReal* dReal) + (dImag* dImag)) < 1.0)
        {
            U.block(0, j, nSize* 2, 1) = eZMPSolver.eigenvectors().col(i);
            j++;
        }
    }
    if(j != nSize)
    {
        printf("Warning! ******* Pelvis Planning *******\n");
    }

    Eigen::MatrixXcd U1 = U.block(0, 0, nSize, nSize);
    Eigen::MatrixXcd U2 = U.block(nSize, 0, nSize, nSize);

    Eigen::MatrixXcd X = U2 * U1.inverse();

    return X.real();
}



/***** ROS callback functions ***/
void gazebo::rok3_plugin::fb_step_callback(const std_msgs::Float64::ConstPtr& msg)
{
    FootPlaner.fb_step = msg->data;
}

void gazebo::rok3_plugin::rl_step_callback(const std_msgs::Float64::ConstPtr& msg)
{
    FootPlaner.rl_step = msg->data;
}

void gazebo::rok3_plugin::rl_turn_callback(const std_msgs::Float64::ConstPtr& msg)
{
    FootPlaner.rl_turn = msg->data;
}

void gazebo::rok3_plugin::is_stop_callback(const std_msgs::Bool::ConstPtr& msg)
{
    FootPlaner.stop_flag = msg->data;
}
