#include "footstep_planner.h"

FootstepPlanner::FootstepPlanner()
  : fb_step_size(0.03), //m
    step_time(1.4),    //sec
    step_num(10),      //step number
    start_foot(0),     //left : 0 right : 1
    dsp_ratio(0.3),
    goal_turn_angle(0), //radian
    foot_height(0.05),     //m
    foot_distance(0.1) //m
{

}

FootstepPlanner::FootstepPlanner(double fb_step_size_, double step_time_, double step_num_, int start_foot_, double dsp_ratio_, double goal_turn_angle_, double foot_height){
  fb_step_size = fb_step_size_;
  step_time = step_time_;
  step_num = step_num_;
  start_foot = start_foot_;
  dsp_ratio = dsp_ratio_;
  goal_turn_angle = goal_turn_angle_;
}

void FootstepPlanner::set_fb_step_size(double fb_step_size_){ fb_step_size = fb_step_size_; }
void FootstepPlanner::set_step_time(double step_time_){ step_time = step_time_; }
void FootstepPlanner::set_step_num(int step_num_){step_num = step_num_;}
void FootstepPlanner::set_start_foot(double start_foot_){ start_foot = start_foot_; }
void FootstepPlanner::set_dsp_ratio(double dsp_ratio_){ dsp_ratio = dsp_ratio_; }
void FootstepPlanner::set_goal_turn_angle(double goal_turn_angle_){ goal_turn_angle = goal_turn_angle_; }

double FootstepPlanner::get_fb_step_size(){ return fb_step_size; }
double FootstepPlanner::get_step_time(){ return step_time; }
int FootstepPlanner::get_step_num(){return step_num;}
double FootstepPlanner::get_start_foot(){ return start_foot; }
double FootstepPlanner::get_dsp_ratio(){ return dsp_ratio; }
double FootstepPlanner::get_goal_turn_angle(){ return goal_turn_angle; }

void FootstepPlanner::Plan(){

  int next_Foot_index = start_foot; //0 : Left, 1 : Right
  MatrixXd GlobalOrigin;(4,4);
  MatrixXd LocalFootTF(4,4);
  MatrixXd GlobalFootTF(4,4);
  GlobalOrigin = MatrixXd::Identity(4,4);
  GlobalFootTF = GlobalOrigin;
  LocalFootTF = MatrixXd::Identity(4,4);

  //NextFoot = MatrixXd::Identity(4,4);

  //double foot_distance = 0.1;

  double Del_X,Del_Y;
  double unit_turn_angle = goal_turn_angle/step_num;

  Eigen::Matrix3d unit_turn_RotMat;
  double cc,ss;
  cc = cos(unit_turn_angle); ss = sin(unit_turn_angle);
  unit_turn_RotMat<<cc,-ss, 0,\
                    ss, cc, 0,\
                     0,  0, 1;

  for(int i=0;i< step_num;i++){
    if(i == 0){
      Del_X = 0.0;
      if(next_Foot_index == 0)
        Del_Y = foot_distance/2.0;
      else
        Del_Y = - foot_distance/2.0;
      LocalFootTF.block(0,0,3,3) = MatrixXd::Identity(3,3);
      LocalFootTF(0,3) = Del_X; LocalFootTF(1,3)= Del_Y;
      GlobalFootTF = GlobalFootTF*LocalFootTF;
    }
    else{
      if(next_Foot_index == 0){ //left foot
          Del_X = fb_step_size - (foot_distance/2.0)*sin(unit_turn_angle);
          Del_Y = +foot_distance-(foot_distance/2.0)*(1-cos(unit_turn_angle));
      }
      else{ //right foot
          Del_X = fb_step_size + (foot_distance/2.0)*sin(unit_turn_angle);
          Del_Y = -foot_distance+(foot_distance/2.0)*(1-cos(unit_turn_angle));
      }
      LocalFootTF.block(0,0,3,3) = unit_turn_RotMat;
      LocalFootTF(0,3) = Del_X; LocalFootTF(1,3)= Del_Y;
      GlobalFootTF = GlobalFootTF*LocalFootTF;
      //std::cout<<"GlobaTF"<<std::endl;
      //std::cout<<GlobalFootTF<<std::endl;
    }
    double global_foot_X = GlobalFootTF(0,3);
    double global_foot_Y = GlobalFootTF(1,3);
    double global_foot_yaw = atan2(GlobalFootTF(0,1),GlobalFootTF(0,0));
    vector<double> one_foot_step;
    //std::cout<<"flll"<<std::endl;
    //std::cout<<global_foot_X<<" "<<global_foot_Y<<" "<<global_foot_yaw<<std::endl;
    one_foot_step.push_back(global_foot_X);
    one_foot_step.push_back(global_foot_Y);
    one_foot_step.push_back(global_foot_yaw);

    FootSteps.push_back(one_foot_step);
    next_Foot_index = (next_Foot_index == 1)?0:1;
  }


}


struct XY FootstepPlanner::get_zmp_ref(double t){

  struct XY zmp_ref;
  double preview_start_wait_time = 1.5;

  if(t < preview_start_wait_time){  //wait when start for preview
    zmp_ref.x = 0.0;
    zmp_ref.y = 0.0;
  }
  else{
    t = t-preview_start_wait_time;
    int step_index_n = (int)(t/step_time + 0.0001);
    if(step_index_n<step_num){
      zmp_ref.x = FootSteps[step_index_n][0];  //assume that zmp_ref = foot_center_point
      zmp_ref.y = FootSteps[step_index_n][1];
    }
    else{
      zmp_ref.x = FootSteps[step_num-1][0];
      zmp_ref.y = FootSteps[step_num-1][1];
    }
  }
  return zmp_ref;
}

MatrixXd FootstepPlanner::get_Left_foot(double t){

  double preview_start_wait_time = 1.5;

  MatrixXd LeftFoot(4,4);
  LeftFoot = MatrixXd::Identity(4,4);
  double global_x, global_y,global_z, global_yaw;
  if(t < preview_start_wait_time){  //wait when start for preview
    global_x = 0.0;
    global_y = 0.05; // foot_distance /2
    global_z = 0.0;
    global_yaw = 0;
  }
  else{

    t = t-preview_start_wait_time;
    int step_index_n = (int)(t/step_time + 0.001);
    //std::cout<<"step_index : "<<step_index_n<<std::endl;
    if(step_index_n<=step_num-1){

      if((start_foot == 0 and step_index_n%2 == 0) or (start_foot == 1 and step_index_n%2 == 1)){ //when left foot is supprot foot
        global_x   = FootSteps[step_index_n][0];
        global_y   = FootSteps[step_index_n][1];
        global_z   = 0.0;
        global_yaw = FootSteps[step_index_n][2];
      }
      else{ //when left foot is moving foot
        double pre_time = step_time * (double)step_index_n;
        double dsp_time = step_time * dsp_ratio;
        double half_dsp_time = 0.5*dsp_time;
        //double foot_height = 0.02;//0.1;
        if(step_index_n == 0){
          //double foot_distance = 0.1;
          //double foot_height = 0.1;
          double pre_x = 0.0;
          double pre_y = FootSteps[step_index_n][1] + foot_distance;
          double pre_yaw = 0;
          int next_step_index = step_index_n+1;
          if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
            global_x   = pre_x   + (FootSteps[next_step_index][0]  -pre_x)*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_y   = pre_y   + (FootSteps[next_step_index][1]  -pre_y)*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_yaw = pre_yaw + (FootSteps[next_step_index][2]-pre_yaw)*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          }
          else if(t-pre_time<half_dsp_time){
            global_x   = pre_x;
            global_y   = pre_y;
            global_z   = 0.0;
            global_yaw = pre_yaw;
          }
          else if(step_time-half_dsp_time < t-pre_time){
            global_x   = FootSteps[next_step_index][0];
            global_y   = FootSteps[next_step_index][1];
            global_z   = 0.0;
            global_yaw = FootSteps[next_step_index][2];
          }
        }
        else if(step_index_n == step_num-1){

          last_foot == 1; //last foot : right, side foot : left
          //double foot_distance = 0.1;
          //double foot_height = 0.1;

          MatrixXd last_foot_TF(4,4);
          MatrixXd side_foot_TF(4,4);
          MatrixXd side_foot_global_TF(4,4);
          double s = sin(FootSteps[step_index_n][2]);
          double c = cos(FootSteps[step_index_n][2]);
          double x = FootSteps[step_index_n][0];
          double y = FootSteps[step_index_n][1];
          last_foot_TF<<c,-s,0,x,\
                        s, c,0,y,\
                        0, 0,1,0,
                        0, 0,0,1;
          double angle = 0;//PI/2; //left foot is side foot
          side_foot_TF << cos(angle), -sin(angle), 0, 0,\
                          sin(angle),  cos(angle), 0, foot_distance,\
                                   0,           0, 1, 0,\
                                   0,           0, 0, 1;
          side_foot_global_TF = last_foot_TF * side_foot_TF;

          double next_x = side_foot_global_TF(0,3);
          double next_y = side_foot_global_TF(1,3);
          double next_yaw = FootSteps[step_index_n][2];

          last_side_foot.push_back(next_x);
          last_side_foot.push_back(next_y);
          last_side_foot.push_back(next_yaw);


          int pre_step_index = step_index_n-1;
          if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
            global_x   = FootSteps[pre_step_index][0] + (next_x   - FootSteps[pre_step_index][0])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_y   = FootSteps[pre_step_index][1] + (next_y   - FootSteps[pre_step_index][1])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_yaw = FootSteps[pre_step_index][2] + (next_yaw - FootSteps[pre_step_index][2])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          }
          else if(t-pre_time < half_dsp_time){

            global_x   = FootSteps[pre_step_index][0];
            global_y   = FootSteps[pre_step_index][1];
            global_z   = 0.0;
            global_yaw = FootSteps[pre_step_index][2];
          }
          else if(step_time-half_dsp_time < t-pre_time){
            global_x   = next_x;
            global_y   = next_y;
            global_z   = 0.0;
            global_yaw = next_yaw;
          }

        }

        else if(step_index_n > 0 and step_index_n < step_num-1){
          int pre_step_index = step_index_n-1;
          int next_step_index = step_index_n+1;

          if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
            global_x   = FootSteps[pre_step_index][0] + (FootSteps[next_step_index][0]-FootSteps[pre_step_index][0])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_y   = FootSteps[pre_step_index][1] + (FootSteps[next_step_index][1]-FootSteps[pre_step_index][1])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_yaw = FootSteps[pre_step_index][2] + (FootSteps[next_step_index][2]-FootSteps[pre_step_index][2])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          }
          else if(t-pre_time<half_dsp_time){
            global_x   = FootSteps[pre_step_index][0];
            global_y   = FootSteps[pre_step_index][1];
            global_z   = 0.0;
            global_yaw = FootSteps[pre_step_index][2];
          }
          else if(step_time-half_dsp_time < t-pre_time){
            global_x   = FootSteps[next_step_index][0];
            global_y   = FootSteps[next_step_index][1];
            global_z   = 0.0;
            global_yaw = FootSteps[next_step_index][2];
          }
        }
      }
    }
    else{ //must be modified
      if(last_foot == 0){ //last foot is left_foot, than right is side foot
        global_x   = FootSteps[step_num-1][0];
        global_y   = FootSteps[step_num-1][1];
        global_z   = 0.0;
        global_yaw = FootSteps[step_num-1][2];
      }
      else{               //last foot is right_foot, than left is side foot
        global_x   = last_side_foot[0];
        global_y   = last_side_foot[1];
        global_z   = 0.0;
        global_yaw = last_side_foot[2];
      }
    }
  }
  double S,C;
  S = sin(global_yaw);
  C = cos(global_yaw);
  LeftFoot<<C, -S, 0, global_x,\
            S,  C, 0, global_y,\
            0,  0, 1, global_z,\
            0,  0, 0, 1;
  return LeftFoot;


}

MatrixXd FootstepPlanner::get_Right_foot(double t){

  double preview_start_wait_time = 1.5;

  MatrixXd RightFoot(4,4);
  RightFoot = MatrixXd::Identity(4,4);
  double global_x, global_y, global_z, global_yaw;
  if(t < preview_start_wait_time){  //wait when start for preview
    global_x = 0.0;
    global_y = -0.05; // foot_distance /2
    global_z = 0.0;
    global_yaw = 0;
  }
  else{
    t = t - preview_start_wait_time;
    int step_index_n = (int)(t/step_time + 0.001);
    //std::cout<<"step_index : "<<step_index_n<<std::endl;
    if(step_index_n<=step_num-1){

      if((start_foot == 0 and step_index_n%2 == 1) or (start_foot == 1 and step_index_n%2 == 0)){ //when Right foot is supprot foot
        global_x   = FootSteps[step_index_n][0];
        global_y   = FootSteps[step_index_n][1];
        global_z   = 0.0;
        global_yaw = FootSteps[step_index_n][2];
      }
      else{ //when right foot is moving foot
        double pre_time = step_time * (double)step_index_n;
        double dsp_time = step_time * dsp_ratio;
        double half_dsp_time = 0.5*dsp_time;
        //double foot_height=0.02;//0.1;

        if(step_index_n == 0){  //when starting
          //double foot_distance = 0.1;
          double pre_x = 0.0;
          double pre_y = FootSteps[step_index_n][1] - foot_distance; //for right foot
          double pre_yaw = 0;
          int next_step_index = step_index_n+1;
          if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
            global_x   = pre_x   + (FootSteps[next_step_index][0]  -pre_x)*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_y   = pre_y   + (FootSteps[next_step_index][1]  -pre_y)*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_yaw = pre_yaw + (FootSteps[next_step_index][2]-pre_yaw)*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          }
          else if(t-pre_time<half_dsp_time){
            global_x   = pre_x;
            global_y   = pre_y;
            global_z   = 0.0;
            global_yaw = pre_yaw;
          }
          else if(step_time-half_dsp_time < t-pre_time){
            global_x   = FootSteps[next_step_index][0];
            global_y   = FootSteps[next_step_index][1];
            global_z   = 0.0;
            global_yaw = FootSteps[next_step_index][2];
          }
        }
        else if(step_index_n == step_num-1){ //when finishing
          //double foot_distance = 0.1;

          last_foot = 0; //last foot : left, side foot : right

          MatrixXd last_foot_TF(4,4);
          MatrixXd side_foot_TF(4,4);
          MatrixXd side_foot_global_TF(4,4);
          double s = sin(FootSteps[step_index_n][2]);
          double c = cos(FootSteps[step_index_n][2]);
          double x = FootSteps[step_index_n][0];
          double y = FootSteps[step_index_n][1];
          last_foot_TF<<c,-s,0,x,\
                        s, c,0,y,\
                        0, 0,1,0,
                        0, 0,0,1;
          double angle = 0;//PI/2; //left foot is side foot          //for right*/
          side_foot_TF << cos(angle), -sin(angle), 0, 0,\
                          sin(angle), cos(angle), 0, -foot_distance,\
                                   0,          0, 1, 0,\
                                   0,          0, 0, 1;
          side_foot_global_TF = last_foot_TF * side_foot_TF;

          double next_x = side_foot_global_TF(0,3);
          double next_y = side_foot_global_TF(1,3);
          double next_yaw = FootSteps[step_index_n][2];

          last_side_foot.push_back(next_x);
          last_side_foot.push_back(next_y);
          last_side_foot.push_back(next_yaw);

          int pre_step_index = step_index_n-1;
          if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
            global_x   = FootSteps[pre_step_index][0] + (next_x   - FootSteps[pre_step_index][0])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_y   = FootSteps[pre_step_index][1] + (next_y   - FootSteps[pre_step_index][1])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_yaw = FootSteps[pre_step_index][2] + (next_yaw - FootSteps[pre_step_index][2])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          }
          else if(t-pre_time < half_dsp_time){
            global_x   = FootSteps[pre_step_index][0];
            global_y   = FootSteps[pre_step_index][1];
            global_z   = 0.0;
            global_yaw = FootSteps[pre_step_index][2];
          }
          else if(step_time-half_dsp_time < t-pre_time){
            global_x   = next_x;
            global_y   = next_y;
            global_z   = 0.0;
            global_yaw = next_yaw;
          }

        }

        else if(step_index_n > 0 or step_index_n < step_num-1){
          int pre_step_index = step_index_n-1;
          int next_step_index = step_index_n+1;

          if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
            global_x   = FootSteps[pre_step_index][0] + (FootSteps[next_step_index][0]-FootSteps[pre_step_index][0])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_y   = FootSteps[pre_step_index][1] + (FootSteps[next_step_index][1]-FootSteps[pre_step_index][1])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_yaw = FootSteps[pre_step_index][2] + (FootSteps[next_step_index][2]-FootSteps[pre_step_index][2])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          }
          else if(t-pre_time<half_dsp_time){
            global_x   = FootSteps[pre_step_index][0];
            global_y   = FootSteps[pre_step_index][1];
            global_z   = 0.0;
            global_yaw = FootSteps[pre_step_index][2];
          }
          else if(step_time-half_dsp_time < t-pre_time){
            global_x   = FootSteps[next_step_index][0];
            global_y   = FootSteps[next_step_index][1];
            global_z   = 0.0;
            global_yaw = FootSteps[next_step_index][2];
          }

        }
      }
    }
    else{ //must be modified
      if(last_foot == 0){ //last foot is left_foot, than right is side foot
        global_x   = last_side_foot[0];
        global_y   = last_side_foot[1];
        global_z   = 0.0;
        global_yaw = last_side_foot[2];
      }
      else{
        global_x   = FootSteps[step_num-1][0];
        global_y   = FootSteps[step_num-1][1];
        global_z   = 0.0;
        global_yaw = FootSteps[step_num-1][2];
      }
    }
  }
  double S,C;
  S = sin(global_yaw);
  C = cos(global_yaw);
  RightFoot<<C, -S, 0, global_x,\
            S,  C, 0, global_y,\
            0,  0, 1, global_z,\
            0,  0, 0, 1;
  return RightFoot;


}

double FootstepPlanner::ellipse_traj(double present_time, double start_time, double T, double foot_Height){
  double _1_cos_t;
  if(present_time-start_time < T){
    _1_cos_t = 0.5*(1.0 - cos(PI*((present_time-start_time)/T)));
  }
  else
    _1_cos_t = 1;
  double result = foot_Height*sin(PI*_1_cos_t);
  return result;
}
double FootstepPlanner::ellipse_traj(double t, double T, double foot_Height){
  double result;
  double _1_cos_t = 0.5*(1.0 - cos(PI*(t/T)));
  result = foot_Height*sin(PI*_1_cos_t);
  return result;
}

