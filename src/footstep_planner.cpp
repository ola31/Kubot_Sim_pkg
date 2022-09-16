/*
#include "footstep_planner.h"

FootstepPlanner::FootstepPlanner()
  : fb_step_size(0.00), //m
    step_time(0.5),    //sec
    step_num(20),      //step number
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
      zmp_ref.y = 0; // must be modify
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
            //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_z = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
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
          //printf("last foot : right");
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

         // last_side_foot.push_back(next_x);
          //last_side_foot.push_back(next_y);
          //last_side_foot.push_back(next_yaw);
          last_side_foot[0] = next_x;
          last_side_foot[1] = next_y;
          last_side_foot[2] = next_yaw;

          int pre_step_index = step_index_n-1;
          if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
            global_x   = FootSteps[pre_step_index][0] + (next_x   - FootSteps[pre_step_index][0])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_y   = FootSteps[pre_step_index][1] + (next_y   - FootSteps[pre_step_index][1])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_z = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
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
            //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_z = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
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
      if(last_foot == 1){ //last foot is left_foot, than right is side foot
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
      printf(":LEFT x : %lf, y : %lf, yaw : %lf\n",global_x,global_y,global_yaw);
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
 //printf("t : %lf\n",t);

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
            //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_z = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
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
          //printf("last foot : left");

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
          double angle = 0;//PI/2; //left foot is side foot          //for right
          side_foot_TF << cos(angle), -sin(angle), 0, 0,\
                          sin(angle), cos(angle), 0, -foot_distance,\
                                   0,          0, 1, 0,\
                                   0,          0, 0, 1;
          side_foot_global_TF = last_foot_TF * side_foot_TF;

          double next_x = side_foot_global_TF(0,3);
          double next_y = side_foot_global_TF(1,3);
          double next_yaw = FootSteps[step_index_n][2];

          last_side_foot[0] = next_x;
          last_side_foot[1] = next_y;
          last_side_foot[2] = next_yaw;

          int pre_step_index = step_index_n-1;
          if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
            global_x   = FootSteps[pre_step_index][0] + (next_x   - FootSteps[pre_step_index][0])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_y   = FootSteps[pre_step_index][1] + (next_y   - FootSteps[pre_step_index][1])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_z   = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
            //global_yaw = FootSteps[pre_step_index][2] + (next_yaw - FootSteps[pre_step_index][2])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
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
            //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_z   = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
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
      if(last_foot == 1){ //last foot is left_foot, than right is side foot
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
    printf(" RIGHT x : %lf, y : %lf, yaw : %lf\n",global_x,global_y,global_yaw);
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


double FootstepPlanner::Bezier_curve_8th(double time, double start_t, double T){
  double t = time-start_t;
  int P[9] = {0,0,0,100,0,100,0,0,0};
  double result = 0.0;
  for(int k=1;k<=9;k++){
    if(k<=8)
      result += P[k-1]*((double)(factorial(8)/(factorial(k-1)*factorial(8-k+1)))*pow((1.0-(t/T)),8-k+1)*pow((t/T),k-1));
    else
      result += (double)(factorial(8)/(factorial(k-1)*factorial(8-k+1)))*pow((1.0-(t/T)),8-k+1)*pow((t/T),k-1);
  }
  return result*0.001;
}

int FootstepPlanner::factorial(int n){
  if(n>1)
    return n*factorial(n-1);
  else
    return 1;
}
*/


#include "footstep_planner.h"

double func_1_cos_yaw(double start, double end, double t, double T);

FootstepPlanner::FootstepPlanner()
  : walking_mode(0),
    fb_step_size(0.000), //m
    step_time(0.3),    //sec
    step_num(1000),      //step number
    start_foot(0),     //left : 0 right : 1
    dsp_ratio(0.3),
    goal_turn_angle(0.0*PI/180.0), //radian
    foot_height(0.00),     //m
    foot_distance(0.10), //m
    RHR_add_q(0.0),
    LHR_add_q(0.0),
    Hip_roll_add_q(7.0*PI/180.0),
    Foot_y_adding(0.00),
    func_1_cos_param(5.0),
    compensation_start_time_param(0.7),
    two_feet_on_ground(false),

    fb_step(0.0),
    rl_step(0),//0.01;
    rl_turn(0.0),//PI/36;

    stop_flag(false)
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
double FootstepPlanner::get_LHR_add_q(){return LHR_add_q;}
double FootstepPlanner::get_RHR_add_q(){return RHR_add_q;}

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
  double unit_turn_angle = goal_turn_angle/(step_num-1);

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
    double global_foot_yaw = -atan2(GlobalFootTF(0,1),GlobalFootTF(0,0));
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
      zmp_ref.x = FootSteps[step_index_n][0] + 0.000;  //assume that zmp_ref = foot_center_point
      zmp_ref.y = 1.0*FootSteps[step_index_n][1];
    }
    else{
      //zmp_ref.x= FootSteps[step_num-1][0];
      if((start_foot == 0 and (int)step_num%2==1)or(start_foot == 1 and (int)step_num%2 == 0)){ //last foot : left
        zmp_ref.x= FootSteps[step_num-1][0] + (foot_distance/2.0)*sin(goal_turn_angle);
        zmp_ref.y = FootSteps[step_num-1][1] - (foot_distance/2.0)*cos(goal_turn_angle); // must be modify
      }
      else{
        zmp_ref.x= FootSteps[step_num-1][0] - (foot_distance/2.0)*sin(goal_turn_angle);
        zmp_ref.y = FootSteps[step_num-1][1] + (foot_distance/2.0)*cos(goal_turn_angle);
      }
    }
  }
  return zmp_ref;
}

double FootstepPlanner::get_CoM_yaw(double t){
    double preview_start_wait_time = 1.5;
    double CoM_yaw= 0.0;
    double unit_turn_angle = goal_turn_angle/step_num;
    if(t < preview_start_wait_time){  //wait when start for preview
      CoM_yaw= 0.0;
    }
    else{
      t = t-preview_start_wait_time;
      int step_index_n = (int)(t/step_time + 0.0001);
      double pre_time = step_time * (double)step_index_n;
      if(t<step_num*step_time){
        CoM_yaw = (step_index_n)*unit_turn_angle + unit_turn_angle*(t-pre_time)/step_time; //FootSteps[step_index_n][2];
      }
      else{
        CoM_yaw = goal_turn_angle;//(step_num)*unit_turn_angle; //FootSteps[step_num-1][2];
      }
    }

    return CoM_yaw;
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

        double pre_time = step_time * (double)step_index_n;
        double dsp_time = step_time * dsp_ratio;
        double half_dsp_time = 0.5*dsp_time;

        LHR_add_q =  0.0;
        RHR_add_q =  0.0;

        if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
          //LHR_add_q =  Hip_roll_add_q*0.5*(1.0-cos(2.0*PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          LHR_add_q = func_1_cos(t,pre_time+compensation_start_time_param*half_dsp_time,step_time-dsp_time+(1.0-compensation_start_time_param)*half_dsp_time,Hip_roll_add_q);
          RHR_add_q =  0.0;//LHR_add_q;//0.0;
          R_foot_y_adding = Foot_y_adding*0.5*(1.0-cos(2.0*PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
        }
      }
      else{ //when left foot is moving foot

        //LHR_add_q = 0.0;
       // RHR_add_q = 0.0;

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
            //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_z = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
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
          //printf("last foot : right");
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

         // last_side_foot.push_back(next_x);
          //last_side_foot.push_back(next_y);
          //last_side_foot.push_back(next_yaw);
          last_side_foot[0] = next_x;
          last_side_foot[1] = next_y;
          last_side_foot[2] = next_yaw;

          int pre_step_index = step_index_n-1;
          if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
            global_x   = FootSteps[pre_step_index][0] + (next_x   - FootSteps[pre_step_index][0])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_y   = FootSteps[pre_step_index][1] + (next_y   - FootSteps[pre_step_index][1])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_z = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
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
            //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_z = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
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

        if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
          L_foot_y_adding = -Foot_y_adding*0.5*(1.0-cos(2.0*PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
        }
      }
    }
    else{ //must be modified
      if(last_foot == 1){ //last foot is left_foot, than right is side foot
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
      //printf(":LEFT x : %lf, y : %lf, yaw : %lf\n",global_x,global_y,global_yaw);
    }
  }
  double S,C;
  S = sin(global_yaw);
  C = cos(global_yaw);

  if(two_feet_on_ground)
    global_z = 0;

  global_y+=L_foot_y_adding;

  LeftFoot<<C, -S, 0, global_x,\
            S,  C, 0, global_y,\
            0,  0, 1, global_z,\
            0,  0, 0, 1;
  return LeftFoot;


}

MatrixXd FootstepPlanner::get_Right_foot(double t){

  double preview_start_wait_time = 1.5;
 //printf("t : %lf\n",t);

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

        double pre_time = step_time * (double)step_index_n;
        double dsp_time = step_time * dsp_ratio;
        double half_dsp_time = 0.5*dsp_time;

        LHR_add_q = 0.0;
        RHR_add_q = 0.0;

        if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
          //LHR_add_q =  0.0;
         // RHR_add_q =  -Hip_roll_add_q*0.5*(1.0-cos(2.0*PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          RHR_add_q = -func_1_cos(t,pre_time+compensation_start_time_param*half_dsp_time,step_time-dsp_time+(1.0-compensation_start_time_param)*half_dsp_time,Hip_roll_add_q);
          LHR_add_q = 0.0;//RHR_add_q;
        }
      }
      else{ //when right foot is moving foot
        //LHR_add_q = 0.0;
        //RHR_add_q = 0.0;

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
            //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_z = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
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
          //printf("last foot : left");

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
          double angle = 0;//PI/2; //left foot is side foot          //for right
          side_foot_TF << cos(angle), -sin(angle), 0, 0,\
                          sin(angle), cos(angle), 0, -foot_distance,\
                                   0,          0, 1, 0,\
                                   0,          0, 0, 1;
          side_foot_global_TF = last_foot_TF * side_foot_TF;

          double next_x = side_foot_global_TF(0,3);
          double next_y = side_foot_global_TF(1,3);
          double next_yaw = FootSteps[step_index_n][2];

          last_side_foot[0] = next_x;
          last_side_foot[1] = next_y;
          last_side_foot[2] = next_yaw;

          int pre_step_index = step_index_n-1;
          if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
            global_x   = FootSteps[pre_step_index][0] + (next_x   - FootSteps[pre_step_index][0])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            global_y   = FootSteps[pre_step_index][1] + (next_y   - FootSteps[pre_step_index][1])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
            //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_z   = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
            //global_yaw = FootSteps[pre_step_index][2] + (next_yaw - FootSteps[pre_step_index][2])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
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
            //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
            global_z   = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
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

        if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
          R_foot_y_adding = Foot_y_adding*0.5*(1.0-cos(2.0*PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
        }
      }
    }
    else{ //must be modified
      if(last_foot == 1){ //last foot is left_foot, than right is side foot
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
    //printf(" RIGHT x : %lf, y : %lf, yaw : %lf\n",global_x,global_y,global_yaw);
  }
  double S,C;
  S = sin(global_yaw);
  C = cos(global_yaw);
  if(two_feet_on_ground)
    global_z = 0;

  global_y+=R_foot_y_adding;

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


double FootstepPlanner::Bezier_curve_8th(double time, double start_t, double T){
  double t = time-start_t;
  int P[9] = {0,0,0,110,0,110,0,0,0};
  //int P[9] = {0,0,0,0,0,0,0,0,0};
  double result = 0.0;
  for(int k=1;k<=9;k++){
    if(k<=8)
      result += P[k-1]*((double)(factorial(8)/(factorial(k-1)*factorial(8-k+1)))*pow((1.0-(t/T)),8-k+1)*pow((t/T),k-1));
    else
      result += (double)(factorial(8)/(factorial(k-1)*factorial(8-k+1)))*pow((1.0-(t/T)),8-k+1)*pow((t/T),k-1);
  }
  return result*0.001;
}

int FootstepPlanner::factorial(int n){
  if(n>1)
    return n*factorial(n-1);
  else
    return 1;
}

double FootstepPlanner::func_1_cos(double t, double start_t, double T, double max){
  if(t-start_t <= (T/func_1_cos_param)){
    return max*0.5*(1.0-cos(PI*((t-start_t)/(T/func_1_cos_param))));
  }
  else if(t-start_t >= ((func_1_cos_param-1.0)*T/func_1_cos_param)){
    return (max + (0.0-max)*0.5*(1.0-cos(PI*((t-start_t-((func_1_cos_param-1.0)*T/func_1_cos_param))/(T/func_1_cos_param)))));
  }
  else return max;
}






MatrixXd FootstepPlanner::get_side_foot_tf(int present_foot_index, MatrixXd pre_step_tf){
  double del_x;
  if(present_foot_index == 0){ //side foot is leftfoot
   del_x = foot_distance;
  }
  else{//side foot is rightfoot
    del_x = -foot_distance;
  }
  MatrixXd tmp_tf(4,4);
  tmp_tf.block(0,0,3,3) = MatrixXd::Identity(3,3);
  tmp_tf(0,3) = del_x;
  tmp_tf(1,3) = 0.0;
  tmp_tf(2,3) = 0.0;
  tmp_tf(3,3) = 1.0;
  return pre_step_tf*tmp_tf;
}

std::vector<double> FootstepPlanner::get_step_vector(double del_x, double del_y, double turn_angle_radian, double foot_index){
  double s = sin(turn_angle_radian);
  double c = cos(turn_angle_radian);
  MatrixXd tmp_tf(4,4);
  tmp_tf<<  c, -s, 0,  del_x,\
            s,  c, 0,  del_y,\
            0,  0, 1.0, 0,\
            0,  0, 0,  1.0;
  this->foot_tf_global = this->foot_tf_global*tmp_tf;
  std::vector<double> step;
  step.push_back(foot_tf_global(0,3)); //x
  step.push_back(foot_tf_global(1,3)); //y

  double global_foot_yaw = -atan2(foot_tf_global(0,1),foot_tf_global(0,0));
  step.push_back(global_foot_yaw); //yaw
  step.push_back(foot_index); //foot_index
   return step;
}
void FootstepPlanner::init_deque(void){
  std::vector<double> step;
  double del_x;
  double del_y;
  int sign=(start_foot==0)?1:-1;

  foot_index = start_foot;
  double start_foot_y = (start_foot==0)?(0.5*foot_distance):(-0.5*foot_distance);
  for(int i=0;i<5;i++){
    del_x = 0.0; //0.0;
    double yaw;
    if(i==0) {
      del_x = 0.0;
      del_y= start_foot_y;
      yaw = 0.0;;
    }
    else{
      del_x = 0.00;
      foot_index = (abs(foot_index-1.0)<0.001)?0.0:1.0;
      del_y = sign*foot_distance;
      yaw = 0;
    }


    step = get_step_vector(del_x,del_y,yaw,foot_index);
    footstep_deque.push_back(step);
    sign *= -1;

  }
  pre_step = footstep_deque.back();

}



void FootstepPlanner::update_footsteps(double t_sec){

  double preview_start_wait_time = 1.5;
  if(t_sec < preview_start_wait_time){
  }
  else{

  int nn = footstep_deque.size();
  //printf("deque num : %d\n",nn);

  t_sec = t_sec - preview_start_wait_time;
  //printf("t_sec : %lf\n",t_sec);
  int t_ms= (int)(t_sec*1000 + 0.00001);
  int step_time_ms=(int)(step_time*1000 + 0.00001);

  if(t_ms%step_time_ms == 0){ //every step_time

    pre_step = footstep_deque[0];
    footstep_deque.pop_front();

    int deque_step_num = footstep_deque.size();
    int N = 1000;
    double dt = 0.001;
    double preview_time = N*dt;
    if(deque_step_num >= ((int)(preview_time/step_time)+2)){
      Fplanner_time+=dt;
      return;
    }
    if(stop_flag){
      foot_index = (foot_index==1)?0:1;
      double del_x, del_y, yaw;
      if(foot_index == 0){ //left foot
        del_x = 0.0;
        del_y = foot_distance;
        yaw = 0.0;
      }
      else if(foot_index == 1){ //right foot
        del_x = 0.0;
        del_y = -foot_distance;
        yaw = 0.0;
      }
      std::vector<double> step;
      step = get_step_vector(del_x,del_y,yaw,foot_index + 2*(int)stop_flag);
      footstep_deque.push_back(step);

      Fplanner_time+=dt;
      return;
    }

    foot_index = (foot_index==1)?0:1; //change foot index(left->right or right->left)
    if(foot_index == 0){//left foot

      double del_x;
      double del_y;

      if(rl_step <= -unit_rl_step){ //rl_step : next_step
        if(rl_turn <= -unit_rl_turn){ //rl_turn : next_step
          del_x =fb_step;
          del_y = foot_distance;

          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,0.0); //left
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step;
          del_y = -foot_distance + rl_step;
          del_x += foot_distance*sin(rl_turn);
          del_y += foot_distance*(1.0-cos(rl_turn));
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,rl_turn,1.0); //right
          footstep_deque.push_back(step2);
          /*
          foot_index = (foot_index==1)?0:1;
          del_x = fb_step; //0.0;
          del_y = foot_distance;
          std::vector<double> step3;
          step3 = get_step_vector(del_x,del_y,0.0,0.0); //left
          footstep_deque.push_back(step3);
          */

        }
        else if(rl_turn >= unit_rl_turn){ //rl_turn : now_step
          del_x = 0;
          del_y = foot_distance;
          del_x -= foot_distance*sin(rl_turn);
          del_y -= foot_distance*(1.0-cos(rl_turn));

          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,0.0); //left
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step;
          del_y = -foot_distance + rl_step;
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,rl_turn,1.0); //right
          footstep_deque.push_back(step2);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step; //0.0;
          del_y = foot_distance;
          std::vector<double> step3;
          step3 = get_step_vector(del_x,del_y,0.0,0.0); //left
          footstep_deque.push_back(step3);
        }
        else{ //rl_turn = 0
          del_x = fb_step;
          del_y = foot_distance;

          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,0.0); //left
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step;
          del_y = -foot_distance + rl_step;
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,0.0,1.0); //right
          footstep_deque.push_back(step2);
        }
      }
      else if(rl_step >= unit_rl_step){ //rl_step : now_step
        if(rl_turn <= -unit_rl_turn){ //rl_turn : next_step
          del_x = fb_step;
          del_y = foot_distance + rl_step;
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,0.0);//left
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step;
          del_y = -foot_distance;
          del_x +=foot_distance*sin(rl_turn);
          del_y += foot_distance*(1.0-cos(rl_turn));
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,rl_turn,1.0);//right
          footstep_deque.push_back(step2);
          /*
          foot_index = (foot_index==1)?0:1;
          del_x = 0.0;
          del_y = foot_distance;
          std::vector<double> step3;
          step3 = get_step_vector(del_x,del_y,0.0,0.0);//left
          footstep_deque.push_back(step3);
          */
        }
        else if(rl_turn >= unit_rl_turn){ //rl_turn : now_step
          del_x = fb_step;
          del_y = foot_distance + rl_step;
          del_x -=foot_distance*sin(rl_turn);
          del_y -= foot_distance*(1.0-cos(rl_turn));
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,rl_turn,0.0);//left
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step;//0.0;
          del_y = -foot_distance;
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,0.0,1.0);//right
          footstep_deque.push_back(step2);
        }
        else{  //rl_turn = 0
          del_x = fb_step;
          del_y = foot_distance + rl_step;
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,0.0);//left
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step; //0.0;
          del_y = -foot_distance;
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,0.0,1.0);//right
          footstep_deque.push_back(step2);
        }
      }
      else{ //rl_step = 0
        if(rl_turn <= -unit_rl_turn){ //rl_turn : next_step
          del_x = 0;
          del_y = foot_distance;
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,0.0);//left
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step;
          del_y = -foot_distance;
          del_x +=foot_distance*sin(rl_turn);
          del_y += foot_distance*(1.0-cos(rl_turn));
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,rl_turn,1.0);//right
          footstep_deque.push_back(step2);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step; //0.0;
          del_y = foot_distance;
          std::vector<double> step3;
          step3 = get_step_vector(del_x,del_y,0.0,0.0);//left
          footstep_deque.push_back(step3);

        }
        else if(rl_turn >= unit_rl_turn){ //rl_turn : now_step
          del_x = fb_step;
          del_y = foot_distance;
          del_x -=foot_distance*sin(rl_turn);
          del_y -= foot_distance*(1.0-cos(rl_turn));
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,rl_turn,0.0);//left
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step; //0.0;
          del_y = -foot_distance;
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,0.0,1.0);//right
          footstep_deque.push_back(step2);
        }
        else{ //rl_turn = 0
          del_x = fb_step;
          del_y = foot_distance;
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,0.0);//left
          footstep_deque.push_back(step);
        }
      }
    }
    else if(foot_index == 1){//right foot

      double del_x;
      double del_y;

      if(rl_step <= -unit_rl_step){ //rl_step : now_step
        if(rl_turn <= -unit_rl_turn){ //rl_turn : now_step
          del_x = fb_step;
          del_y = -foot_distance + rl_step;
          del_x +=foot_distance*sin(rl_turn);
          del_y += foot_distance*(1.0-cos(rl_turn));
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,rl_turn,1.0);//right
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step; //0.0;
          del_y = foot_distance;
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,0.0,0.0);//left
          footstep_deque.push_back(step2);
        }
        else if(rl_turn >= unit_rl_turn){ //rl_turn : next_step
          del_x = fb_step;
          del_y = -foot_distance + rl_step;
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,1.0);//right
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step; //0.0;
          del_y = foot_distance;
          del_x -=foot_distance*sin(rl_turn);
          del_y -= foot_distance*(1.0-cos(rl_turn));
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,rl_turn,0.0);//left
          footstep_deque.push_back(step2);
          /*
          foot_index = (foot_index==1)?0:1;
          del_x = 0.0;
          del_y = -foot_distance;
          std::vector<double> step3;
          step3 = get_step_vector(del_x,del_y,0.0,1.0);//right
          footstep_deque.push_back(step3);
          */

        }
        else{ //rl_turn = 0
          del_x = fb_step;
          del_y = -foot_distance + rl_step;
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,1.0);//right
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step; //0.0;
          del_y = foot_distance;
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,0.0,0.0);//left
          footstep_deque.push_back(step2);
        }
      }
      else if(rl_step >= unit_rl_step){ //rl_step : next_step
        if(rl_turn <= -unit_rl_turn){ //rl_turn : now_step
          del_x = fb_step;
          del_y = -foot_distance;
          del_x += foot_distance*sin(rl_turn);
          del_y += foot_distance*(1.0-cos(rl_turn));

          std::vector<double> step;
          step = get_step_vector(del_x,del_y,rl_turn,1.0); //right
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step;
          del_y = foot_distance + rl_step;
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,0.0,0.0); //left
          footstep_deque.push_back(step2);
          /*
          foot_index = (foot_index==1)?0:1;
          del_x = fb_step; //0.0;
          del_y = -foot_distance;
          std::vector<double> step3;
          step3 = get_step_vector(del_x,del_y,0.0,0.0); //right
          footstep_deque.push_back(step3);
          */

        }
        else if(rl_turn >= unit_rl_turn){ //rl_turn : next_step
          del_x = fb_step;
          del_y = -foot_distance;

          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,1.0); //right
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step;
          del_y = foot_distance + rl_step;
          del_x -= foot_distance*sin(rl_turn);
          del_y -= foot_distance*(1.0-cos(rl_turn));
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,rl_turn,0.0); //left
          footstep_deque.push_back(step2);
          /*
          foot_index = (foot_index==1)?0:1;
          del_x = fb_step; //0.0;
          del_y = -foot_distance;
          std::vector<double> step3;
          step3 = get_step_vector(del_x,del_y,0.0,0.0); //right
          footstep_deque.push_back(step3);
          */
        }
        else{
          del_x = 0.0;
          del_y = -foot_distance;
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,1.0);//right
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step;
          del_y = foot_distance + rl_step;
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,0.0,0.0);//left
          footstep_deque.push_back(step2);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step;
          del_y = -foot_distance;
          std::vector<double> step3;
          step3 = get_step_vector(del_x,del_y,0.0,1.0);//right
          footstep_deque.push_back(step3);
        }
      }
      else{ //rl_step = 0
        if(rl_turn <= -unit_rl_turn){ //rl_turn : now_step

          del_x = fb_step;
          del_y = -foot_distance;
          del_x +=foot_distance*sin(rl_turn);
          del_y += foot_distance*(1.0-cos(rl_turn));
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,rl_turn,1.0);//right
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step;//0.0;
          del_y = foot_distance;
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,0.0,0.0);//left
          footstep_deque.push_back(step2);

        }
        else if(rl_turn >= unit_rl_turn){ //rl_turn : next_step
          del_x = 0;
          del_y = -foot_distance;
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,1.0);//right
          footstep_deque.push_back(step);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step;
          del_y = foot_distance;
          del_x -=foot_distance*sin(rl_turn);
          del_y -= foot_distance*(1.0-cos(rl_turn));
          std::vector<double> step2;
          step2 = get_step_vector(del_x,del_y,rl_turn,0.0);//left
          footstep_deque.push_back(step2);

          foot_index = (foot_index==1)?0:1;
          del_x = fb_step; //0.0;
          del_y = -foot_distance;
          std::vector<double> step3;
          step3 = get_step_vector(del_x,del_y,0.0,1.0);//right
          footstep_deque.push_back(step3);
        }
        else{ //rl_turn = 0
          del_x = fb_step;
          del_y = -foot_distance;
          std::vector<double> step;
          step = get_step_vector(del_x,del_y,0.0,1.0);//right
          footstep_deque.push_back(step);
        }
      }
    }
  }
  else{
  }
  Fplanner_time+=dt;
  //printf("fplaneer time %lf\n",Fplanner_time);
  }
}

MatrixXd FootstepPlanner::get_Left_foot2(double t){

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


  int step_index_n = (int)(t/step_time + 0.0001);
  //printf("step_index : %d\n",step_index_n);
  //int step_index_n = (int)((t-(step_num*step_time))/step_time + 0.0001);

  std::vector<double>step;
  step = footstep_deque[0];
  std::vector<double>next_step;
  if(footstep_deque.size() > 1){
    next_step = footstep_deque[1];
  }


  if(true/*step_index_n<=step_num-1*/){

    //if((start_foot == 0 and step_index_n%2 == 0) or (start_foot == 1 and step_index_n%2 == 1)){ //when left foot is supprot foot

    if(abs(step[3] - 0)< 0.001 or abs(step[3] - 2)< 0.001){ //when left foot is supprot foot
      global_x   = step[0];
      global_y   = step[1];
      global_z   = 0.0;
      global_yaw = step[2];

      double pre_time = step_time * (double)step_index_n;
      double dsp_time = step_time * dsp_ratio;
      double half_dsp_time = 0.5*dsp_time;

      LHR_add_q =  0.0;
      RHR_add_q =  0.0;

      if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
        //LHR_add_q =  Hip_roll_add_q*0.5*(1.0-cos(2.0*PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
        LHR_add_q = func_1_cos(t,pre_time+compensation_start_time_param*half_dsp_time,step_time-dsp_time+(1.0-compensation_start_time_param)*half_dsp_time,Hip_roll_add_q);
        RHR_add_q =  0.0;//LHR_add_q;//0.0;
        R_foot_y_adding = Foot_y_adding*0.5*(1.0-cos(2.0*PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
      }
    }
    else{ //when left foot is moving foot

      //LHR_add_q = 0.0;
     // RHR_add_q = 0.0;

      double pre_time = step_time * (double)step_index_n;
      double dsp_time = step_time * dsp_ratio;
      double half_dsp_time = 0.5*dsp_time;
      //double foot_height = 0.02;//0.1;
      if(step_index_n == 0){
        //double foot_distance = 0.1;
        //double foot_height = 0.1;
        double pre_x = 0.0;
        double pre_y = step[1] + foot_distance;
        double pre_yaw = 0;
        int next_step_index = step_index_n+1;
        if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
          global_x   = pre_x   + (next_step[0]  -pre_x)*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          global_y   = pre_y   + (next_step[1]  -pre_y)*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
          global_z = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
          //global_yaw = pre_yaw + (next_step[2]-pre_yaw)*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          global_yaw = func_1_cos_yaw(pre_yaw, next_step[2],(t-pre_time-half_dsp_time),(step_time-dsp_time));
        }
        else if(t-pre_time<half_dsp_time){
          global_x   = pre_x;
          global_y   = pre_y;
          global_z   = 0.0;
          global_yaw = pre_yaw;
        }
        else if(step_time-half_dsp_time < t-pre_time){
          global_x   = next_step[0];
          global_y   = next_step[1];
          global_z   = 0.0;
          global_yaw = next_step[2];
        }
      }
      else if(step_index_n == step_num-1){

        last_foot == 1; //last foot : right, side foot : left
        //printf("last foot : right");
        //double foot_distance = 0.1;
        //double foot_height = 0.1;

        MatrixXd last_foot_TF(4,4);
        MatrixXd side_foot_TF(4,4);
        MatrixXd side_foot_global_TF(4,4);
        double s = sin(step[2]);
        double c = cos(step[2]);
        double x = step[0];
        double y = step[1];
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
        double next_yaw = step[2];

       // last_side_foot.push_back(next_x);
        //last_side_foot.push_back(next_y);
        //last_side_foot.push_back(next_yaw);
        last_side_foot[0] = next_x;
        last_side_foot[1] = next_y;
        last_side_foot[2] = next_yaw;

        int pre_step_index = step_index_n-1;
        if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
          global_x   = pre_step[0] + (next_x   - pre_step[0])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          global_y   = pre_step[1] + (next_y   - pre_step[1])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
          global_z = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
          //global_yaw = pre_step[2] + (next_yaw - pre_step[2])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          global_yaw = func_1_cos_yaw(pre_step[2], next_yaw,(t-pre_time-half_dsp_time),(step_time-dsp_time));
        }
        else if(t-pre_time < half_dsp_time){

          global_x   = pre_step[0];
          global_y   = pre_step[1];
          global_z   = 0.0;
          global_yaw = pre_step[2];
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
          global_x   = pre_step[0] + (next_step[0]-pre_step[0])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          global_y   = pre_step[1] + (next_step[1]-pre_step[1])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
          if(step[3] >1.9 and next_step[3]>1.9)
            global_z = 0.0;
          else
            global_z = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
          ////global_yaw = pre_step[2] + (next_step[2]-pre_step[2])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          global_yaw = func_1_cos_yaw(pre_step[2], next_step[2],(t-pre_time-half_dsp_time),(step_time-dsp_time));
        }
        else if(t-pre_time<half_dsp_time){
          global_x   = pre_step[0];
          global_y   = pre_step[1];
          global_z   = 0.0;
          global_yaw = pre_step[2];
        }
        else if(step_time-half_dsp_time < t-pre_time){
          global_x   = next_step[0];
          global_y   = next_step[1];
          global_z   = 0.0;
          global_yaw = next_step[2];
        }
      }

      if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
        L_foot_y_adding = -Foot_y_adding*0.5*(1.0-cos(2.0*PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
      }
    }
  }

  else{ //must be modified
    if(last_foot == 1){ //last foot is left_foot, than right is side foot
      global_x   = step[0];
      global_y   = step[1];
      global_z   = 0.0;
      global_yaw = step[2];
    }
    else{               //last foot is right_foot, than left is side foot
      global_x   = step[0];
      global_y   = step[1];
      global_z   = 0.0;
      global_yaw = step[2];
    }
    //printf(":LEFT x : %lf, y : %lf, yaw : %lf\n",global_x,global_y,global_yaw);
  }
  }

  double S,C;
  S = sin(global_yaw);
  C = cos(global_yaw);


  if(two_feet_on_ground)
    global_z = 0;

  global_y+=L_foot_y_adding;

  LeftFoot<<C, -S, 0, global_x,\
            S,  C, 0, global_y,\
            0,  0, 1, global_z,\
            0,  0, 0, 1;
  return LeftFoot;


}

MatrixXd FootstepPlanner::get_Right_foot2(double t){

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
    t = t-preview_start_wait_time;


  int step_index_n = (int)(t/step_time + 0.0001);
  //int step_index_n = (int)((t-(step_num*step_time))/step_time + 0.0001);

  std::vector<double>step;
  step = footstep_deque[0];
  std::vector<double>next_step;
  if(footstep_deque.size() > 1){
    next_step = footstep_deque[1];
  }


  if(true/*step_index_n<=step_num-1*/){

    //if((start_foot == 0 and step_index_n%2 == 1) or (start_foot == 1 and step_index_n%2 == 0)){ //when Right foot is supprot foot
    if(abs(step[3] - 1.0)< 0.001 or abs(step[3] - 1.0 -2)< 0.001){//when Right foot is support foot
      global_x   = step[0];
      global_y   = step[1];
      global_z   = 0.0;
      global_yaw = step[2];

      double pre_time = step_time * (double)step_index_n;
      double dsp_time = step_time * dsp_ratio;
      double half_dsp_time = 0.5*dsp_time;

      LHR_add_q = 0.0;
      RHR_add_q = 0.0;

      if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
        //LHR_add_q =  0.0;
       // RHR_add_q =  -Hip_roll_add_q*0.5*(1.0-cos(2.0*PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
        RHR_add_q = -func_1_cos(t,pre_time+compensation_start_time_param*half_dsp_time,step_time-dsp_time+(1.0-compensation_start_time_param)*half_dsp_time,Hip_roll_add_q);
        LHR_add_q = 0.0;//RHR_add_q;
      }
    }
    else{ //when right foot is moving foot
      //LHR_add_q = 0.0;
      //RHR_add_q = 0.0;

      double pre_time = step_time * (double)step_index_n;
      double dsp_time = step_time * dsp_ratio;
      double half_dsp_time = 0.5*dsp_time;
      //double foot_height=0.02;//0.1;

      if(step_index_n == 0){  //when starting
        //double foot_distance = 0.1;
        double pre_x = 0.0;
        double pre_y = step[1] - foot_distance; //for right foot
        double pre_yaw = 0;
        int next_step_index = step_index_n+1;
        if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
          global_x   = pre_x   + (next_step[0]  -pre_x)*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          global_y   = pre_y   + (next_step[1]  -pre_y)*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
          global_z = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
          ////global_yaw = pre_yaw + (next_step[2]-pre_yaw)*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          global_yaw = func_1_cos_yaw(pre_yaw,next_step[2],(t-pre_time-half_dsp_time),(step_time-dsp_time));
        }
        else if(t-pre_time<half_dsp_time){
          global_x   = pre_x;
          global_y   = pre_y;
          global_z   = 0.0;
          global_yaw = pre_yaw;
        }
        else if(step_time-half_dsp_time < t-pre_time){
          global_x   = next_step[0];
          global_y   = next_step[1];
          global_z   = 0.0;
          global_yaw = next_step[2];
        }
      }
      else if(step_index_n == step_num-1){ //when finishing
        //double foot_distance = 0.1;

        last_foot = 0; //last foot : left, side foot : right
        //printf("last foot : left");

        MatrixXd last_foot_TF(4,4);
        MatrixXd side_foot_TF(4,4);
        MatrixXd side_foot_global_TF(4,4);
        double s = sin(step[2]);
        double c = cos(step[2]);
        double x = step[0];
        double y = step[1];
        last_foot_TF<<c,-s,0,x,\
                      s, c,0,y,\
                      0, 0,1,0,
                      0, 0,0,1;
        double angle = 0;//PI/2; //left foot is side foot          //for right
        side_foot_TF << cos(angle), -sin(angle), 0, 0,\
                        sin(angle), cos(angle), 0, -foot_distance,\
                                 0,          0, 1, 0,\
                                 0,          0, 0, 1;
        side_foot_global_TF = last_foot_TF * side_foot_TF;

        double next_x = side_foot_global_TF(0,3);
        double next_y = side_foot_global_TF(1,3);
        double next_yaw = step[2];

        last_side_foot[0] = next_x;
        last_side_foot[1] = next_y;
        last_side_foot[2] = next_yaw;

        int pre_step_index = step_index_n-1;
        if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
          global_x   = pre_step[0] + (next_x   - pre_step[0])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          global_y   = pre_step[1] + (next_y   - pre_step[1])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
          global_z   = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);
          //global_yaw = FootSteps[pre_step_index][2] + (next_yaw - FootSteps[pre_step_index][2])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
        }
        else if(t-pre_time < half_dsp_time){
          global_x   = pre_step[0];
          global_y   = pre_step[1];
          global_z   = 0.0;
          global_yaw = pre_step[2];
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
          global_x   = pre_step[0] + (next_step[0]-pre_step[0])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          global_y   = pre_step[1] + (next_step[1]-pre_step[1])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          //global_z   = ellipse_traj(t, pre_time+half_dsp_time, step_time-dsp_time, foot_height);
          if(step[3] >1.9 and next_step[3]>1.9)
            global_z = 0.0;
          else
            global_z   = Bezier_curve_8th(t, pre_time+half_dsp_time, step_time-dsp_time);

          //global_yaw = pre_step[2] + (next_step[2]-pre_step[2])*0.5*(1.0-cos(PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
          global_yaw = func_1_cos_yaw(pre_step[2], next_step[2],(t-pre_time-half_dsp_time),(step_time-dsp_time));
        }
        else if(t-pre_time<half_dsp_time){
          global_x   = pre_step[0];
          global_y   = pre_step[1];
          global_z   = 0.0;
          global_yaw = pre_step[2];
        }
        else if(step_time-half_dsp_time < t-pre_time){
          global_x   = next_step[0];
          global_y   = next_step[1];
          global_z   = 0.0;
          global_yaw = next_step[2];
        }
      }

      if(half_dsp_time<=t-pre_time and t-pre_time<=(step_time-half_dsp_time)){
        R_foot_y_adding = Foot_y_adding*0.5*(1.0-cos(2.0*PI*((t-pre_time-half_dsp_time)/(step_time-dsp_time))));
      }
    }
  }
  else{ //must be modified
    if(last_foot == 1){ //last foot is left_foot, than right is side foot
      global_x   = last_side_foot[0];
      global_y   = last_side_foot[1];
      global_z   = 0.0;
      global_yaw = last_side_foot[2];
    }
    else{
      global_x   = pre_step[0];
      global_y   = pre_step[1];
      global_z   = 0.0;
      global_yaw = pre_step[2];
    }
  }
  //printf(" RIGHT x : %lf, y : %lf, yaw : %lf\n",global_x,global_y,global_yaw);
  }
  double S,C;
  S = sin(global_yaw);
  C = cos(global_yaw);
  if(two_feet_on_ground)
    global_z = 0;

  global_y+=R_foot_y_adding;

  RightFoot<<C, -S, 0, global_x,\
            S,  C, 0, global_y,\
            0,  0, 1, global_z,\
            0,  0, 0, 1;
  return RightFoot;

}

struct XY FootstepPlanner::get_zmp_ref2(double t){

  struct XY zmp_ref;

  double preview_start_wait_time = 1.5;

  if(t < preview_start_wait_time){  //wait when start for preview
    zmp_ref.x = 0.0;
    zmp_ref.y = 0.0;
  }
  else{
    t = t-preview_start_wait_time;
    //int t_ms = (int)(t*1000+ 0.0001);
    int Fplanner_time_ms= (int)((Fplanner_time)*1000 + 0.00001);
    int step_time_ms = (int)(step_time*1000 + 0.0001);
    int step_num_;
    //step_num = t_ms/step_time_ms; //Nth step 0,1,2~
    step_num_ = Fplanner_time_ms/step_time_ms; //Nth step 0,1,2~
    int step_index_n = (int)((t-(step_num_*step_time))/step_time + 0.00001);

    //int step_index_n = (int)(t/step_time + 0.0001);
    std::vector<double>step;
    step = footstep_deque[step_index_n];
    if(step[3]<1.9/*stop flag false*/){
      //zmp_ref.x = FootSteps[step_index_n][0] + 0.000;  //assume that zmp_ref = foot_center_point
      //zmp_ref.y = 1.0*FootSteps[step_index_n][1];
      zmp_ref.x = step[0] + 0.000;  //assume that zmp_ref = foot_center_point
      zmp_ref.y = 1.0*step[1];
    }
    else{
/*
      double s = sin(turn_angle_radian);
      double c = cos(turn_angle_radian);
      MatrixXd tmp_tf(4,4);
      tmp_tf<<  c, -s, 0,  del_x,\
                s,  c, 0,  del_y,\
                0,  0, 1.0, 0,\
                0,  0, 0,  1.0;
      this->foot_tf_global = this->foot_tf_global*tmp_tf;
      std::vector<double> step;
      step.push_back(foot_tf_global(0,3)); //x
      step.push_back(foot_tf_global(1,3)); //y
*/
      if(step[3]<1.5){ //left_foot
        zmp_ref.x = step[0] + (foot_distance/2.0)*sin(goal_turn_angle);
        zmp_ref.y = step[1] - (foot_distance/2.0)*cos(goal_turn_angle); // must be modify
      }
      else{
        zmp_ref.x = step[0] - (foot_distance/2.0)*sin(goal_turn_angle);
        zmp_ref.y = step[1] + (foot_distance/2.0)*cos(goal_turn_angle);
      }
      zmp_ref.x = 0.0;
      zmp_ref.y = 0.0;
    }
  }

  return zmp_ref;
}


double FootstepPlanner::get_CoM_yaw2(double t){

  double preview_start_wait_time = 1.5;
  double CoM_yaw= 0.0;
  double unit_turn_angle = goal_turn_angle/step_num;
  if(t < preview_start_wait_time){  //wait when start for preview
    CoM_yaw= 0.0;
  }
  else{
    t = t-preview_start_wait_time;
    int step_index_n = (int)(t/step_time + 0.0001);
    double pre_time = step_time * (double)step_index_n;

    std::vector<double>step;
    step = footstep_deque[0];
    std::vector<double>next_step;
    if(footstep_deque.size() > 1){
      next_step = footstep_deque[1];
    }

    if(t<step_num*step_time){
      if(next_step[2]*step[2] < 0 && abs(step[2])>(PI/2.0)){
        double a = (next_step[2]>=0.0)?(-2.0*PI):(2.0*PI);
        double delta = (next_step[2]+a) - step[2];
        CoM_yaw = step[2] + delta*(t-pre_time)/step_time;
        if(CoM_yaw>PI)
          CoM_yaw - 2.0*PI;
        else if(CoM_yaw<-PI)
          CoM_yaw + 2.0*PI;

      }
      else{
        CoM_yaw = step[2] + (next_step[2]-step[2])*(t-pre_time)/step_time; //FootSteps[step_index_n][2];
      }
    }
    else{ //must be modified
      CoM_yaw = step[2];//(step_num)*unit_turn_angle; //FootSteps[step_num-1][2];
    }
  }

    return CoM_yaw;
}

double func_1_cos_yaw(double start, double end, double t, double T){
  double yaw;

  if((start*end < 0) && abs(start)>(PI/2.0)){
    double a = (end>=0.0)?(-2.0*PI):(2.0*PI);
    double delta = (end+a) - start;
    yaw = start + delta*0.5*(1.0-cos(PI*(t/T)));
    if(yaw>PI)
      yaw - 2.0*PI;
    else if(yaw<-PI)
      yaw + 2.0*PI;

  }
  else{
    yaw = start + (end-start)*0.5*(1.0-cos(PI*(t/T))); //FootSteps[step_index_n][2];
  }
  return yaw;
}


