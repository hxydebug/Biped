#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <Python.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <time.h>
#include <sys/time.h>
#include "bikebot_control.h"
#include "leg_controller.h"

// 必须的头文件
#include <pthread.h>
#include <mutex>
#include <thread>
// 使用ros
#include <ros/ros.h> // 包含ROS的头文件
#include <serial/serial.h>
//RT
#include <limits.h>
#include <sched.h>
#include <sys/mman.h>
#include "bikebot_timer.h"
#include <sys/timerfd.h>
#include <signal.h>
// imu
// #define OPENZEN_CXX14
// #include "OpenZen.h"
// #include <iostream>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TransformStamped.h"

#include "swing_leg_controller.h"
#include "stance_leg_controller.h"
#include "gait_generator.h"
#include "PosVelEstimator.h"

using namespace std;

int socket0;
int socket1;

//全局变量
char ch[64] = {0};
Angle angle[2];
Angle angleV[2];
Position p0[2];
Position v0[2];
Position pdes[2];
Position vdes[2];
Position nextpos[2];
Angle nextangle[2];
Angle L_angle;
Angle R_angle;
int Iter = 500;//设置2000ms
pthread_t tids1[4];
pthread_t tids2[4];
float global_time = 0;

//共享全局变量
//leg_data
CANMessage cbmsg[6];
//command
Eigen::VectorXd user_cmd;
//yaw bias
float yaw_bias = 0;

// 互斥锁
//leg_state
mutex legState_m;

//全局变量
Leg_state leg_state;
CANMessage mes;
CANMessage L_msgs[3];
CANMessage R_msgs[3];
Motor_cmd Mcmd;
Posdiff poserror;
float body_v = 0.1;
float Max_p,Max_r;
float esti_runtime,esti_periodtime;
float swing_runtime,swing_periodtime;
float stc_runtime,stc_periodtime;
float record_runtime,record_periodtime;
float safe_runtime,safe_periodtime;
Position l_leg_p;
Position r_leg_p;
Eigen::Vector3d l_footP_begin;
Eigen::Vector3d r_footP_begin;
Eigen::Vector3d l_leg_v;
Eigen::Vector3d r_leg_v;
Angle l_desired_angle;
Angle r_desired_angle;
Eigen::Vector3d vel_imu;
Eigen::Vector3d vel_kine[2];

//if can start
int bike_begin = 0;
int can0_recieved = 0;
int can1_recieved = 0;
int safety_det_begin = 0;
static int shut_down;

// if imu start
int imu_received = 0;

// if vicon start
int vicon_received = 0;

//record
int stance = 0;

// gait_generator
float v_body = 0;
gait_generator gait_gen;
stance_leg_controller stc(&leg_state,&gait_gen,v_body);
Eigen::VectorXd stc_tau(6);

ros::MultiThreadedSpinner spinner(2);
Eigen::Vector3d last_vicon_pos;

static void sighand(int sig)
{
	shut_down = 1;
}

// Can0_thread
void* Can0_thread(void* args)
{
    cout << "Can0_thread" << endl;
    int nbytes;
    struct can_frame frame;
    while(!shut_down)
    {
        nbytes = read(socket0, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
                perror("can raw socket0 read");
                continue;
        }
        /* paranoid check ... */
        if (nbytes < sizeof(struct can_frame)) {
                fprintf(stderr, "read: incomplete CAN0 frame\n");
                continue;
        }

        //读取指定ID的数据
        if( frame.data[0] == 1){
            for(int i=0;i<6;i++){
                cbmsg[0].data[i] = frame.data[i];
            }
        }
        else if( frame.data[0] == 2){
            for(int i=0;i<6;i++){
                cbmsg[1].data[i] = frame.data[i];
            }
        }
        else{
            if( frame.data[0] == 3){
                for(int i=0;i<6;i++){
                    cbmsg[2].data[i] = frame.data[i];
                }		
            }
        }

        can0_recieved = 1;

    }

    return NULL;
}
// Can1_thread
void* Can1_thread(void* args)
{
    cout << "Can1_thread" << endl;
    int nbytes;
    struct can_frame frame;
    while(!shut_down)
    {
        nbytes = read(socket1, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
                perror("can raw socket1 read");
                continue;
        }
        /* paranoid check ... */
        if (nbytes < sizeof(struct can_frame)) {
                fprintf(stderr, "read: incomplete CAN1 frame\n");
                continue;
        }

        //读取指定ID的数据
        if( frame.data[0] == 1+bias){
            for(int i=0;i<6;i++){
                cbmsg[3].data[i] = frame.data[i];
            }
        }
        else if( frame.data[0] == 2+bias){
            for(int i=0;i<6;i++){
                cbmsg[4].data[i] = frame.data[i];
            }		
        }
        else{
            if( frame.data[0] == 3+bias){
                for(int i=0;i<6;i++){
                    cbmsg[5].data[i] = frame.data[i];
                }	
            }
        }

        //can_msg recieved
        can1_recieved = 1;

    }

    return NULL;
}
// void accCallback(const geometry_msgs::Vector3StampedConstPtr &free_acc){
//     leg_state.acc[0] = free_acc->vector.x;
//     leg_state.acc[1] = free_acc->vector.y;
//     leg_state.acc[2] = free_acc->vector.z;
// }
void imuCallback(const sensor_msgs::ImuConstPtr &imu){
    
    leg_state.acc[0] = imu->linear_acceleration.x;
    leg_state.acc[1] = imu->linear_acceleration.y;
    leg_state.acc[2] = imu->linear_acceleration.z;

    leg_state.omega[0] = imu->angular_velocity.x;
    leg_state.omega[1] = imu->angular_velocity.y;
    leg_state.omega[2] = imu->angular_velocity.z;

    Eigen::Vector4d qua;
    Eigen::Vector3d rpy;
    qua << imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z;
    quaToRpy(qua,rpy);
    leg_state.rpy[0] = rpy[0]-0.01;
    leg_state.rpy[1] = rpy[1];
    leg_state.vicon_rpy[2] = rpy[2];
    imu_received = 1;
    // cout<<rpy[2]<<endl;
}

//imu_thread
void* imu_thread(void* args)
{
    cout << "imu_thread" << endl;

    int argc = 0;
    ros::init(argc, NULL, "node");
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe("/imu/data",1,imuCallback);
    // ros::Subscriber acc_sub = nh.subscribe("/filter/free_acceleration",1,accCallback);
    spinner.spin();
    
    return NULL;
}

void viconCallback(const geometry_msgs::TransformStampedConstPtr &vicon){
    
    leg_state.vicon_pos[0] = vicon->transform.translation.x;
    leg_state.vicon_pos[1] = vicon->transform.translation.y;
    leg_state.vicon_pos[2] = vicon->transform.translation.z;

    leg_state.vicon_vel[0] = (leg_state.vicon_pos[0]-last_vicon_pos[0])*150.0;
    leg_state.vicon_vel[1] = (leg_state.vicon_pos[1]-last_vicon_pos[1])*150.0;
    leg_state.vicon_vel[2] = (leg_state.vicon_pos[2]-last_vicon_pos[2])*150.0;

    last_vicon_pos[0] = leg_state.vicon_pos[0];
    last_vicon_pos[1] = leg_state.vicon_pos[1];
    last_vicon_pos[2] = leg_state.vicon_pos[2];

    Eigen::Vector4d qua;
    Eigen::Vector3d rpy;
    qua << vicon->transform.rotation.w, vicon->transform.rotation.x, vicon->transform.rotation.y, vicon->transform.rotation.z;
    quaToRpy(qua,rpy);
    leg_state.vicon_rpy[0] = rpy[0];
    leg_state.vicon_rpy[1] = rpy[1];
    leg_state.rpy[2] = rpy[2];
    vicon_received = 1;
    // cout<<rpy[2]<<endl;
}

//vicon_thread
void* vicon_thread(void* args)
{
    cout << "vicon_thread" << endl;

    int argc = 0;
    ros::init(argc, NULL, "vicon_node");
    ros::NodeHandle nh;
    ros::Subscriber vicon_sub = nh.subscribe("/vicon/Xinyan/Xinyan",1,viconCallback);
    // ros::Subscriber acc_sub = nh.subscribe("/filter/free_acceleration",1,accCallback);
    spinner.spin();
    
    return NULL;
}

//safety_thread
void* safety_thread(void* args)
{
    cout << "safety_thread" << endl;

    cout << "safety_thread start !" << endl;
    //初始化定时器
    float _period = 0.002;
    auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
    int seconds = (int)_period;
    int nanoseconds = (int)(1e9 * std::fmod(_period, 1.f));
    Timer t;
    itimerspec timerSpec;
    timerSpec.it_interval.tv_sec = seconds;
    timerSpec.it_value.tv_sec = seconds;
    timerSpec.it_value.tv_nsec = nanoseconds;
    timerSpec.it_interval.tv_nsec = nanoseconds;
    timerfd_settime(timerFd, 0, &timerSpec, nullptr);
    unsigned long long missed = 0;
    float _lastRuntime = 0;
    float _lastPeriodTime = 0;
    float _maxPeriod = 0;
    float _maxRuntime = 0;

    int print_flag = 1;

    while(!shut_down)
    {
        //计时
        _lastPeriodTime = (float)t.getSeconds();
        t.start();

        //angle_limit
        if(leg_state.cbdata[0].p < -50.0*PI/180.0 || leg_state.cbdata[0].p > 60.0*PI/180.0) {
            damp_motors();
            if(print_flag==1) {
                cout<<"angle0 error"<<endl;
                cout<<leg_state.cbdata[0].p<<endl;
                print_flag = 0;
            }  
            shut_down = 1;
        }
        if(leg_state.cbdata[1].p < -130.0*PI/180.0 || leg_state.cbdata[1].p > 60.0*PI/180.0) {
            damp_motors();
            if(print_flag==1) {
                cout<<"angle1 error"<<endl;
                cout<<leg_state.cbdata[1].p<<endl;
                print_flag = 0;
            } 
            shut_down = 1;
        }
        if(leg_state.cbdata[2].p < 35.0*PI/180.0 || leg_state.cbdata[2].p > 150.0*PI/180.0) {
            damp_motors();
            if(print_flag==1) {
                cout<<"angle2 error"<<endl;
                cout<<leg_state.cbdata[2].p<<endl;
                print_flag = 0;
            } 
            shut_down = 1;
        }
        if(leg_state.cbdata[3].p < -60.0*PI/180.0 || leg_state.cbdata[3].p > 50.0*PI/180.0) {
            damp_motors();
            if(print_flag==1) {
                cout<<"angle3 error"<<endl;
                cout<<leg_state.cbdata[3].p<<endl;
                print_flag = 0;
            } 
            shut_down = 1;
        }
        if(leg_state.cbdata[4].p < -60.0*PI/180.0 || leg_state.cbdata[4].p > 130.0*PI/180.0) {
            damp_motors();
            if(print_flag==1) {
                cout<<"angle4 error"<<endl;
                cout<<leg_state.cbdata[4].p<<endl;
                print_flag = 0;
            } 
            shut_down = 1;
        }
        if(leg_state.cbdata[5].p < -150.0*PI/180.0 || leg_state.cbdata[5].p > -35.0*PI/180.0) {
            damp_motors();
            if(print_flag==1) {
                cout<<"angle5 error"<<endl;
                cout<<leg_state.cbdata[5].p<<endl;
                print_flag = 0;
            } 
            shut_down = 1;
        }
        //torque_limit
        for(int i(0);i<2;i++){
            if(leg_state.cbdata[i].t < -30.0 || leg_state.cbdata[i].t > 30.0) {
                damp_motors();
                if(print_flag==1) {
                    cout<<"error: torque"<<i<<endl;
                    print_flag = 0;
                } 
                shut_down = 1;
            }
        }
        for(int i(3);i<5;i++){
            if(leg_state.cbdata[i].t < -30.0 || leg_state.cbdata[i].t > 30.0) {
                damp_motors();
                if(print_flag==1) {
                    cout<<"error: torque"<<i<<endl;
                    print_flag = 0;
                } 
                shut_down = 1;
            }
        }
        if(leg_state.cbdata[2].t < -45.0 || leg_state.cbdata[2].t > 45.0) {
            damp_motors();
            if(print_flag==1) {
                cout<<"error: torque"<<2<<endl;
                print_flag = 0;
            } 
            shut_down = 1;
        }
        if(leg_state.cbdata[5].t < -45.0 || leg_state.cbdata[5].t > 45.0) {
            damp_motors();
            if(print_flag==1) {
                cout<<"error: torque"<<5<<endl;
                print_flag = 0;
            } 
            shut_down = 1;
        }


        _lastRuntime = (float)t.getSeconds();
        _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
        _maxRuntime = std::max(_maxRuntime, _lastRuntime);
        // cout<<"maxPeriod:"<<_maxPeriod<<endl;
        // cout<<"maxRuntime:"<<_maxRuntime<<endl;
        safe_runtime = _lastRuntime;
        safe_periodtime = _lastPeriodTime;
        //延时
        int m = read(timerFd, &missed, sizeof(missed));
        (void)m;
    }

    return NULL;
}

//compute_foot_grf_thread
void* compute_foot_grf_thread(void* args)
{

    cout<<"compute_foot_grf start!"<<endl;

    //初始化定时器
    float _period = 0.001;
    auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
    int seconds = (int)_period;
    int nanoseconds = (int)(1e9 * std::fmod(_period, 1.f));
    Timer t;
    itimerspec timerSpec;
    timerSpec.it_interval.tv_sec = seconds;
    timerSpec.it_value.tv_sec = seconds;
    timerSpec.it_value.tv_nsec = nanoseconds;
    timerSpec.it_interval.tv_nsec = nanoseconds;
    timerfd_settime(timerFd, 0, &timerSpec, nullptr);
    unsigned long long missed = 0;
    float _lastRuntime = 0;
    float _lastPeriodTime = 0;
    float _maxPeriod = 0;
    float _maxRuntime = 0;

    while(!shut_down)
    {
        //计时
        _lastPeriodTime = (float)t.getSeconds();
        t.start();

        /********************** running begin **********************/
        //更新数据
        legstate_update();

        float desire_v = 0.5;//0.8
        stc.desired_xspeed = desire_v;
        //calculate grf  500HZ
        stc_tau = stc.get_action(user_cmd);


        /********************** running end **********************/

        _lastRuntime = (float)t.getSeconds();
        _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
        _maxRuntime = std::max(_maxRuntime, _lastRuntime);
        Max_p = _maxPeriod;
        Max_r = _maxRuntime;
        // cout<<"_maxPeriod:"<<_maxPeriod<<endl;
        // cout<<"_lastRuntime:"<<_lastRuntime<<endl;
        stc_runtime = _lastRuntime;
        stc_periodtime = _lastPeriodTime;
        /// 延时
        int m = read(timerFd, &missed, sizeof(missed));
        (void)m;

    }
    return NULL;

}

//legcontrol_thread
void* legcontrol_thread(void* args)
{

    cout<<"leg controller start!"<<endl;

    //初始化控制器
 	swing_leg_controller swc(&leg_state,&gait_gen,v_body);
    leg_controller l_controller(&leg_state,&gait_gen,&swc,&stc);

    //初始化定时器
    float _period = 0.001;
    auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
    int seconds = (int)_period;
    int nanoseconds = (int)(1e9 * std::fmod(_period, 1.f));
    Timer t;
    itimerspec timerSpec;
    timerSpec.it_interval.tv_sec = seconds;
    timerSpec.it_value.tv_sec = seconds;
    timerSpec.it_value.tv_nsec = nanoseconds;
    timerSpec.it_interval.tv_nsec = nanoseconds;
    timerfd_settime(timerFd, 0, &timerSpec, nullptr);
    unsigned long long missed = 0;
    float _lastRuntime = 0;
    float _lastPeriodTime = 0;
    float _maxPeriod = 0;
    float _maxRuntime = 0;

    while(!shut_down)
    {
        //计时
        _lastPeriodTime = (float)t.getSeconds();
        t.start();

        /********************** running begin **********************/
        //更新数据
        // legstate_update();

        float desire_v = 0.5;//0.8
        swc.desired_xspeed = desire_v;
        //leg控制  500HZ
        //0 initial
        //1 pd+force

        l_controller.get_action(&Mcmd,1,stc_tau,user_cmd);
        stance = gait_gen.leg_state[0];
        l_leg_p = swc.postarget[0];
        r_leg_p = swc.postarget[1];
        l_leg_v = swc.veltarget[0];
        r_leg_v = swc.veltarget[1];
        // l_footP_begin = swc.foot_position_body[0];
        // r_footP_begin = swc.foot_position_body[1];
        l_desired_angle = swc.angleTarget[0];
        r_desired_angle = swc.angleTarget[1];
        global_time = l_controller.timer;
        //驱动leg执行器
        motor_cmd_write(Mcmd);
        
        
        //打印数据
        // for(int j=0;j<6;j++){
        //     for(int i=0;i<6;i++){
        //         printf("%x ",cbmsg[j].data[i]);
        //     }
        //     printf("\n");
        // }
        // for(int i=0;i<6;i++){
        //     cb_Inf(leg_state.cbdata+i);
        // }
        // printf("\r\n");

        /********************** running end **********************/

        _lastRuntime = (float)t.getSeconds();
        // cout<<"lastPeriodTime:"<<_lastPeriodTime<<endl;
        _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
        _maxRuntime = std::max(_maxRuntime, _lastRuntime);
        Max_p = _maxPeriod;
        Max_r = _maxRuntime;
        // cout<<"maxPeriod:"<<_maxPeriod<<endl;
        // cout<<"maxRuntime:"<<_maxRuntime<<endl;
        // cout<<"Runtime:"<<_lastRuntime<<endl;
        swing_runtime = _lastRuntime;
        swing_periodtime = _lastPeriodTime;
        // cout<<leg_state.varphi/ PI * 180.0<<endl;
        
        /// 延时
        int m = read(timerFd, &missed, sizeof(missed));
        (void)m;

    }
    return NULL;
}

//estimator_thread
void* estimator_thread(void* args)
{

    cout<<"state estimator start!"<<endl;

    //variables
    float timestep_length = 0.001;
    //initialize the estimator
    PosVelEstimator Estimator(&leg_state,&gait_gen,timestep_length);

    //初始化定时器
    float _period = timestep_length;
    auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
    int seconds = (int)_period;
    int nanoseconds = (int)(1e9 * std::fmod(_period, 1.f));
    Timer t;
    itimerspec timerSpec;
    timerSpec.it_interval.tv_sec = seconds;
    timerSpec.it_value.tv_sec = seconds;
    timerSpec.it_value.tv_nsec = nanoseconds;
    timerSpec.it_interval.tv_nsec = nanoseconds;
    timerfd_settime(timerFd, 0, &timerSpec, nullptr);
    unsigned long long missed = 0;
    float _lastRuntime = 0;
    float _lastPeriodTime = 0;
    float _maxPeriod = 0;
    float _maxRuntime = 0;

    while(!shut_down)
    {
        //计时
        _lastPeriodTime = (float)t.getSeconds();
        t.start();

        /********************** running begin **********************/
        Estimator.run();
        vel_imu = Estimator.vel_imu;
        vel_kine[0] = Estimator.vel_kine[0];
        vel_kine[1] = Estimator.vel_kine[1];
        /********************** running end **********************/

        _lastRuntime = (float)t.getSeconds();
        // cout<<"lastPeriodTime:"<<_lastPeriodTime<<endl;
        _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
        _maxRuntime = std::max(_maxRuntime, _lastRuntime);
        Max_p = _maxPeriod;
        Max_r = _maxRuntime;
        // cout<<"maxPeriod:"<<_maxPeriod<<endl;
        // cout<<"maxRuntime:"<<_maxRuntime<<endl;
        // cout<<"Runtime:"<<_lastRuntime<<endl;

        // cout<<leg_state.varphi/ PI * 180.0<<endl;
        esti_runtime = _lastRuntime;
        esti_periodtime = _lastPeriodTime;
        /// 延时
        int m = read(timerFd, &missed, sizeof(missed));
        (void)m;

    }
    return NULL;
}

//record_thread
void* record_thread(void* args)
{
    cout << "record_thread start!" << endl;

    //生成数据编号
    char result[100] = {0};
    sprintf(result, "/home/hesam/0725/dataFile%s.txt", ch);
    ofstream dataFile;
    dataFile.open(result, ofstream::app);

    //初始化定时器
    float _period = 0.002;
    auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
    int seconds = (int)_period;
    int nanoseconds = (int)(1e9 * std::fmod(_period, 1.f));
    Timer t;
    itimerspec timerSpec;
    timerSpec.it_interval.tv_sec = seconds;
    timerSpec.it_value.tv_sec = seconds;
    timerSpec.it_value.tv_nsec = nanoseconds;
    timerSpec.it_interval.tv_nsec = nanoseconds;
    timerfd_settime(timerFd, 0, &timerSpec, nullptr);
    unsigned long long missed = 0;
    float _lastRuntime = 0;
    float _lastPeriodTime = 0;
    float _maxPeriod = 0;
    float _maxRuntime = 0;

    while(!shut_down)
    {
        //计时
        _lastPeriodTime = (float)t.getSeconds();
        t.start();

        /*** record the data ***/
        // 朝TXT文档中写入数据
        dataFile <<Mcmd.cmd[0].t << ", " << Mcmd.cmd[1].t << ", " << Mcmd.cmd[2].t << ", " 
                << Mcmd.cmd[3].t << ", " << Mcmd.cmd[4].t << ", " << Mcmd.cmd[5].t << ", " 
                << leg_state.cbdata[0].p << ", " << leg_state.cbdata[1].p << ", " << leg_state.cbdata[2].p << ", " 
                << leg_state.cbdata[3].p << ", " << leg_state.cbdata[4].p << ", " << leg_state.cbdata[5].p << ", " 
                << leg_state.cbdata[0].v << ", " << leg_state.cbdata[1].v << ", " << leg_state.cbdata[2].v << ", " 
                << leg_state.cbdata[3].v << ", " << leg_state.cbdata[4].v << ", " << leg_state.cbdata[5].v << ", "
                << leg_state.cbdata[0].t << ", " << leg_state.cbdata[1].t << ", " << leg_state.cbdata[2].t << ", " 
                << leg_state.cbdata[3].t << ", " << leg_state.cbdata[4].t << ", " << leg_state.cbdata[5].t << ", " 
                // << poserror.error[0] << ", " << poserror.error[1] << ", " << poserror.error[2] << ", " 
                // << poserror.error[3] << ", " << poserror.error[4] << ", " << poserror.error[5] << ", " 
                << stance << ", "
                << l_leg_p.x << ", "<< l_leg_p.y << ", "<< l_leg_p.z << ", "
                << r_leg_p.x << ", "<< r_leg_p.y << ", "<< r_leg_p.z << ", "
                << stc_tau[0] << ", "<< stc_tau[1] << ", "<< stc_tau[2] << ", "
                << stc_tau[3] << ", "<< stc_tau[4] << ", "<< stc_tau[5] << ", "
                << leg_state.rpy[0] << ", "<< leg_state.rpy[1] << ", "<< leg_state.rpy[2] << ", "
                << leg_state.com_velocity[0] << ", "<< leg_state.com_velocity[1] << ", "<< leg_state.com_velocity[2] << ", "
                << leg_state.com_height << ", "<< esti_runtime << ", " << esti_periodtime  << ", "
                << swing_runtime << ", " << swing_periodtime  << ", "
                << stc_runtime << ", " << stc_periodtime  << ", "
                << record_runtime << ", " << record_periodtime << ", " << global_time << ", "
                // << leg_state.omega[0] << ", "<< leg_state.omega[1] << ", " << leg_state.omega[2] << ", "
                // << leg_state.acc[0] << ", "<< leg_state.acc[1] << ", " << leg_state.acc[2] << ", "
                << leg_state.com_position[0] << ", "<< leg_state.com_position[1] << ", " << leg_state.com_position[2] << ", "
                << safe_runtime << ", " << safe_periodtime << ", "
                << leg_state.foot_p[0][0] << ", "<< leg_state.foot_p[0][1] << ", " << leg_state.foot_p[0][2] << ", "
                << leg_state.foot_p[1][0] << ", "<< leg_state.foot_p[1][1] << ", " << leg_state.foot_p[1][2] << ", "
                << l_leg_v[0] << ", "<< l_leg_v[1] << ", "<< l_leg_v[2] << ", "
                << r_leg_v[0] << ", "<< r_leg_v[1] << ", "<< r_leg_v[2] << ", "
                << leg_state.vicon_COMpos[0] << ", "<< leg_state.vicon_COMpos[1] << ", " << leg_state.vicon_COMpos[2] << ", "
                << leg_state.vicon_COMvel[0] << ", "<< leg_state.vicon_COMvel[1] << ", " << leg_state.vicon_COMvel[2] << ", "
                << leg_state.vicon_pos[0] << ", "<< leg_state.vicon_pos[1] << ", " << leg_state.vicon_pos[2] << ", "
                << leg_state.vicon_rpy[0] << ", "<< leg_state.vicon_rpy[1] << ", " << leg_state.vicon_rpy[2] << ", "
                // << stc.p_com_des[0] << ", "<< stc.p_com_des[1] << ", "<< stc.p_com_des[2] << ", "
                // << stc.w_com_des[0] << ", "<< stc.w_com_des[1] << ", "<< stc.w_com_des[2] << ", "
                // << leg_state.acc[0] << ", "<< leg_state.acc[1] << ", " << leg_state.acc[2] << ", "
                << vel_kine[0][0] << ", "<< vel_kine[0][1] << ", " << vel_kine[0][2] << ", "
                << vel_kine[1][0] << ", "<< vel_kine[1][1] << ", " << vel_kine[1][2] << ", "
                << vel_imu[0] << ", "<< vel_imu[1] << ", " << vel_imu[2] << ", "
                << leg_state.omega[0] << ", "<< leg_state.omega[1] << ", " << leg_state.omega[2] << ", "
                << stc.w_com_des[2] << ", " << stc.tau_pd[0] << ", " << stc.tau_pd[1] << ", " << stc.tau_pd[2] << ", "
                << leg_state.acc[0] << ", "<< leg_state.acc[1] << ", " << leg_state.acc[2]
                // << l_desired_angle.q[0] << ", " << l_desired_angle.q[1] << ", " << l_desired_angle.q[2] << ", "
                // << r_desired_angle.q[0] << ", " << r_desired_angle.q[1] << ", " << r_desired_angle.q[2] 
                << std::endl;


        _lastRuntime = (float)t.getSeconds();
        _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
        _maxRuntime = std::max(_maxRuntime, _lastRuntime);
        // cout<<"maxPeriod:"<<_maxPeriod<<endl;
        // cout<<"maxRuntime:"<<_maxRuntime<<endl;
        record_runtime = _lastRuntime;
        record_periodtime = _lastPeriodTime;
        //延时
        int m = read(timerFd, &missed, sizeof(missed));
        (void)m;
    }
    dataFile.close();
    return NULL;
}

int main(int argc, char **argv)
{
    cout << "main_thread" << endl;

    // initial variables
    stc_tau.setConstant(0);
    user_cmd.resize(4);
    user_cmd << 0,0,0.41,0.0;   //vx,vy,height,dyaw
    leg_state.com_height = user_cmd[2];
    leg_state.com_velocity[0] = 0;
    leg_state.com_velocity[1] = 0;
    leg_state.com_velocity[2] = 0;
    leg_state.omega_world[0] = 0;
    leg_state.omega_world[1] = 0;
    leg_state.omega_world[2] = 0;
    leg_state.com_position[0] = 0;
    leg_state.com_position[1] = 0;
    leg_state.com_position[2] = user_cmd[2];

    //初始化can0，can1
    CAN_init();
    
    //开启多线程
    thread_setup();

    //启动电机
    setup_motors();

    //判断leg硬件是否就绪
    while(can0_recieved == 0 || can1_recieved == 0);
    cout << "Can is Ready!" << endl;

    legstate_update();
    sleep(1);

    // reset_motors();
    //test
    // cmd_transfer(3,&L_msgs[2],69*PI/180.0,0,8,0.2,0);
    // can0_tx(L_msgs[2].data,3);

    // //腿初始化
    setpoint(0.06,0.11,-0.26); // 0.06 0.11 -0.26
    cout<<"first step"<<endl;
    legstate_update();
    int ret = pthread_create(&tids1[3], NULL, safety_thread, NULL);
    if (ret != 0){
        cout << "pthread_create3 error: error_code=" << ret << endl;
    }
    setpoint1(-0.003,0.1,-0.295);//0.04
    cout<<"leg init finished!"<<endl;

    legstate_update();
    for(int i=0;i<6;i++){
        cb_Inf(leg_state.cbdata+i);
    }
    printf("\r\n");

    //infer if imu is ready
    // while(imu_received == 0);
    cout << "Hardware is Ready!" << endl;
    //infer if vicon is ready
    while(vicon_received == 0);
    cout << "Hardware2 is Ready!" << endl;
    //record yaw bias and xy bias
    yaw_bias = leg_state.rpy[2];
    leg_state.pos_offset[0] = leg_state.vicon_pos[0];
    leg_state.pos_offset[1] = leg_state.vicon_pos[1];
    //wait 1s
    sleep(1);

    time_t tt = time(NULL);
    strftime(ch, sizeof(ch) - 1, "%H%M", localtime(&tt));

    control_threadcreate();    

    signal(SIGINT, sighand);

    while(!shut_down){

        // reset_motors();
        // legstate_update();
        // for(int i=0;i<6;i++){
        //     cb_Inf(leg_state.cbdata+i);
        // }
        // // footPoint_pos_Inf();
        // printf("\r\n");

        // // // printf("\r\n");
        // cout<<stc.GRF<<endl;
        // cout<<" "<<endl;
        // sleep(1);
        // Sleep_us(200000);
        Sleep_us(20000);
    }
    Sleep_us(20000);
    reset_motors();
    cout<<"done!"<<endl;

    /* Join the thread and wait until it is done */
    for (int i(0); i < 4; i ++){
        int ret = pthread_join(tids1[i], NULL);
        if (ret)  printf("join pthread1 failed\n");
        ret = pthread_join(tids2[i], NULL);
        if (ret)  printf("join pthread2 failed\n");
    }

    return 0;

}

void thread_setup(void){
    //定义线程的 id 变量，多个变量使用数组
    int ret;

    struct sched_param param;
    pthread_attr_t attr;

    /* Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr);
    if (ret) {
            printf("init pthread attributes failed\n");
    }
    /* Set a specific stack size  */
    ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
    }
    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret) {
            printf("pthread setschedpolicy failed\n");
    }
    param.sched_priority = 99;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret) {
            printf("pthread setschedparam failed\n");
    }
    // /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
    }
    pthread_attr_getschedparam(&attr, &param);
    cout<<"can_thread prior:"<<param.sched_priority<<endl;
    //参数依次是：创建的线程id，线程参数，调用的函数，传入的函数参数
    ret = pthread_create(&tids1[0], &attr, Can0_thread, NULL);
    if (ret != 0){
        cout << "pthread_create0 error: error_code=" << ret << endl;
    }

    ret = pthread_create(&tids1[1], &attr, Can1_thread, NULL);
    if (ret != 0){
        cout << "pthread_create1 error: error_code=" << ret << endl;
    }

    cpu_set_t mask;
    printf("pid=%d\n", getpid());
    CPU_ZERO(&mask);
    CPU_SET(1, &mask);//绑定cpu1
    pthread_setaffinity_np(tids1[0], sizeof(cpu_set_t), &mask) ;
    pthread_setaffinity_np(tids1[1], sizeof(cpu_set_t), &mask) ;



    ret = pthread_create(&tids1[2], NULL, imu_thread, NULL);
    if (ret != 0){
        cout << "pthread_create2 error: error_code=" << ret << endl;
    }

    ret = pthread_create(&tids1[3], NULL, vicon_thread, NULL);
    if (ret != 0){
        cout << "pthread_create3 error: error_code=" << ret << endl;
    }

}

void control_threadcreate(void){
    struct sched_param param;
    pthread_attr_t attr;

    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
            printf("mlockall failed: %m\n");
    }
    /* Initialize pthread attributes (default values) */
    int ret = pthread_attr_init(&attr);
    if (ret) {
            printf("init pthread attributes failed\n");
    }
    /* Set a specific stack size  */
    ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
    }
    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret) {
            printf("pthread setschedpolicy failed\n");
    }
    param.sched_priority = 79;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret) {
            printf("pthread setschedparam failed\n");
    }
    // /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
            printf("pthread setinheritsched failed\n");
    }
    pthread_attr_getschedparam(&attr, &param);
    cout<<"compute_grf_thread prior:"<<param.sched_priority<<endl;
    ret = pthread_create(&tids2[0], &attr, compute_foot_grf_thread, NULL);
    if (ret != 0){
        cout << "grf_pthread error: error_code=" << ret << endl;
    }

    param.sched_priority = 99;
    ret = pthread_attr_setschedparam(&attr, &param);
    pthread_attr_getschedparam(&attr, &param);
    cout<<"legc_thread prior:"<<param.sched_priority<<endl;
    ret = pthread_create(&tids2[1], &attr, legcontrol_thread, NULL);
    if (ret != 0){
        cout << "ctr_pthread error: error_code=" << ret << endl;
    } 

    param.sched_priority = 98;
    ret = pthread_attr_setschedparam(&attr, &param);
    pthread_attr_getschedparam(&attr, &param);
    cout<<"estimator_thread prior:"<<param.sched_priority<<endl;
    ret = pthread_create(&tids2[2], &attr, estimator_thread, NULL);
    if (ret != 0){
        cout << "estimate_pthread error: error_code=" << ret << endl;
    }   

    cpu_set_t mask;
    printf("pid=%d\n", getpid());
    CPU_ZERO(&mask);
    CPU_SET(1, &mask);//绑定cpu1
    pthread_setaffinity_np(tids2[1], sizeof(cpu_set_t), &mask) ;

    CPU_ZERO(&mask);
    CPU_SET(2, &mask);//绑定cpu2
    pthread_setaffinity_np(tids2[2], sizeof(cpu_set_t), &mask) ;

    CPU_ZERO(&mask);
    CPU_SET(3, &mask);//绑定cpu3
    pthread_setaffinity_np(tids2[0], sizeof(cpu_set_t), &mask) ;

    // param.sched_priority = 48;
    // ret = pthread_attr_setschedparam(&attr, &param);
    // pthread_attr_getschedparam(&attr, &param);
    // cout<<"record_thread prior:"<<param.sched_priority<<endl;
    ret = pthread_create(&tids2[3], NULL, record_thread, NULL);
    if (ret != 0){
        cout << "record_pthread error: error_code=" << ret << endl;
    }
}

void legstate_update(){
    legState_m.lock();
    cb_trans(cbmsg,leg_state.cbdata);
    legState_m.unlock();
}

void CAN_init(){
    struct sockaddr_can addr0;
	struct ifreq ifr0;
    struct sockaddr_can addr1;
	struct ifreq ifr1;

    if ((socket0 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1) {
		perror("Error while opening socket0");
	}
    const char *ifname0 = "can0";
	strcpy(ifr0.ifr_name, ifname0);
	ioctl(socket0, SIOCGIFINDEX, &ifr0);
    addr0.can_family  = AF_CAN;
	addr0.can_ifindex = ifr0.ifr_ifindex;
    if (bind(socket0, (struct sockaddr *)&addr0, sizeof(addr0)) == -1) {
		perror("Error in socket0 bind");
	}

    if ((socket1 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1) {
		perror("Error while opening socket1");
	}
    const char *ifname1 = "can1";
	strcpy(ifr1.ifr_name, ifname1);
	ioctl(socket1, SIOCGIFINDEX, &ifr1);
    addr1.can_family  = AF_CAN;
	addr1.can_ifindex = ifr1.ifr_ifindex;
    if (bind(socket1, (struct sockaddr *)&addr1, sizeof(addr1)) == -1) {
		perror("Error in socket1 bind");
	}
}

void can0_tx(uint8_t tdata[],uint8_t id){
	struct can_frame frame;
    frame.can_id  = id;
    frame.can_dlc = 8;
    memcpy(frame.data,tdata,8);
    write(socket0, &frame, sizeof(struct can_frame));
}

void can1_tx(uint8_t tdata[],uint8_t id){
	struct can_frame frame;
    frame.can_id  = id;
    frame.can_dlc = 8;
    memcpy(frame.data,tdata,8);
    write(socket1, &frame, sizeof(struct can_frame));
}

void reset_motors(){
    for(int i=1;i<=3;i++){
		can0_tx(reset,i);
		can1_tx(reset,i+bias);
        //sleep
        Sleep_us(300);
	}
}

void damp_motors(){
    for(int i=1;i<=3;i++){
        cmd_transfer(i+1,&mes,0,0,0,2,0);
        can0_tx(mes.data,i+1);
        cmd_transfer(i+4,&mes,0,0,0,2,0);
        can1_tx(mes.data,i+1+bias);
        //sleep
        Sleep_us(300);
	}
}

void setup_motors(){
    for(int i=1;i<=3;i++){
        can0_tx(set_foc,i);
        can1_tx(set_foc,i+bias);
        //sleep
        Sleep_us(300);

	}
}

void motor_control(Leg_command leg_cmd){
    float kd = 1.5;
    int i;
    for(i=0;i<2;i++){
        cmd_transfer(i+1,&L_msgs[i],0,0,0,kd,leg_cmd.torque[i]);
        can0_tx(L_msgs[i].data,i+1);
        cmd_transfer(i+4,&R_msgs[i],0,0,0,kd,leg_cmd.torque[i+3]);
        can1_tx(R_msgs[i].data,i+1+bias);
        //延时
        Sleep_us(300);
	}
    i = 2;
    cmd_transfer(i+1,&L_msgs[i],0,0,0,kd,leg_cmd.torque[i]);
    can0_tx(L_msgs[i].data,i+1);
    cmd_transfer(i+4,&R_msgs[i],0,0,0,kd,leg_cmd.torque[i+3]);
    can1_tx(R_msgs[i].data,i+1+bias);
}

void motor_cmd_write(Motor_cmd Mcmd){
    int i;
    for(i=0;i<2;i++){
        cmd_transfer(i+1, &L_msgs[i], Mcmd.cmd[i].p, Mcmd.cmd[i].v, Mcmd.cmd[i].kp, Mcmd.cmd[i].kd, Mcmd.cmd[i].t);
        can0_tx(L_msgs[i].data,i+1);
        cmd_transfer(i+4, &R_msgs[i], Mcmd.cmd[i+3].p, Mcmd.cmd[i+3].v, Mcmd.cmd[i+3].kp, Mcmd.cmd[i+3].kd, Mcmd.cmd[i+3].t);
        can1_tx(R_msgs[i].data,i+1+bias);
        //延时
        Sleep_us(300);
	}
    i = 2;
    cmd_transfer(i+1, &L_msgs[i], Mcmd.cmd[i].p, Mcmd.cmd[i].v, Mcmd.cmd[i].kp, Mcmd.cmd[i].kd, Mcmd.cmd[i].t);
    can0_tx(L_msgs[i].data,i+1);
    cmd_transfer(i+4, &R_msgs[i], Mcmd.cmd[i+3].p, Mcmd.cmd[i+3].v, Mcmd.cmd[i+3].kp, Mcmd.cmd[i+3].kd, Mcmd.cmd[i+3].t);
    can1_tx(R_msgs[i].data,i+1+bias);
}

void footPoint_pos_Inf(){
    CBData cbdata[6];
    //获取当前关节位置
    cb_trans(cbmsg,cbdata);
    angle[0].q[0] = cbdata[0].p;
    angle[0].q[1] = cbdata[1].p;
    angle[0].q[2] = cbdata[2].p;
    angle[1].q[0] = cbdata[3].p;
    angle[1].q[1] = cbdata[4].p;
    angle[1].q[2] = cbdata[5].p;
    //求正运动学，计算当前末端位置
    Kinematics_ref(&angle[0],&p0[0],0);
    Kinematics_ref(&angle[1],&p0[1],1);
    cout<<"left foot position: "<<p0[0].x<<","<<p0[0].y<<","<<p0[0].z<<endl;
    cout<<"right foot position: "<<p0[1].x<<","<<p0[1].y<<","<<p0[1].z<<endl;

}

void setpoint(float x,float y,float z){
    /***  进行点位控制  ***/
    CBData cbdata[6];
    //获取当前关节位置和速度
    cb_trans(cbmsg,cbdata);
    angle[0].q[0] = cbdata[0].p;
    angle[0].q[1] = cbdata[1].p;
    angle[0].q[2] = cbdata[2].p;
    angleV[0].q[0] = cbdata[0].v;
    angleV[0].q[1] = cbdata[1].v;
    angleV[0].q[2] = cbdata[2].v;
    angle[1].q[0] = cbdata[3].p;
    angle[1].q[1] = cbdata[4].p;
    angle[1].q[2] = cbdata[5].p;
    angleV[1].q[0] = cbdata[3].v;
    angleV[1].q[1] = cbdata[4].v;
    angleV[1].q[2] = cbdata[5].v;

    //求正运动学，计算当前末端位置；求雅可比，计算当前末端速度
    Kinematics_ref(&angle[0],&p0[0],0);
    Kinematics_ref(&angle[1],&p0[1],1);
    v0[0].x = 0;
    v0[0].y = 0;
    v0[0].z = 0;
    v0[1].x = 0;
    v0[1].y = 0;
    v0[1].z = 0;
//		pos_Inf(&p0[0]);
//		pos_Inf(&p0[1]);
    
    //输入期望位置和速度
    pdes[0].x = x;
    pdes[0].y = y;
    pdes[0].z = z;
    vdes[0].x = 0;
    vdes[0].y = 0;
    vdes[0].z = 0;   
    pdes[1].x = x;
    pdes[1].y = -y;
    pdes[1].z = z;
    vdes[1].x = 0;
    vdes[1].y = 0;
    vdes[1].z = 0;
    
    //三次样条插补，逆运动学，pd控制
    init_chabu(&pdes[0],&vdes[0],&p0[0],&v0[0],Iter,0);
    init_chabu(&pdes[1],&vdes[1],&p0[1],&v0[1],Iter,1);
    for(int j=0;j<Iter;j++){	
        chabu(&nextpos[0],j+1,0);
        Inv_kinematics_ref(&L_angle,&nextpos[0],0);
        chabu(&nextpos[1],j+1,1);
        Inv_kinematics_ref(&R_angle,&nextpos[1],1);
        for(int i=0;i<3;i++){
            cmd_transfer(i+1,&L_msgs[i],L_angle.q[i],0,8,0.2,0);
            can0_tx(L_msgs[i].data,i+1);
            cmd_transfer(i+4,&R_msgs[i],R_angle.q[i],0,8,0.2,0);
            can1_tx(R_msgs[i].data,i+1+bias);
            Sleep_us(300);
        }
        // cout<<L_angle.q[0]*180/PI<<","<<L_angle.q[1]*180/PI<<","<<L_angle.q[2]*180/PI<<","
        //     <<R_angle.q[0]*180/PI<<","<<R_angle.q[1]*180/PI<<","<<R_angle.q[2]*180/PI<<endl;
    }
}

void setpoint1(float x,float y,float z){
    /***  进行点位控制  ***/
    CBData cbdata[6];
    //获取当前关节位置和速度
    cb_trans(cbmsg,cbdata);
    angle[0].q[0] = cbdata[0].p;
    angle[0].q[1] = cbdata[1].p;
    angle[0].q[2] = cbdata[2].p;
    angleV[0].q[0] = cbdata[0].v;
    angleV[0].q[1] = cbdata[1].v;
    angleV[0].q[2] = cbdata[2].v;
    angle[1].q[0] = cbdata[3].p;
    angle[1].q[1] = cbdata[4].p;
    angle[1].q[2] = cbdata[5].p;
    angleV[1].q[0] = cbdata[3].v;
    angleV[1].q[1] = cbdata[4].v;
    angleV[1].q[2] = cbdata[5].v;

    //求正运动学，计算当前末端位置；求雅可比，计算当前末端速度
    Kinematics_ref(&angle[0],&p0[0],0);
    Kinematics_ref(&angle[1],&p0[1],1);
    v0[0].x = 0;
    v0[0].y = 0;
    v0[0].z = 0;
    v0[1].x = 0;
    v0[1].y = 0;
    v0[1].z = 0;
//		pos_Inf(&p0[0]);
//		pos_Inf(&p0[1]);
    
    //输入期望位置和速度
    pdes[0].x = x;
    pdes[0].y = y;
    pdes[0].z = z;
    vdes[0].x = 0;
    vdes[0].y = 0;
    vdes[0].z = 0;   
    pdes[1].x = x;
    pdes[1].y = -y;
    pdes[1].z = z;
    vdes[1].x = 0;
    vdes[1].y = 0;
    vdes[1].z = 0;
    
    //三次样条插补，逆运动学，pd控制
    init_chabu(&pdes[0],&vdes[0],&p0[0],&v0[0],Iter,0);
    init_chabu(&pdes[1],&vdes[1],&p0[1],&v0[1],Iter,1);
    for(int j=0;j<Iter;j++){	
        chabu(&nextpos[0],j+1,0);
        Inv_kinematics_ref(&L_angle,&nextpos[0],0);
        chabu(&nextpos[1],j+1,1);
        Inv_kinematics_ref(&R_angle,&nextpos[1],1);
        for(int i=0;i<3;i++){
            cmd_transfer(i+1,&L_msgs[i],L_angle.q[i],0,20,0.2,0);
            can0_tx(L_msgs[i].data,i+1);
            cmd_transfer(i+4,&R_msgs[i],R_angle.q[i],0,20,0.2,0);
            can1_tx(R_msgs[i].data,i+1+bias);
            Sleep_us(300);
        }
        // cout<<L_angle.q[0]*180/PI<<","<<L_angle.q[1]*180/PI<<","<<L_angle.q[2]*180/PI<<","
        //     <<R_angle.q[0]*180/PI<<","<<R_angle.q[1]*180/PI<<","<<R_angle.q[2]*180/PI<<endl;
    }
    cout<<L_angle.q[0]*180/PI<<","<<L_angle.q[1]*180/PI<<","<<L_angle.q[2]*180/PI<<","
        <<R_angle.q[0]*180/PI<<","<<R_angle.q[1]*180/PI<<","<<R_angle.q[2]*180/PI<<endl;

}

void Sleep_us(int us)
{
    struct timeval delay;
    delay.tv_sec = 0;
    delay.tv_usec = us;
    select(0, NULL, NULL, NULL, &delay);
}