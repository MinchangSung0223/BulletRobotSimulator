#include "control_run.h"
#include "waypoint.h"

LR_Control *control;


void* control_run(void* param) {

    //thread setting
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);

    //Load Controller Settings
    control = new LR_Control();
    control->LRSetup(urdfFilePath);    


    double dt = fixedTimeStep;
    JVec q;
    JVec q_dot;
    Vector3d eef_forces;
    Vector3d eef_moments;
    

//Joint Trajectory
    std::vector<JVec> way_points;
    std::vector<double> delays;
    JVec q_des=JVec::Zero();
    JVec q_dot_des=JVec::Zero();
    JVec q_ddot_des=JVec::Zero();    
    JVec eint = JVec::Zero();
    JointWayPointGenerator(way_points,delays);
//Task Trajectory
    std::vector<SE3> task_way_points;
    std::vector<double> task_delays;
    Vector6d Task_eint = Vector6d::Zero();
    SE3 T0 = FKinBody(control->M,control->Blist,q0);
    Vector6d V0 = Vector6d::Zero();
    Vector6d VT = Vector6d::Zero();
    Vector6d V0_dot = Vector6d::Zero();
    Vector6d VT_dot = Vector6d::Zero();
    SE3 T_des=T0;
    Vector6d V_des = Vector6d::Zero();
    Vector6d V_des_dot = Vector6d::Zero();    
    TaskWayPointGenerator(task_way_points,task_delays,T0);


    JVec torques;
    while (run) {
        // 다음 주기 시간 계산
        next_period.tv_nsec += CYCLE_NS;
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_nsec -= 1000000000;
            next_period.tv_sec++;
        }
        // 실제 작업 수행
        JVec q = robot_info.act.q;
        JVec q_dot = robot_info.act.q_dot;

        //Trajectory Generation
        control->WayPointTaskTrajectory(task_way_points, task_delays, gt, T_des, V_des, V_des_dot);
        control->WayPointJointTrajectory(way_points, delays, gt,q_des,q_dot_des,q_ddot_des);       

        //Control
                 
        //torques = control->HinfControl(  q, q_dot, q_des, q_dot_des,q_ddot_des, eint, eef_mass);
        torques =control->TaskHinfControl(  q, q_dot, JVec::Zero(),  T_des, V_des, V_des_dot, Task_eint);        
        JVec e = q_des-q;
        eint+=e*dt;
        //eint = tanh_JVec(eint);
        std::lock_guard<std::mutex> lock(mtx);
        robot_info.des.q = q_des;
        robot_info.des.q_dot = q_dot_des;
        robot_info.des.tau = torques;
        robot_info.act.e = e;
        robot_info.act.eint = eint;
        // 다음 주기까지 대기
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }
    return NULL;
}

