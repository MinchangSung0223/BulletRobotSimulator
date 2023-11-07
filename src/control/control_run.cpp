#include "control_run.h"

JVec q;
JVec q_dot;
JVec q_des=JVec::Zero();
Vector3d eef_forces;
Vector3d eef_moments;
LR_Control *control;

void* control_run(void* param) {
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);
    control = new LR_Control();
    control->LRSetup(urdfFilePath);    

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
        JVec torques = lr::GravityForces(q,control->g,control->Mlist, control->Glist, control->Slist,eef_mass);
        robot_info.des.tau = torques;

        
        // 다음 주기까지 대기
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }
    return NULL;
}

