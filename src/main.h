#ifndef MAIN_H  // 헤더 파일 중복 포함 방지를 위한 전처리기
#define MAIN_H
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
#include <sys/mman.h>
#include <string.h>		// string function definitions
#include <fcntl.h>		// File control definitions
#include <errno.h>		// Error number definitions
#include <termios.h>	// POSIX terminal control definitions
#include <time.h>		// time calls
#include <sys/ioctl.h>
#include <math.h>
#include <string>
#include "iostream"
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <pthread.h>
#include <chrono>

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <thread>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>





#include "physics/physics_run.h"
#include "control/control_run.h"
#include "qt/qt_run.h"
#include "print/print_run.h"

#define CYCLE_NS 1000000

#define NSEC_PER_SEC 			1000000000

static int run = 1;
#define ASSERT_EQ(a, b) assert((a) == (b));

typedef struct STATE{
	JVec q;
	JVec q_dot;
	JVec q_ddot;
	JVec tau;
	JVec tau_ext;
	JVec e;
	JVec eint;
	JVec G;

	Vector6d x;                           //Task space
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6d F;
	Vector6d F_CB;
    Vector6d F_ext;

    double s_time;
}state;

typedef struct ROBOT_INFO{
	int Position;
	int aq_inc[JOINTNUM];
	int atoq_per[JOINTNUM];
	short dtor_per[JOINTNUM];
	int statusword[JOINTNUM];

	JVec q_target;
	JVec qdot_target;
	JVec qddot_target;
	JVec traj_time;
	

	STATE act;
	STATE des;
	STATE nom;

}ROBOT_INFO;
extern ROBOT_INFO robot_info;
extern std::mutex g_pages_mutex;
extern double gt;
extern b3RobotSimulatorClientAPI* sim;
extern const char* urdfFilePath;
extern double CONTROL_RATE;
extern btScalar fixedTimeStep;
extern Robot *robot;
extern float eef_mass;
extern double gt;
extern bool with_gui;
extern bool use_log;
extern std::mutex mtx; // 전역 뮤텍스
extern JVec q0;


#endif  // MAIN_H

