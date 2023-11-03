
#include "main.h"
// RT_TASK Physics_task;
ROBOT_INFO robot_info;
b3RobotSimulatorClientAPI* sim;
double CONTROL_RATE = 1000.0;
btScalar fixedTimeStep = 1. / CONTROL_RATE;
Robot *robot;
LR_Control *control;
JVec q;
JVec q_dot;
JVec q_des=JVec::Zero();

JVec kMaxTorques=JVec::Ones()*1000;
Vector3d eef_forces;
Vector3d eef_moments;


unsigned int cycle_ns = 1000000; /* 1 ms */
double gt=0;

void signal_handler(int signum)
{
	//rt_task_delete(&Physics_task);
	//rt_task_delete(&safety_task);
	//rt_task_delete(&print_task);
	printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGWINCH)
		printf("╔═══════════════[SIGNAL INPUT SIGWINCH]══════════════╗\n");		
	else if(signum==SIGHUP)
		printf("╔════════════════[SIGNAL INPUT SIGHUP]═══════════════╗\n");
    printf("║                       Stopped!                     ║\n");
	printf("╚════════════════════════════════════════════════════╝\n");	
    exit(1);

    
}
#define CYCLE_NS 1000000   // 1ms
void *physics_run(void *param) {
    (void)param; // 파라미터를 사용하지 않는 경우
    double dt=((double) CYCLE_NS)/((double) NSEC_PER_SEC);	//period in second unit

    // 다음 주기까지의 시간을 저장하기 위한 변수
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);
    std::chrono::time_point<std::chrono::high_resolution_clock> prevClock, nowClock;
    std::chrono::microseconds duration;
    prevClock = std::chrono::high_resolution_clock::now();
    while (1) {
        // 다음 주기 시간 계산
        next_period.tv_nsec += CYCLE_NS;
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_nsec -= 1000000000;
            next_period.tv_sec++;
        }

        // 실제 작업 수행
        q = robot->get_q();
	    q_dot = robot->get_q_dot();
	    JVec torques = lr::GravityForces(q,control->g,control->Mlist, control->Glist, control->Slist);

	    robot->set_torques(torques,kMaxTorques);
	    //robot->apply_ext_forces(Vector3d(robot_info.act.F_ext[0],robot_info.act.F_ext[1],robot_info.act.F_ext[2]));
	    robot->apply_ext_forces(Vector3d(10,10,10));        
        //robot_info.act.q = JVec(sin(gt*M_PI*1),sin(gt*M_PI*2),sin(gt*M_PI*4),sin(gt*M_PI*8),sin(gt*M_PI*16),sin(gt*M_PI*32),sin(gt*M_PI*64)) ;

    	sim->stepSimulation();
        gt+=dt;
	    robot_info.act.q = q;
	    robot_info.act.q_dot = q_dot;
	    robot_info.act.tau = torques;        
        static int cnt_=0;
        if(++cnt_>999){
           nowClock = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(nowClock-prevClock);            
            printf("gt : %.3f, elapsed time : %.6f [s] \n",gt,duration.count()/1e6);
            cnt_ = 0;
            prevClock = nowClock;
        }

        // 다음 주기까지 대기
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }

    return NULL;
}

#define DRAW_CYCLE_NS 16666667 // 60hz
void *draw_run(void *param) {
    (void)param; // 파라미터를 사용하지 않는 경우
    // 다음 주기까지의 시간을 저장하기 위한 변수
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);
    std::chrono::time_point<std::chrono::high_resolution_clock> prevClock, nowClock;
    std::chrono::microseconds duration;
    prevClock = std::chrono::high_resolution_clock::now();
    while (1) {
        // 다음 주기 시간 계산
        next_period.tv_nsec += DRAW_CYCLE_NS; 
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_nsec -= 1000000000;
            next_period.tv_sec++;
        }

        // 실제 작업 수행

        btVector3 fromXYZ(0,0,0);
        btVector3 toXYZ(0,0,1);
        b3RobotSimulatorAddUserDebugLineArgs line_args;
        line_args.m_parentLinkIndex=8;
        sim->addUserDebugLine(fromXYZ,toXYZ,line_args);
        
        // 다음 주기까지 대기
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }

    return NULL;
}


void runQtApplication(int argc, char* argv[]) {
  QApplication a(argc, argv);
  // style our application with custom dark style
  QApplication::setStyle(new DarkStyle);

  // create frameless window (and set windowState or title)
  FramelessWindow framelessWindow;

  // create our mainwindow instance
  MainWindow *mainWindow = new MainWindow;
  // add the mainwindow to our custom frameless window
  framelessWindow.resize(1600,600);
  framelessWindow.setContent(mainWindow);
  framelessWindow.show();
  a.exec();
}

int main(int argc, char* argv[]){
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGWINCH, signal_handler);
	signal(SIGHUP, signal_handler);    




    const char* urdfFilePath = "/opt/RobotInfo/urdf/satellite/arm.urdf";
    bool with_gui=0;
    //-----------------Sim Setup------------------------
    sim = new b3RobotSimulatorClientAPI();
    bool isConnected;
    isConnected = sim->connect(eCONNECT_GUI);

    if (!isConnected)
    {
        printf("Cannot connect\n");
        return -1;
    }
    sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
    sim->setTimeOut(10);
    sim->syncBodies();	
    sim->setTimeStep(fixedTimeStep);
    sim->setGravity(btVector3(0, 0, -9.8));
    b3RobotSimulatorSetPhysicsEngineParameters args;
    sim->getPhysicsEngineParameters(args);
    int robotId = sim->loadURDF(urdfFilePath);
    int planeId = sim->loadURDF("/opt/RobotInfo/urdf/plane/plane.urdf");
    sim->setRealTimeSimulation(false);
    robot = new Robot(sim,robotId);	
    robot_info.act.F_ext=Vector6d::Zero();
    control = new LR_Control();
    control->LRSetup();


    std::thread qtThread(runQtApplication, argc, argv);

    //pthread setup
    pthread_t physics_thread,draw_thread;
    pthread_attr_t physics_attr,draw_attr;
    struct sched_param physics_param,draw_param;

    pthread_attr_init(&physics_attr);
    pthread_attr_init(&draw_attr);
    pthread_attr_setschedpolicy(&physics_attr, SCHED_FIFO);
    pthread_attr_setschedpolicy(&draw_attr, SCHED_FIFO);
    physics_param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    draw_param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_setschedparam(&physics_attr, &physics_param);
    pthread_attr_setschedparam(&draw_attr, &draw_param);
    pthread_create(&physics_thread, &physics_attr, physics_run, NULL);
    pthread_create(&draw_thread, &draw_attr, draw_run, NULL);



    pause();
    qtThread.join();	
    pthread_join(physics_thread, NULL);


	signal_handler(0);
    return 0;
}