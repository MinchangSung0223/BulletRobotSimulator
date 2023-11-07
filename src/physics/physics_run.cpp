#include "physics_run.h"
Robot *robot;
JVec q0 = JVec(0,0,0,1.5708,0,1.5708,0);
JVec kMaxTorques=JVec::Ones()*1000;

double gt=0;
void* physics_run(void* param) {
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);

    //-----------------Sim Setup------------------------
    sim = new b3RobotSimulatorClientAPI();
    bool isConnected;
    if(with_gui)  isConnected = sim->connect(eCONNECT_GUI);
    else  isConnected = sim->connect(eCONNECT_DIRECT);
    if (!isConnected)
    {
        printf("Cannot connect\n");
        return NULL;
    }
    sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
    sim->setTimeOut(10);
    sim->syncBodies();	
    sim->setTimeStep(fixedTimeStep);
    sim->setGravity(btVector3(0, 0, -9.8));
    b3RobotSimulatorSetPhysicsEngineParameters args;
    sim->getPhysicsEngineParameters(args);
    int robotId = sim->loadURDF(urdfFilePath);
    sim->setRealTimeSimulation(false);
    robot = new Robot(sim,robotId);	
    robot->reset_q(q0);

    double dt = fixedTimeStep;

    robot_info.act.F_ext=Vector6d::Zero();
    std::chrono::time_point<std::chrono::high_resolution_clock> prevClock, nowClock;
    std::chrono::microseconds duration;
    prevClock = std::chrono::high_resolution_clock::now();
    while (run) {
        // 다음 주기 시간 계산
        next_period.tv_nsec += CYCLE_NS;
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_nsec -= 1000000000;
            next_period.tv_sec++;
        }

        // 실제 작업 수행
        JVec q = robot->get_q();
 	    JVec q_dot = robot->get_q_dot();        

        JVec torques = robot_info.des.tau;
        robot->set_torques(torques,kMaxTorques);
        sim->stepSimulation();
        robot_info.act.q = q;
        robot_info.act.q_dot = q_dot;
        robot_info.act.tau = robot_info.des.tau;
        gt +=dt;
      
        // 다음 주기까지 대기
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }
    return NULL;
}



// #include <random>
// #define CYCLE_NS 1000000   // 1ms
// void *physics_run(void *param) {
//     (void)param; // 파라미터를 사용하지 않는 경우
//     double dt=((double) CYCLE_NS)/((double) NSEC_PER_SEC);	//period in second unit

//     // 다음 주기까지의 시간을 저장하기 위한 변수
//     struct timespec next_period;
//     clock_gettime(CLOCK_MONOTONIC, &next_period);
//     std::chrono::time_point<std::chrono::high_resolution_clock> prevClock, nowClock;
//     std::chrono::microseconds duration;
//     prevClock = std::chrono::high_resolution_clock::now();
    
//     int val = 99;

//     robot->draw_eef_T(0.25,5);

//     std::vector<JVec> way_points;
//     std::vector<double> delays;

//     JVec RandJVec1=JVec::Zero();
//     JVec RandJVec2=JVec::Zero();
//     JVec RandJVec3=JVec::Zero();
//     JVec RandJVec4=JVec::Zero();
//     JVec RandJVec5=JVec::Zero();
//     JVec RandJVec6=JVec::Zero();
//     double min_range = -1.5708;
//     double max_range = 1.5708;    
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_real_distribution<> dis(min_range, max_range);

//     for (int i = 0; i < RandJVec1.size(); ++i) {
//         RandJVec1[i] = dis(gen);
//         RandJVec2[i] = dis(gen);
//         RandJVec3[i] = dis(gen);
//         RandJVec4[i] = dis(gen);
//         RandJVec5[i] = dis(gen);
//     }


//     way_points.push_back(q0);
//     way_points.push_back(JVec(1.5708,1.5708/2.0,0,0,0,0,0));
//     way_points.push_back(JVec(-1.5708,1.5708/2.0,0,0,0,0,0));
//     way_points.push_back(JVec(-1.5708,-1.5708/2.0,0,0,0,0,0));
//     way_points.push_back(JVec(1.5708,-1.5708/2.0,1.5708,0,0,0,0));
//     way_points.push_back(JVec(1.5708,1.5708/2.0,1.5708,0,0,0,0));
//     way_points.push_back(JVec(-1.5708,-1.5708/2.0,-1.5708,1.5708,0,0,0));
//     way_points.push_back(JVec(-1.5708,1.5708/2.0,-1.5708,1.5708,0,0,0));
//     way_points.push_back(JVec(0,0,0,1.5708,0,1.5708,0));
//     way_points.push_back(JVec(0,0,0,-1.5708,0,-1.5708,0));
//     way_points.push_back(RandJVec1);
//     way_points.push_back(RandJVec2);
//     way_points.push_back(RandJVec3);
//     way_points.push_back(RandJVec4);
//     way_points.push_back(RandJVec5);
//     way_points.push_back(RandJVec6);
    

//     for(int i =0;i<15;i++){
//         delays.push_back(10.0);
//     }
        

//     way_points.push_back(q0);
//     way_points.push_back(JVec(1.5708,1.5708/2.0,0,0,0,0,0));
//     way_points.push_back(JVec(-1.5708,1.5708/2.0,0,0,0,0,0));
//     way_points.push_back(JVec(-1.5708,-1.5708/2.0,0,0,0,0,0));
//     way_points.push_back(JVec(1.5708,-1.5708/2.0,1.5708,0,0,0,0));
//     way_points.push_back(JVec(1.5708,1.5708/2.0,1.5708,0,0,0,0));
//     way_points.push_back(JVec(-1.5708,-1.5708/2.0,-1.5708,1.5708,0,0,0));
//     way_points.push_back(JVec(-1.5708,1.5708/2.0,-1.5708,1.5708,0,0,0));
//     way_points.push_back(JVec(0,0,0,1.5708,0,1.5708,0));
//     way_points.push_back(JVec(0,0,0,-1.5708,0,-1.5708,0));
//     way_points.push_back(RandJVec1);
//     way_points.push_back(RandJVec2);
//     way_points.push_back(RandJVec3);
//     way_points.push_back(RandJVec4);
//     way_points.push_back(RandJVec5);
//     way_points.push_back(RandJVec6);
    

//     for(int i =0;i<15;i++){
//         delays.push_back(5.0);
//     }
        


//     way_points.push_back(q0);
//     way_points.push_back(JVec(1.5708,1.5708/2.0,0,0,0,0,0));
//     way_points.push_back(JVec(-1.5708,1.5708/2.0,0,0,0,0,0));
//     way_points.push_back(JVec(-1.5708,-1.5708/2.0,0,0,0,0,0));
//     way_points.push_back(JVec(1.5708,-1.5708/2.0,1.5708,0,0,0,0));
//     way_points.push_back(JVec(1.5708,1.5708/2.0,1.5708,0,0,0,0));
//     way_points.push_back(JVec(-1.5708,-1.5708/2.0,-1.5708,1.5708,0,0,0));
//     way_points.push_back(JVec(-1.5708,1.5708/2.0,-1.5708,1.5708,0,0,0));
//     way_points.push_back(JVec(0,0,0,1.5708,0,1.5708,0));
//     way_points.push_back(JVec(0,0,0,-1.5708,0,-1.5708,0));
//     way_points.push_back(RandJVec1);
//     way_points.push_back(RandJVec2);
//     way_points.push_back(RandJVec3);
//     way_points.push_back(RandJVec4);
//     way_points.push_back(RandJVec5);
//     way_points.push_back(RandJVec6);
    



//     for(int i =0;i<15;i++){
//         delays.push_back(3.0);
//     }
        


//     std::vector<SE3> task_way_points;
//     std::vector<double> task_delays;
//     SE3 X0 = FKinBody(control->M,control->Blist,q0);
//     SE3 XT = X0*MatrixExp6(VecTose3(Vector6d(-0.5,0.0,0.0,0.0,0.0,0.0)));

//     task_way_points.push_back(X0);
//     task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(-0.5,0.0,0.0,0.0,0.0,0.0))));
//     task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(-0.0,0.0,0.0,0.5,0.0,0.0))));
//     task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(-0.0,0.0,0.5,-0.5,0.0,0.0))));
//     task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(-0.0,0.0,0.0,0.5,0.5,0.0))));
//     task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(-0.0,0.0,0.0,0.5,-0.5,0.0))));
//     task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(-0.0,0.0,0.5,0.5,-0.5,0.5))));
//     task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(-0.0,0.0,0.0,0.5,-0.5,-0.5))));
//     task_delays.push_back(5.0);
//     task_delays.push_back(5.0);
//     task_delays.push_back(5.0);
//     task_delays.push_back(5.0);
//     task_delays.push_back(5.0);
//     task_delays.push_back(5.0);
//     task_delays.push_back(5.0);

//     JVec eint = JVec::Zero();

//     JVec q_des=JVec::Zero();
//     JVec q_dot_des=JVec::Zero();
//     JVec q_ddot_des=JVec::Zero();
//     Vector6d Task_eint = Vector6d::Zero();


//     Vector6d V0 = Vector6d::Zero();
//     Vector6d VT = Vector6d::Zero();
//     Vector6d V0_dot = Vector6d::Zero();
//     Vector6d VT_dot = Vector6d::Zero();
//     SE3 T_des=X0;
//     Vector6d V_des = Vector6d::Zero();
//     Vector6d V_des_dot = Vector6d::Zero();

   

//     while (run) {
//         // 다음 주기 시간 계산
//         next_period.tv_nsec += CYCLE_NS;
//         if (next_period.tv_nsec >= 1000000000) {
//             next_period.tv_nsec -= 1000000000;
//             next_period.tv_sec++;
//         }

//         // 실제 작업 수행
//         q = robot->get_q();
// 	    q_dot = robot->get_q_dot();
//         control->WayPointJointTrajectory(way_points, delays, gt,q_des,q_dot_des,q_ddot_des);        
//         //lr::LieScrewTrajectory( X0,XT,V0,VT,V0_dot,VT_dot,5,gt,T_des,V_des,V_des_dot);
//         //control->WayPointTaskTrajectory(task_way_points, task_delays, gt, T_des, V_des, V_des_dot);
	    
//         JVec q_ddot = JVec::Zero();
//         //JVec torques =control->TaskHinfControl(  q, q_dot, q_ddot,  T_des, V_des, V_des_dot, Task_eint);
//         JVec torques =control->HinfControl(q,q_dot,q_des,q_dot_des,q_ddot_des,eint,eef_mass);
//         //JVec torques = lr::GravityForces(q,control->g,control->Mlist, control->Glist, control->Slist,5.0);
//         JVec e = q_des-q;
//         eint +=e*dt;

// 	    robot->set_torques(torques,kMaxTorques);
// 	    //robot->apply_ext_forces(Vector3d(robot_info.act.F_ext[0],robot_info.act.F_ext[1],robot_info.act.F_ext[2]));
// 	    //robot->apply_ext_forces(Vector3d(10,10,10));        
//         //robot_info.act.q = JVec(sin(gt*M_PI*1),sin(gt*M_PI*2),sin(gt*M_PI*4),sin(gt*M_PI*8),sin(gt*M_PI*16),sin(gt*M_PI*32),sin(gt*M_PI*64)) ;
// //        spdlog::info("Torques: {}", torques.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]")));
//         //spdlog::info("{:03.3f} ,{:03.3f} ,{:03.3f} ,{:03.3f} ,{:03.3f} ,{:03.3f} ,{:03.3f}",q_ddot_des(0),q_ddot_des(1),q_ddot_des(2),q_ddot_des(3),q_ddot_des(4),q_ddot_des(5),q_ddot_des(6));
//         //spdlog::info("{:03.3f} ,{:03.3f} ,{:03.3f} ,{:03.3f} ,{:03.3f} ,{:03.3f} ,{:03.3f}",q_dot(0),q_dot(1),q_dot(2),q_dot(3),q_dot(4),q_dot(5),q_dot(6));
//         spdlog::info("{:03.3f} ,{:03.3f} ,{:03.3f} ,{:03.3f} ,{:03.3f} ,{:03.3f} ,{:03.3f}",torques(0),torques(1),torques(2),torques(3),torques(4),torques(5),torques(6));


//         static int cnt_=0;
//         if(++cnt_>1000){
//            nowClock = std::chrono::high_resolution_clock::now();
//             duration = std::chrono::duration_cast<std::chrono::microseconds>(nowClock-prevClock);        

//             //spdlog::info("gt : {:03.3f} , elapsed time : {:03.6f}[s] \n",gt,duration.count()/1e6);
//             printf("gt : %.3f, elapsed time : %.6f [s] \n",gt,duration.count()/1e6);
//             //std::cout<<e.transpose()<<std::endl;
//             cnt_ = 0;
//             prevClock = nowClock;
//         }
//     	sim->stepSimulation();
//         gt+=dt;
// 	    robot_info.act.q = q;
// 	    robot_info.act.q_dot = q_dot;
// 	    robot_info.act.tau = torques;       
//         // 다음 주기까지 대기
//         if(gt>10*15+5*15+3*15){ 
//             run=0;
//             break;
//         }
//         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
//     }
//     return NULL;
// }
