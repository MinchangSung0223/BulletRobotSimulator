#include "physics_run.h"
Robot *robot;
//JVec q0 = JVec(0,0,0,1.5708,0,1.5708,0);
JVec kMaxTorques=JVec::Ones()*1000;
JVec  q0= JVec(0,0,0,0,0,0,0);
double gt=0;
void draw_T(SE3 T,float line_length,float line_width,int& draw_id_x,int& draw_id_y,int& draw_id_z,double life_time){
    // sim->removeUserDebugItem(draw_id_x);
    // sim->removeUserDebugItem(draw_id_y);
    // sim->removeUserDebugItem(draw_id_z);

    Vector4d p =Vector4d::Zero();
    p(3) = 1;
    Vector4d p_x =Vector4d::Zero();
    p_x(0) = line_length;
    p_x(3) = 1;
    Vector4d p_y =Vector4d::Zero();
    p_y(1) = line_length;
    p_y(3) = 1;
    Vector4d p_z =Vector4d::Zero();
    p_z(2) = line_length;
    p_z(3) = 1;

    p= T*p;
    p_x =  T*p_x;
    p_y =  T*p_y;
    p_z =  T*p_z;
	btVector3 fromXYZ(p(0),p(1),p(2));
    btVector3 fromXYZ_x(p_x(0),p_x(1),p_x(2));
    btVector3 fromXYZ_y(p_y(0),p_y(1),p_y(2));
    btVector3 fromXYZ_z(p_z(0),p_z(1),p_z(2));
    
    b3RobotSimulatorAddUserDebugLineArgs line_args;
    line_args.m_parentObjectUniqueId=-1;
    line_args.m_parentLinkIndex=-1;
    line_args.m_lifeTime = life_time;
    line_args.m_lineWidth = line_width;   

	line_args.m_colorRGB[0]=1;
	line_args.m_colorRGB[1]=0;
	line_args.m_colorRGB[2]=0;
	draw_id_x=sim->addUserDebugLine(fromXYZ,fromXYZ_x,line_args);
	line_args.m_colorRGB[0]=0;
	line_args.m_colorRGB[1]=1;
	line_args.m_colorRGB[2]=0;
	draw_id_y=sim->addUserDebugLine(fromXYZ,fromXYZ_y,line_args);
	line_args.m_colorRGB[0]=0;
	line_args.m_colorRGB[1]=0;
	line_args.m_colorRGB[2]=1;
	draw_id_z=sim->addUserDebugLine(fromXYZ,fromXYZ_z,line_args);
    

}

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
	sim->configureDebugVisualizer( COV_ENABLE_SHADOWS, 0);
    btVector3 targetPos(0,0,0);
    sim->resetDebugVisualizerCamera(3.5, -45, 135, targetPos);
    sim->setTimeOut(10);
    
    sim->setTimeStep(fixedTimeStep);
    sim->setGravity(btVector3(0, 0, 0));
    b3RobotSimulatorSetPhysicsEngineParameters args;
    sim->getPhysicsEngineParameters(args);
    int robotId = sim->loadURDF(urdfFilePath);
    sim->setRealTimeSimulation(false);
    robot = new Robot(sim,robotId);	
    robot->reset_q(q0);
    sim->stepSimulation();

    robot->draw_eef_T(0.25,2);
    double dt = fixedTimeStep;
    int draw_x,draw_y,draw_z;
    robot_info.act.F_ext=Vector6d::Zero();
    std::chrono::time_point<std::chrono::high_resolution_clock> prevClock, nowClock;
    std::chrono::microseconds duration;
    prevClock = std::chrono::high_resolution_clock::now();
    draw_T(SE3::Identity(),10,1,draw_x,draw_y,draw_z,0);            
    int count=0;
    JVec q = robot->get_q();
    JVec q_dot = robot->get_q_dot();       
    robot_info.act.q = q;
    robot_info.act.q_dot = q_dot;
    robot_info.act.tau = robot_info.des.tau;    
    while (run) {
        // 다음 주기 시간 계산
        next_period.tv_nsec += CYCLE_NS;
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_nsec -= 1000000000;
            next_period.tv_sec++;
        }

        // 실제 작업 수행    
        //robot->apply_ext_forces()
        //robot->apply_ext_torques()
        robot->set_torques(robot_info.des.tau,kMaxTorques);
        sim->stepSimulation();
        //로봇 state 획득
        q = robot->get_q();
 	    q_dot = robot->get_q_dot();    
        std::lock_guard<std::mutex> lock(mtx);
        robot_info.act.q = q;
        robot_info.act.q_dot = q_dot;
        robot_info.act.tau = robot_info.des.tau;
        gt +=dt;
        
        // 다음 주기까지 대기
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }
    return NULL;
}
