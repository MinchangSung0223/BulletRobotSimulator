#ifndef WAYPOINT_RUN_H
#define WAYPOINT_RUN_H


//waypoint
#include "control_run.h"



void JointWayPointGenerator(std::vector<JVec>& way_points,std::vector<double>& delays){


    JVec RandJVec1=JVec::Zero();
    JVec RandJVec2=JVec::Zero();
    JVec RandJVec3=JVec::Zero();
    JVec RandJVec4=JVec::Zero();
    JVec RandJVec5=JVec::Zero();
    JVec RandJVec6=JVec::Zero();
    double min_range = -1.5708;
    double max_range = 1.5708;    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min_range, max_range);

    for (int i = 0; i < RandJVec1.size(); ++i) {
        RandJVec1[i] = dis(gen);
        RandJVec2[i] = dis(gen);
        RandJVec3[i] = dis(gen);
        RandJVec4[i] = dis(gen);
        RandJVec5[i] = dis(gen);
    }


    way_points.push_back(q0);
    way_points.push_back(JVec(1.5708,1.5708/2.0,0,0,0,0,0));
    way_points.push_back(JVec(-1.5708,1.5708/2.0,0,0,0,0,0));
    way_points.push_back(JVec(-1.5708,-1.5708/2.0,0,0,0,0,0));
    way_points.push_back(JVec(1.5708,-1.5708/2.0,1.5708,0,0,0,0));
    way_points.push_back(JVec(1.5708,1.5708/2.0,1.5708,0,0,0,0));
    way_points.push_back(JVec(-1.5708,-1.5708/2.0,-1.5708,1.5708,0,0,0));
    way_points.push_back(JVec(-1.5708,1.5708/2.0,-1.5708,1.5708,0,0,0));
    way_points.push_back(JVec(0,0,0,1.5708,0,1.5708,0));
    way_points.push_back(JVec(0,0,0,-1.5708,0,-1.5708,0));
    way_points.push_back(RandJVec1);
    way_points.push_back(RandJVec2);
    way_points.push_back(RandJVec3);
    way_points.push_back(RandJVec4);
    way_points.push_back(RandJVec5);
    way_points.push_back(RandJVec6);
    

    for(int i =0;i<15;i++){
        delays.push_back(10.0);
    }
        

    way_points.push_back(q0);
    way_points.push_back(JVec(1.5708,1.5708/2.0,0,0,0,0,0));
    way_points.push_back(JVec(-1.5708,1.5708/2.0,0,0,0,0,0));
    way_points.push_back(JVec(-1.5708,-1.5708/2.0,0,0,0,0,0));
    way_points.push_back(JVec(1.5708,-1.5708/2.0,1.5708,0,0,0,0));
    way_points.push_back(JVec(1.5708,1.5708/2.0,1.5708,0,0,0,0));
    way_points.push_back(JVec(-1.5708,-1.5708/2.0,-1.5708,1.5708,0,0,0));
    way_points.push_back(JVec(-1.5708,1.5708/2.0,-1.5708,1.5708,0,0,0));
    way_points.push_back(JVec(0,0,0,1.5708,0,1.5708,0));
    way_points.push_back(JVec(0,0,0,-1.5708,0,-1.5708,0));
    way_points.push_back(RandJVec1);
    way_points.push_back(RandJVec2);
    way_points.push_back(RandJVec3);
    way_points.push_back(RandJVec4);
    way_points.push_back(RandJVec5);
    way_points.push_back(RandJVec6);
    

    for(int i =0;i<15;i++){
        delays.push_back(5.0);
    }
        


    way_points.push_back(q0);
    way_points.push_back(JVec(1.5708,1.5708/2.0,0,0,0,0,0));
    way_points.push_back(JVec(-1.5708,1.5708/2.0,0,0,0,0,0));
    way_points.push_back(JVec(-1.5708,-1.5708/2.0,0,0,0,0,0));
    way_points.push_back(JVec(1.5708,-1.5708/2.0,1.5708,0,0,0,0));
    way_points.push_back(JVec(1.5708,1.5708/2.0,1.5708,0,0,0,0));
    way_points.push_back(JVec(-1.5708,-1.5708/2.0,-1.5708,1.5708,0,0,0));
    way_points.push_back(JVec(-1.5708,1.5708/2.0,-1.5708,1.5708,0,0,0));
    way_points.push_back(JVec(0,0,0,1.5708,0,1.5708,0));
    way_points.push_back(JVec(0,0,0,-1.5708,0,-1.5708,0));
    way_points.push_back(RandJVec1);
    way_points.push_back(RandJVec2);
    way_points.push_back(RandJVec3);
    way_points.push_back(RandJVec4);
    way_points.push_back(RandJVec5);
    way_points.push_back(RandJVec6);
    
    for(int i =0;i<15;i++){
        delays.push_back(3.0);
    }    
};
void TaskWayPointGenerator(std::vector<SE3>& task_way_points,std::vector<double>& task_delays,const SE3 X0){
    task_way_points.push_back(X0);
    task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(-0.3,0.0,0.0,0.0,0.0,0.0))));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(0.0,0.0,0.0,0.0,0.0,0.0))));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(0.3,0.0,0.0,0.0,0.0,0.0))));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(0.0,0.0,0.0,0.0,0.0,0.0))));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(0.0,0.3,0.0,0.0,0.0,0.0))));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(0.0,0.0,0.0,0.0,0.0,0.0))));
    task_way_points.push_back(X0*MatrixExp6(VecTose3(Vector6d(0.0,-0.3,0.0,0.0,0.0,0.0))));
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
    task_delays.push_back(5.0);
};
#endif // WAYPOINT_RUN_H
