
#include "main.h"
#include <cxxopts.hpp>
#include <sched.h>

ROBOT_INFO robot_info;
b3RobotSimulatorClientAPI* sim;
double CONTROL_RATE = 1000.0;
btScalar fixedTimeStep = 1. / CONTROL_RATE;
const char* urdfFilePath =NULL;
float eef_mass = 0;
bool with_gui = false;
bool use_log;
std::mutex mtx; 



// 스레드 생성 후 호출할 함수
void set_thread_affinity(pthread_t thread, int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset); // core_id에 스레드를 할당

    int rc = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        // 에러 처리
    }
}

void signal_handler(int signum)
{
	printf("\n\n");
    run=0;
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

int main(int argc, char* argv[]){
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGWINCH, signal_handler);
	signal(SIGHUP, signal_handler);    

    cxxopts::Options options("BulletRobotSim", "BulletRobotSim");
    // Define options
    bool use_control;
    bool use_qt;
    bool use_print;
    
    static std::string urdfPathStr; // Make this static to retain throughout the program

    options.add_options()
        ("g,gui", "Include GUI")
        ("e,eef_mass", "End-Effector Mass", cxxopts::value<float>()->default_value("0"))
        ("u,urdf", "URDF File Path", cxxopts::value<std::string>())
        ("c,control", "Use control", cxxopts::value<bool>()->default_value("false")->implicit_value("true"))
        ("q,qt", "Use Qt", cxxopts::value<bool>()->default_value("false")->implicit_value("true"))
        ("p,print", "Use Print", cxxopts::value<bool>()->default_value("false")->implicit_value("true"))
        ("l,log", "Save Log", cxxopts::value<bool>()->default_value("false")->implicit_value("true"))
        ("h,help", "Print usage");

    // Parse command line arguments
    auto result = options.parse(argc, argv);

    // If no arguments were provided or help was requested, print the help message.
    if (argc == 1 || result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0; // Exit after displaying help
    }

    // GUI option
    with_gui = result["gui"].as<bool>();

    // URDF file path option
    if (result.count("urdf")) {
        urdfPathStr = result["urdf"].as<std::string>();
        urdfFilePath = urdfPathStr.c_str();
    } else {
        std::cout << "No URDF file path provided. Please specify a path using the --urdf option.\n";
        std::cout << options.help() << std::endl;
        return 1; // Exit with error code
    }

    // EEF mass option
    eef_mass = result["eef_mass"].as<float>();

    // Control option
    use_control = result["control"].as<bool>();

    // Qt option
    use_qt = result["qt"].as<bool>();
    use_print = result["print"].as<bool>();
    use_log = result["log"].as<bool>();

    std::cout<<"=========================================================="<<std::endl;
    std::cout<<""<<std::endl;
    std::cout << "\tURDF file path: " << urdfFilePath << std::endl;
    std::cout << "\tEnd-Effector mass: " << eef_mass << std::endl;
    std::cout << "\tUse control: " << (use_control ? "Yes" : "No") << std::endl;
    std::cout << "\tUse Qt: " << (use_qt ? "Yes" : "No") << std::endl;
    std::cout << "\tUse Print: " << (use_print ? "Yes" : "No") << std::endl;    
    std::cout << "\tSave Log: " << (use_log ? "Yes" : "No") << std::endl;    
    std::cout<<""<<std::endl;
    std::cout<<"=========================================================="<<std::endl;



    //control thread
    pthread_t control_thread;
    pthread_attr_t control_attr;
    struct sched_param control_param;
    if(use_control){
        pthread_attr_init(&control_attr);
        pthread_attr_setschedpolicy(&control_attr, SCHED_RR );
        control_param.sched_priority = sched_get_priority_max(SCHED_RR )-2;
        pthread_attr_setschedparam(&control_attr, &control_param);
        pthread_create(&control_thread, &control_attr, control_run, NULL);
        //set_thread_affinity(control_thread, 1); // 1번 코어에 할당
    }
    //pthread thread
    pthread_t physics_thread;
    pthread_attr_t physics_attr;
    struct sched_param physics_param;
    pthread_attr_init(&physics_attr);
    pthread_attr_setschedpolicy(&physics_attr, SCHED_FIFO );
    physics_param.sched_priority = sched_get_priority_max(SCHED_FIFO )-1;
    pthread_attr_setschedparam(&physics_attr, &physics_param);
    pthread_create(&physics_thread, &physics_attr, physics_run, NULL);
    //set_thread_affinity(physics_thread, 0); // 0번 코어에 할당

    //qt thread
    pthread_t print_thread;
    pthread_attr_t print_attr;
    struct sched_param print_param;
    if(use_print){
    pthread_attr_init(&print_attr);
    pthread_attr_setschedpolicy(&print_attr, SCHED_RR );
    print_param.sched_priority = sched_get_priority_max(SCHED_RR )-49;
    pthread_attr_setschedparam(&print_attr, &print_param);
    pthread_create(&print_thread, &print_attr, print_run, NULL);
    //set_thread_affinity(qt_thread, 2); // 2번 코어에 할당
    }

    //qt thread
    pthread_t qt_thread;
    pthread_attr_t qt_attr;
    struct sched_param qt_param;
    if(use_qt){
    pthread_attr_init(&qt_attr);
    pthread_attr_setschedpolicy(&qt_attr, SCHED_RR );
    qt_param.sched_priority = sched_get_priority_max(SCHED_RR )-50;
    pthread_attr_setschedparam(&qt_attr, &qt_param);
    pthread_create(&qt_thread, &qt_attr, qt_run, NULL);
    //set_thread_affinity(qt_thread, 2); // 2번 코어에 할당
    }

    pause();
    pthread_join(physics_thread, NULL);
    if(use_control) pthread_join(control_thread, NULL);
    if(use_qt) pthread_join(qt_thread, NULL);
    
	signal_handler(0);
    return 0;
}


