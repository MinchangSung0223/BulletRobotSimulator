#include "print_run.h"
std::shared_ptr<spdlog::logger> my_logger;
void init_logger(const char* urdf_path) {
    try {
        std::string log_file_name = "logs/";
        log_file_name += urdf_path;  // urdf_path를 log 파일 이름에 추가
        log_file_name += "-log.txt"; // 확장자 추가

        // 기존 로그 파일 제거 시도
        if (std::remove(log_file_name.c_str()) == 0) {
            spdlog::info("Existing log file '{}' removed.", log_file_name);
        } else {
            spdlog::warn("Could not remove '{}'. It may not exist or be in use.", log_file_name);
        }

        // 로거 설정
        my_logger = spdlog::basic_logger_mt("basic_logger", log_file_name);
        my_logger->set_pattern("%M:%S.%e,%v");

        spdlog::set_default_logger(my_logger);  // Default logger로 설정
        spdlog::flush_every(std::chrono::seconds(3));  // 자동 flush 설정
    }
    catch (const spdlog::spdlog_ex& ex) {
        std::cout << "Log init failed: " << ex.what() << std::endl;
    }
}
void* print_run(void* param) {
    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);
    //sdplog setup
    if(use_log)init_logger(urdfFilePath);
    

    while (run) {
        // 다음 주기 시간 계산
        next_period.tv_nsec += CYCLE_NS*100;
        if (next_period.tv_nsec >= 1000000000) {
            next_period.tv_nsec -= 1000000000;
            next_period.tv_sec++;
        }
        //
            spdlog::info("gt : {:03.3f} \n",gt);
        //
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);
    }
    return NULL;
}

