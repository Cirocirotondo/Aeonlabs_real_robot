#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <zmq.hpp>
#include <chrono>
#include <nlohmann/json.hpp>

#include "pose.hpp"
#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_receive_interface.h"

using json = nlohmann::json;

class UR5Estimator
{
public:
    UR5Estimator(const std::string& robot_ip, const json &config)
        : receive_(robot_ip), stop_flag_(false), context_(1), publisher_(context_, ZMQ_PUB), frequency_(parse_frequency(config))
    {
        const int publisher_port = parse_port(config);

        const std::string publisher_address = "tcp://127.0.0.1:" + std::to_string(publisher_port);
        publisher_.bind(publisher_address);

        std::cout << "started UR5 Estimator on ip " << robot_ip << " with frequency " << frequency_ << " Hz\n";
    }
  
    void update_robot_state()
    {
        curr_pose_ = receive_.getActualTCPPose();
        curr_vel_ = utils::std_to_eigen(receive_.getActualTCPSpeed());
        curr_Q_ = utils::std_to_eigen(receive_.getActualQ());
        curr_Qd_ = utils::std_to_eigen(receive_.getTargetQd());
        curr_force_ = utils::std_to_eigen(receive_.getActualTCPForce());
        curr_force_lowpass_ = 0.9 * curr_force_lowpass_ + 0.1 * curr_force_;
    }

    void send_state(){
        const json state = {
            {"pos", curr_pose_.to_std_vector()},
            {"vel", utils::eigen_to_std(curr_vel_)},
            {"Q", utils::eigen_to_std(curr_Q_)},
            {"Qd", utils::eigen_to_std(curr_Qd_)},
            {"force", utils::eigen_to_std(curr_force_lowpass_)}
        };

        const std::string message = state.dump();
        zmq::message_t zmq_message(message.begin(), message.end());
        publisher_.send(zmq_message, zmq::send_flags::none);
    }

    void run() {
        while (!stop_flag_.load())
        {
            const auto start = std::chrono::high_resolution_clock::now();
            
            update_robot_state();
            send_state();

            const auto next_cycle_time = start + std::chrono::duration<double>(1.0 / frequency_);
            std::this_thread::sleep_until(next_cycle_time);
        }
        std::cout << "UR5 Estimator stopped gracefully\n";
    }

    void stop() {
        stop_flag_.store(true);
        std::cout << "UR5Estimator speedStop applied\n";
    }

private:
    static double parse_frequency(const json& config) {
        if (!config.contains("frequency") || !config["frequency"].is_number()) {
            throw std::invalid_argument("Missing or invalid 'frequency' in config.");
        }
        return config["frequency"].get<double>();
    }

    static int parse_port(const json& config) {
        if (!config.contains("publisher_port") || !config["publisher_port"].is_number_integer()) {
            throw std::invalid_argument("Missing or invalid 'publisher_port' in config.");
        }
        return config["publisher_port"].get<int>();
    }

    ur_rtde::RTDEReceiveInterface receive_;
    std::atomic<bool> stop_flag_;
    zmq::context_t context_;
    zmq::socket_t publisher_;

    const double frequency_;

    Pose7D curr_pose_;
    Eigen::Vector<double, 6> curr_vel_;
    Eigen::Vector<double, 6> curr_Q_;
    Eigen::Vector<double, 6> curr_Qd_;
    Eigen::Vector<double, 6> curr_force_;
    Eigen::Vector<double, 6> curr_force_lowpass_;
};

int main(int argc, char* argv[]) {
    const std::string config_path = (argc > 1) ? argv[1] : "config.json";

    json config;
    try {
        std::ifstream config_file(config_path);
        if (!config_file.is_open()) throw std::runtime_error("Unable to open " + config_path);
        config_file >> config;
        std::cout << "Loaded configuration from " << config_path << "\n";
    } catch (const std::exception &e) {
        std::cerr << "Error loading config.json: " << e.what() << "\nUsing default values.\n";
    }

    UR5Estimator estimator(
        config["robot_ip"].get<std::string>(), config
    );

    try {
        // Handle Ctrl+C signal
        static UR5Estimator *estimator_ptr = &estimator;
        static std::atomic_flag signal_handled = ATOMIC_FLAG_INIT;
        std::signal(SIGINT, [](int)
                    {
            if (!signal_handled.test_and_set()) {
                std::cout << "\nCtrl+C detected. Stopping the estimator...\n";
                if (estimator_ptr!=nullptr) {
                    estimator_ptr->stop();
                }
            }
        });

        estimator.run();
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
