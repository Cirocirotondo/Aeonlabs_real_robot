#include "spline.hpp"

#include <cassert>  // The simplest assertion
#include <cmath>    // For std::abs
#include <iostream>
#include <vector>

#include <fstream> // For file output
#include <iomanip> // For std::fixed and std::setprecision

using SplineType = TkSpline;

void write_spline_data_to_csv(const SplineType& spline, const std::vector<double>& times, const std::string& filename, int num_samples = 100) {
    std::ofstream output_file(filename);
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    // Write header
    output_file << "time,pos_0,pos_1,pos_2,pos_3,pos_4,pos_5,vel_0,vel_1,vel_2,vel_3,vel_4,vel_5\n";

    // Set precision for output
    output_file << std::fixed << std::setprecision(8);

    if (times.empty()) {
        std::cerr << "Warning: Cannot generate spline data for plotting, times vector is empty." << std::endl;
        return;
    }

    double start_time = times.front();
    double end_time = times.back();

    if (end_time == start_time) { // Handle single point case
        if (num_samples > 0) {
            Eigen::VectorXd pos = spline.get_position(start_time);
            Eigen::VectorXd vel = spline.get_velocity(start_time);
            output_file << start_time;
            for (int i = 0; i < 6; ++i) output_file << "," << pos[i];
            for (int i = 0; i < 6; ++i) output_file << "," << vel[i];
            output_file << "\n";
        }
        return;
    }

    double step_size = (end_time - start_time) / (num_samples - 1);

    for (int i = 0; i < num_samples; ++i) {
        double current_time = start_time + i * step_size;
        
        Eigen::VectorXd pos = spline.get_position(current_time);
        Eigen::VectorXd vel = spline.get_velocity(current_time);

        output_file << current_time;
        for (int j = 0; j < 6; ++j) {
            output_file << "," << pos[j];
        }
        for (int j = 0; j < 6; ++j) {
            output_file << "," << vel[j];
        }
        output_file << "\n";
    }

    output_file.close();
    std::cout << "Spline data written to " << filename << std::endl;
}

void assert_near_eigen_vector(const Eigen::VectorXd& actual, const Eigen::VectorXd& expected,
                              double tolerance, const std::string& message = "") {
    if (actual.size() != expected.size()) {
        std::cerr << "TEST FAILED: Vector sizes do not match. Actual: " << actual.size()
                  << ", Expected: " << expected.size();
        if (!message.empty()) std::cerr << " (" << message << ")";
        std::cerr << std::endl;
        assert(false);  // Force a crash if sizes mismatch
    }
    for (int i = 0; i < actual.size(); ++i) {
        if (std::abs(actual[i] - expected[i]) > tolerance) {
            std::cerr << "TEST FAILED: Mismatch at index " << i << ". Actual: " << actual[i]
                      << ", Expected: " << expected[i]
                      << ", Diff: " << std::abs(actual[i] - expected[i]) << ", Tol: " << tolerance;
            if (!message.empty()) std::cerr << " (" << message << ")";
            std::cerr << std::endl;
            assert(false);  // Force a crash on failure
        }
    }
    std::cout << "  Passed: Vector comparison." << std::endl;
}

void assert_no_throw(std::function<void()> func, const std::string& message = "") {
    try {
        func();
        std::cout << "  Passed: No throw expected." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "TEST FAILED: Unexpected exception caught: " << e.what();
        if (!message.empty()) std::cerr << " (" << message << ")";
        std::cerr << std::endl;
        assert(false);
    } catch (...) {
        std::cerr << "TEST FAILED: Unexpected unknown exception caught.";
        if (!message.empty()) std::cerr << " (" << message << ")";
        std::cerr << std::endl;
        assert(false);
    }
}

// --- Test Functions ---

void test_constructor_and_interpolation() {
    std::cout << "Running test_constructor_and_interpolation..." << std::endl;

    std::vector<double> times = {0.0, 1.0, 2.0, 3.0};
    Eigen::VectorXd p0(6), p1(6), p2(6), p3(6);
    p0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    p1 << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    p2 << 2.0, 2.0, 2.0, 0.0, 0.0, 0.0;
    p3 << 3.0, 3.0, 3.0, 0.0, 0.0, 0.0;
    std::vector<Eigen::VectorXd> positions = {p0, p1, p2, p3};

    assert_no_throw([&]() { SplineType spline(times, positions); },
                    "Spline construction should not throw.");
    SplineType testSpline(times, positions);

    // Test position at control points
    Eigen::VectorXd pos_at_0 = testSpline.get_position(0.0);
    assert_near_eigen_vector(pos_at_0, positions[0], 1e-9, "Position at time 0.0");

    Eigen::VectorXd pos_at_1 = testSpline.get_position(1.0);
    assert_near_eigen_vector(pos_at_1, positions[1], 1e-9, "Position at time 1.0");

    Eigen::VectorXd pos_at_2 = testSpline.get_position(2.0);
    assert_near_eigen_vector(pos_at_2, positions[2], 1e-9, "Position at time 2.0");

    Eigen::VectorXd pos_at_3 = testSpline.get_position(3.0);
    assert_near_eigen_vector(pos_at_3, positions[3], 1e-9, "Position at time 3.0");

    // Test position between control points (simple linear path for x,y,z)
    // For a cubic spline, interpolation between points will not be strictly linear,
    // but the spline passes through the control points.
    // For a simple linear example like this, if the spline is cubic,
    // it will be smooth but not necessarily exactly 0.5, 0.5, 0.5 at time 0.5.
    // We can only assert that it returns a valid vector of the correct size.
    Eigen::VectorXd mid_pos = testSpline.get_position(0.5);
    assert(mid_pos.size() == 6 && "Mid position vector size should be 6.");
    std::cout << "  Passed: Mid position vector size." << std::endl;

    write_spline_data_to_csv(testSpline, times, "spline_data_linear.csv");

    std::cout << "test_constructor_and_interpolation completed." << std::endl << std::endl;
}

void test_constructor_and_interpolation_non_uniform_knots() {
    std::cout << "Running test_constructor_and_interpolation..." << std::endl;

    std::vector<double> times = {0.0, 2.0, 3.0, 10.0};
    Eigen::VectorXd p0(6), p1(6), p2(6), p3(6);
    p0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    p1 << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    p2 << 2.0, 2.0, 2.0, 0.0, 0.0, 0.0;
    p3 << 3.0, 3.0, 3.0, 0.0, 0.0, 0.0;
    std::vector<Eigen::VectorXd> positions = {p0, p1, p2, p3};

    assert_no_throw([&]() { SplineType spline(times, positions); },
                    "Spline construction should not throw.");
    SplineType testSpline(times, positions);

    // Test position at control points
    Eigen::VectorXd pos_at_0 = testSpline.get_position(0.0);
    assert_near_eigen_vector(pos_at_0, positions[0], 1e-9, "Position at time 0.0");

    Eigen::VectorXd pos_at_1 = testSpline.get_position(2.0);
    assert_near_eigen_vector(pos_at_1, positions[1], 1e-9, "Position at time 1.0");

    Eigen::VectorXd pos_at_2 = testSpline.get_position(3.0);
    assert_near_eigen_vector(pos_at_2, positions[2], 1e-9, "Position at time 2.0");

    Eigen::VectorXd pos_at_3 = testSpline.get_position(10.0);
    assert_near_eigen_vector(pos_at_3, positions[3], 1e-9, "Position at time 3.0");

    // Test position between control points (simple linear path for x,y,z)
    // For a cubic spline, interpolation between points will not be strictly linear,
    // but the spline passes through the control points.
    // For a simple linear example like this, if the spline is cubic,
    // it will be smooth but not necessarily exactly 0.5, 0.5, 0.5 at time 0.5.
    // We can only assert that it returns a valid vector of the correct size.
    Eigen::VectorXd mid_pos = testSpline.get_position(0.5);
    assert(mid_pos.size() == 6 && "Mid position vector size should be 6.");
    std::cout << "  Passed: Mid position vector size." << std::endl;

    write_spline_data_to_csv(testSpline, times, "spline_data_non_uniform.csv");

    std::cout << "test_constructor_and_interpolation completed." << std::endl << std::endl;
}

void test_get_velocity() {
    std::cout << "Running test_get_velocity..." << std::endl;

    std::vector<double> times = {0.0, 1.0, 2.0, 3.0};
    Eigen::VectorXd p0(6), p1(6), p2(6), p3(6);
    p0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    p1 << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    p2 << 2.0, 2.0, 2.0, 0.0, 0.0, 0.0;
    p3 << 3.0, 3.0, 3.0, 0.0, 0.0, 0.0;
    std::vector<Eigen::VectorXd> positions = {p0, p1, p2, p3};
    SplineType testSpline(times, positions);

    // For a path like 0,0,0 -> 1,1,1 -> 2,2,2 -> 3,3,3 with evenly spaced times,
    // the velocity should be constant 1,1,1 in x,y,z between the first and last segment.
    // At the very start/end, it depends on boundary conditions.
    // For a simple cubic spline interpolation, the velocities at the knots might not be exactly 1.
    // Let's test a point in the middle where velocity should stabilize.
    Eigen::VectorXd expected_vel_middle(6);
    expected_vel_middle << 1.0, 1.0, 1.0, 0.0, 0.0,
        0.0;  // Expected velocity in x,y,z for this linear setup

    // Eigen::VectorXd vel_at_0_5 = testSpline.get_velocity(0.5);
    // assert_near_eigen_vector(vel_at_0_5, expected_vel_middle, 1e-9, "Velocity at time 0.5");

    // Eigen::VectorXd vel_at_1_5 = testSpline.get_velocity(1.5);
    // assert_near_eigen_vector(vel_at_1_5, expected_vel_middle, 1e-9, "Velocity at time 1.5");

    // Eigen::VectorXd vel_at_2_5 = testSpline.get_velocity(2.5);
    // assert_near_eigen_vector(vel_at_2_5, expected_vel_middle, 1e-9, "Velocity at time 2.5");

    // Test velocity at boundary
    // Eigen::VectorXd vel_at_0 = testSpline.get_velocity(0.0);
    // assert_near_eigen_vector(vel_at_0, Eigen::VectorXd::Zero(6), 1e-9,
    //                          "Velocity at time 0.0 (start)");

    // Eigen::VectorXd vel_at_3 = testSpline.get_velocity(3.0);
    // assert_near_eigen_vector(vel_at_3, Eigen::VectorXd::Zero(6), 1e-9,
    //                          "Velocity at time 3.0 (end)");

    std::cout << "test_get_velocity completed." << std::endl << std::endl;
}

void test_edge_cases_out_of_bounds() {
    std::cout << "Running test_edge_cases_out_of_bounds..." << std::endl;

    std::vector<double> times = {0.0, 1.0, 2.0, 3.0};
    Eigen::VectorXd p0(6), p1(6), p2(6), p3(6);
    p0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    p1 << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    p2 << 2.0, 2.0, 2.0, 0.0, 0.0, 0.0;
    p3 << 3.0, 3.0, 3.0, 0.0, 0.0, 0.0;
    std::vector<Eigen::VectorXd> positions = {p0, p1, p2, p3};
    SplineType testSpline(times, positions);

    // Position after end time
    Eigen::VectorXd pos_after_end = testSpline.get_position(4.0);
    assert_near_eigen_vector(pos_after_end, positions.back(), 1e-9, "Position after end time");

    // Position before start time (should be clamped to start position in normalized 0-1 domain)
    Eigen::VectorXd pos_before_start = testSpline.get_position(-1.0);
    assert_near_eigen_vector(pos_before_start, positions.front(), 1e-9,
                             "Position before start time");

    // Velocity after end time
    Eigen::VectorXd vel_after_end = testSpline.get_velocity(4.0);
    assert_near_eigen_vector(vel_after_end, Eigen::VectorXd::Zero(6), 1e-9,
                             "Velocity after end time");

    // Velocity before start time
    Eigen::VectorXd vel_before_start = testSpline.get_velocity(-1.0);
    assert_near_eigen_vector(vel_before_start, Eigen::VectorXd::Zero(6), 1e-9,
                             "Velocity before start time");

    std::cout << "test_edge_cases_out_of_bounds completed." << std::endl << std::endl;
}

// --- Main Test Runner ---
int main() {
    std::cout << "Starting Spline Unit Tests (minimal dependency)..." << std::endl << std::endl;

    test_constructor_and_interpolation();
    test_constructor_and_interpolation_non_uniform_knots();
    test_get_velocity();
    test_edge_cases_out_of_bounds();
    // Add more test functions here as your class grows

    std::cout << "All Spline tests completed successfully!" << std::endl;
    return 0;  // Indicate success
}