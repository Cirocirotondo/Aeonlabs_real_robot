#include <Eigen/Dense>
#include <iostream>
#include <unsupported/Eigen/Splines>

#include "tk_spline.h"

class BaseSpline {
   public:
    // Virtual destructor is crucial for proper polymorphic deletion.
    virtual ~BaseSpline() = default;

    // Pure virtual function to get the position at a given time.
    // Any derived class must implement this.
    virtual Eigen::VectorXd get_position(double time) const = 0;

    // Pure virtual function to get the velocity at a given time.
    // Any derived class must implement this.
    virtual Eigen::VectorXd get_velocity(double time) const = 0;
};

class EigenSpline : public BaseSpline {
    typedef Eigen::Spline<double, 6> Spline6d;
    typedef Eigen::SplineFitting<Spline6d> Spline6dFitting;

   public:
    EigenSpline(){};
    EigenSpline(const std::vector<double> &times, const std::vector<Eigen::VectorXd> &positions)
        : _times(times), _points(positions) {
        const double time_scaling = _times[_times.size() - 1];

        std::vector<double> scaled_times;
        for (const auto &t : _times) {
            scaled_times.push_back(t / time_scaling);
        }

        Eigen::Map<const Eigen::VectorXd> knot_vector(scaled_times.data(), scaled_times.size());
        // knot_vector /= time_scaling;

        const int num_points = _points.size();
        const int pose_size = _points[0].size();

        Eigen::MatrixXd points_matrix(pose_size, num_points);  // pose_size rows, num_points cols

        for (int i = 0; i < num_points; ++i) {
            points_matrix.col(i) = _points[i];  // Copy each Eigen::VectorXd into a column
        }

        // std::cout << knot_vector << std::endl;
        // std::cout << points_matrix << std::endl;

        Eigen::MatrixXd zero_derivative_matrix(pose_size, 2);  // pose_size rows, num_points cols
        std::vector<int> idx{0, pose_size};

        spline = Spline6dFitting::Interpolate(points_matrix, 2, knot_vector);
        // spline = Spline6dFitting::InterpolateWithDerivatives(points_matrix,
        // zero_derivative_matrix, idx, 3, knot_vector);
    };

    Eigen::VectorXd get_position(const double time) const {
        if (time < _times[_times.size() - 1]) {
            const double time_scaling = _times[_times.size() - 1];
            return spline(time / time_scaling);
        }
        return _points[_points.size() - 1];
    }
    Eigen::VectorXd get_velocity(const double time) const {
        if (time < _times[_times.size() - 1]) {
            const double time_scaling = _times[_times.size() - 1];

            // std::cout << "A" << std::endl;
            // std::cout << "T:" << time << std::endl;
            // std::cout << "Scaled: " << time / time_scaling << std::endl;
            // std::cout << spline.derivatives(time / time_scaling, 1).col(1) / time_scaling <<
            // std::endl;
            return Eigen::VectorXd(spline.derivatives(time / time_scaling, 1).col(1)) /
                   time_scaling;
        }

        // return zero
        return Eigen::VectorXd::Zero(6);
    }

   private:
    Spline6d spline;

    std::vector<double> _times;
    std::vector<Eigen::VectorXd> _points;
};

class TkSpline : public BaseSpline {
   public:
    TkSpline(){};

    TkSpline(const std::vector<double> &times, const std::vector<Eigen::VectorXd> &positions){
        const auto v0 = Eigen::VectorXd::Zero(6);
        *this = TkSpline(times, positions, v0);
    }

    TkSpline(const std::vector<double> &times, const std::vector<Eigen::VectorXd> &positions, const Eigen::VectorXd &initial_velocities)
        : _times(times), _points(positions) {
        const int pose_size = 6;
        // _splines.resize(pose_size);  // Allocate space for 6 tk::spline objects

        // Prepare data for each dimension
        std::vector<std::vector<double>> dim_data(pose_size);
        for (const auto &p : _points) {
            if (p.size() != pose_size) {
                throw std::runtime_error(
                    "Spline construction failed: All position vectors must be 6-dimensional.");
            }
            for (int i = 0; i < pose_size; ++i) {
                dim_data[i].push_back(p(i));
            }
        }

        for (int i = 0; i < pose_size; ++i) {
            // _splines[i].set_points(_times, dim_data[i]);
            _splines.push_back(tk::spline(_times, dim_data[i], tk::spline::cspline_hermite, true,
                                          tk::spline::first_deriv, initial_velocities[i], tk::spline::first_deriv,
                                          0.));
        }
    };

    Eigen::VectorXd get_position(const double time) const {
        if (time < _times[_times.size() - 1]) {
            Eigen::VectorXd position(6);
            for (int i = 0; i < 6; ++i) {
                position(i) = _splines[i](time);  // Query each dimension's spline
            }
            return position;
        }
        return _points[_points.size() - 1];
    }
    Eigen::VectorXd get_velocity(const double time) const {
        if (time < _times[_times.size() - 1]) {
            Eigen::VectorXd velocity(6);
            for (int i = 0; i < 6; ++i) {
                velocity(i) =
                    _splines[i].deriv(1, time);  // Query each dimension's spline derivative
            }
            return velocity;
        }

        // return zero
        return Eigen::VectorXd::Zero(6);
    }

   private:
    std::vector<tk::spline> _splines;  // A vector of tk::spline objects, one for each dimension

    std::vector<double> _times;
    std::vector<Eigen::VectorXd> _points;
};