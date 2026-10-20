#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

namespace utils {
Eigen::Quaterniond rotvec_to_quat(const Eigen::Vector3d &v) {
    double angle = v.norm();
    if (angle < 1e-6) return Eigen::Quaterniond(1, 0, 0, 0);

    Eigen::Vector3d axis = v.normalized();
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

Eigen::VectorXd quat_to_rotvec(const Eigen::Quaterniond &q) {
    Eigen::AngleAxisd aa(q);
    return aa.angle() * aa.axis();
}

Eigen::VectorXd std_to_eigen(const std::vector<double> &v) {
    Eigen::VectorXd e(v.size());
    for (size_t i = 0; i < v.size(); i++) {
        e[i] = v[i];
    }
    return e;
}

std::vector<double> eigen_to_std(const Eigen::VectorXd &v) {
    std::vector<double> s(v.size());
    for (int i = 0; i < v.size(); i++) {
        s[i] = v[i];
    }
    return s;
}

Eigen::VectorXd clamp(const Eigen::VectorXd &x, const Eigen::VectorXd &lower,
                      const Eigen::VectorXd &upper) {
    return x.cwiseMax(lower).cwiseMin(upper);
}

}  // namespace utils

using namespace utils;

class Pose7D {
   public:
    Eigen::Vector3d position;  // Position (x, y, z)
    Eigen::Quaterniond quat;   // Orientation (w, x, y, z)

    Pose7D() : position(Eigen::Vector3d::Zero()), quat(Eigen::Quaterniond::Identity()) {}

    // constructor from position and quaternion
    Pose7D(const Eigen::Vector3d &p, const Eigen::Quaterniond &q) : position(p), quat(q) {}

    Pose7D(const Pose7D &other) = default;

    // constructor from position and rotvec
    Pose7D(const Eigen::Vector<double, 6> &v) {
        position = v.head<3>();
        quat = rotvec_to_quat(v.tail<3>());
    }

    // constructor from position and quaternion
    Pose7D(const Eigen::Vector<double, 7> &v) {
        position = v.head<3>();
        quat = Eigen::Quaterniond(v[3], v[4], v[5], v[6]);
    }

    // Convert to a 7D std::vector
    std::vector<double> to_std_vector() const {
        std::vector<double> v(7);
        v[0] = position.x();
        v[1] = position.y();
        v[2] = position.z();
        v[3] = quat.x();
        v[4] = quat.y();
        v[5] = quat.z();
        v[6] = quat.w();  // scalar last here
        return v;
    }

    // Overload the assignment operator (position, rtvec)
    Pose7D &operator=(const std::vector<double> &TCPPose) {
        if (TCPPose.size() == 6) {
            position = Eigen::Vector3d(TCPPose[0], TCPPose[1], TCPPose[2]);
            quat = rotvec_to_quat(Eigen::Vector3d(TCPPose[3], TCPPose[4], TCPPose[5]));
            return *this;
        } else if (TCPPose.size() == 7) {
            position = Eigen::Vector3d(TCPPose[0], TCPPose[1], TCPPose[2]);
            quat = Eigen::Quaterniond(TCPPose[6], TCPPose[3], TCPPose[4],
                                      TCPPose[5]);  // watch out, scalar first!
            return *this;
        } else {
            std::cerr << "Invalid pose vector size: Expected 6 or 7 elements, got "
                      << TCPPose.size() << "\n";
            return *this;
        }
    }

    Pose7D copy() const { return Pose7D(position, quat); }

    // Overload the assignment operator
    Pose7D &operator=(const Pose7D &other) {
        if (this != &other) {
            position = other.position;
            quat = other.quat;
        }
        return *this;
    }

    // Overload mult operator for scalar multiplication
    Pose7D operator*(const double &scalar) const {
        Pose7D result;
        result.position = position * scalar;
        result.quat = Eigen::Quaterniond::Identity().slerp(scalar, quat);
        return result;
    }

    // Overload minus operator for the difference
    Pose7D operator-(const Pose7D &other) const {
        Pose7D result;
        result.position = position - other.position;
        result.quat = quat * other.quat.inverse();
        return result;
    }

    double get_rotation_angle() {
        Eigen::AngleAxisd angleAxis(quat);
        return angleAxis.angle();
    }

    Eigen::Quaterniond scale_and_return_quat(double scale) {
        return Eigen::Quaterniond::Identity().slerp(scale, quat);
    }

    // return the pose to the speedL command in UR RTDE (also apply deadzone)
    std::vector<double> get_actual_speed_6D(float min_val) {
        std::vector<double> vec;
        for (auto p : position) {
            if (std::abs(p) < min_val)
                vec.push_back(0.0);
            else
                vec.push_back(p);
        }
        auto rotvec = quat_to_rotvec(quat);
        if (rotvec.norm() < min_val) {
            vec.insert(vec.end(), {0.0, 0.0, 0.0});
        } else {
            vec.insert(vec.end(), rotvec.begin(), rotvec.end());
        }
        return vec;
    }

    // Print the pose
    std::string str() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        int width = 5;
        oss << "[ ";
        for (int i = 0; i < position.size(); ++i) {
            oss << std::setw(width) << position[i] << " ";
        }
        oss << std::setw(width) << quat.w() << " " << std::setw(width) << quat.x() << " "
            << std::setw(width) << quat.y() << " " << std::setw(width) << quat.z() << " ]";

        return oss.str();
    }
};

// exponential filter has to be calculated this way,
void apply_exp_filter(Pose7D &speed, const Pose7D &oldSpeed, float exp_filter_) {
    speed.position = exp_filter_ * oldSpeed.position + (1 - exp_filter_) * speed.position;
    speed.quat = oldSpeed.quat.slerp(1 - exp_filter_, speed.quat);
}