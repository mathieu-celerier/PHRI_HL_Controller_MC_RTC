#include <Eigen/Dense>

inline Eigen::Matrix3d skew(Eigen::Vector3d v)
{
    return (Eigen::Matrix3d() << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0).finished();
}