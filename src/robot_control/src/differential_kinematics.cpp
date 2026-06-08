#include "differential_kinematics.hpp"

#include <stdexcept>

namespace robot_control
{

DifferentialKinematics::DifferentialKinematics(double wheel_separation_mm)
: wheel_separation_mm_(wheel_separation_mm)
{
	if (wheel_separation_mm_ <= 0.0) {
		throw std::invalid_argument("wheel_separation_mm must be greater than zero");
	}
}

WheelSpeedsMmPerS DifferentialKinematics::inverseKinematics(
	double vx_m_s, double vy_m_s, double omega_rad_s) const
{
	static_cast<void>(vy_m_s);

	const double vx_mm_s = vx_m_s * 1000.0;
	const double angular_term_mm_s = omega_rad_s * wheel_separation_mm_ / 2.0;

	return WheelSpeedsMmPerS{vx_mm_s - angular_term_mm_s, vx_mm_s + angular_term_mm_s};
}

double DifferentialKinematics::wheelSeparationMm() const
{
	return wheel_separation_mm_;
}

}  // namespace robot_control
