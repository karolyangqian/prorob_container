#pragma once

#include <cstdint>

namespace robot_control
{

struct WheelSpeedsMmPerS
{
	double left_mm_s;
	double right_mm_s;
};

class DifferentialKinematics
{
public:
	explicit DifferentialKinematics(double wheel_separation_mm);

	WheelSpeedsMmPerS inverseKinematics(double vx_m_s, double vy_m_s, double omega_rad_s) const;

	double wheelSeparationMm() const;

private:
	double wheel_separation_mm_;
};

}  // namespace robot_control
