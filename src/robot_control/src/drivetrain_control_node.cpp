#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <termios.h>
#include <unistd.h>

#include "differential_kinematics.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using robot_control::DifferentialKinematics;
using robot_control::WheelSpeedsMmPerS;

speed_t baudRateToSpeed(int baud_rate)
{
	switch (baud_rate) {
		case 9600:
			return B9600;
		case 19200:
			return B19200;
		case 38400:
			return B38400;
		case 57600:
			return B57600;
		case 115200:
		default:
			return B115200;
	}
}

int openSerialPort(const std::string & device, int baud_rate)
{
	const int fd = ::open(device.c_str(), O_WRONLY | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		throw std::runtime_error("failed to open serial port '" + device + "': " + std::strerror(errno));
	}

	termios tty {};
	if (tcgetattr(fd, &tty) != 0) {
		const std::string message = "failed to read serial settings for '" + device + "': " +
			std::strerror(errno);
		::close(fd);
		throw std::runtime_error(message);
	}

	cfmakeraw(&tty);
	const speed_t speed = baudRateToSpeed(baud_rate);
	if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
		const std::string message = "failed to set baud rate for '" + device + "': " +
			std::strerror(errno);
		::close(fd);
		throw std::runtime_error(message);
	}

	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_oflag &= ~OPOST;
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		const std::string message = "failed to apply serial settings for '" + device + "': " +
			std::strerror(errno);
		::close(fd);
		throw std::runtime_error(message);
	}

	return fd;
}

ssize_t writeAll(int fd, const uint8_t * data, size_t size)
{
	size_t total_written = 0;
	while (total_written < size) {
		const ssize_t written = ::write(fd, data + total_written, size - total_written);
		if (written < 0) {
			if (errno == EINTR) {
				continue;
			}
			return -1;
		}
		total_written += static_cast<size_t>(written);
	}

	return static_cast<ssize_t>(total_written);
}

int16_t clampToInt16(double value)
{
	const double min_value = static_cast<double>(std::numeric_limits<int16_t>::min());
	const double max_value = static_cast<double>(std::numeric_limits<int16_t>::max());
	const double clamped = std::clamp(value, min_value, max_value);
	return static_cast<int16_t>(std::lround(clamped));
}

std::array<uint8_t, 8> makePayload(
	int16_t front_left_speed_mm_s,
	int16_t front_right_speed_mm_s,
	int16_t back_left_speed_mm_s,
	int16_t back_right_speed_mm_s)
{
	std::array<uint8_t, 8> payload {};
	const auto encode = [](uint8_t * destination, int16_t value) {
			const uint16_t raw = static_cast<uint16_t>(value);
			destination[0] = static_cast<uint8_t>(raw & 0xFFU);
			destination[1] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
		};

	encode(payload.data(), front_left_speed_mm_s);
	encode(payload.data() + 2, front_right_speed_mm_s);
	encode(payload.data() + 4, back_left_speed_mm_s);
	encode(payload.data() + 6, back_right_speed_mm_s);
	return payload;
}

}  // namespace

class DrivetrainControlNode : public rclcpp::Node
{
public:
	DrivetrainControlNode()
	: Node("drivetrain_control"),
		serial_port_(declare_parameter<std::string>("serial_port", "/dev/ttyUSB0")),
		serial_baud_(declare_parameter<int>("baudrate", 115200))
	{
		wheel_separation_mm_ = declare_parameter<double>("wheel_separation", 300.0);
		kinematics_ = std::make_unique<DifferentialKinematics>(wheel_separation_mm_);
		initializeSerialPort();

		cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
			"cmd_vel",
			rclcpp::QoS(10),
			[this](const geometry_msgs::msg::Twist::SharedPtr msg) {
				onCmdVel(*msg);
			});

		RCLCPP_INFO(
			get_logger(),
			"robot_control started with serial port %s, baud %d, wheel separation %.2f mm",
			serial_port_.c_str(),
			serial_baud_,
			wheel_separation_mm_);
	}

	~DrivetrainControlNode() override
	{
		if (serial_fd_ >= 0) {
			::close(serial_fd_);
			serial_fd_ = -1;
		}
	}

private:
	void initializeSerialPort()
	{
		if (access(serial_port_.c_str(), F_OK) != 0) {
			RCLCPP_WARN(
				get_logger(),
				"serial port %s is not available yet, node will stay alive and skip writes",
				serial_port_.c_str());
			serial_fd_ = -1;
			return;
		}

		try {
			serial_fd_ = openSerialPort(serial_port_, serial_baud_);
			RCLCPP_INFO(get_logger(), "serial port %s opened successfully", serial_port_.c_str());
		} catch (const std::exception & error) {
			serial_fd_ = -1;
			RCLCPP_WARN(
				get_logger(),
				"serial port %s could not be opened (%s); node will keep running without writes",
				serial_port_.c_str(),
				error.what());
		}
	}

	void onCmdVel(const geometry_msgs::msg::Twist & twist)
	{

		const WheelSpeedsMmPerS wheel_speeds = kinematics_->inverseKinematics(
			twist.linear.x,
			twist.linear.y,
			twist.angular.z);

		const int16_t left_speed_mm_s = clampToInt16(wheel_speeds.left_mm_s);
		const int16_t right_speed_mm_s = clampToInt16(wheel_speeds.right_mm_s);
		const std::array<uint8_t, 8> payload = makePayload(
			left_speed_mm_s,
			right_speed_mm_s,
			left_speed_mm_s,
			right_speed_mm_s);

		RCLCPP_INFO(
			get_logger(),
			"motor speeds mm/s: front_l=%d front_r=%d back_l=%d back_r=%d",
			static_cast<int>(left_speed_mm_s),
			static_cast<int>(right_speed_mm_s),
			static_cast<int>(left_speed_mm_s),
			static_cast<int>(right_speed_mm_s));

        if (serial_fd_ < 0) {
            RCLCPP_WARN(
                get_logger(),
                "cannot write to serial port %s because it is not open; skipping write",
                serial_port_.c_str());
			return;
		}

		const ssize_t written = writeAll(serial_fd_, payload.data(), payload.size());
		if (written != static_cast<ssize_t>(payload.size())) {
			RCLCPP_ERROR(
				get_logger(),
				"failed to write payload to %s: %s",
				serial_port_.c_str(),
				std::strerror(errno));
			return;
		}

		if (tcdrain(serial_fd_) != 0) {
			RCLCPP_WARN(
				get_logger(),
				"payload written to %s, but tcdrain failed: %s",
				serial_port_.c_str(),
				std::strerror(errno));
		}
	}

	double wheel_separation_mm_ {0.0};
	std::unique_ptr<DifferentialKinematics> kinematics_;
	std::string serial_port_;
	int serial_baud_ {115200};
	int serial_fd_ {-1};
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<DrivetrainControlNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
