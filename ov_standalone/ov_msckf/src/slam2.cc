#include <functional>

#include "common/component.hh"
#include "common/switchboard.hh"
#include "common/data_format.hh"

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Dense>

#include "core/VioManager.h"
// #include "core/RosVisualizer.h"
#include "state/State.h"

using namespace ILLIXR;

using namespace ov_msckf;

class slam1 : public component {
public:
	/* Provide handles to slam1 */
	slam1(std::unique_ptr<writer<pose_type>>&& pose, std::unique_ptr<writer<bool>>&& ready)
		: _m_pose{std::move(pose)}
		, _m_ready{std::move(ready)}
		, _m_begin{std::chrono::system_clock::now()}
	{ }

	void feed_cam(const cam_type* cam_frame) {
		const std::lock_guard<std::mutex> lock{_m_mutex};
		// okvis_estimator.addImage(cvtTime(cam_frame->time), cam_frame->id, *cam_frame->img);
		assert(cam_frame->img0);
		assert(cam_frame->img1);
		cv::Mat img0 {*cam_frame->img0};
		cv::Mat img1 {*cam_frame->img1};
		open_vins_estimator.feed_measurement_stereo(cvtTime(cam_frame->time), img0, img1, 0, 1);
		State* state = open_vins_estimator.get_state();

		auto quat = state->imu()->quat();

		if (open_vins_estimator.intialized()) {
			if (isUninitialized) {
				isUninitialized = false;
				_m_ready->put(new bool{true});
			}
			_m_pose->put(new pose_type{
				cam_frame->time,
				(state->imu()->pos()).cast<float>(),
				Eigen::Quaternionf{quat(3), quat(0), quat(1), quat(2)},
			});
		}
	}

	void feed_imu(const imu_type* imu_reading) {
		const std::lock_guard<std::mutex> lock{_m_mutex};
		open_vins_estimator.feed_measurement_imu(cvtTime(imu_reading->time), (imu_reading->angular_v).cast<double>(), (imu_reading->linear_a).cast<double>());
	}

	virtual void _p_start() override {
		/* All of my work is already scheduled synchronously. Nohting to do here. */
	}

	virtual void _p_stop() override { }

	virtual ~slam1() override { }

private:
	std::unique_ptr<writer<pose_type>> _m_pose;
	std::unique_ptr<writer<bool>> _m_ready;
	time_type _m_begin;
	std::mutex _m_mutex;

	VioManager open_vins_estimator;

	bool isUninitialized = true;
	
	double cvtTime(time_type t) {
		auto diff = t - _m_begin;
		return static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(diff).count()) / 1000000000.0;
	}

};

extern "C" component* create_component(switchboard* sb) {
	/* First, we declare intent to read/write topics. Switchboard
	   returns handles to those topics. */
	auto pose_ev = sb->publish<pose_type>("slow_pose");
	auto slam_ready = sb->publish<bool>("slam_ready");
	pose_ev->put(new pose_type{std::chrono::system_clock::now(), Eigen::Vector3f{0, 0, 0}, Eigen::Quaternionf{1, 0, 0, 0}});
	slam_ready->put(new bool{false});

	auto this_slam1 = new slam1{std::move(pose_ev), std::move(slam_ready)};
	sb->schedule<cam_type>("cams", std::bind(&slam1::feed_cam, this_slam1, std::placeholders::_1));
	sb->schedule<imu_type>("imu0", std::bind(&slam1::feed_imu, this_slam1, std::placeholders::_1));

	return this_slam1;
}