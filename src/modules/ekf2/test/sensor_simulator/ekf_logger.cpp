#include "ekf_logger.h"

#include <iomanip>

EkfLogger::EkfLogger(std::shared_ptr<Ekf> ekf):
	_ekf{ekf},
	_ekf_wrapper(ekf)
{
}

void EkfLogger::setFilePath(std::string file_path)
{
	_file_path = file_path;
}

void EkfLogger::writeStateToFile(const int gps_nsat)
{
	if (!_file_opened) {
		_file.open(_file_path);
		_file_opened = true;
		_file << "Timestamp";

		if (_state_logging_enabled) {
			for (int i = 0; i < 24; i++) {
				_file << ",state[" << i << "]";
			}
		}

		if (_variance_logging_enabled) {
			for (int i = 0; i < 24; i++) {
				_file << ",variance[" << i << "]";
			}
		}

		_file << ", gps_nsta";

		_file << ", estHeading, predictedMagHeading" ;

		_file << std::endl;
	}

	if (_file) {
		writeState(gps_nsat);
	}
	else {
		std::cout << "Can not write to output file" << std::endl;
	}
}

void EkfLogger::writeState(const int gps_nsat)
{
	if (_state_logging_enabled) {
		uint64_t time_us = _ekf->get_imu_sample_delayed().time_us;
		_file << time_us;

		if (_state_logging_enabled) {
			matrix::Vector<float, 24> state = _ekf->getStateAtFusionHorizonAsVector();

			for (int i = 0; i < 24; i++) {
				_file << "," << std::setprecision(3) << state(i);
			}
		}

		if (_variance_logging_enabled) {
			matrix::Vector<float, 24> variance = _ekf->covariances_diagonal();

			for (int i = 0; i < 24; i++) {
				_file << "," << variance(i);
			}
		}
		//get heading
		float h1 = _ekf->getEstHeading();
		float h2 = _ekf->getPredMagHeading();

		_file << "," << gps_nsat;
		_file << "," << h1 <<"," << h2;
		_file << std::endl;
	}
}

#include <sstream>
using namespace std;
std::string EkfLogger::printParams() const
{
	parameters* p = _ekf->getParamHandle();

	ostringstream os;
	os << "mag_fusion_type=" << p->mag_fusion_type << endl;
	os << "vdist_sensor_type=" << p->vdist_sensor_type << endl;
	os << "terrain_fusion_mode" << p->terrain_fusion_mode << endl;
	os << "sensor_interval_max_ms" << p->sensor_interval_max_ms << endl;

	// measurement time delays
	os << "mag_delay_ms=" << p->mag_delay_ms<< endl;
	os << "baro_delay_ms=" << p->baro_delay_ms << endl;
	os << "gps_delay_ms=" << p->gps_delay_ms << endl;
	os << "airspeed_delay_ms=" << p->airspeed_delay_ms << endl;
	os << "flow_delay_ms=" << p->flow_delay_ms << endl;
	os << "range_delay_ms=" << p->range_delay_ms << endl;
	os << "ev_delay_ms=" << p->ev_delay_ms << endl;
	os << "auxvel_delay_ms=" << p->auxvel_delay_ms << endl;

	// input noise
	os << "gyro_noise=" << p->gyro_noise << endl;
	os << "accel_noise=" << p->accel_noise << endl;

	// process noise
	os << "gyro_bias_p_noise=" << p->gyro_bias_p_noise << endl;
	os << "accel_bias_p_noise=" << p->accel_bias_p_noise << endl;
	os << "mage_p_noise=" << p->mage_p_noise << endl;
	os << "magb_p_noise=" << p->magb_p_noise << endl;
	os << "wind_vel_p_noise=" << p->wind_vel_p_noise << endl;
	os << "wind_vel_p_noise_scaler=" << p->wind_vel_p_noise_scaler << endl;
	os << "terrain_p_noise=" << p->terrain_p_noise << endl;
	os << "terrain_gradient=" << p->terrain_gradient << endl;
	os << "terrain_timeout=" << p->terrain_timeout << endl;

	// initialization errors
	os << "switch_on_gyro_bias=" << p->switch_on_gyro_bias << endl;
	os << "switch_on_accel_bias=" << p->switch_on_accel_bias << endl;
	os << "initial_tilt_err=" << p->initial_tilt_err << endl;
	os << "initial_wind_uncertainty=" << p->initial_wind_uncertainty << endl;

	// position and velocity fusion
	os << "gps_vel_noise=" << p->gps_vel_noise << endl;
	os << "gps_pos_noise=" << p->gps_pos_noise << endl;
	os << "pos_noaid_noise=" << p->pos_noaid_noise << endl;
	os << "baro_noise=" << p->baro_noise << endl;
	os << "baro_drift_rate=" << p->baro_drift_rate << endl;
	os << "baro_innov_gate=" << p->baro_innov_gate << endl;
	os << "gps_pos_innov_gate=" << p->gps_pos_innov_gate << endl;
	os << "gps_vel_innov_gate=" << p->gps_vel_innov_gate << endl;
	os << "gnd_effect_deadzone=" << p->gnd_effect_deadzone << endl;
	os << "gnd_effect_max_hgt=" << p->gnd_effect_max_hgt << endl;

	// magnetometer fusion
	os << "mag_heading_noise=" << p->mag_heading_noise << endl;
	os << "mag_noise=" << p->mag_noise << endl;
	os << "mag_declination_deg=" << p->mag_declination_deg << endl;
	os << "heading_innov_gate=" << p->heading_innov_gate << endl;
	os << "mag_innov_gate=" << p->mag_innov_gate << endl;
	os << "mag_declination_source = " << p->mag_declination_source << endl;
	os << "mag_fusion_type=" << p->mag_fusion_type << endl;
	os << "mag_acc_gate=" << p->mag_acc_gate << endl;
	os << "mag_yaw_rate_gate=" << p->mag_yaw_rate_gate << endl;
	os << "quat_max_variance=" << p->quat_max_variance << endl;

	// GNSS heading fusion
	os << "gps_heading_noise=" << p->gps_heading_noise << endl;

	// airspeed fusion
	os << "tas_innov_gate=" << p->tas_innov_gate << endl;
	os << "eas_noise=" << p->eas_noise << endl;
	os << "arsp_thr=" << p->arsp_thr << endl;

	// synthetic sideslip fusion
	os << "beta_innov_gate=" << p->beta_innov_gate << endl;
	os << "beta_noise=" << p->beta_noise << endl;
	os << "beta_avg_ft_us=" << p->beta_avg_ft_us << endl;

	// range finder fusion
	os << "range_noise=" << p->range_noise << endl;
	os << "range_innov_gate=" << p->range_innov_gate << endl;
	os << "rng_gnd_clearance=" << p->rng_gnd_clearance << endl;
	os << "rng_sens_pitch=" << p->rng_sens_pitch << endl;
	os << "range_noise_scaler=" << p->range_noise_scaler << endl;
	os << "vehicle_variance_scaler=" << p->vehicle_variance_scaler << endl;
	os << "max_hagl_for_range_aid=" << p->max_hagl_for_range_aid << endl;
	os << "max_vel_for_range_aid=" << p->max_vel_for_range_aid << endl;
	os << "range_aid=" << p->range_aid << endl;
	os << "range_aid_innov_gate=" << p->range_aid_innov_gate << endl;
	os << "range_valid_quality_s=" << p->range_valid_quality_s << endl;
	os << "range_cos_max_tilt=" << p->range_cos_max_tilt << endl;

	// vision position fusion
	os << "ev_vel_innov_gate=" << p->ev_vel_innov_gate << endl;
	os << "ev_pos_innov_gate=" << p->ev_pos_innov_gate << endl;

	// optical flow fusion
	os << "flow_noise=" << p->flow_noise << endl;
	os << "flow_noise_qual_min=" << p->flow_noise_qual_min << endl;
	os << "flow_qual_min = " << p->flow_qual_min << endl;
	os << "flow_innov_gate=" << p->flow_innov_gate << endl;

	// these parameters control the strictness of GPS quality checks used to determine if the GPS is
	// good enough to set a local origin and commence aiding
	os << "gps_check_mask = " << p->gps_check_mask << endl;
	os << "req_hacc=" << p->req_hacc << endl;
	os << "req_vacc=" << p->req_vacc << endl;
	os << "req_sacc=" << p->req_sacc << endl;
	os << "req_nsats = " << p->req_nsats << endl;
	os << "req_pdop=" << p->req_pdop << endl;
	os << "req_hdrift=" << p->req_hdrift << endl;
	os << "req_vdrift=" << p->req_vdrift << endl;

	// XYZ offset of sensors in body axes (m)
/*
	Vector3f imu_pos_body = " << << endl;
	Vector3f gps_pos_body = " << << endl;
	Vector3f rng_pos_body = " << << endl;
	Vector3f flow_pos_body = " << << endl;
	Vector3f ev_pos_body = " << << endl;
*/

	// output complementary filter tuning
	os << "vel_Tau=" << p->vel_Tau << endl;
	os << "pos_Tau=" << p->pos_Tau << endl;

	// accel bias learning control
	os << "acc_bias_lim=" << p->acc_bias_lim << endl;
	os << "acc_bias_learn_acc_lim=" << p->acc_bias_learn_acc_lim << endl;
	os << "acc_bias_learn_gyr_lim=" << p->acc_bias_learn_gyr_lim << endl;
	os << "acc_bias_learn_tc=" << p->acc_bias_learn_tc << endl;

	os << "valid_timeout_max=" << p->valid_timeout_max << endl;

	// static barometer pressure position error coefficient along body axes
	os << "static_pressure_coef_xp=" << p->static_pressure_coef_xp << endl;
	os << "static_pressure_coef_xn=" << p->static_pressure_coef_xn << endl;
	os << "static_pressure_coef_yp=" << p->static_pressure_coef_yp << endl;
	os << "static_pressure_coef_yn=" << p->static_pressure_coef_yn << endl;
	os << "static_pressure_coef_z=" << p->static_pressure_coef_z << endl;
	// upper limit on airspeed used for correction  (m/s**2)
	os << "max_correction_airspeed=" << p->max_correction_airspeed << endl;

	// multi-rotor drag specific force fusion
	os << "drag_noise=" << p->drag_noise << endl;
	os << "bcoef_x=" << p->bcoef_x << endl;
	os << "bcoef_y=" << p->bcoef_y << endl;
	os << "mcoef=" << p->mcoef << endl;

	// control of accel error detection and mitigation (IMU clipping)
	os << "vert_innov_test_lim=" << p->vert_innov_test_lim << endl;
	os << "bad_acc_reset_delay_us=" << p->bad_acc_reset_delay_us << endl;

	// auxiliary velocity fusion
	os << "auxvel_noise=" << p->auxvel_noise << endl;
	os << "auxvel_gate=" << p->auxvel_gate << endl;

	// control of on-ground movement check
	os << "is_moving_scaler=" << p->is_moving_scaler << endl;

	// compute synthetic magnetomter Z value if possible
	os << "synthesize_mag_z=" << p->synthesize_mag_z << endl;
	os << "check_mag_strength=" << p->check_mag_strength << endl;

	// Parameters used to control when yaw is reset to the EKF-GSF yaw estimator value
	os << "EKFGSF_tas_default=" << p->EKFGSF_tas_default << endl;
	os << "EKFGSF_reset_delay=" << p->EKFGSF_reset_delay << endl;
	os << "EKFGSF_yaw_err_max=" << p->EKFGSF_yaw_err_max << endl;
	os << "EKFGSF_reset_count_limit=" << p->EKFGSF_reset_count_limit << endl;

	return os.str();
}
