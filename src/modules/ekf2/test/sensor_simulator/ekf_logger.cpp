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

void EkfLogger::writeStateToFile(const int gps_flag)
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

		if (gps_flag >= 0) {
			_file << ", gpsOnOff";
		}

		_file << ", estHeading, predictedMagHeading" ;

		_file << std::endl;
	}

	if (_file) {
		writeState( gps_flag );

	} else {
		std::cerr << "Can not write to output file" << std::endl;
#if _WINDOWS
		std::exit(-1);
#else
		system_exit(-1);
#endif
	}

}

void EkfLogger::writeState(const int gps_flag)
{
	if (_state_logging_enabled) {
		uint64_t time = _ekf->get_imu_sample_delayed().time_us;
		_file << time;

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

		if (gps_flag >= 0) {
			_file << "," << gps_flag;
		}
		
		float h1 = _ekf->getEstHeading();
		float h2 = _ekf->getPredMagHeading();
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

	return os.str();

}
