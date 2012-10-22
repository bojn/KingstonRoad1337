#define _WIN32_WINDOWS 0x0501
#include <string>
#include <iostream>
#include <fstream>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include "TimeoutSerial.h"

using namespace boost;

std::string readLine(boost::asio::serial_port& serial)
{
	//Reading data char by char, code is optimized for simplicity, not speed
	using namespace boost;
	char c;
	std::string result;
	for(;;)
	{
		asio::read(serial,asio::buffer(&c,1));
		switch(c)
		{
		case '\r':
			break;
		case '\n':
			return result;
		default:
			result+=c;
		}
	}
}

template <typename T>
struct vector3 {
	vector3() : x(0),y(0),z(0) {}
	vector3(const T& ix, const T& iy, const T& iz) : x(ix), y(iy), z(iz) {}

	void operator+=(const vector3<T>& v) {
		x += v.x;
		y += v.y;
		z += v.z;
	}

	vector3<T> operator/=(const double& v) {
		vector3<T> out(x/v, y/v, z/v);
	}

	vector3<double> operator-(const vector3<double>& v) const {
		return vector3<double>((double)x - v.x, (double)y - v.y, (double)z - v.z);
	}

	T x,y,z;
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const vector3<T>& v)
{
	os << "<" << v.x << " ; " << v.y << " ; " << v.z << ">";
	return os;
}

class MPU6050 {
	typedef int DataType;
public:
	MPU6050(TimeoutSerial& serial) : _serial(serial)
	{
	}

	bool readSample() 
	{
		try {
			char numSamples = 1;
			_serial.write(&numSamples, 1);
			char ack;
			_serial.read(&ack, 1); // read ack
			if (ack != numSamples) return false;
		}
		catch (std::exception& e) {
			std::cerr << "readSamples() : " << e.what() << std::endl;
			return false;
		}
		try {
			std::string ln = _serial.readStringUntil("\n");
 			sscanf(ln.c_str(), "%d\t%d\t%d\t%d\t%d\t%d\n", &_a.x, &_a.y, &_a.z, &_g.x, &_g.y, &_g.z);
//sscanf(ln.c_str(), "%d\t%d\t%d\n", &_g.x, &_g.y, &_g.z);
			return true;
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			return false;
		}
	}


	void readSamples(unsigned int numSamples)
	{
		unsigned int succeededReads = 0;
		while (succeededReads < numSamples){
			if (readSample()) {
				_gyroSamples.push_back(g());
				_accSamples.push_back(a());
				succeededReads++;
				std::cerr << succeededReads << std::endl;
			}
			else {
				std::cerr << "!" << std::endl;
			}
		}
	}


	const vector3<DataType>& g() const {
		return _g;
	}

	const vector3<DataType>& a() const {
		return _a;
	}

	const std::vector<vector3<DataType>>& gyroSamples() const {
		return _gyroSamples;
	}

	const std::vector<vector3<DataType>>& accSamples() const {
		return _accSamples;
	}

private:
	TimeoutSerial& _serial;

	vector3<DataType> _g; // Gyroscope
	vector3<DataType> _a; // Accelerometer

	std::vector<vector3<DataType>> _gyroSamples;
	std::vector<vector3<DataType>> _accSamples;
};

template <typename T>
vector3<double> computeMean(const std::vector<vector3<T>>& v) {
	vector3<int> sum;
	for (std::vector<vector3<T>>::const_iterator vIt = v.begin(); vIt != v.end(); vIt++) 
	{
		sum += *vIt;
	}
	double length = v.size();
	return vector3<double>(sum.x / length, sum.y / length, sum.z / length);
}

void main()
{
	unsigned int baudRate = 115200;
	boost::asio::io_service io;
	TimeoutSerial serial;
	try {
		serial.open("COM3", baudRate);
		serial.close();
		serial.open("COM3", baudRate);
		serial.setTimeout(posix_time::milliseconds(200));
	}
	catch (...) {
		std::cerr << "could not open the COM-port" << std::endl;
	}

	MPU6050 mpu(serial);

	// Compute calibration
	std::cerr << "waiting for buffer..." << std::endl;
	//mpu.readSamples(1000);
	//vector3<double> gyroMean = computeMean(mpu.gyroSamples());
	//std::cerr << "gyro mean : " <<  gyroMean << std::endl;


	vector3<double> gyroSum;
	//std::vector<vector3<double>> gyroData;
	//unsigned int succeededReads = 0;
/**	while (succeededReads < 1000) {
		if (mpu.readSample()) {
			vector3<double> calbratedGyro = (mpu.g() - gyroMean);
			//std::cerr << succeededReads << " : " << calbratedGyro << std::endl;
      gyroSum += calbratedGyro;
			gyroData.push_back(calbratedGyro);
			succeededReads++;
		}
		else {
			//std::cerr << "waiting for sync..." << std::endl;
		}
	}
	*/
  
	//std::cerr << "gyro mean : " <<  gyroMean << std::endl;
	//std::cerr << "gyro sum : " << gyroSum << std::endl;
  mpu.readSamples(10000);
	//gyroSamples = mpu.gyroSamples();
	std::ofstream ofs("c:/temp/gyro.dlm", std::ios::out);
	for (std::vector<vector3<int>>::const_iterator vIt = mpu.gyroSamples().begin(); vIt != mpu.gyroSamples().end(); vIt++) {
		ofs << vIt->x << " ; " << vIt->y << " ; " << vIt->z << std::endl;
	}

}