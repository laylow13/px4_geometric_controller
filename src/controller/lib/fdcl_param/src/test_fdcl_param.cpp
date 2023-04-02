#include <iostream>
#include <string>
#include "Eigen/Dense"

#include "fdcl/param.hpp"

using namespace std;
using std::placeholders::_1;
int main(void)
{
	double m;
	string dev;
	int addr;
	Eigen::Matrix<double, 3, 3> J;
	bool GPS;
	
    fdcl::param pfile;
	
	pfile.open("config.txt");

	pfile.read("UAV.m",m);
	cout << "m=" << m << endl;
	pfile.read("UAV.J",J);
	cout << "J=" << J << endl;
	pfile.read("IMU.dev",dev);
	cout << "dev=" << dev << endl;
	pfile.read("GPS.available",GPS);
	cout << "GPS=" << GPS << endl;

	pfile.save("I2C.addr",50);
	pfile.read("I2C.addr",addr);
	cout << "addr=" << addr << endl;
	pfile.save("GPS.available",true);
	pfile.read("GPS.available",GPS);
	cout << "GPS=" << GPS << endl;


	pfile.close();
}


