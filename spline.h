#ifndef __SPLINE_H
#define __SPLINE_H
#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;
class CubicSpline1D {
private:
	vector<double> a;
	vector<double> b;
	std::vector<std::vector<double>> A;
	vector<double> B;
	vector<double> c;
	vector<double> d;
	vector<double> x;
	void calc_bd(const vector<double> a, vector<double>& b, const vector<double> c, vector<double>& d, const vector<double>h);
	vector<double> diff(const std::vector<double>& arr);
	void calc_A(const std::vector<double>& h, std::vector<std::vector<double>>& A);
	void calc_B(const std::vector<double>& h, std::vector<double> a, std::vector<double>& B );
	std::vector<double> solve_linear_system(const std::vector<std::vector<double>>& A, const std::vector<double>& B);
public:
	CubicSpline1D();
	CubicSpline1D(const vector<double>& x, const vector<double>& y);
	//CubicSpline1D();
	double calc_position(double x_val, const vector<double> s);
	vector<double> calc_first_derivative(vector<double> x_val) const;
	vector<double> calc_second_derivative(vector<double> x_val) const;
	void display();
};
class CubicSpline2D {
private:
	CubicSpline1D sx;
	CubicSpline1D sy;
	vector<double> calc_s(const std::vector<double>& x, const std::vector<double>& y);

public:
	vector<double> s;
	CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y);
	pair<double, double> calc_position(double x_val, const vector<double> s);
	void display();
};
#endif