#include "spline.h"
CubicSpline1D::CubicSpline1D() {
}
CubicSpline1D::CubicSpline1D(const vector<double>& x, const vector<double>& y) { 
	//	x:compute cucumlative sum
	std::vector<double> h = diff(x);
    for (int i = 0; i < (int)h.size(); i++) {
		if (h[i] < 0) {
			throw std::runtime_error("x coordinates must be sorted in ascending order");
		}
	}
	a = y;
	b.resize(h.size(), 0.0);
	d.resize(h.size(), 0.0);
	calc_A(h, A);
	calc_B(h, a, B);
	c = solve_linear_system(A, B);
	calc_bd(a, b, c, d, h);
}
void CubicSpline1D::calc_A(const std::vector<double>& h, std::vector<std::vector<double>>& A) {
	int nx = h.size() + 1; // 3
	A.resize(nx, std::vector<double>(h.size() + 1, 0.0));
	A[0][0] = 1.0;
	for (int i = 0; i < nx - 1 ; i++) {
		if (i != nx - 2) {
			A[i + 1][i + 1] = 2.0 * (h[i] + h[i + 1]);
		}
		A[i + 1][i] = h[i];
		A[i][i + 1] = h[i];
	}
	A[0][1] = 0.0;
	A[nx - 1][nx - 2] = 0.0;
	A[nx - 1][nx - 1] = 1.0;
}
void CubicSpline1D::calc_B(const std::vector<double>& h, std::vector<double> a, std::vector<double>& B) {
	int nx = h.size() + 1;
	B.resize(nx , 0.0);
	for (int i = 0; i < nx - 2 ; i++) {
		B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
	}
}
vector<double> CubicSpline1D::diff(const std::vector<double>& arr) {
	std::vector<double> result(arr.size() - 1, 0.0);
	//adjacent_difference(arr.begin(), arr.end(), result.begin());
	for (int i = 0; i < result.size(); i++) {
		result[i]  = arr[i + 1] - arr[i];
	}
	return result;
}
std::vector<double> CubicSpline1D::solve_linear_system(const std::vector<std::vector<double>>& A, const std::vector<double>& B) {
	// Kiểm tra kích thước hợp lệ của ma trận A và vector B
	if (A.size() != B.size() || A.empty() || A[0].size() != A.size()) {
		throw std::invalid_argument("Invalid dimensions of A and B");
	}

	// Số phần tử của hệ phương trình tuyến tính
	const int n = A.size();

	// Sao chép ma trận A vào một ma trận tạm thời
	std::vector<std::vector<double>> tempA = A;

	// Sao chép vector B vào một vector tạm thời
	std::vector<double> tempB = B;

	// Khởi tạo vector kết quả
	std::vector<double> X(n, 0.0);

	// Phương pháp điều chỉnh (Gauss-Jordan elimination)
	for (int i = 0; i < n; ++i) {
		// Tìm vị trí phần tử chính không bằng 0
		int maxRow = i;
		for (int j = i + 1; j < n; ++j) {
			if (std::abs(tempA[j][i]) > std::abs(tempA[maxRow][i])) {
				maxRow = j;
			}
		}

		// Hoán đổi hai hàng
		std::swap(tempA[i], tempA[maxRow]);
		std::swap(tempB[i], tempB[maxRow]);

		// Không thể giải hệ phương trình tuyến tính
		if (std::abs(tempA[i][i]) < 1e-12) {
			throw std::runtime_error("Unable to solve linear system");
		}

		// Đưa các phần tử phía dưới đường chéo chính về 0
		for (int j = i + 1; j < n; ++j) {
			double ratio = tempA[j][i] / tempA[i][i];
			tempA[j][i] = 0.0;
			for (int k = i + 1; k < n; ++k) {
				tempA[j][k] -= ratio * tempA[i][k];
			}
			tempB[j] -= ratio * tempB[i];
		}
	}

	// Phương pháp lùi (Backward substitution)
	for (int i = n - 1; i >= 0; --i) {
		double sum = 0.0;
		for (int j = i + 1; j < n; ++j) {
			sum += tempA[i][j] * X[j];
		}
		X[i] = (tempB[i] - sum) / tempA[i][i];
	}

	return X;
}
void CubicSpline1D::calc_bd(const vector<double> a,vector<double>& b, const vector<double> c, vector<double>& d,const vector<double>h){
		
    for (int i = 0; i < (int)a.size() - 1 ; i++) {
		double dd = (c[i + 1] - c[i]) / (3.0 * h[i]);
		double db = ((a[i + 1] - a[i]) / h[i]) - (h[i] / 3.0) * (2.0 * c[i] + c[i + 1]);

		b[i] = db;
		d[i] = dd;
	}
}
   double CubicSpline1D:: calc_position(double x_val, const vector<double> s)  {
	int nIndex = 0;
	if (x_val < s[0]) {
		return 0.0;
	}
	else if (x_val > s[s.size() - 1]) {
		return 0.0;
	}

    for (int i = 0; i < (int)s.size(); i++) {
		if (x_val > s[i]) {
			continue;
		}
		else if (x_val <= s[i]) {
			nIndex = i - 1 ;
			if (nIndex < 0)
			{
				nIndex = 0;
			}
			break;
		}
	}
	double dx = x_val - s[nIndex];
//	cout << "  " << dx;
	double position = a[nIndex] + b[nIndex] * dx + c[nIndex] * dx * dx + d[nIndex] * dx * dx * dx;
	return position;
}
void CubicSpline1D::display() {
    cout << "a:";
    for (int i = 0; i < (int)a.size(); i++) {
        cout << "  " << a[i];
    }
    cout << "\nb:";
    for (int i = 0; i < (int)b.size(); i++) {
        cout << "  " << b[i];
    }
    cout << "\nc:";
    for (int i = 0; i < (int)c.size(); i++) {
        cout << "  " << c[i];
    }
    cout << "\nd:";
    for (int i = 0; i < (int)d.size(); i++) {
        cout << "  " << d[i];
    }
    cout << "\n";
}
/*-------------------------------------------------------------------------------------------------------------------------------------------------*/
CubicSpline2D::CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y) {
	s = calc_s(x, y);
	sx = CubicSpline1D(s, x);
	sy = CubicSpline1D(s, y);
}
vector<double> CubicSpline2D::calc_s(const std::vector<double>& x, const std::vector<double>& y) {
	std::vector<double> dx(x.size() - 1);
	std::vector<double> dy(y.size() - 1);
	for (size_t i = 0; i < dx.size(); ++i) {
		dx[i] = x[i + 1] - x[i];
		dy[i] = y[i + 1] - y[i];
	}
	std::vector<double> ds(dx.size());
	for (size_t i = 0; i < ds.size(); ++i) {
		ds[i] = hypot(dx[i], dy[i]);
	}
	std::vector<double> s(ds.size() + 1);
	s[0] = 0.0;
	for (size_t i = 0; i < ds.size(); ++i) {
		s[i + 1] = s[i] + ds[i];
	}
	return s;
}
pair<double, double> CubicSpline2D::calc_position(double x_val, const vector<double> s) {
	double x = sx.calc_position(x_val, s);
	double y = sy.calc_position(x_val, s);
	return make_pair(x, y);
}
void CubicSpline2D::display() {
//	sx.display();
//	sy.display();
}
