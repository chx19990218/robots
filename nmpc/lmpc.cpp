#include"Eigen/Eigen"
using namespace std;
int main()
{
	int Np = 30;
	int Nc = 1;
	float inter = 0.1
	MatrixXd A(4,4);
	MatrixXd B(4,2);
	int row_A = 4;
	int col_A = 4;
	int row_B = 4;
	int col_B = 2;
	MatrixXd R(Nc*col_B, Nc*col_B)
	R.setIdentity();
	cout << R;
	return 0;
}
