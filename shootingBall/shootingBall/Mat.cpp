#include "Mat.h"
#include <vector>
#include <cassert>

using namespace std;

Mat::Mat(void)
{
}

Mat::Mat(int n, int m) {
	mat.resize(n);
	for (int i = 0; i < n; i++) mat[i].resize(m);
}

Mat mul(const Mat& A, const Mat& B) {
	int n = A.mat.size();
	int m = B.mat[0].size();
	int l = A.mat[0].size();
	Mat ret(n, m);
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			for (int k = 0; k < l; k++) {
				ret.mat[i][j] += A.mat[i][k] * B.mat[k][j];
			}
		}
	}
	return ret;
}

void Mat::resize(int n, int m) {
	mat.resize(n);
	for (int i = 0; i < n; i++) mat[i].resize(m);
}

Mat::~Mat(void)
{
}
