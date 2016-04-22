#pragma once
#include <vector>
class Mat
{
public:
	Mat(void);
	Mat(int n, int m);
	~Mat(void);
	std::vector<std::vector<double> > mat;
	void resize(int n, int m);
};

Mat mul(const Mat& A, const Mat& B);