#pragma once
#include <complex>
#include <vector>
#include <utility>

typedef double Real;
typedef std::complex<Real> Pt; // 点
typedef std::pair<Pt,Pt> Line; // 線分
typedef std::vector<Pt> Poly; // 多角形
// 円
struct Cir {
  Pt p; Real r;
  Cir() {}
  Cir(const Pt &p, Real r) : p(p), r(r) { }
};
bool operator < (const Pt& a, const Pt& b);
Real cross(const Pt& a, const Pt& b);
Real dot(const Pt& a, const Pt& b);
bool near(const Pt& p, const Pt& q);
/* ccw :
CD  : counter direction
CW  : clock wise
OS  : on segment
CCW : counter clock wise
D   : direction
 */
enum LPposit { P_CD = -2, P_CW = -1, P_OS = 0, P_CCW = 1, P_D = 2};
LPposit ccw(const Pt& p, const Pt& q, const Pt& r);
// 線分の長さ
Real Sabs(const Line& l);
// 直線と点の距離
Real LPdist(const Line& l, const Pt& p);
// 点と線分の距離
Real SPdist(Line l, Pt p);
// 線分交差判定
bool crossS(const Line& p, const Line& q);
// 直線の交差判定
Pt intersect(const Line& p, const Line& q);
// 直線の交点
// tested: AOJ 2003
Pt line_line_intersect(const Line &p, const Line &q);
// 円と直線の交点
std::vector<Pt> circle_line_intersect(Line l,Cir c);
bool eq(Real l, Real r);
// 点の内部/境界/外部のフラグ
enum {GEOMETRY_IN, GEOMETRY_ON, GEOMETRY_OUT};
// 点が多角形の内部/境界/外部のどこにあるのかを判定する
int contains(const Poly& P, const Pt& p);
// 正規化
Pt normalize(const Pt& p);
// 反射させるやつ(vという速度がLine lに進入してきたとき,vを反射させる)
Pt reflection(const Pt& v, const Line& l);
// 直線に垂直なベクトルを提示する
Pt vertical(const Line& l);