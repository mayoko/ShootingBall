#include "geometry.h"
#include <complex>
#include <vector>
#include <utility>

#define rep2(i,m,n) for(int i=(int)(m);i<(int)(n);i++)
#define rep(i,n) rep2(i,0,n)
#define squere(x) ((x)*(x))

using namespace std;
const double EPS = 1e-8;
const double INF = 1e12;

typedef long long ll;
typedef unsigned long long ull;
typedef pair<int,int> P;

static int dx[4] = {-1,0,1,0};
static int dy[4] = {0,1,0,-1};

bool operator < (const Pt& a, const Pt& b) {
  return real(a) != real(b) ? real(a) < real(b) : imag(a) < imag(b);
}

Real cross(const Pt& a, const Pt& b) {
  return imag(conj(a)*b);
}
Real dot(const Pt& a, const Pt& b) {
  return real(conj(a)*b);
}

bool near(const Pt& p, const Pt& q){return abs(p - q) < EPS;}

LPposit ccw(const Pt& p, const Pt& q, const Pt& r) {
  Real c = cross(q-p,r-p);
  if (c < -EPS) return P_CW;
  if (c >  EPS) return P_CCW;
  if (dot(q - p, r - p) < -EPS) return P_CD;
  if (dot(p - q, r - q) < -EPS) return P_D;
  return P_OS;
}

// 線分の長さ
Real Sabs(const Line& l) {return abs(l.first - l.second); }
// 直線と点の距離
Real LPdist(const Line& l, const Pt& p) {return abs(cross(l.second-l.first,p-l.first)) / Sabs(l); }
// 点と線分の距離
Real SPdist(Line l, Pt p) {
    Real a = abs(l.first  - p);
    Real b = abs(l.second - p);
    Real c = Sabs(l);
    if (b * b + c * c > a * a && a * a + c * c > b * b){
        return LPdist(l, p);
    }
    return min(a, b);
}

// 線分交差判定
bool crossS(const Line& p, const Line& q){
  return
    ccw(p.first, p.second, q.first) * ccw(p.first, p.second, q.second) <= 0 &&
    ccw(q.first, q.second, p.first) * ccw(q.first, q.second, p.second) <= 0;
}

// 直線の交差判定
Pt intersect(const Line& p, const Line& q) {
  Pt vp = p.second - p.first;
  Pt vq = q.second - q.first;
  Pt c(cross(vp, p.first), cross(vq, q.first));
  return Pt(cross(c, Pt(vp.real(), vq.real())), cross(c, Pt(vp.imag(), vq.imag()))) / cross(vp, vq);
}

// 直線の交点
// tested: AOJ 2003
Pt line_line_intersect(const Line &p, const Line &q)
{
    Pt b = q.second-q.first;
    Real d1 = abs(cross(b, p.first-q.first));
    Real d2 = abs(cross(b, p.second-q.first));
    Real t = d1 / (d1 + d2);
    return p.first+(p.second-p.first)*t;
}

// 円と直線の交点
vector<Pt> circle_line_intersect(Line l,Cir c){
  vector<Pt> ret;
  Real di = LPdist(l,c.p);
  Real r=c.r;
  if(di+EPS > r) return ret;
  Pt v=(l.second-l.first);
  v/=abs(v);  
  Pt rv=v*Pt(0,1);
  rv*=di;  
  if(LPdist(l,c.p+rv) > di+EPS) rv = -rv;
  v*=sqrt(r*r-di*di);
  ret.push_back(c.p+rv-v);
  ret.push_back(c.p+rv+v);
  return ret;
}

bool eq(Real l, Real r)
{
    return (abs(l-r) < EPS);
}

int contains(const Poly& P, const Pt& p) {
  bool in = false;
  int n = P.size();
  for (int i = 0; i < n; ++i) {
    Pt a = P[i] - p, b = P[(i+1)%n] - p;
    if (imag(a) > imag(b)) swap(a, b);
    if (imag(a) <= 0 && 0 < imag(b))
      if (cross(a, b) < 0) in = !in;
    if (cross(a, b) == 0 && dot(a, b) <= 0) return GEOMETRY_ON;
  }
  return in ? GEOMETRY_IN : GEOMETRY_OUT;
}

Pt normalize(const Pt& p) {
    return p/abs(p);
}

Pt reflection(const Pt& v, const Line& l) {
    Pt p = l.first-l.second;
    p = normalize(p);
    Pt vp = p*dot(p, v);
    Pt vn = v-vp;
    return vp-vn;
}

Pt vertical(const Line& l) {
    Pt p = l.first - l.second;
    return Pt(-imag(p), real(p));
}