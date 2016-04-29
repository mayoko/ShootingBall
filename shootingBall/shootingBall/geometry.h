#pragma once
#include <complex>
#include <vector>
#include <utility>

typedef double Real;
typedef std::complex<Real> Pt; // �_
typedef std::pair<Pt,Pt> Line; // ����
typedef std::vector<Pt> Poly; // ���p�`
// �~
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
// �����̒���
Real Sabs(const Line& l);
// �����Ɠ_�̋���
Real LPdist(const Line& l, const Pt& p);
// �_�Ɛ����̋���
Real SPdist(Line l, Pt p);
// ������������
bool crossS(const Line& p, const Line& q);
// �����̌�������
Pt intersect(const Line& p, const Line& q);
// �����̌�_
// tested: AOJ 2003
Pt line_line_intersect(const Line &p, const Line &q);
// �~�ƒ����̌�_
std::vector<Pt> circle_line_intersect(Line l,Cir c);
bool eq(Real l, Real r);
// �_�̓���/���E/�O���̃t���O
enum {GEOMETRY_IN, GEOMETRY_ON, GEOMETRY_OUT};
// �_�����p�`�̓���/���E/�O���̂ǂ��ɂ���̂��𔻒肷��
int contains(const Poly& P, const Pt& p);
// ���K��
Pt normalize(const Pt& p);
// ���˂�������(v�Ƃ������x��Line l�ɐi�����Ă����Ƃ�,v�𔽎˂�����)
Pt reflection(const Pt& v, const Line& l);
// �����ɐ����ȃx�N�g����񎦂���
Pt vertical(const Line& l);