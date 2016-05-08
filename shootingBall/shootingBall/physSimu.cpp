#include "physSimu.h"
#include "field.h"
#include "geometry.h"
#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

const int dx[4] = {1, 0, -1, 0};
const int dy[4] = {0, -1, 0, 1};

// 減衰項
const Real damp = 5;
// 矢印パネルでの加速項
const Real cdAccel = 1000;
// フィールドの幅
const Real width = 1010;
//const Real width = 640;
// フィールドの高さ
const Real height = 580;
//const Real height = 520;
//ボールの半径
const Real radius = 25;

physSimu::physSimu() {
	ballIsMoving = false;
	ballIsClear = false;
	ballIsOver = false;
	circle.r = radius;
}

void physSimu::wallDetect() {
	if (real(circle.p) + circle.r + 0.01 > width) {
		Line l;
		l.first = Pt(width, 0); l.second = Pt(width, height);
		Pt V = vertical(l);
		Pt a = circle.p - l.first;
		if (dot(V, a) * dot(V, v) < 0) {
			v = reflection(v, l);
			return;
		}
	}
	if (real(circle.p) - circle.r - 0.01 < 0) {
		Line l;
		l.first = Pt(0, 0); l.second = Pt(0, height);
		Pt V = vertical(l);
		Pt a = circle.p - l.first;
		if (dot(V, a) * dot(V, v) < 0) {
			v = reflection(v, l);
			return;
		}
	}
	if (imag(circle.p) + circle.r + 0.01 > height) {
		Line l;
		l.first = Pt(0, height); l.second = Pt(width, height);
		Pt V = vertical(l);
		Pt a = circle.p - l.first;
		if (dot(V, a) * dot(V, v) < 0) {
			v = reflection(v, l);
			return;
		}
	}
	if (imag(circle.p) - circle.r - 0.01 < 0) {
		Line l;
		l.first = Pt(0, 0); l.second = Pt(width, 0);
		Pt V = vertical(l);
		Pt a = circle.p - l.first;
		if (dot(V, a) * dot(V, v) < 0) {
			v = reflection(v, l);
			return;
		}
	}
}

void physSimu::simulate(const Field& field, Real t) {
	// 以前規定されていた速度を元にオブジェクトを動かす
	Real dt = t-this->t;
	circle.p += Pt(real(v) * dt, imag(v)*dt);
	this->t = t;
	// 壁にぶつかるなら速度を反転させる
	wallDetect();
	// firldにおかれている各オブジェクトに対して作用するものがあればそのように動かす
	int n = field.boards.size();
	for (Field::Board board : field.boards) {
		int id = board.id;
		//if (id == Field::Board::OBSTACLE) {
		//	// 円と直線が交わっていたら跳ね返る
		//	//! 頂点が時計回り,または反時計回りに並んでいることを前提としている
		//	for (int i = 0; i < 4; i++) {
		//		bool flag = false;
		//		Line l;
		//		l.first = board.position[i];
		//		l.second = board.position[(i+1)%4];
		//		if (circle.r + 0.01 > LPdist(l, circle.p)) {
		//			{
		//				Pt V = vertical(l);
		//				Pt a = circle.p - l.first;
		//				if (dot(V, a) * dot(V, v) > 0) continue;
		//			}
		//			vector<Pt> intersect = circle_line_intersect(l, circle);
		//			for (Pt p : intersect) {
		//				if (eq(0, SPdist(l, p))) {
		//					flag = true;
		//					break;
		//				}
		//			}
		//		}
		//		if (flag) {
		//			// ボールが直線にぶつかったとき跳ね返る処理
		//			v = reflection(v, l);
		//			break;
		//		}
		//	}\
		//} else
		if (id == Field::Board::CHANGE_DIRECTION) {
			
			if (contains(board.position, circle.p) == GEOMETRY_IN) {
				// 矢印の向きに応じて速度を変化させる

				v += cdAccel*dt*(board.position[(4-board.dir)%4] - board.position[(7-board.dir)%4]) /  abs(board.position[0] - board.position[1]);
				// 単純に向きを変えるだけ
				//v = abs(v) * (board.position[(4-board.dir)%4] - board.position[(7-board.dir)%4]) /  abs(board.position[0] - board.position[1]);
			}
		} else if (id == Field::Board::ACCELERATION){
            if (contains(board.position, circle.p) == GEOMETRY_IN){
             v *= 1.05;
            }
        } else if (id == Field::Board::DECELERATION){
            if (contains(board.position, circle.p) == GEOMETRY_IN){
             v *= 0.9;
            }
        } 
		//else if(id == Field::Board::HOLE){
		//	if (contains(board.position, circle.p) == GEOMETRY_IN) {
		//		//落ちたらボール消える
		//		fallIntoHole(field);
		//	}
  //      }
		else if(id == Field::Board::BLACKHOLE){
			if (contains(board.position, circle.p) == GEOMETRY_IN) {
				//落ちたらボール消える
				Real startV = abs(v);
				fallIntoBlackHole(field);
				shootFromWhiteHole(field, startV);
			}
		} else if(id == Field::Board::GOAL){
			if (contains(board.position, circle.p) == GEOMETRY_IN) {
				//ゴールに到達
				arrivedAtGoal(field);
			}
		}
	}
	// 任意のシミュレーションで行う処理:摩擦を受けて速度を微減させる
	Real length = abs(v);
	if (eq(length, 0)) return;
	//v *= 0.998;
}

void physSimu::shootBall(const Field& field, const Real& startV){
	cout << "shoot Ball" << endl;
	// firldにおかれている各オブジェクトのなかでスタート用パネルを探し、処理する
	int n = field.boards.size();
	for (Field::Board board : field.boards) {
		int id = board.id;
		cout << id << endl;
		if (id == Field::Board::START) {
			cout << "detect START" << endl;
			//4頂点の重心が中心座標
			Pt ballStartPos = (board.position[0] +board.position[1] +board.position[2] +board.position[3]) / 4.0;
			Cir ball(ballStartPos,radius);
			circle = ball;
			//マーカの縦方向でのワールド座標系での単位ベクトルに初期速さをかけて初期速度に
			//(4-board.dir)%4で常に左上になるらしい。そして順番は時計回りらしい。
			v = (board.position[(3-board.dir)%4] - board.position[(3-board.dir+3)%4]) /  abs(board.position[0] - board.position[1]) * startV;
		}
	}
	t = 0;
	ballIsMoving = true;
	ballIsOver = false;
	ballIsClear = false;
}

void physSimu::fallIntoHole(const Field& field){
	v = 0;
	ballIsMoving = false;	
	std::cout << "GAME OVER..." << std::endl;
	ballIsOver = true;
}
void physSimu::fallIntoBlackHole(const Field& field){
	ballIsMoving = false;
	ballIsOver = false;
}
void physSimu::shootFromWhiteHole(const Field& field,const Real& startV){
	cout << "shoot Ball" << endl;
	// firldにおかれている各オブジェクトのなかでスタート用パネルを探し、処理する
	int n = field.boards.size();
	for (Field::Board board : field.boards) {
		int id = board.id;
		cout << id << endl;
		if (id == Field::Board::WHITEHOLE) {
			cout << "detect START" << endl;
			//4頂点の重心が中心座標
			Pt ballStartPos = (board.position[0] +board.position[1] +board.position[2] +board.position[3]) / 4.0;
			//Cir ball(ballStartPos,radius);
			//circle = ball;
			////マーカの縦方向でのワールド座標系での単位ベクトルに初期速さをかけて初期速度に
			////(4-board.dir)%4で常に左上になるらしい。そして順番は時計回りらしい。
			//v = (board.position[(3-board.dir)%4] - board.position[(3-board.dir+3)%4]) /  abs(board.position[0] - board.position[1]) * startV;
			circle.p = ballStartPos;
		}
	}
	ballIsMoving = true;
	ballIsOver = false;
	ballIsClear = false;
}





void physSimu::arrivedAtGoal(const Field& field){
	v = 0;
	ballIsMoving = false;
	std::cout << "GOAL!!!!!" << std::endl;
	ballIsClear = true;
}

void physSimu::changeState(Real x, Real y, Real vx, Real vy) {
	circle.p = Pt(x, y);
	v = Pt(vx, vy);
}

void physSimu::print() const {
	cout << t << " " << circle.r << " #t, r" << endl;
	cout << real(circle.p) << " " << imag(circle.p) << " #x, y" << endl;
	cout << real(v) << " " << imag(v) << " #vx, vy" << endl;
}