#pragma once
#include "geometry.h"
#include "field.h"

// およそ530*782らしいです
class physSimu
{
public:
	Cir circle; // 座標および半径
	Pt v; // 速度
	Real t; // 時間
	physSimu();
	// 物理シミュレーション関係なく位置,速度を変更する
	void changeState(Real x, Real y, Real vx, Real vy);
	// 壁にぶつかるかを探知してぶつかるなら速度を反転させる
	void wallDetect();
	// Fieldおよび時間を読み込んだとき,ボールの運動を規定する
	void simulate(const Field& field, Real t);
	//ボールを発射するための処理
	void shootBall(const Field& field, const Real& startV);
	//ボールを発射したか否か
	bool ballIsMoving;
	bool ballIsClear;
	bool ballIsOver;
	//穴に落ちたらボールを消す処理
	void fallIntoHole(const Field&);
	//ゴール処理
	void arrivedAtGoal(const Field&);
	// ボールの運動を記述する(デバッグ用)
	void print() const ;
};