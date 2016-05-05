#pragma once
#include "geometry.h"
#include "Mat.h"
#include <vector>
#include <map>
#include <AR/config.h>
#include <AR/video.h>
#include <AR/param.h>			// arParamDisp()
#include <AR/ar.h>
#include <AR/gsub_lite.h>



/* カメラから受け取った情報でフィールドを構成するクラス */

class Field
{
public:
	// 各ARマーカーの役割を規定するクラス
	struct Board {
		enum {
			OBSTACLE = 0,
			CHANGE_DIRECTION = 1,
			START = 2,
			GOAL =  3,
			HOLE = 4,
			BLACKHOLE = 5,
			WHITEHOLE = 6,
			DECELERATION = 7,
			ACCELERATION = 8,
			EFFECT_NUM = 9,
		};
		// 何のボードか(いまのところ障害物と方向変換のみ)
		// 上で定義したenumで管理する
		int id;
		// 向き
		int dir;
		// 各頂点の位置
		Poly position;
		// デバッグ用　Boardの情報を記載する
		void print() const;
	};
	std::vector<Board> boards;
	// カメラ変換行列
	Mat H_cw;
	Field();
	// ARToolKitのidとBoardクラスのidのすり合わせ
	// trans[patt_id] = AR内部でpatt_idと解釈されるのがBoardではどのidになるか
	std::map<int, int> trans;
	// カメラから受け取った情報でFieldを構成する
	void receiveData(int marker_num, ARMarkerInfo* marker_info);
	// 情報をすべてなしにする(毎周期呼び出す)
	void clear();
};