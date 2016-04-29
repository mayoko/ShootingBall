#include "field.h"
#include "geometry.h"
#include "Mat.h"

#include <map>
#include <utility>
#include <fstream>
#include <iostream>

using namespace std;

void Field::Board::print() const {
	cout << "id is " << id << endl;
	switch(id) {
	case OBSTACLE:
		cout << "OBSTACLE" << endl;
		break;
	case CHANGE_DIRECTION:
		cout << "CHANGE_DIRECTION" << endl;
		break;
	case START:
		cout << "START" << endl;
		break;
	case GOAL:
		cout << "GOAL" << endl;
		break;
	case HOLE:
		cout << "HOLE" << endl;
		break;
	case BLACKHOLE:
		cout << "BLACKHOLE" << endl;
		break;
	case WHITEHOLE:
		cout << "WHITEHOLE" << endl;
		break;
	case ACCELERATION:
        cout << "ACCELERATION" << endl;
    break;
    case DECELERATION:
        cout << "DECELERATION" << endl;
    break;

	default:
		cout << "NONE" << endl;
		break;
	}
	cout << "dir is " << dir << endl;
	// 場所
	Pt tmp = Pt(0, 0);
	for (auto c : position) {
		tmp += c;
	}
	tmp /= 4.0;
	cout << "position(x, y) is " << endl;
	cout << real(tmp) << " " << imag(tmp) << endl;
}

Field::Field(void)
{
	H_cw.resize(3, 3);
	ifstream ifs("Data/H_wc.txt");
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) ifs >> H_cw.mat[i][j];
	cout << "initialize H_wc mat" << endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			cout << H_cw.mat[i][j] << " ";
		}
		cout << endl;
	}
}

void Field::receiveData(int marker_num, ARMarkerInfo* marker_info) {
	//認識したマーカーのidと座標
	//TODO ほかのファイルとの整合性をあわせる
	for(int  j=0; j<marker_num; j++ ){
		if (marker_info[j].cf < 0.15) continue;
		Board new_board;
		new_board.id = trans[marker_info[j].id];
		new_board.dir = marker_info[j].dir;
		for(int vertex_num = 0; vertex_num < 4;vertex_num++){
			Mat vec(3, 1);
			for (int i = 0; i < 2; i++) vec.mat[i][0] = marker_info[j].vertex[vertex_num][i];
			vec.mat[2][0] = 1;
			vec = mul(H_cw, vec);
			new_board.position.emplace_back(vec.mat[0][0]/vec.mat[2][0],vec.mat[1][0]/vec.mat[2][0]+150);
		}
		boards.push_back(new_board);
	}
	// デバッグ用に情報を記載する
	/*cout << "Find Boards" << endl;
	for (Board board : boards) {
		board.print();
	}*/
}

void Field::clear() {
	boards.clear();
}