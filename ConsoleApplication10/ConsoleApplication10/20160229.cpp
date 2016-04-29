//
//  20160229
//
//
//

//そもそもカメラが起動しなくて、既存のプログラムを動かすのができないです
//なので、担当部分で、追加する内容だけメモ書きしてます
//内容もほとんど真似して書いただけで中身ないです、ごめんなさい



//加速と減速


//field.cppの中身
//print()の中身
case ACCELERATION:
    cout << "ACCELERATION" << endl;
    break;
case DECELERATION:
    cout << "DECELERATION" << endl;
    break;

//physSimu.cpp の中身
//simulateの中身
//forの中身、それぞれのマーカーの役割のところ
//加速させる
else if (id == Field::Board::ACCELERATION){
    if (contains(board.position, circle.p) == GEOMETRY_IN){
        v *= 1.1;
    }
}
//減速させる
else if (id == Field::Board::DECELERATION){
    if (contains(board.position, circle.p) == GEOMETRY_IN){
        v *= 0.9;
    }
}


//具体的に新しいマーカーの模様を決めたり、それを読み取るのは ARTK_MultiMarker.cpp ですか？
//その中身のどこをどう変えたら良いかは、よくわからないです。追いついてないです。



//方向転換


//既にできている、CHANGE_DIRECTION　のままでもOK？？

//physSimu.cpp　の中身
//CHANGE_DIRECTIONのところ、コピペ
else if (id == Field::Board::CHANGE_DIRECTION) {
    if (contains(board.position, circle.p) == GEOMETRY_IN) {
        // 矢印の向きに応じて速度を変化させる
        v += cdAccel*dt*(board.position[(4-board.dir)%4] - board.position[(7-board.dir)%4]) /  abs(board.position[0] - board.position[1]);
        // 単純に向きを変えるだけ
        //v = abs(v) * (board.position[(4-board.dir)%4] - board.position[(7-board.dir)%4]) /  abs(board.position[0] - board.position[1]);
    }
    
//ここで、　ひとつめの　v += の文は、走ってきたボールの速度に矢印方向の速度ベクトルを加えている（あってますか？）
//ふたつめの　v = の文は、ボールの速度を矢印の方向に変える
//後者は矢印マーカーを通過した時点で直前の方向は無視される。ー＞ゲームとしてはつまらない？
//逆に前者では、直前の速さが無視される。
    
v += abs(v) * (board.position[(4-board.dir)%4] - board.position[(7-board.dir)%4]) /  abs(board.position[0] - board.position[1]);

//にするのはどうでしょうか？混ぜただけですが


