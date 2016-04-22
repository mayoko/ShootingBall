#pragma once
#include "geometry.h"
#include "field.h"

// ���悻530*782�炵���ł�
class physSimu
{
public:
	Cir circle; // ���W����є��a
	Pt v; // ���x
	Real t; // ����
	physSimu();
	// �����V�~�����[�V�����֌W�Ȃ��ʒu,���x��ύX����
	void changeState(Real x, Real y, Real vx, Real vy);
	// �ǂɂԂ��邩��T�m���ĂԂ���Ȃ瑬�x�𔽓]������
	void wallDetect();
	// Field����ю��Ԃ�ǂݍ��񂾂Ƃ�,�{�[���̉^�����K�肷��
	void simulate(const Field& field, Real t);
	//�{�[���𔭎˂��邽�߂̏���
	void shootBall(const Field& field, const Real& startV);
	//�{�[���𔭎˂������ۂ�
	bool ballIsMoving;
	bool ballIsClear;
	bool ballIsOver;
	//���ɗ�������{�[������������
	void fallIntoHole(const Field&);
	//�S�[������
	void arrivedAtGoal(const Field&);
	// �{�[���̉^�����L�q����(�f�o�b�O�p)
	void print() const ;
};