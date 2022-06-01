#pragma once
#include<iostream>
#include<opencv2\opencv.hpp>
#include"KF.h"
using namespace std;
using namespace cv;

class Kalman {
public:
		struct Kalman_InitDef {
		double Z_k[100];//����ֵ ��solvepnpʵ��
		double E_mea;//�������
		double X_k[100];//����ֵ
		double K_k[100];//����ֵ
		double E_east[100];//�������
	};
	Kalman_InitDef KF;
	void Generating_Random_Measurement(double  distanceW) {
		for (int i = 0; i < 100; i++) {
			KF.Z_k[i] = distanceW;//���������ֵ
		}
	}
	float Kalman_Gain_Calculaitioin(double E_east, char E_mea) {
		double result;
		result = E_east / (E_east + E_mea);
		return result;
	}
	void Kalman_X_k_Calculation() {
		double kalmanGain;
		for (int i = 0; i < 100; i++) {
			kalmanGain = Kalman_Gain_Calculaitioin(KF.E_east[i - 1], KF.E_mea);
			KF.X_k[i] = KF.X_k[i - 1] + kalmanGain * (KF.Z_k[i] - KF.Z_k[i - 1]);
			KF.E_east[i] = ((double)1 - kalmanGain) * KF.E_east[i];
			//cout << KF.X_k[i] << '\t' << KF.E_east[i] << endl;
		}
	}
	void KF_Init() {
		KF.X_k[0] = 1200;
		KF.E_east[0] = 20;
		KF.E_mea = 5;
	}
};
