#pragma once

#include <vector>
#include <sstream>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <iostream>

using namespace std;


class Robot;
class HG6Cobot         
{

public:
     HG6Cobot(Robot* rob);
    ~HG6Cobot();

	void		calTorq_HandGuide(int* out_torq);	//用於手拉		(重力 + 部分摩擦輔助)
	void		calTorq_Collision(int* out_torq);	//用於碰撞偵測	(重力 + 摩擦力 + 慣性)
	void		calTorq_Gravity(int* out_torq);		//用於Torque offset 	(重力)  //PMC Modified 11412//1229

private:

	Robot*		robot;
     //int        alarm,status,opmode;   //**工作狀態
     int        nAxis;                 //**URDF軸數
     double     urdf[10][10];         //**URDF定義 10:10個軸  10:xyzabc + w + center xyz
     double     mx[10][3][4];          //**matrix  operation
     double     my[10][3][4];          //**gravity & inertia
     double     xMass[8][8],yMass[8][8];
     //double     limit[8],gainM[8],gainG[8],ticks[8];
	 double     m_Inertia[6];  //PMC Modified 11411 //1114

     void reset();

     
     double *deg (double[],int);
	 void    deg (double[],double[],int);
	 double *rad (double[],int);
	 void    rad (double[],double[],int);

     int  doURDF   (int,double[]);
     void setURDF  (double[]);
     void doForward(double[]);
     void doInertia(double[]);
     void doGravity(double[]);

	 //----added by Grace, 1080917. 
	 void calGesture(double[], double[]);
	 void calTorq_G(double[]);	//將doInertia/doGravity 的結果(N.m) 轉成馬達Torq單位(0.1%)， 包含調參。 
	 void calTorq_F_HG(double[]);
	 void calTorq_F(double[]);
	 void calTorq_M(double[]); //PMC Modified 11411 //1114
	 void calTorq_C(double[]); //PMC Modified 11411 //1114

	 bool isReachAngleLimit(const double angle_Deg, const int axis_NO, double buffer_deg);  //11408 PMC modified
	 double AngleLimitTorque(double angle_Deg, double angle_Vel,const int axis_NO, double buffer_deg); //11408 PMC modified
	 //--------------------


     void    cross  (double[],double[],double[]);
     double  dot    (double[],double[]);
     void    norm   (double[]);


     void    matrix34(double[3][4],double p[]);
	 void    matrix34(double[3][4],double,double,double,double,double,double);
	 void    pose34(double[],double[3][4]);
	 double *abc(double[3][4]);
	 void    abc(double[],double[3][4]);
	 double *abc(         double[],double[],double[]);
	 void    abc(double[],double[],double[],double[]);
	 double *zyz(         double[3][4]);
	 void    zyz(double[],double[3][4]);
	 void    mul34(             double[3][4],double[3][4]);
	 void    mul34(double[3][4],double[3][4],double[3][4]);
	 double *mul34(             double[3][4],double[3]);
	 void    mul34(double[3],   double[3][4],double[3]);
	 void    inv34(double[3][4],double[3][4]);
	 void    set34(double[3][4],double[3][4]);

};


