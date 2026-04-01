#include "MathTool_Robot.h"
#include "Shm.h"

extern SHMData* shm;

//====== tool & base function
void removeBase(double out_Pose[6], const double Pose[6], const int baseIndex)
{
	double world2BaseT[4][4], world2ToolT[4][4], base2ToolT[4][4];

	transferPose2T(base2ToolT, Pose);							//  base2ToolT :  base T tool
	transferPose2T(world2BaseT, shm->bases[baseIndex].Pose);		// world2BaseT : world T base

	multiplyMatrix_4x4(world2ToolT, world2BaseT, base2ToolT); // world T tool = world T base * base T tool

	transferT2Pose(out_Pose, world2ToolT);
};
void attachBase(double out_Pose[6], const double Pose[6], const int baseIndex) 
{
	double world2BaseT[4][4], base2WorldT[4][4], world2ToolT[4][4], base2ToolT[4][4];

	transferPose2T(world2ToolT, Pose);							// world2ToolT : world T tool
	transferPose2T(world2BaseT, shm->bases[baseIndex].Pose);		// world2BaseT : world T base																																																							
	inverseHT(base2WorldT, world2BaseT);						// base2WorldT :  base T world

	multiplyMatrix_4x4(base2ToolT, base2WorldT, world2ToolT); // base T tool = base T world * world T tool

	transferT2Pose(out_Pose, base2ToolT);
};
void removeTool(double out_Pose[6], const double Pose[6], const int toolIndex)
{
	if(toolIndex <=0) {
		for(int i = 0; i<6; i++) out_Pose[i] = Pose[i];
		return;
	}

	double Root2FlangeT[4][4], Root2ToolT[4][4], Flange2ToolT[4][4], Tool2FlangeT[4][4];

	transferPose2T(Root2ToolT, Pose);
	transferPose2T(Flange2ToolT, shm->tools[toolIndex].Pose);	
	inverseHT(Tool2FlangeT, Flange2ToolT);
	multiplyMatrix_4x4(Root2FlangeT, Root2ToolT, Tool2FlangeT); // root T Flange = root T Tool * Tool T Flange
	transferT2Pose(out_Pose, Root2FlangeT);

};
void attachTool(double out_Pose[6], const double Pose[6], const int toolIndex) 
{
	double Root2FlangeT[4][4], Root2ToolT[4][4], Flange2ToolT[4][4];

	transferPose2T(Root2FlangeT, Pose);
	transferPose2T(Flange2ToolT, shm->tools[toolIndex].Pose);	
	multiplyMatrix_4x4(Root2ToolT, Root2FlangeT, Flange2ToolT); // root T Tool = root T Flange * Flange T Tool
	transferT2Pose(out_Pose, Root2ToolT);
};



Matrix4d tempT, tempInvT, temp_out_T, delta_T;

// 依照輸入 Pose 的座標系統(C.S.) , 繞其 X , Y , Z 軸旋轉
void moveABC(double out_Pose[MAX_REDUNDANCY], const double PoseAtBase[MAX_REDUNDANCY], double deltaTheta, int ind, eFrame::e frame )
{
	int i;
	double axis_vector[3] = {0};
	
	// 先複製一份完整的 gesture data (Pose) 到 out_Pose
	// 這樣 out_Pose 才帶有 redundancy 資訊
	for(i = 0; i < MAX_REDUNDANCY; i++)
		out_Pose[i] = PoseAtBase[i];

	tempT = tempInvT = temp_out_T = delta_T = Matrix4d::Identity(); //Identity_4x4;

	transferPose2T(tempT , PoseAtBase);

	tempInvT = tempT.inverse();

	switch(ind)
	{
		// A: 繞 Z 軸
		case 3:
			{
				if(frame == eFrame::BASE) {	// base frame
					axis_vector[0] = tempInvT(0,2);
					axis_vector[1] = tempInvT(1,2);
					axis_vector[2] = tempInvT(2,2);
				}
				else {										// tool frame
					axis_vector[0] = 0;
					axis_vector[1] = 0;
					axis_vector[2] = 1;
				}
			}
			break;

		// B: 繞 Y 軸
		case 4:
			{
				if(frame == eFrame::BASE) {	// base frame
					axis_vector[0] = tempInvT(0,1);
					axis_vector[1] = tempInvT(1,1);
					axis_vector[2] = tempInvT(2,1);
				}
				else {										// tool frame
					axis_vector[0] = 0;
					axis_vector[1] = 1;
					axis_vector[2] = 0;
				}
			}
			break;

		// C: 繞 X 軸
		case 5:
			{
				if(frame == eFrame::BASE) {	// base frame
					axis_vector[0] = tempInvT(0,0);
					axis_vector[1] = tempInvT(1,0);
					axis_vector[2] = tempInvT(2,0);
				}
				else {										// tool frame
					axis_vector[0] = 1;
					axis_vector[1] = 0;
					axis_vector[2] = 0;
				}
			}
			break;
	}

	AxisAngleMethod_T(delta_T , axis_vector , deltaTheta); // get ΔT

	temp_out_T = tempT * delta_T; // T' = T x ΔT

	transferT2Pose(out_Pose , temp_out_T);
}
// 依照輸入 Pose 的座標系統(C.S.) , 沿其 X , Y , Z 軸移動
void moveXYZ(double out_Pose[MAX_REDUNDANCY], const double PoseAtBase[MAX_REDUNDANCY], double deltaDist, int ind, eFrame::e frame)
{
	tempT = temp_out_T = delta_T = Matrix4d::Identity(); //Identity_4x4;

	transferPose2T(tempT , PoseAtBase); // tempT : base T tool

	switch(ind)
	{
		// X
		case 0:
			{
				if(frame == eFrame::BASE) {	// base frame
					tempT(0,3) += deltaDist;
				}
				else {									// tool frame
					delta_T(0,3) = deltaDist;
				}
			}
			break;

		// Y
		case 1:
			{
				if(frame == eFrame::BASE) {	// base frame
					tempT(1,3) += deltaDist;
				}
				else {									// tool frame
					delta_T(1,3) = deltaDist;
				}
			}
			break;

		// Z
		case 2:
			{
				if(frame == eFrame::BASE) {	// base frame
					tempT(2,3) += deltaDist;
				}
				else {									// tool frame
					delta_T(2,3) = deltaDist;
				}
			}
			break;
	}

	temp_out_T = tempT * delta_T; // T' = T x ΔT

	transferT2Pose(out_Pose , temp_out_T);
}



//======= Circle Class for Circle-Path-Planning. ===========

Circle::Circle()
{
	Zero_3 = Vector3d::Zero(3);
}
Circle::Circle(const CircleData& cData)
{
	_cData = cData;
	Zero_3 = Vector3d::Zero(3);
}
// 三點求 圓心, 半徑, 角度, 圓平面法向量, 起始方向
void Circle::calculateBasicParameter(double startPose[6], double midPose[6], double endPose[6], double thetaUser)
{
	int i;
	double a, b, c, A, B, C, s, Area;
	double sin2A, sin2B, sin2C;
	Vector3d vector_MS, vector_SE, vector_EM;
	Vector3d u_MS, u_SE, u_ME, u_SM, u_ES, u_EM;
	
	for(i = 0; i < 3; i++)
	{
		vector_MS(i) = startPose[i] -   midPose[i]; // vec_MS : M (Mid)   → S (Start)
		vector_SE(i) =   endPose[i] - startPose[i]; // vec_SE : S (Start) → E (End)
		vector_EM(i) =   midPose[i] -   endPose[i]; // vec_EM : E (End)   → M (Mid)
	}

	u_MS = vector_MS.normalized();	 u_SM = - u_MS;
	u_SE = vector_SE.normalized();	 u_ES = - u_SE;
	u_EM = vector_EM.normalized();	 u_ME = - u_EM;


	a = vector_MS.norm(); // Length_MS
	b = vector_SE.norm(); // Length_SE
	c = vector_EM.norm(); // Length_EM

	// 求圓半徑
	//-----------------------------//
	// Heron's formula
	s = (a+b+c)/2;
	Area = sqrt(s*(s-a)*(s-b)*(s-c));

	_cData.R = a*b*c / (4*Area); // R = abc / 4A
	//-----------------------------//

	// 求圓心
	//----------------------------------------------------------------------------------------------------------//
	A = acosd( u_SM.dot(u_SE) );
	B = acosd( u_MS.dot(u_ME) );
	C = acosd( u_EM.dot(u_ES) );

	sin2A = sind(2*A);
	sin2B = sind(2*B);
	sin2C = sind(2*C);

	for(i = 0; i < 3; i++)
	{
		_cData.center[i] = ( sin2A*startPose[i] + sin2B*midPose[i] + sin2C*endPose[i] ) / (sin2A + sin2B + sin2C);
	}
	//----------------------------------------------------------------------------------------------------------//


		double temptheta, arc_theta;
		Vector3d CS_Vector, CM_Vector, CE_Vector;
		Vector3d u_CS, u_CM, u_CE;
			

		for(i = 0; i < 3; i++)
		{
			CS_Vector(i) = startPose[i] - _cData.center[i]; // C (center) → S (start)
			CM_Vector(i) =   midPose[i] - _cData.center[i]; // C (center) → M (mid)
			CE_Vector(i) =   endPose[i] - _cData.center[i]; // C (center) → E (end)
		}

		// u : unit vecotr
		u_CS = CS_Vector.normalized(); 
		u_CM = CM_Vector.normalized(); 
		u_CE = CE_Vector.normalized();

		_cData.uStart = u_CS;

		// 利用向量內積求夾角
		arc_theta = acosd( u_MS.dot(u_ME) ); // θr = acos( u_MS．u_ME ) (．:dot product)
		temptheta = acosd( u_CS.dot(u_CE) ); 

		// 利用圓弧角(θr)來判斷 θ_circle = θ or 360-θ 和 法向量 (u_normal)
		if(arc_theta >= 90) // 當圓形夾角 θ_circle ≦ 180 時, 利用 u_CS 與 u_CM 求法向量 (u_normal)
		{
			_cData.uNormal  = ( u_CS.cross(u_CM) ).normalized(); // u_normal = u_CS X u_CM (X:cross product) 
			_cData.th_Final = temptheta;
		}
		else // 當圓形夾角 θ_circle > 180 時, 利用 u_CS 與 u_CE 求法向量 (u_normal)
		{
			_cData.uNormal  = -1 * ( u_CS.cross(u_CE) ).normalized(); // u_normal = u_CS X u_CE (X:cross product) 
			_cData.th_Final = (360 - temptheta);
		}

		
		// 若使用者有指定角度，採用該角度。
		if(thetaUser >= 0)
			_cData.th_Final = thetaUser;
}
void Circle::getCData(CircleData& out_cData)
{
	out_cData = _cData;
}
int Circle::theta2XYZ(double out_Pose[6], const double deltaTheta)
{
	// 添加轉 u_normal 的判斷
	AxisAngleMethod_R(_deltaR, _cData.uNormal, deltaTheta); // ΔR : 繞 u_normal 軸轉 Δθ 角的對應旋轉矩陣

	_uNow = _deltaR* _cData.uStart;

	for(int i = 0; i < 3; i++)
		out_Pose[i] =_cData.center[i] + _uNow(i)* _cData.R;

	return 1;
}