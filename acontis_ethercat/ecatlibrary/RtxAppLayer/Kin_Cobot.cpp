#include "Kinematics.h"


Kin_Cobot::Kin_Cobot(int rInd) : KinSerial(rInd)
{
	_axisNum = 6;
	_gestureNum = 6;
	_solutionNum = 8;
	T01 = T16 = T45 = T56 = T14 = T12 = T23 = T34 = Identity_4x4;
}

eKin::e Kin_Cobot::IK()
{
	bool is_Out_Workspace[2] = {false, false};

	// get wrist position (relative to root frame)
	_wrist.x = IK_inMat(0,3) - DH->d[5] * IK_inMat(0,2);
	_wrist.y = IK_inMat(1,3) - DH->d[5] * IK_inMat(1,2);
	_wrist.z = IK_inMat(2,3) - DH->d[5] * IK_inMat(2,2);


	// ---------- calculate θ1 -------------	
	if(getTheta1Deg(Theta1_Deg_Array) == true)
	{
		return eKin::WRIST_SINGULAR; //11406 PMC Modified
	}
	

	// assign θ1 to _thetaSolution
	for(int i = 1; i <= 4; i++) {
		_thetaSolution.totalThetas[i]  [0] = changeAxisAngle(Theta1_Deg_Array(0) , 1); // change the θ1 to a suitable value
		_thetaSolution.totalThetas[i+4][0] = changeAxisAngle(Theta1_Deg_Array(1) , 1); // change the θ1 to a suitable value
	}

	
	// ---------- calculate θ5 & θ6 -------------
	for(int j = 1; j <= 8; j += 4) // j = 1 , 5
	{

		// calculate θ5 & θ6
		is_Out_Workspace[j/4] = getTheta56Deg(Theta5_Deg_Array , Theta6_Deg_Array , _thetaSolution.totalThetas[j][0]);

		if(is_Out_Workspace[j/4] == true) {
			_thetaSolution.ret[j+0] = eKin::OUT_OF_WORKSPACE;
			_thetaSolution.ret[j+1] = eKin::OUT_OF_WORKSPACE;
			_thetaSolution.ret[j+2] = eKin::OUT_OF_WORKSPACE;
			_thetaSolution.ret[j+3] = eKin::OUT_OF_WORKSPACE;
			continue;
		}

			
		// assign θ5 & θ6 to _thetaSolution
		for(int i = 0; i < 2; i++)
		{
			// θ5
			_thetaSolution.totalThetas[j+i]  [4] = changeAxisAngle(Theta5_Deg_Array(0) , 5); // 1,2 ; 5,6
			_thetaSolution.totalThetas[j+i+2][4] = changeAxisAngle(Theta5_Deg_Array(1) , 5); // 3,4 ; 7,8
				
			// θ6
			_thetaSolution.totalThetas[j+i]  [5] = changeAxisAngle(Theta6_Deg_Array(0) , 6); // 1,2 ; 5,6
			_thetaSolution.totalThetas[j+i+2][5] = changeAxisAngle(Theta6_Deg_Array(1) , 6); // 3,4 ; 7,8
		}
	}

	// check out of workspace 
	if(	is_Out_Workspace[0] == true && 
		is_Out_Workspace[0] == true		) {
		return eKin::OUT_OF_WORKSPACE;
	}
		

	// ------ calculate θ2 & θ3 & θ4 ------------
		
	for(int j = 1; j <= 8; j += 2) // j : 1 , 3 , 5 , 7
	{	
		getTheta234Deg( 
			Theta2_Deg_Array , 
			Theta3_Deg_Array , 
			Theta4_Deg_Array , 
			_thetaSolution.totalThetas[j][0] , //Theta1
			_thetaSolution.totalThetas[j][4] , //Theta5
			_thetaSolution.totalThetas[j][5]   //Theta6
			);
			
		// assign θ2 & θ3 & θ4 to _thetaSolution
		for(int i = 0; i < 2; i++)
		{
			_thetaSolution.totalThetas[j+i][1] = changeAxisAngle(Theta2_Deg_Array(i) , 2);
			_thetaSolution.totalThetas[j+i][2] = changeAxisAngle(Theta3_Deg_Array(i) , 3);
			_thetaSolution.totalThetas[j+i][3] = changeAxisAngle(Theta4_Deg_Array(i) , 4);
		}
		
	}
	
	//11406 PMC Add
	if(!isWristSingular()) return eKin::WRIST_SINGULAR;

	return chooseSolution(_thetaSolution.totalThetas); 
}

bool Kin_Cobot::isWristSingular()
{
	bool is_Writst_Singularity[4] = { false,false,false,false };

	for (int j = 1; j <= 8; j += 2) // j : 1 , 3 , 5 , 7
	{
		//det(J)=s3s5a2a3(c2a2+c23a3+s234d5)為0會有奇異點
		//1.當四六軸共軸，Wrist_Singular(s5=0)
		double value = abs(sind(_thetaSolution.totalThetas[j][4]));
		if(value < 0.05 ) 
		{
       		_thetaSolution.ret[j + 0] = eKin::WRIST_SINGULAR;
       		_thetaSolution.ret[j + 1] = eKin::WRIST_SINGULAR;
       		is_Writst_Singularity[j/2] = true;
		}

		//2.內部空間限制(c2a2+c23a3+s234d5=0,wrist投影)
		if(sqrt(pow(_wrist.x,2)+pow(_wrist.y,2)) < 150)
		{
			_thetaSolution.ret[j + 0] = eKin::WRIST_SINGULAR;
			_thetaSolution.ret[j + 1] = eKin::WRIST_SINGULAR;
			is_Writst_Singularity[j/2] = true;
		}

		//3.當J3趨近(s3=0)
		double value_3 = abs(_thetaSolution.totalThetas[j][2]);
		if(value_3 < 10 ) 
		{
       		_thetaSolution.ret[j + 0] = eKin::WRIST_SINGULAR;
       		_thetaSolution.ret[j + 1] = eKin::WRIST_SINGULAR;
       		is_Writst_Singularity[j/2] = true;
		}
	}

	// check in singularity	
	if (is_Writst_Singularity[0] == true &&
		is_Writst_Singularity[1] == true &&
		is_Writst_Singularity[2] == true &&
		is_Writst_Singularity[3] == true)
	{
		return false;
	}

	return true;
}
bool Kin_Cobot::getTheta1Deg(Vector2d &out_Theta1_Deg)
{
	double d2 , d4;

	d2 = DH->d[1];
	d4 = DH->d[3];

	double A, B , C , D1;

	A = d2 + d4 + _wrist.y;
	B = 2 * _wrist.x;
	C = d2 + d4 - _wrist.y;

	D1 = B*B - 4*A*C;

	D1 = fabs(D1) < 1e-5? 0:D1 ; // eliminate the Round-off error

	if(D1 < 0)  //11406 PMC Modified
	{
		return true; // out of workspace
	}
	else
	{
		out_Theta1_Deg(0) = 2 * atan2d( - B + sqrt(D1) , 2*A);
		out_Theta1_Deg(1) = 2 * atan2d( - B - sqrt(D1) , 2*A);

		return false;
	}
}
bool Kin_Cobot::getTheta56Deg(Vector2d &out_Theta5_Deg , Vector2d &out_Theta6_Deg , const double Theta1Deg)
{
	int i;
	double d2 , d4 , d6;

	d2 = DH->d[1];
	d4 = DH->d[3];
	d6 = DH->d[5];

	double r21 , r22 , py , c5 , s5;

	DH->getT(T01 , Theta1Deg , 1);

	T16 = T01.inverse() * IK_inMat; // T16 = T10 * T06

	py  = T16(1,3);
	r21 = T16(1,0);
	r22 = T16(1,1);

	c5 = ( py - (d2 + d4) ) / d6; //Modify by yunyu 20250331

	c5 = fabs( fabs(c5) - 1 ) < 1e-5? sign(c5)*1:c5 ; // eliminate the Round-off error
	
	//s5 = sqrt( 1 - pow(c5,2) );
	s5 = sqrt( r21*r21 + r22*r22);

	if(fabs(c5) > 1) return true; // out of workspace
	
	// ---- calculate θ5 ---------
	out_Theta5_Deg(0) = atan2d( s5 , c5);
	out_Theta5_Deg(1) = atan2d(-s5 , c5);

	// ---- calculate θ6 ---------
	if( fabs( r21 ) <= 1e-5 && fabs( r22 ) <= 1e-5 ) // sin(θ5) = 0 的情況
	{
		out_Theta6_Deg(0) = out_Theta6_Deg(1) = _refThetas[5];
	}
	else
	{		
		for(i = 0; i < 2; i++)
		{
			s5 = sind(out_Theta5_Deg(i));
			out_Theta6_Deg(i) =  atan2d( - r22/s5 , r21/s5 );
		}
	}
	return false;	
}
bool Kin_Cobot::getTheta234Deg(Vector2d &out_Theta2_Deg , Vector2d &out_Theta3_Deg , Vector2d &out_Theta4_Deg , double theta1 , double theta5 , double theta6)
{
	int i;
	double a2 , a3;

	a2 = DH->a[2];
	a3 = DH->a[3];

	double px , py , pz;
	DH->getT(T01 , theta1 , 1);
	DH->getT(T45 , theta5 , 5);
	DH->getT(T56 , theta6 , 6);

	T14 = T01.inverse() * IK_inMat * (T45*T56).inverse(); // T16 = T10 * T06 * T65 * T54

	px = T14(0,3);
	py = T14(1,3);
	pz = T14(2,3);


	// calculate θ3
	//---------------------------------------------------------------//
	double c3 , s3;
	
	//c3 = ( pow(px,2) + pow(pz,2) - pow(a2,2) - pow(a3,2) ) / (2*a2*a3);
	c3 = ( px*px + pz*pz - a2*a2 - a3*a3 ) / (2*a2*a3);

	c3 = fabs( fabs(c3) - 1 ) < 1e-5? sign(c3)*1:c3 ; // eliminate the Round-off error

	
	//s3 = sqrt( 1 - pow(c3,2) );
	s3 = sqrt( 1 - c3*c3 );  

	out_Theta3_Deg(0) = atan2d(   s3 , c3 );
	out_Theta3_Deg(1) = atan2d( - s3 , c3 );
	//---------------------------------------------------------------//

	// calculate θ2
	//---------------------------------------------//
	double u1 , v1 , r1 , u2 , v2 , r2 , A , c2 , s2;
	
	u1 = a2 + a3*c3;    r1 =   px;
	v2 = u1;			r2 = - pz;
	
	for(i = 0; i < 2; i++)
	{
		s3 = sind(out_Theta3_Deg(i));

		u2 = a3*s3;    v1 = - u2;

		A = u1*v2 - u2*v1;

		c2 = (r1*v2 - r2*v1) / A;
		s2 = (u1*r2 - u2*r1) / A;

		out_Theta2_Deg(i) = atan2d( s2 , c2 );
	}
	//---------------------------------------------//

	// calculate θ4
	//------------------------------------//
	double r21 , r11;

	for(i = 0; i < 2; i++)
	{
		DH->getT(T12 , out_Theta2_Deg(i) , 2);
		DH->getT(T23 , out_Theta3_Deg(i) , 3);

		T34 = (T12*T23).inverse() * T14; // T34 = T32 * T21 * T14

		r21 = T34(1,0);
		r11 = T34(0,0);

		out_Theta4_Deg(i) = atan2d( r21 , r11 );
	}
	//------------------------------------//    

	return true;
}
