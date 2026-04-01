#include "HG6Cobot.h"
#include "HelpFunctions.h"
#include "Robot.h"
#include "Motor.h"
#include "Def.h"
#include "Shm.h"
#include "ErrorHandler.h"
#include "Debug.h"

extern SHMData* shm;

HG6Cobot::HG6Cobot(Robot* rob)
{
    this->robot = rob;
    this->nAxis = rob->motorNum;
    this->reset();
}
HG6Cobot::~HG6Cobot()
{
}

bool HG6Cobot::isReachAngleLimit(const double angle_Deg, const int axis_NO, double buffer_deg) // 角度極限 +/- buffer_deg
{
    double* axisPositiveLimit = shm->robots[robot->rId].spec.DH.axisPositiveLimit;
    double* axisNegativeLimit = shm->robots[robot->rId].spec.DH.axisNegativeLimit;

    if (angle_Deg > axisPositiveLimit[axis_NO] - buffer_deg) return true;
    if (angle_Deg < axisNegativeLimit[axis_NO] + buffer_deg) return true;

    return false;
}

double HG6Cobot::AngleLimitTorque(double angle_Deg, double angle_Vel, const int axis_NO, double buffer_deg)  //手拉越靠近角度極限，阻力越大  //PMC Modified 11501//0123
{
    int gearratio_direction[6] = { -1, -1, 1, -1 ,-1 ,-1 }; //PMC Modified 11501 //0115
    double corrective_torque = 0.0;
    double* axisPositiveLimit = shm->robots[robot->rId].spec.DH.axisPositiveLimit;
    double* axisNegativeLimit = shm->robots[robot->rId].spec.DH.axisNegativeLimit;

    const double K_stiffness = 50.0; // K 從0往上加，調整到適合的block
    const double D_damping = 5.0; // set K = 0, 調整D值讓超過角度極限後有阻尼感

    if (angle_Deg > axisPositiveLimit[axis_NO] - buffer_deg) {
        double penetration = angle_Deg - (axisPositiveLimit[axis_NO] - buffer_deg);
        corrective_torque = -K_stiffness * penetration * gearratio_direction[axis_NO] - D_damping * angle_Vel * gearratio_direction[axis_NO];   //PMC Modified 11501//0123
    }
    else if (angle_Deg < axisNegativeLimit[axis_NO] + buffer_deg) {
        double penetration = angle_Deg - (axisNegativeLimit[axis_NO] + buffer_deg);
        corrective_torque = -K_stiffness * penetration * gearratio_direction[axis_NO] - D_damping * angle_Vel * gearratio_direction[axis_NO];   //PMC Modified 11501//0123
    }

    return corrective_torque;
}

void HG6Cobot::calTorq_HandGuide(int* out_torq)	//用於手拉		(重力 + 部分摩擦輔助)
{
    double m[6], f[6], g[6];
    double* aidRatio = shm->HGParams[robot->rId].aidRatio;
    double* axisVelNow = robot->axisVelNow;
    double* axisDegNow = robot->axisDegNow;

    reset();
    calGesture(g, m);
    calTorq_G(g);
    calTorq_F_HG(f);

    for (int i = 0; i < nAxis; i++) {
        out_torq[i] = (int)(g[i] + f[i] * aidRatio[i]);

        if (isReachAngleLimit(axisDegNow[i], i, 5.0)) { //手拉卡角度極限 //5.0 角度緩衝區 // 跟isReachAngleLimit的buffer_deg要一致  //11408 PMC modified
            out_torq[i] = (int)(g[i] + f[i] * aidRatio[i] + AngleLimitTorque(axisDegNow[i], axisVelNow[i], i, 5.0));
        }

        if (isReachAngleLimit(axisDegNow[i], i, -5.0))  //安全機制避免暴衝  //11408 PMC modified
            out_torq[i] = (int)(g[i] + f[i] * aidRatio[i]);
    }
}

void HG6Cobot::calTorq_Collision(int* out_torq)	//用於碰撞偵測	(重力 + 摩擦力) //1114
{
    double c[6], m[6], f[6], g[6];
    reset();
    calGesture(g, m);
    calTorq_G(g);
    calTorq_F(f);
    calTorq_M(m);
    calTorq_C(c);

    for (int i = 0; i < nAxis; i++)
    {
        out_torq[i] = (int)(g[i] + f[i] + m[i] + c[i]);
    }
    static int print_count[MAX_ROBOT_NUM] = { 0 };
    if (shm->isScriptRunning) {
        if (print_count[robot->rId]++ > 0) {
            print_count[robot->rId] = 0;
            Debug::writeln(3, "Robot[%d], gG,%f, %f, %f, %f, %f, %f, fG,%f, %f, %f, %f, %f, %f, mG,%f, %f, %f, %f, %f, %f, cG,%f, %f, %f, %f, %f, %f  ",
                robot->rId,
                g[0], g[1], g[2], g[3], g[4], g[5],
                f[0], f[1], f[2], f[3], f[4], f[5],
                m[0], m[1], m[2], m[3], m[4], m[5],
                c[0], c[1], c[2], c[3], c[4], c[5]
            );
        }
    }
}

void HG6Cobot::calTorq_Gravity(int* out_torq) {	//用於Torque offset 	(重力)  //PMC Modified 11412//1229
    double m[6], g[6];
    reset();
    calGesture(g, m);
    calTorq_G(g);

    for (int i = 0; i < nAxis; i++)
    {
        out_torq[i] = (int)(g[i]);
    }
}

void HG6Cobot::calGesture(double g[], double m[]) //建立機構參數
{
    double axis[6];
    double pose[6];
    double* thetaInit = robot->getThetaInit();
    for (int i = 0; i < nAxis; i++)
        axis[i] = robot->axisDegNow[i] - thetaInit[i];

    setURDF(axis);
    doForward(pose);
    doInertia(m);
    doGravity(g);
}

void HG6Cobot::calTorq_G(double g[]) // 重力 g
{
    double* g_shift = shm->HGParams[robot->rId].g_shift;
    double* g_ratio = shm->HGParams[robot->rId].g_ratio;
    double* ratedTorq = shm->HGParams[robot->rId].ratedTorq;
    double getGearRatio[6] = { 81, 81, 81, 121, 121, 121 }; //PMC Modified 11501 //0115

    for (int i = 0; i < nAxis; i++)
        g[i] = (-1 * g[i] * 1000 * 9.80665 / (getGearRatio[i] / 0.7) / ratedTorq[i]) * g_ratio[i] + g_shift[i]; // 重力力矩 * 千分比 * 重力常數 / (減速比 / 減速機效率) / 額定扭矩
}

void HG6Cobot::calTorq_F_HG(double f[]) // 摩擦 f
{
    double* KV = shm->HGParams[robot->rId].KV;
    double* KV0 = shm->HGParams[robot->rId].KV0;
    double* axisVelNow = robot->axisVelNow;

    for (int i = 0; i < nAxis; i++) {
        double vel_low = (i == 0 || i == 4 || i == 5) ? 0.15 : 0.3;
        double vel = axisVelNow[i];

        if (abs(axisVelNow[i]) < vel_low) { // 沒有速度
            f[i] = 0;
        }
        else {
            // 限制最大速度
            if (axisVelNow[i] > 50.0)       axisVelNow[i] = 50.0;
            else if (axisVelNow[i] < -50.0) axisVelNow[i] = -50.0;

            if (abs(axisVelNow[i]) < 15.0) {  // 手拉離開緩停
                double t = (abs(axisVelNow[i]) - vel_low) / (15.0 - vel_low);
                f[i] = sqrt(sqrt(t)) * (KV0[i] * sign(axisVelNow[i]) + KV[i] * axisVelNow[i]);
            }
            else {
                f[i] = KV0[i] * sign(axisVelNow[i]) + KV[i] * axisVelNow[i];
            }
        }
    }
}

void HG6Cobot::calTorq_F(double f[]) // 摩擦 f
{
    double* KV = shm->HGParams[robot->rId].KV;
    double* KV0 = shm->HGParams[robot->rId].KV0;
    double* axisVelNow = robot->axisVelNow;

    for (int i = 0; i < nAxis; i++) {
        if (abs(axisVelNow[i]) < 0.5) //沒有速度
            f[i] = 0;
        else
            f[i] = (KV0[i] * sign(axisVelNow[i]) + KV[i] * axisVelNow[i]);
    }
}

void HG6Cobot::calTorq_M(double m[]) // 慣性m //PMC Modified 11411 //1118
{
    double* KA = shm->HGParams[robot->rId].KA;
    double* KA0 = shm->HGParams[robot->rId].KA0;
    double* axisAccNow = robot->axisAccNow;
    doInertia(m_Inertia);
    for (int i = 0; i < nAxis; i++) {
        if (abs(axisAccNow[i]) < 0.5)
            m[i] = 0;
        else
            m[i] = axisAccNow[i] * (m_Inertia[i] * KA[i]) + KA0[i] * (axisAccNow[i] / 150); //PMC Modified 11411 //1121
    }
}

void HG6Cobot::calTorq_C(double c[]) // 科氏力c //PMC Modified 11411 //1118
{
    double* KC = shm->HGParams[robot->rId].KC;
    double* axisVelNow = robot->axisVelNow;

    for (int i = 0; i < nAxis; i++) {
        if (abs(axisVelNow[i]) < 0.5)
            c[i] = 0;
        else
            c[i] = KC[i] * abs(axisVelNow[i]) * axisVelNow[i];
    }
}

void HG6Cobot::reset()
{
    double pi = atan(1.0) * 4;
    double toRad = pi / 180.;
    double toDeg = 180. / pi;
    double payload = shm->HGParams[robot->rId].payload;
    double centerX = shm->HGParams[robot->rId].centerX;//PMC Modified 11501 //0127
    double centerY = shm->HGParams[robot->rId].centerY;
    double centerZ = shm->HGParams[robot->rId].centerZ;
    RobotSpec* spec = &(shm->robots[robot->rId].spec); //PMC Modified 11412 //1209
    /*
    //Wistron 7
    //x (mm)			    //y					 //z					    //Rx				//Ry				//Rz

    urdf[1][0] = 0;	    urdf[1][1] = 0;		 urdf[1][2] = 164.0;        urdf[1][3] = 0;		urdf[1][4] = 0;		urdf[1][5] = 1;
    urdf[2][0] = 0;	    urdf[2][1] = 178.2;  urdf[2][2] = 0;	        urdf[2][3] = 0;		urdf[2][4] = 1;		urdf[2][5] = 0;
    urdf[3][0] = 0;	    urdf[3][1] = 0;		 urdf[3][2] = 443.5;	    urdf[3][3] = 0;		urdf[3][4] = 1;		urdf[3][5] = 0;
    urdf[4][0] = 0;	    urdf[4][1] = -121.4; urdf[4][2] = 392.0;	    urdf[4][3] = 0;		urdf[4][4] = 1;		urdf[4][5] = 0;
    urdf[5][0] = 0;	    urdf[5][1] = 121.4;	 urdf[5][2] = 0;	        urdf[5][3] = 0;		urdf[5][4] = 0;		urdf[5][5] = 1;
    urdf[6][0] = 0;		urdf[6][1] = 89.5;  	 urdf[6][2] = 103.3;	    urdf[6][3] = 0;		urdf[6][4] = 1;		urdf[6][5] = 0;

    //weight (kg)			         //center-x(mm)		     //center-y(mm)																		//center-z(mm)
    urdf[1][6] = 0;			     urdf[1][7] = 0;		 urdf[1][8] = 0;																	urdf[1][9] = 0;
    urdf[2][6] = 10.304;			 urdf[2][7] = 0.111;     urdf[2][8] = 5.112;																urdf[2][9] = -19.015;
    urdf[3][6] = 17.092;			 urdf[3][7] = -0.035;    urdf[3][8] = -14.543;																urdf[3][9] = 155.539;
    urdf[4][6] = 3.417;		   	 urdf[4][7] = -0.125;    urdf[4][8] = -113.785;	 															urdf[4][9] = 288.676;
    urdf[5][6] = 2.304;			 urdf[5][7] = 0.189;     urdf[5][8] = 114.624;																urdf[5][9] = 9.007;
    urdf[6][6] = 2.463 + payload;	 urdf[6][7] = -0.169;    urdf[6][8] = (2.463 * 17.015 + payload * (centerZ + urdf[6][1])) / (2.463 + payload);	urdf[6][9] = 100.193;
    */

    //PMC Modified 11412 //1209
    for (int i = 0;i <= nAxis;i++)
    {
        urdf[i + 1][0] = spec->URDF.x[i];
        urdf[i + 1][1] = spec->URDF.y[i];
        urdf[i + 1][2] = spec->URDF.z[i];
        urdf[i + 1][3] = spec->URDF.Rx[i];
        urdf[i + 1][4] = spec->URDF.Ry[i];
        urdf[i + 1][5] = spec->URDF.Rz[i];
        urdf[i + 1][6] = spec->URDF.weight[i];
        urdf[i + 1][7] = spec->URDF.center_x[i];
        urdf[i + 1][8] = spec->URDF.center_y[i];
        urdf[i + 1][9] = spec->URDF.center_z[i];
    }

    if (nAxis >= 6) {
        double armMass = urdf[6][6];
        urdf[6][6] = armMass + payload;
        urdf[6][7] = (armMass * urdf[6][7] + payload * (centerX + urdf[6][0])) / (urdf[6][6]);
        urdf[6][8] = (armMass * urdf[6][8] + payload * (centerZ + urdf[6][1])) / (urdf[6][6]);
        urdf[6][9] = (armMass * urdf[6][9] + payload * (-centerY + urdf[6][2])) / (urdf[6][6]);
    }


    //mm -> m
    for (int j = 1; j <= 6; j++) {
        urdf[j][0] /= 1000;
        urdf[j][1] /= 1000;
        urdf[j][2] /= 1000;
        urdf[j][7] /= 1000;
        urdf[j][8] /= 1000;
        urdf[j][9] /= 1000;
    }
    double* RefRoot = robot->getRefRoot();
    double base[6] = { 0,0,0,RefRoot[5],RefRoot[4],RefRoot[3] };
    matrix34(my[0], base);
}

//**** radius轉degree *****
double* HG6Cobot::deg(double q[], int n)
{
    static double d[8] = { 0,0,0,0,0,0,0,0 }; deg(d, q, n); return(d);
}
void    HG6Cobot::deg(double d[], double q[], int n)
{
    for (int i = 0; i < n; i++) d[i] = q[i] * RAD2DEG;
}
//**** degree轉radius *****
double* HG6Cobot::rad(double d[], int n)
{
    static double q[8] = { 0,0,0,0,0,0,0,0 }; rad(q, d, n); return(q);
}
void    HG6Cobot::rad(double q[], double d[], int n)
{
    for (int i = 0; i < n; i++) q[i] = d[i] * DEG2RAD;
}

//***** 計算機械手座標 *****  
int  HG6Cobot::doURDF(int n, double v[])
{
    int i; double* p;
    if (n <= 10)              //**cmd=10:設置base模型
    {
        matrix34(my[0], v);        return(0);
    }
    if (n == 19)              //**cmd=19:計算forward函數
    {
        setURDF(v); doForward(v); return(6);
    }
    if (n == 20)              //**cmd=20:取得inertia數組
    {
        doInertia(v);             return(nAxis);
    }
    if (n == 21)              //**cmd=21:取得gravity數組
    {
        doGravity(v);             return(nAxis);
    }
    nAxis = n - 10;             //**cmd<19:設置URDF模型 // n:11~18,-> nAxis = 1~8
    for (i = 0, p = urdf[nAxis]; i < 8; i++) p[i] = v[i];
    return(0);
}
//***** 設置各軸的M[3*4] *****
void HG6Cobot::setURDF(double v[])
{
    int i, k; double* p, q[6] = { 0,0,0,0,0,0 };
    for (i = 1; i <= nAxis; i++)
    {
        p = urdf[i];
        for (k = 0; k < 3; k++) q[k] = p[k];
        for (; k < 6; k++) q[k] = p[k] * v[i - 1];
        matrix34(my[i], q);
    }
}
//***** 計算機械手座標 *****
void HG6Cobot::doForward(double v[])
{
    int i; double m0[3][4], m1[3][4], m2[3][4];
    for (i = nAxis; i > 0; i--)
    {
        if (i == nAxis) { set34(m0, my[i]); continue; }
        set34(m2, m0);  set34(m1, my[i]); mul34(m0, m1, m2);
    }
    pose34(v, m0);
}
//***** 計算機械手慣量 *****
void HG6Cobot::doInertia(double v[])
{
    int i, k; double d, dx, dy, x, y, z, m0[3][4], m1[3][4], m2[3][4];
    for (i = nAxis; i > 0; i--)
    {
        for (k = i; k > 0; k--)
        {
            if (k == i)               //**取得質心位置
            {
                set34(m0, my[i]);
                m0[0][3] = urdf[i][7]; //x分量
                m0[1][3] = urdf[i][8]; //y分量
                m0[2][3] = urdf[i][9]; //z分量
                continue;
            }                       //**計算旋轉平面分量
            x = m0[0][3]; y = m0[1][3]; z = m0[2][3];
            if (abs(urdf[k][3]) > 0.5) { dx = y; dy = z; } //**旋轉軸=X軸
            if (abs(urdf[k][4]) > 0.5) { dx = z; dy = x; } //**旋轉軸=Y軸
            if (abs(urdf[k][5]) > 0.5) { dx = x; dy = y; } //**旋轉軸=Z軸

            xMass[k][i] = dx; yMass[k][i] = dy;
            set34(m2, m0);  set34(m1, my[k]);  mul34(m0, m1, m2);
        }
    }
    for (i = 1; i <= nAxis; i++)          //**計算各軸慣量
    {
        for (k = i + 1, d = 0;k <= nAxis;k++) //**累計各節慣量
        {
            x = xMass[i][k]; y = yMass[i][k];
            d += urdf[k][6] * sqrt(x * x + y * y);
        }    v[i - 1] = d;               //**回覆各軸慣量
    }


}
//***** 計算重力向量 *****
void HG6Cobot::doGravity(double v[])
{
    // This function returns "unit-gravity" torque in (kg*m). The caller scales by 1000*9.80665 etc.
    // Keep the original (legacy) gravity model for J1~J5 so existing tuning stays close,
    // but add the missing payload coupling due to J6 rotation, and compute a correct J6 torque.
    //
    // Key idea:
    //   legacy model effectively treats the payload CoM as NOT rotating with J6 (hence no coupling).
    //   We compute payload torque with J6 rotation (correct) and without J6 rotation (legacy-equivalent),
    //   then add the delta to the legacy torques for upstream joints.

    // ----------------------------
    // 1) Legacy gravity (as-is) for joints 1..(nAxis-1)
    // ----------------------------
    double v_legacy[8] = { 0,0,0,0,0,0,0,0 };

    int i, k;
    double x, y, z, d, dx, dy;
    double m0[3][4], m1[3][4], m2[3][4];

    for (i = 0; i < nAxis; i++)
    {
        if (i == 0) { set34(m0, my[i]); continue; }

        set34(m1, m0);
        set34(m2, my[i]);
        mul34(m0, m1, m2);

        inv34(m2, m0); //**取得Z軸(x,y,z)
        x = m2[0][2]; y = m2[1][2]; z = m2[2][2];

        // joint axis selector (legacy)
        if (urdf[i][3] > 0.5) { dx = y;  dy = z; } //**旋轉軸=X軸
        if (urdf[i][4] > 0.5) { dx = z;  dy = x; } //**旋轉軸=Y軸
        if (urdf[i][5] > 0.5) { dx = x;  dy = y; } //**旋轉軸=Z軸
        if (urdf[i][3] < -0.5) { dx = -y; dy = -z; } //**旋轉軸=X軸
        if (urdf[i][4] < -0.5) { dx = -z; dy = -x; } //**旋轉軸=Y軸
        if (urdf[i][5] < -0.5) { dx = -x; dy = -y; } //**旋轉軸=Z軸

        // legacy sums only downstream links (k=i+1..nAxis) //**累計各節重力
        for (k = i + 1, d = 0; k <= nAxis; k++)
            d += urdf[k][6] * (dx * yMass[i][k] - dy * xMass[i][k]);

        v_legacy[i - 1] = d;
    }

    // legacy behavior: J6 torque is always 0
    if (nAxis >= 6) v_legacy[nAxis - 1] = 0.0;

    // ----------------------------
    // 2) Build kinematics for payload coupling (uses real joint angles)
    // ----------------------------
    const double payload = shm->HGParams[robot->rId].payload;
    const double centerX = shm->HGParams[robot->rId].centerX;
    const double centerY = shm->HGParams[robot->rId].centerY;
    const double centerZ = shm->HGParams[robot->rId].centerZ;

    // If no payload mass, just return legacy output (plus fixed J6=0)
    if (payload <= 1e-9)
    {
        for (int j = 0; j < nAxis; j++) v[j] = v_legacy[j];
        return;
    }

    // Current joint angles (deg) used by setURDF() in calGesture()
    double axis_deg[8] = { 0,0,0,0,0,0,0,0 };
    double* thetaInit = robot->getThetaInit();
    for (int j = 0; j < nAxis; j++) axis_deg[j] = robot->axisDegNow[j] - thetaInit[j];

    // Payload CoM offset expressed in the flange/tool frame (mm) -> (m)
    // Keep the same axis mapping convention used elsewhere:
    //   payload_x = centerX
    //   payload_y = centerZ
    //   payload_z = -centerY
    const double c_local[3] = { centerX / 1000.0, centerZ / 1000.0, -centerY / 1000.0 };

    // Build joint origins & axes in world for the ACTUAL pose, and also compute:
    //   T_base_flange_actual (includes J6 rotation)
    //   T_base_flange_norot  (same but J6 angle forced to 0)
    double p_joint[8][3];    // 1..6 (axis point, actual pose)
    double p_joint_nr[8][3]; // 1..6 (axis point, J6-no-rot pose)
    double axis_w[8][3];     // 1..6

    double T_act[3][4], T_nr[3][4], A[3][4], Ttmp[3][4];

    // start from base/root transform already prepared in reset(): my[0]
    set34(T_act, my[0]);
    set34(T_nr, my[0]);

    for (i = 1; i <= nAxis; i++)
    {
        // IMPORTANT:
        // The revolute joint axis passes through the joint "origin" defined by the fixed
        // translation (urdf[i][0..2]) in the *parent* frame, then rotates about Rx/Ry/Rz.
        // The previous v2 code used the parent frame origin as the axis point, which can
        // incorrectly produce a non-zero J6 torque even when centerX/Y/Z = 0.
        //
        // Compute a point on the joint axis in world:
        //   p_joint = p_parent + R_parent * t_fixed
        const double tx = urdf[i][0];
        const double ty = urdf[i][1];
        const double tz = urdf[i][2];
        p_joint[i][0] = T_act[0][3] + T_act[0][0] * tx + T_act[0][1] * ty + T_act[0][2] * tz;
        p_joint[i][1] = T_act[1][3] + T_act[1][0] * tx + T_act[1][1] * ty + T_act[1][2] * tz;
        p_joint[i][2] = T_act[2][3] + T_act[2][0] * tx + T_act[2][1] * ty + T_act[2][2] * tz;

        // Same axis point but for the "J6-no-rotation" chain (used when computing tau_nr)
        p_joint_nr[i][0] = T_nr[0][3] + T_nr[0][0] * tx + T_nr[0][1] * ty + T_nr[0][2] * tz;
        p_joint_nr[i][1] = T_nr[1][3] + T_nr[1][0] * tx + T_nr[1][1] * ty + T_nr[1][2] * tz;
        p_joint_nr[i][2] = T_nr[2][3] + T_nr[2][0] * tx + T_nr[2][1] * ty + T_nr[2][2] * tz;

        // joint axis local selector from URDF (Rx/Ry/Rz as +/-1)
        double ax_local[3] = { 0.0, 0.0, 0.0 };
        if (urdf[i][3] > 0.5) ax_local[0] = 1.0;
        else if (urdf[i][3] < -0.5) ax_local[0] = -1.0;
        else if (urdf[i][4] > 0.5) ax_local[1] = 1.0;
        else if (urdf[i][4] < -0.5) ax_local[1] = -1.0;
        else if (urdf[i][5] > 0.5) ax_local[2] = 1.0;
        else if (urdf[i][5] < -0.5) ax_local[2] = -1.0;
        else                        ax_local[2] = 1.0;

        // axis in world = R_parent * ax_local (using T_act rotation)
        axis_w[i][0] = T_act[0][0] * ax_local[0] + T_act[0][1] * ax_local[1] + T_act[0][2] * ax_local[2];
        axis_w[i][1] = T_act[1][0] * ax_local[0] + T_act[1][1] * ax_local[1] + T_act[1][2] * ax_local[2];
        axis_w[i][2] = T_act[2][0] * ax_local[0] + T_act[2][1] * ax_local[1] + T_act[2][2] * ax_local[2];

        // joint transform (actual)
        const double ang_act = axis_deg[i - 1];
        matrix34(A,
            urdf[i][0], urdf[i][1], urdf[i][2],
            urdf[i][3] * ang_act, urdf[i][4] * ang_act, urdf[i][5] * ang_act);
        set34(Ttmp, T_act);
        mul34(T_act, Ttmp, A);

        // joint transform (no-rot for J6 only)
        const double ang_nr = (i == nAxis ? 0.0 : ang_act);
        matrix34(A,
            urdf[i][0], urdf[i][1], urdf[i][2],
            urdf[i][3] * ang_nr, urdf[i][4] * ang_nr, urdf[i][5] * ang_nr);
        set34(Ttmp, T_nr);
        mul34(T_nr, Ttmp, A);
    }

    // Payload CoM in world:
    // p = p_flange + R_flange * c_local
    double p_payload_act[3] = {
        T_act[0][3] + T_act[0][0] * c_local[0] + T_act[0][1] * c_local[1] + T_act[0][2] * c_local[2],
        T_act[1][3] + T_act[1][0] * c_local[0] + T_act[1][1] * c_local[1] + T_act[1][2] * c_local[2],
        T_act[2][3] + T_act[2][0] * c_local[0] + T_act[2][1] * c_local[1] + T_act[2][2] * c_local[2]
    };
    double p_payload_nr[3] = {
        T_nr[0][3] + T_nr[0][0] * c_local[0] + T_nr[0][1] * c_local[1] + T_nr[0][2] * c_local[2],
        T_nr[1][3] + T_nr[1][0] * c_local[0] + T_nr[1][1] * c_local[1] + T_nr[1][2] * c_local[2],
        T_nr[2][3] + T_nr[2][0] * c_local[0] + T_nr[2][1] * c_local[1] + T_nr[2][2] * c_local[2]
    };

    // Unit-gravity force (kg, not N)
    const double F[3] = { 0.0, 0.0, -payload };

    // ----------------------------
    // 3) Apply coupling correction
    // ----------------------------
    for (i = 1; i <= nAxis; i++)
    {
        // torque = ((r x F) · axis)
        const double r_act[3] = {
            p_payload_act[0] - p_joint[i][0],
            p_payload_act[1] - p_joint[i][1],
            p_payload_act[2] - p_joint[i][2]
        };
        const double t_act[3] = {
            r_act[1] * F[2] - r_act[2] * F[1],
            r_act[2] * F[0] - r_act[0] * F[2],
            r_act[0] * F[1] - r_act[1] * F[0]
        };
        const double tau_act = t_act[0] * axis_w[i][0] + t_act[1] * axis_w[i][1] + t_act[2] * axis_w[i][2];

        const double r_nr[3] = {
            p_payload_nr[0] - p_joint_nr[i][0],
            p_payload_nr[1] - p_joint_nr[i][1],
            p_payload_nr[2] - p_joint_nr[i][2]
        };
        const double t_nr[3] = {
            r_nr[1] * F[2] - r_nr[2] * F[1],
            r_nr[2] * F[0] - r_nr[0] * F[2],
            r_nr[0] * F[1] - r_nr[1] * F[0]
        };
        const double tau_nr = t_nr[0] * axis_w[i][0] + t_nr[1] * axis_w[i][1] + t_nr[2] * axis_w[i][2];

        if (i < nAxis)
        {
            // Upstream joints: keep legacy baseline, add only the missing J6-coupling delta
            v[i - 1] = v_legacy[i - 1] + (tau_act - tau_nr);
        }
        else
        {
            // J6: legacy is always 0; output the physically-correct payload torque
            v[i - 1] = tau_act;
        }
    }
}


//***** 計算cross(三維) *****
void   HG6Cobot::cross(double v[], double a[], double b[])
{
    v[0] = (a[1] * b[2]) - (a[2] * b[1]);
    v[1] = (a[2] * b[0]) - (a[0] * b[2]);
    v[2] = (a[0] * b[1]) - (a[1] * b[0]);
}                               //***** 計算dot(三維) *****
double HG6Cobot::dot(double a[], double b[])
{
    int i; double d;
    for (i = 0, d = 0; i < 3; i++) d += a[i] * b[i];
    return(d);
}                         //***** 計算normalize(三維) *****
void   HG6Cobot::norm(double v[])
{
    int i; double d;
    for (i = 0, d = 0; i < 3; i++) d += (v[i] * v[i]);
    for (i = 0, d = sqrt(d); i < 3; i++) v[i] /= d;
}


//*********************************************************
// 機械手相關的轉移函數,將[4*4]矩陣用[3*4]矩陣表示
//   XYZ模式: a=Rz,b=Ry,c=Rx
//      {ca*cb, ca*sb*sc-sa*cc, ca*sb*cc+sa*sc, x}
//      {sa*cb, sa*sb*sc+ca*cc, sa*sb*cc-ca*sc, y}
//      {  -sb,    cb*sc,          cb*cc,       z}
//      {    0,        0,              0,       1}
//   ZYZ模式: a=q5,b=q6,c=q7
//      {ca*cb*cc-sa*sc, -ca*cb*sc-sa*cc, ca*sb}
//      {sa*cb*cc+ca*sc, -sa*cb*sc+ca*cc, sa*sb}
//      {  -sb*cc,           sb*sc,          cb}
//*********************************************************
            //***** 轉移函數=matrix34(x,y,x,Rx,Ry,Rz) *****
void HG6Cobot::matrix34(double d[3][4], double p[])
{
    matrix34(d, p[0], p[1], p[2], p[3], p[4], p[5]);
}
void HG6Cobot::matrix34(double d[3][4], double  x, double  y, double  z, //**平移量
    double rx, double ry, double rz) //**旋轉量
{
    double a, b, c, ca, cb, cc, sa, sb, sc;
    a = rx * DEG2RAD; ca = cos(a); sa = sin(a);
    b = ry * DEG2RAD; cb = cos(b); sb = sin(b);
    c = rz * DEG2RAD; cc = cos(c); sc = sin(c);

    d[0][0] = cb * cc;                  d[0][1] = -cb * sc;           d[0][2] = sb;         d[0][3] = x;
    d[1][0] = sa * sb * cc + ca * sc;          d[1][1] = -sa * sb * sc + ca * cc;   d[1][2] = -sa * cb;    d[1][3] = y;;
    d[2][0] = -ca * sb * cc + sa * sc;          d[2][1] = ca * sb * sc + sa * cc;   d[2][2] = ca * cb;    d[2][3] = z;
}
//***** 從matrix34換算(x,y,z,Rx,Ry,Rz) *****
void HG6Cobot::pose34(double d[], double mx[3][4])
{
    double* dx = deg(abc(mx), 3);
    d[0] = mx[0][3]; d[1] = mx[1][3]; d[2] = mx[2][3];
    d[3] = dx[2];    d[4] = dx[1];    d[5] = dx[0];
}
//***** 從matrix34換算(a,b,c) *****
double* HG6Cobot::abc(double mx[3][4])
{
    static double d[3] = { 0,0,0 }; abc(d, mx); return(d);
}
void    HG6Cobot::abc(double d[], double mx[3][4])
{
    double a, b, c, ca, cb, cc, sa, sb, sc;
    sa = mx[1][0]; ca = mx[0][0];      a = atan2(sa, ca);
    sb = -mx[2][0]; cb = sqrt(1 - sb * sb); b = atan2(sb, cb);
    sc = mx[2][1]; cc = mx[2][2];      c = atan2(sc, cc);
    if (cb < 0.0001)          //**b=+/-90度時,(a,c)=多重解
    {
        a = 0.; sc = mx[0][1] / sb; cc = mx[0][2] / sb; c = atan2(sc, cc);
    }
    d[0] = a; d[1] = b; d[2] = c;
}
//***** 從(vX,vY,vZ)換算(a,b,c) *****
double* HG6Cobot::abc(double vX[3], double vY[3], double vZ[3])
{
    static double d[3] = { 0,0,0 }; abc(d, vX, vY, vZ); return(d);
}
void    HG6Cobot::abc(double d[3], double vX[3], double vY[3], double vZ[3])
{
    double a, b, c, ca, cb, cc, sa, sb, sc;
    sa = vX[1]; ca = vX[0];         a = atan2(sa, ca);
    sb = -vX[2]; cb = sqrt(1 - sb * sb); b = atan2(sb, cb);
    sc = vY[2]; cc = vZ[2];         c = atan2(sc, cc);
    if (cb < 0.0001)          //**b=+/-90度時,(a,c)=多重解
    {
        a = 0.; sc = vY[0] / sb; cc = vZ[0] / sb; c = atan2(sc, cc);
    }
    d[0] = a; d[1] = b; d[2] = c;
}
//***** 從matrix34換算(z,y,z) *****
double* HG6Cobot::zyz(double mx[3][4])
{
    static double d[3] = { 0,0,0 }; zyz(d, mx); return(d);
}
void    HG6Cobot::zyz(double d[3], double mx[3][4])
{
    double a, b, c, ca, cb, cc, sa, sc;
    sa = mx[1][2]; ca = mx[0][2]; a = atan2(sa, ca);
    cb = mx[2][2]; b = acos(cb);
    sc = mx[2][1]; cc = -mx[2][0]; c = atan2(sc, cc);
    if ((1 - cb * cb) < 0.00001)  //**b=0度時,(a,c)=多重解
    {
        a = 0.; sc = -mx[0][1] / cb; cc = mx[0][0] / cb; c = atan2(sc, cc);
    }
    d[0] = a; d[1] = b; d[2] = c;
}
//***** 轉移函數 [d]=[a][b] *****
void HG6Cobot::mul34(double d[3][4], double a[3][4], double b[3][4])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            d[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + ((j == 3) ? a[i][3] : 0);
}                             //***** 座標轉換 d=[a]b *****
double* HG6Cobot::mul34(double a[3][4], double b[3])
{
    static double d[3] = { 0,0,0 }; mul34(d, a, b); return(d);
}
void    HG6Cobot::mul34(double d[3], double a[3][4], double b[3])
{
    for (int i = 0; i < 3; i++)
        d[i] = a[i][0] * b[0] + a[i][1] * b[1] + a[i][2] * b[2] + a[i][3];
}
//***** 反函數 [inv]=inv([mx]) *****
void HG6Cobot::inv34(double v[3][4], double m[3][4])
{
    v[0][0] = -m[1][2] * m[2][1] + m[1][1] * m[2][2];
    v[0][1] = m[0][2] * m[2][1] - m[0][1] * m[2][2];
    v[0][2] = -m[0][2] * m[1][1] + m[0][1] * m[1][2];
    v[0][3] = m[0][3] * m[1][2] * m[2][1] - m[0][2] * m[1][3] * m[2][1] - m[0][3] * m[1][1] * m[2][2] + m[0][1] * m[1][3] * m[2][2] + m[0][2] * m[1][1] * m[2][3] - m[0][1] * m[1][2] * m[2][3];
    v[1][0] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    v[1][1] = -m[0][2] * m[2][0] + m[0][0] * m[2][2];
    v[1][2] = m[0][2] * m[1][0] - m[0][0] * m[1][2];
    v[1][3] = m[0][2] * m[1][3] * m[2][0] - m[0][3] * m[1][2] * m[2][0] + m[0][3] * m[1][0] * m[2][2] - m[0][0] * m[1][3] * m[2][2] - m[0][2] * m[1][0] * m[2][3] + m[0][0] * m[1][2] * m[2][3];
    v[2][0] = -m[1][1] * m[2][0] + m[1][0] * m[2][1];
    v[2][1] = m[0][1] * m[2][0] - m[0][0] * m[2][1];
    v[2][2] = -m[0][1] * m[1][0] + m[0][0] * m[1][1];
    v[2][3] = m[0][3] * m[1][1] * m[2][0] - m[0][1] * m[1][3] * m[2][0] - m[0][3] * m[1][0] * m[2][1] + m[0][0] * m[1][3] * m[2][1] + m[0][1] * m[1][0] * m[2][3] - m[0][0] * m[1][1] * m[2][3];
}                             //***** 設置d[][]=s[][] *****
void    HG6Cobot::set34(double d[3][4], double s[3][4])
{
    for (int k = 0; k < 3; k++) { for (int i = 0; i < 4; i++) d[k][i] = s[k][i]; }
}


