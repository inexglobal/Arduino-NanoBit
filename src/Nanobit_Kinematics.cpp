#include "Nanobit_Kinematics.h"
#include <math.h>

static inline float d2r(float x) { return x * (float)M_PI / 180.0f; }
static inline float r2d(float x) { return x * 180.0f / (float)M_PI; }
static inline float wrapPI(float a)
{
    while (a > (float)M_PI)
        a -= (float)M_PI * 2;
    while (a < -(float)M_PI)
        a += (float)M_PI * 2;
    return a;
}

void NBKin::dhT(float a, float alpha_deg, float d, float theta_deg, float T[16])
{
    float ca = cosf(d2r(alpha_deg)), sa = sinf(d2r(alpha_deg));
    float ct = cosf(d2r(theta_deg)), st = sinf(d2r(theta_deg));
    T[0] = ct;
    T[1] = -st * ca;
    T[2] = st * sa;
    T[3] = a * ct;
    T[4] = st;
    T[5] = ct * ca;
    T[6] = -ct * sa;
    T[7] = a * st;
    T[8] = 0.0f;
    T[9] = sa;
    T[10] = ca;
    T[11] = d;
    T[12] = 0.0f;
    T[13] = 0.0f;
    T[14] = 0.0f;
    T[15] = 1.0f;
}

void NBKin::mul4(const float A[16], const float B[16], float C[16])
{
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
        {
            C[r * 4 + c] = A[r * 4 + 0] * B[0 * 4 + c] + A[r * 4 + 1] * B[1 * 4 + c] + A[r * 4 + 2] * B[2 * 4 + c] + A[r * 4 + 3] * B[3 * 4 + c];
        }
}

void NBKin::fk(int dof, const float q_deg[], const NB_DH dh[], float T_out[16])
{
    float T[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    float Ti[16], Tmp[16];
    for (int i = 0; i < dof; i++)
    {
        float theta = (dh[i].theta0) + q_deg[i];
        dhT(dh[i].a, dh[i].alpha, dh[i].d, theta, Ti);
        mul4(T, Ti, Tmp);
        for (int k = 0; k < 16; k++)
            T[k] = Tmp[k];
    }
    for (int k = 0; k < 16; k++)
        T_out[k] = T[k];
}

void NBKin::rpy_zyx(const float R[9], float &roll_deg, float &pitch_deg, float &yaw_deg)
{
    float r00 = R[0], r01 = R[1], r02 = R[2];
    float r10 = R[3], r11 = R[4], r12 = R[5];
    float r20 = R[6], r21 = R[7], r22 = R[8];
    float yaw = atan2f(r10, r00);
    float pitch = asinf(-r20);
    float roll = atan2f(r21, r22);
    roll_deg = r2d(roll);
    pitch_deg = r2d(pitch);
    yaw_deg = r2d(yaw);
}

void NBKin::jacobian_xyz_yaw(int dof, const float q_deg[], const NB_DH dh[], float J[4 * 4])
{
    float T0[16];
    fk(dof, q_deg, dh, T0);
    float R0[9] = {T0[0], T0[1], T0[2], T0[4], T0[5], T0[6], T0[8], T0[9], T0[10]};
    float r, p, y;
    rpy_zyx(R0, r, p, y);
    const float eps = 0.1f; // deg
    for (int j = 0; j < dof; j++)
    {
        float q1[8];
        for (int k = 0; k < dof; k++)
            q1[k] = q_deg[k];
        q1[j] += eps;
        float T1[16];
        fk(dof, q1, dh, T1);
        float R1[9] = {T1[0], T1[1], T1[2], T1[4], T1[5], T1[6], T1[8], T1[9], T1[10]};
        float r1, p1, y1;
        rpy_zyx(R1, r1, p1, y1);
        float dyaw = y1 - y; // deg
        while (dyaw > 180)
            dyaw -= 360;
        while (dyaw < -180)
            dyaw += 360;
        J[0 * 4 + j] = (T1[3] - T0[3]) / eps;   // dx/dqj
        J[1 * 4 + j] = (T1[7] - T0[7]) / eps;   // dy/dqj
        J[2 * 4 + j] = (T1[11] - T0[11]) / eps; // dz/dqj
        J[3 * 4 + j] = dyaw / eps;              // dyaw/dqj
    }
}

bool NBKin::ik_dls4(int dof, const NB_DH dh[], const float q_init[],
                    float x, float y, float z, float yaw_deg,
                    float q_out[], const NB_IKOptions &opt)
{
    float q[8];
    for (int i = 0; i < dof; i++)
        q[i] = q_init[i];
    for (int it = 0; it < opt.max_iter; ++it)
    {
        float T[16];
        fk(dof, q, dh, T);
        float R[9] = {T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]};
        float rr, pp, yy;
        rpy_zyx(R, rr, pp, yy);
        float e[4];
        e[0] = x - T[3];
        e[1] = y - T[7];
        e[2] = z - T[11];
        float dyaw = yaw_deg - yy;
        while (dyaw > 180)
            dyaw -= 360;
        while (dyaw < -180)
            dyaw += 360;
        e[3] = dyaw;
        float pos_norm = fabsf(e[0]) + fabsf(e[1]) + fabsf(e[2]);
        if (pos_norm <= opt.tol_pos && fabsf(e[3]) <= opt.tol_angdeg)
        {
            for (int i = 0; i < dof; i++)
                q_out[i] = q[i];
            return true;
        }
        float J[16] = {0};
        jacobian_xyz_yaw(dof, q, dh, J);
        // A = J*J^T (4x4)
        float A[16] = {0};
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                for (int k = 0; k < dof; k++)
                    A[r * 4 + c] += J[r * 4 + k] * J[c * 4 + k];
        float lam2 = opt.lambda * opt.lambda;
        for (int d = 0; d < 4; d++)
            A[d * 4 + d] += lam2;
        // Solve A y = e (Gauss-Jordan 4x4)
        float aug[4][5];
        for (int r = 0; r < 4; r++)
        {
            for (int c = 0; c < 4; c++)
                aug[r][c] = A[r * 4 + c];
            aug[r][4] = e[r];
        }
        for (int i = 0; i < 4; i++)
        {
            float piv = aug[i][i];
            if (fabsf(piv) < 1e-6f)
                piv = (aug[i][i] = (piv >= 0 ? 1e-6f : -1e-6f));
            float invp = 1.0f / piv;
            for (int c = 0; c < 5; c++)
                aug[i][c] *= invp;
            for (int r = 0; r < 4; r++)
                if (r != i)
                {
                    float f = aug[r][i];
                    for (int c = 0; c < 5; c++)
                        aug[r][c] -= f * aug[i][c];
                }
        }
        float yv[4];
        for (int r = 0; r < 4; r++)
            yv[r] = aug[r][4];
        float dq[8] = {0};
        for (int k = 0; k < dof; k++)
            for (int r = 0; r < 4; r++)
                dq[k] += J[r * 4 + k] * yv[r];
        for (int k = 0; k < dof; k++)
            q[k] += opt.gain * dq[k];
    }
    for (int i = 0; i < dof; i++)
        q_out[i] = q[i];
    return false;
}