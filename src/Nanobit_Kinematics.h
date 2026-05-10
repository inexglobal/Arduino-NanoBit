#include <Arduino.h>

// DH-based FK + DLS IK for up to 4 DOF
#ifndef NANOBIT_TRAJ_MAX_DOF
#define NANOBIT_TRAJ_MAX_DOF 8
#endif

struct NB_DH
{
    float a;
    float alpha;
    float d;
    float theta0;
};

struct NB_IKOptions
{
    int max_iter = 150;
    float tol_pos = 0.5f;     // same unit as DH (e.g., mm)
    float tol_angdeg = 1.0f;  // deg
    float lambda = 5.0f;      // damping
    float gain = 0.6f;        // step gain
    bool use_yaw_only = true; // 4-DOF: match yaw only
};

namespace NBKin
{
    void dhT(float a, float alpha_deg, float d, float theta_deg, float T[16]);
    void mul4(const float A[16], const float B[16], float C[16]);
    void fk(int dof, const float q_deg[], const NB_DH dh[], float T_out[16]);
    void rpy_zyx(const float R[9], float &roll_deg, float &pitch_deg, float &yaw_deg);
    void jacobian_xyz_yaw(int dof, const float q_deg[], const NB_DH dh[], float J[4 * 4]);
    bool ik_dls4(int dof, const NB_DH dh[], const float q_init[],
                 float x, float y, float z, float yaw_deg,
                 float q_out[], const NB_IKOptions &opt);
}