#include <Arduino.h>
#include "Nanobit_Kinematics.h" // for NB_DH and FK helpers

#ifndef NANOBIT_TRAJ_MAX_DOF
#define NANOBIT_TRAJ_MAX_DOF 8
#endif

#ifdef NANOBIT_TRAJ_USE_DOUBLE
using nfloat = double;
#else
using nfloat = float;
#endif

struct NTrajPoint
{
    nfloat q[NANOBIT_TRAJ_MAX_DOF];
    nfloat dq[NANOBIT_TRAJ_MAX_DOF];
    nfloat ddq[NANOBIT_TRAJ_MAX_DOF];
};
struct NTrajLimits
{
    nfloat vmax[NANOBIT_TRAJ_MAX_DOF];
    nfloat amax[NANOBIT_TRAJ_MAX_DOF];
    nfloat jmax[NANOBIT_TRAJ_MAX_DOF];
};
struct NTrajCfg
{
    uint8_t dof = 4;
    bool useMicros = false;
    nfloat timeScale = 1.0f;
};

// State struct for convenience reads (q + pose + timestamp)
struct NBState
{
    uint32_t tEpoch;        // millis() or micros() depending on cfg
    float q[4];             // first 4 joints (deg)
    float x, y, z;          // position (units of DH, e.g., mm)
    float roll, pitch, yaw; // RPY ZYX (deg)
};

enum class NProfile : uint8_t
{
    None,
    LSPB,
    Cubic,
    Quintic,
    SCurve
};

class NanobitTrajectory
{
public:
    NanobitTrajectory();
    bool begin(const NTrajCfg &cfg, const NTrajLimits &limits);
    void setLimits(const NTrajLimits &limits);
    void reset();

    // ===== New: Cubic Path =====
    nfloat planCubic(const nfloat *q0, const nfloat *q1,
                     const nfloat *v0 = nullptr, const nfloat *v1 = nullptr,
                     nfloat *T_out = nullptr, bool sync = true);

    // LSPB (as before)
    nfloat planLSPB(const nfloat *q0, const nfloat *q1, nfloat *T_out = nullptr, bool sync = true);

    // ===== Convenience Quintic =====
    nfloat planQuintic(const nfloat *q0, const nfloat *q1, nfloat *T_out, bool sync = true);
    nfloat planQuintic(const nfloat *q0, const nfloat *q1,
                       const nfloat *v0, const nfloat *v1,
                       nfloat *T_out, bool sync = true);
    nfloat planQuintic(const nfloat *q0, const nfloat *q1,
                       const nfloat *v0, const nfloat *v1,
                       const nfloat *a0, const nfloat *a1,
                       nfloat *T_out = nullptr, bool sync = true);

    // ===== Home configuration =====
    void setHome(const nfloat *qHome);
    bool getHome(nfloat *qOut) const;
    nfloat planToHomeCubic(const nfloat *v0 = nullptr, const nfloat *v1 = nullptr,
                           nfloat *T_out = nullptr, bool sync = true);
    nfloat planToHomeQuintic(nfloat *T_out = nullptr, bool sync = true);

    // ===== Sampling =====
    bool sample(uint32_t tEpoch, NTrajPoint &out) const;
    bool clearWaypoints();
    bool pushWaypoint(const nfloat *qTarget);

    // ===== Convenience getters (q1..q4 + pose + time) =====
    bool getStateNow4(const NB_DH dh4[4], NBState &out); // uses current time
    bool getStateAt4(uint32_t tEpoch, const NB_DH dh4[4], NBState &out);

    nfloat duration() const { return _Tsync; }
    NProfile activeProfile() const { return _profile; }
    uint8_t dof() const { return _cfg.dof; }

private:
    struct Poly3
    {
        nfloat a0, a1, a2, a3;
    };
    struct Poly5
    {
        nfloat a0, a1, a2, a3, a4, a5;
    };
    struct LSPBSeg
    {
        nfloat Tacc, Tflat, Tdec, sgn, dist, vmax, amax;
    };
    struct SCurveSeg
    {
        nfloat Tj, Ta, Tv, Td, T, sgn, dist, vmax, amax, jmax;
    };

    NTrajCfg _cfg;
    NTrajLimits _lim;
    uint8_t _dof = 4;
    NProfile _profile = NProfile::None;
    nfloat _T[NANOBIT_TRAJ_MAX_DOF];
    nfloat _Tsync = 0;
    uint32_t _t0 = 0;

    Poly3 _q3[NANOBIT_TRAJ_MAX_DOF];
    Poly5 _q5[NANOBIT_TRAJ_MAX_DOF];
    LSPBSeg _lspb[NANOBIT_TRAJ_MAX_DOF];
    SCurveSeg _scv[NANOBIT_TRAJ_MAX_DOF];
    nfloat _qStart[NANOBIT_TRAJ_MAX_DOF];
    nfloat _qEnd[NANOBIT_TRAJ_MAX_DOF];
    nfloat _qHome[NANOBIT_TRAJ_MAX_DOF];
    bool _hasHome = false;

    void _syncScale();
    void _evalCubic(uint8_t i, nfloat t, nfloat &q, nfloat &dq, nfloat &ddq) const;
    void _evalLSPB(uint8_t i, nfloat t, nfloat &q, nfloat &dq, nfloat &ddq) const;
    void _evalQuintic(uint8_t i, nfloat t, nfloat &q, nfloat &dq, nfloat &ddq) const;
    void _evalSCurve(uint8_t i, nfloat t, nfloat &q, nfloat &dq, nfloat &ddq) const;
    uint32_t _now() const;
};