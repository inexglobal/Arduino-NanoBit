#include "Nanobit_Trajectory.h"

// Avoid Windows/Arduino min/max macro issues by using our own helpers
#ifndef NB_MINMAX
template <typename T>
static inline T nb_min(T a, T b) { return (a < b) ? a : b; }
template <typename T>
static inline T nb_max(T a, T b) { return (a > b) ? a : b; }
#define NB_MINMAX
#endif

static inline nfloat _absf(nfloat x) { return x >= 0 ? x : -x; }
static inline nfloat _sgn(nfloat x) { return (x > 0) - (x < 0); }

NanobitTrajectory::NanobitTrajectory() {}

bool NanobitTrajectory::begin(const NTrajCfg &cfg, const NTrajLimits &limits)
{
    _cfg = cfg;
    _lim = limits;
    _dof = (cfg.dof ? nb_min<uint8_t>(cfg.dof, NANOBIT_TRAJ_MAX_DOF) : 4);
    reset();
    return true;
}
void NanobitTrajectory::setLimits(const NTrajLimits &limits) { _lim = limits; }
void NanobitTrajectory::reset()
{
    _profile = NProfile::None;
    _Tsync = 0;
    _t0 = 0;
    for (uint8_t i = 0; i < _dof; i++)
    {
        _T[i] = 0;
        _qStart[i] = 0;
        _qEnd[i] = 0;
        _q3[i] = {0, 0, 0, 0};
        _q5[i] = {0, 0, 0, 0, 0, 0};
        _qHome[i] = 0;
    }
    _hasHome = false;
}
void NanobitTrajectory::_syncScale()
{
    nfloat Tmax = 0;
    for (uint8_t i = 0; i < _dof; i++)
        if (_T[i] > Tmax)
            Tmax = _T[i];
    _Tsync = nb_max<nfloat>(Tmax * _cfg.timeScale, 1e-6f);
}

// ===== Cubic (q,v at both ends; v defaults to 0 if nullptr) =====
nfloat NanobitTrajectory::planCubic(const nfloat *q0, const nfloat *q1, const nfloat *v0, const nfloat *v1, nfloat *T_out, bool sync)
{
    _profile = NProfile::Cubic;
    for (uint8_t i = 0; i < _dof; i++)
    {
        _qStart[i] = q0[i];
        _qEnd[i] = q1[i];
        nfloat dist = _absf(q1[i] - q0[i]);
        nfloat vlim = nb_max<nfloat>(1e-6f, _lim.vmax[i]);
        nfloat alim = nb_max<nfloat>(1e-6f, _lim.amax[i]);
        nfloat v0i = v0 ? v0[i] : 0;
        nfloat v1i = v1 ? v1[i] : 0;
        nfloat Tbox = dist / vlim + vlim / alim; // cruise + accel allowance
        nfloat Ttri = 2 * sqrt(dist / alim);
        nfloat Tv0 = _absf(v0i) / alim * 2; // time to ramp from v0 to 0 and back (rough)
        nfloat Tv1 = _absf(v1i) / alim * 2;
        _T[i] = nb_max(nb_max(Tbox, Ttri), nb_max(Tv0, Tv1));
    }
    if (sync)
    {
        _syncScale();
    }
    else
    {
        _Tsync = _T[0];
        for (uint8_t i = 1; i < _dof; i++)
            if (_T[i] > _Tsync)
                _Tsync = _T[i];
    }
    for (uint8_t i = 0; i < _dof; i++)
    {
        nfloat T = _Tsync;
        nfloat T2 = T * T;
        nfloat T3 = T2 * T;
        nfloat q0i = _qStart[i], q1i = _qEnd[i];
        nfloat v0i = v0 ? v0[i] : 0;
        nfloat v1i = v1 ? v1[i] : 0;
        nfloat a0 = q0i;
        nfloat a1 = v0i;
        nfloat D = q1i - q0i - v0i * T;             // Δq adjusted by v0
        nfloat a2 = (3 * D - (v1i - v0i) * T) / T2; // cubic solution
        nfloat a3 = (-2 * D + (v1i - v0i) * T) / T3;
        _q3[i] = {a0, a1, a2, a3};
    }
    if (T_out)
        for (uint8_t i = 0; i < _dof; i++)
            T_out[i] = _T[i];
    _t0 = _now();
    return _Tsync;
}

// ===== LSPB (unchanged) =====
nfloat NanobitTrajectory::planLSPB(const nfloat *q0, const nfloat *q1, nfloat *T_out, bool sync)
{
    _profile = NProfile::LSPB;
    for (uint8_t i = 0; i < _dof; i++)
    {
        nfloat dq = q1[i] - q0[i];
        _qStart[i] = q0[i];
        _qEnd[i] = q1[i];
        nfloat dist = _absf(dq);
        nfloat vmax = nb_max<nfloat>(1e-6f, _lim.vmax[i]);
        nfloat amax = nb_max<nfloat>(1e-6f, _lim.amax[i]);
        nfloat Tacc = vmax / amax;
        nfloat xacc = 0.5f * amax * Tacc * Tacc;
        if (2 * xacc > dist)
        {
            Tacc = sqrt(dist / amax);
            _lspb[i] = {Tacc, 0.0f, Tacc, _sgn(dq), dist, vmax, amax};
            _T[i] = 2 * Tacc;
        }
        else
        {
            nfloat Tflat = (dist - 2 * xacc) / vmax;
            _lspb[i] = {Tacc, Tflat, Tacc, _sgn(dq), dist, vmax, amax};
            _T[i] = 2 * Tacc + Tflat;
        }
    }
    if (sync)
    {
        _syncScale();
    }
    else
    {
        _Tsync = _T[0];
        for (uint8_t i = 1; i < _dof; i++)
            if (_T[i] > _Tsync)
                _Tsync = _T[i];
    }
    if (T_out)
        for (uint8_t i = 0; i < _dof; i++)
            T_out[i] = _T[i];
    _t0 = _now();
    return _Tsync;
}

// ===== Quintic overloads =====
nfloat NanobitTrajectory::planQuintic(const nfloat *q0, const nfloat *q1, nfloat *T_out, bool sync) { return planQuintic(q0, q1, nullptr, nullptr, nullptr, nullptr, T_out, sync); }

nfloat NanobitTrajectory::planQuintic(const nfloat *q0, const nfloat *q1, const nfloat *v0, const nfloat *v1, nfloat *T_out, bool sync) { return planQuintic(q0, q1, v0, v1, nullptr, nullptr, T_out, sync); }

nfloat NanobitTrajectory::planQuintic(const nfloat *q0, const nfloat *q1, const nfloat *v0, const nfloat *v1, const nfloat *a0, const nfloat *a1, nfloat *T_out, bool sync)
{
    _profile = NProfile::Quintic;
    for (uint8_t i = 0; i < _dof; i++)
    {
        _qStart[i] = q0[i];
        _qEnd[i] = q1[i];
        nfloat dist = _absf(q1[i] - q0[i]);
        nfloat v = nb_max<nfloat>(1e-6f, _lim.vmax[i]);
        nfloat a = nb_max<nfloat>(1e-6f, _lim.amax[i]);
        nfloat Ttri = 2 * sqrt(dist / a);
        nfloat Tbox = dist / v + v / a;
        _T[i] = nb_max<nfloat>(Ttri, Tbox);
    }
    _syncScale();
    for (uint8_t i = 0; i < _dof; i++)
    {
        nfloat T = _Tsync;
        nfloat T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
        nfloat A = _qStart[i];
        nfloat B = v0 ? v0[i] : 0;
        nfloat C = a0 ? a0[i] / 2.0f : 0;
        nfloat D = (20 * (_qEnd[i] - _qStart[i]) - (12 * (v0 ? v0[i] : 0) + 8 * (v1 ? v1[i] : 0)) * T - (3 * (a0 ? a0[i] : 0) - (a1 ? a1[i] : 0)) * T2) / (2 * T3);
        nfloat E = (-30 * (_qEnd[i] - _qStart[i]) + (16 * (v0 ? v0[i] : 0) + 14 * (v1 ? v1[i] : 0)) * T + (3 * (a0 ? a0[i] : 0) - 2 * (a1 ? a1[i] : 0)) * T2) / (2 * T4);
        nfloat F = (12 * (_qEnd[i] - _qStart[i]) - (6 * (v0 ? v0[i] : 0) + 6 * (v1 ? v1[i] : 0)) * T - ((a0 ? a0[i] : 0) - (a1 ? a1[i] : 0)) * T2) / (2 * T5);
        _q5[i] = {A, B, C, D, E, F};
    }
    if (T_out)
        for (uint8_t i = 0; i < _dof; i++)
            T_out[i] = _T[i];
    _t0 = _now();
    return _Tsync;
}

uint32_t NanobitTrajectory::_now() const { return _cfg.useMicros ? micros() : millis(); }

// ===== Home configuration API =====
void NanobitTrajectory::setHome(const nfloat *qHome)
{
    for (uint8_t i = 0; i < _dof; i++)
        _qHome[i] = qHome[i];
    _hasHome = true;
}
bool NanobitTrajectory::getHome(nfloat *qOut) const
{
    if (!_hasHome)
        return false;
    for (uint8_t i = 0; i < _dof; i++)
        qOut[i] = _qHome[i];
    return true;
}

nfloat NanobitTrajectory::planToHomeCubic(const nfloat *v0, const nfloat *v1, nfloat *T_out, bool sync)
{
    if (!_hasHome)
        return 0;
    return planCubic(_qStart, _qHome, v0, v1, T_out, sync);
}

nfloat NanobitTrajectory::planToHomeQuintic(nfloat *T_out, bool sync)
{
    if (!_hasHome)
        return 0;
    return planQuintic(_qStart, _qHome, T_out, sync);
}

bool NanobitTrajectory::sample(uint32_t tEpoch, NTrajPoint &out) const
{
    if (_profile == NProfile::None || _Tsync <= 0)
        return false;
    nfloat t = (_cfg.useMicros ? (tEpoch - _t0) / 1e6f : (tEpoch - _t0) / 1e3f);
    if (t < 0)
        t = 0;
    if (t > _Tsync)
        t = _Tsync;
    for (uint8_t i = 0; i < _dof; i++)
    {
        nfloat q = 0, dq = 0, ddq = 0;
        switch (_profile)
        {
        case NProfile::Cubic:
            _evalCubic(i, t, q, dq, ddq);
            break;
        case NProfile::LSPB:
            _evalLSPB(i, t, q, dq, ddq);
            break;
        case NProfile::Quintic:
            _evalQuintic(i, t, q, dq, ddq);
            break;
        case NProfile::SCurve:
            _evalSCurve(i, t, q, dq, ddq);
            break;
        default:
            break;
        }
        out.q[i] = q;
        out.dq[i] = dq;
        out.ddq[i] = ddq;
    }
    return true;
}

// ===== Convenience getters (q1..q4 + pose + current/absolute time) =====
bool NanobitTrajectory::getStateAt4(uint32_t tEpoch, const NB_DH dh4[4], NBState &out)
{
    NTrajPoint p;
    if (!sample(tEpoch, p))
        return false;
    // copy q1..q4
    uint8_t n = (_cfg.dof >= 4) ? 4 : _cfg.dof;
    for (uint8_t i = 0; i < n; i++)
        out.q[i] = (float)p.q[i];
    for (uint8_t i = n; i < 4; i++)
        out.q[i] = 0.0f;
    // FK (4-DOF assumed)
    float T[16];
    NBKin::fk(4, out.q, dh4, T);
    out.x = T[3];
    out.y = T[7];
    out.z = T[11];
    float R[9] = {T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]};
    NBKin::rpy_zyx(R, out.roll, out.pitch, out.yaw);
    out.tEpoch = tEpoch;
    return true;
}

bool NanobitTrajectory::getStateNow4(const NB_DH dh4[4], NBState &out)
{
    uint32_t t = _now();
    return getStateAt4(t, dh4, out);
}

void NanobitTrajectory::_evalCubic(uint8_t i, nfloat t, nfloat &q, nfloat &dq, nfloat &ddq) const
{
    const auto &c = _q3[i];
    nfloat x = t, x2 = x * x, x3 = x2 * x;
    q = c.a0 + c.a1 * x + c.a2 * x2 + c.a3 * x3;
    dq = c.a1 + 2 * c.a2 * x + 3 * c.a3 * x2;
    ddq = 2 * c.a2 + 6 * c.a3 * x;
}

void NanobitTrajectory::_evalLSPB(uint8_t i, nfloat t, nfloat &q, nfloat &dq, nfloat &ddq) const
{
    const auto &S = _lspb[i];
    nfloat T = _Tsync;
    nfloat t1 = S.Tacc, t2 = S.Tacc + S.Tflat;
    nfloat a = S.amax * _sgn(_qEnd[i] - _qStart[i]);
    nfloat v = S.vmax * _sgn(_qEnd[i] - _qStart[i]);
    if (t <= t1)
    {
        dq = a * t;
        q = _qStart[i] + 0.5f * a * t * t;
        ddq = a;
    }
    else if (t <= t2)
    {
        dq = v;
        q = _qStart[i] + 0.5f * a * t1 * t1 + v * (t - t1);
        ddq = 0;
    }
    else
    {
        nfloat td = t - t2;
        dq = v - a * td;
        q = _qEnd[i] - 0.5f * a * (T - t) * (T - t);
        ddq = -a;
    }
}

void NanobitTrajectory::_evalQuintic(uint8_t i, nfloat t, nfloat &q, nfloat &dq, nfloat &ddq) const
{
    const auto &c = _q5[i];
    nfloat x = t, x2 = x * x, x3 = x2 * x, x4 = x3 * x, x5 = x4 * x;
    q = c.a0 + c.a1 * x + c.a2 * x2 + c.a3 * x3 + c.a4 * x4 + c.a5 * x5;
    dq = c.a1 + 2 * c.a2 * x + 3 * c.a3 * x2 + 4 * c.a4 * x3 + 5 * c.a5 * x4;
    ddq = 2 * c.a2 + 6 * c.a3 * x + 12 * c.a4 * x2 + 20 * c.a5 * x3;
}

void NanobitTrajectory::_evalSCurve(uint8_t i, nfloat t, nfloat &q, nfloat &dq, nfloat &ddq) const
{
    const auto &S = _scv[i];
    nfloat Tj = S.Tj, Ta = S.Ta, Tv = S.Tv, Td = S.Td;
    nfloat t1 = Tj, t2 = Ta, t3 = Ta + Tv, t4 = Ta + Tv + Td;
    nfloat s = _sgn(_qEnd[i] - _qStart[i]);
    nfloat j = S.jmax * s, amax = S.amax * s, vmax = S.vmax * s;
    if (t <= t1)
    {
        ddq = j * t;
        dq = 0.5f * j * t * t;
        q = _qStart[i] + (1.0f / 6.0f) * j * t * t * t;
    }
    else if (t <= t2)
    {
        nfloat dt = t - t1;
        ddq = amax;
        dq = 0.5f * j * Tj * Tj + amax * dt;
        q = _qStart[i] + (1.0f / 6.0f) * j * Tj * Tj * Tj + 0.5f * amax * dt * dt + 0.5f * j * Tj * Tj * dt;
    }
    else if (t <= t3)
    {
        ddq = 0;
        dq = vmax;
        q = _qStart[i] + (vmax * (t - t2)) + (vmax * Ta / 2.0f);
    }
    else if (t <= t4)
    {
        nfloat td = t - t3;
        ddq = -amax;
        dq = vmax - amax * td;
        q = _qEnd[i] - (vmax * (t4 - t)) - 0.5f * amax * (t4 - t) * (t4 - t);
    }
    else
    {
        ddq = 0;
        dq = 0;
        q = _qEnd[i];
    }
}
