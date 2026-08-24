#include "headcam.h"

#include <api/SWA.h>
#include <app.h>
#include <gpu/video.h>
#include <kernel/memory.h>
#include <os/logger.h>
#include <user/config.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <vector>

namespace
{
    // ---------------- Tunables ----------------
    // Head cam height/offset relative to the character proxy position, in
    // game units (meters). The proxy sits at the character's ground
    // reference, so 0.85 up + 0.35 forward lands just above and in front of
    // Sonic's head.
    constexpr float HEAD_HEIGHT = 0.85f;
    constexpr float HEAD_FORWARD = 0.35f;
    // A slight downward tilt reads better than perfectly level in first
    // person. Positive value pitches the view down.
    constexpr float HEAD_PITCH_DEG = 7.0f;
    constexpr float BLEND_TAU_IN = 0.20;   // seconds to ramp in
    constexpr float BLEND_TAU_OUT = 0.35;  // seconds to ramp out
    constexpr float YAW_TAU = 0.08;        // seconds to follow a new facing
    constexpr float VELOCITY_THRESHOLD = 0.8f; // m/s before the yaw follows

    const uint32_t WORLD_MAP_CAMERA_VTABLE = 0x8202BF1C; // SWA::CWorldMapCamera

    inline bool InCodeRange(uint32_t addr)
    {
        return addr >= PPC_CODE_BASE && addr < PPC_CODE_BASE + PPC_CODE_SIZE;
    }

    inline bool ValidGuestPtr(uint32_t addr)
    {
        // Guest addresses are 32-bit; the only sentinel is null. (Range is
        // implicit in the type, so an explicit bound would always be true.)
        return addr != 0;
    }

    // ---------------- Small math ----------------
    struct Vec3
    {
        float x = 0.0f, y = 0.0f, z = 0.0f;
    };

    inline Vec3 operator+(const Vec3& a, const Vec3& b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
    inline Vec3 operator-(const Vec3& a, const Vec3& b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
    inline Vec3 operator*(const Vec3& a, float s) { return { a.x * s, a.y * s, a.z * s }; }
    inline Vec3 operator*(float s, const Vec3& a) { return a * s; }
    inline float Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline Vec3 Cross(const Vec3& a, const Vec3& b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }
    inline float Length(const Vec3& a) { return std::sqrt(Dot(a, a)); }
    inline Vec3 Normalize(const Vec3& a)
    {
        float l = Length(a);
        return l > 1e-6f ? a * (1.0f / l) : Vec3{};
    }
    inline bool Finite(const Vec3& a)
    {
        return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
    }

    struct Quat
    {
        float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;
    };

    // Row-major (math) matrix: m[row][col].
    struct Mat4
    {
        float m[4][4]{};

        static Mat4 Identity()
        {
            Mat4 r{};
            r.m[0][0] = r.m[1][1] = r.m[2][2] = r.m[3][3] = 1.0f;
            return r;
        }
    };

    inline Mat4 Mul(const Mat4& a, const Mat4& b)
    {
        Mat4 r{};
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] +
                    a.m[i][2] * b.m[2][j] + a.m[i][3] * b.m[3][j];
        return r;
    }

    // Determinant of a 3x3 matrix given as row-major rows.
    static inline float Det3(float a00, float a01, float a02,
                             float a10, float a11, float a12,
                             float a20, float a21, float a22)
    {
        return a00 * (a11 * a22 - a12 * a21)
             - a01 * (a10 * a22 - a12 * a20)
             + a02 * (a10 * a21 - a11 * a20);
    }

    // General 4x4 inverse via cofactor expansion: (A^-1)[i][j] = C(j,i)/det.
    bool Mat4Invert(const Mat4& a, Mat4& out)
    {
        // Cofactor C(r,c) = (-1)^(r+c) * minor(r,c), minor = det of the 3x3
        // left with row r / column c removed.
        auto minor = [&](int r, int c)
        {
            int rs[3], cs[3];
            int ri = 0, ci = 0;
            for (int i = 0; i < 4; i++) if (i != r) rs[ri++] = i;
            for (int j = 0; j < 4; j++) if (j != c) cs[ci++] = j;
            float m[3][3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    m[i][j] = a.m[rs[i]][cs[j]];
            return Det3(m[0][0], m[0][1], m[0][2],
                        m[1][0], m[1][1], m[1][2],
                        m[2][0], m[2][1], m[2][2]);
        };

        float det = a.m[0][0] * minor(0, 0)
                  - a.m[0][1] * minor(0, 1)
                  + a.m[0][2] * minor(0, 2)
                  - a.m[0][3] * minor(0, 3);
        if (std::fabs(det) < 1e-12f)
            return false;
        float invDet = 1.0f / det;

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            {
                float sign = ((i + j) & 1) ? -1.0f : 1.0f;
                out.m[i][j] = sign * minor(j, i) * invDet;
            }
        return true;
    }

    // Inverse of a rigid (rotation + translation) matrix.
    inline Mat4 InvertRigid(const Mat4& a)
    {
        Mat4 r{};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                r.m[i][j] = a.m[j][i];
        Vec3 t{ a.m[0][3], a.m[1][3], a.m[2][3] };
        r.m[0][3] = -Dot(Vec3{ a.m[0][0], a.m[1][0], a.m[2][0] }, t);
        r.m[1][3] = -Dot(Vec3{ a.m[0][1], a.m[1][1], a.m[2][1] }, t);
        r.m[2][3] = -Dot(Vec3{ a.m[0][2], a.m[1][2], a.m[2][2] }, t);
        r.m[3][3] = 1.0f;
        return r;
    }

    inline bool CloseToIdentity(const Mat4& a, float eps)
    {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                if (std::fabs(a.m[i][j] - (i == j ? 1.0f : 0.0f)) > eps)
                    return false;
        return true;
    }

    inline bool CloseMat(const Mat4& a, const Mat4& b, float eps)
    {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                if (std::fabs(a.m[i][j] - b.m[i][j]) > eps * std::fabs(b.m[i][j]) + 1e-3f)
                    return false;
        return true;
    }

    Quat QuatFromBasis(const Vec3& right, const Vec3& up, const Vec3& forward)
    {
        // Rows of R are the basis vectors.
        float r00 = right.x, r01 = right.y, r02 = right.z;
        float r10 = up.x, r11 = up.y, r12 = up.z;
        float r20 = forward.x, r21 = forward.y, r22 = forward.z;

        float trace = r00 + r11 + r22;
        Quat q{};
        if (trace > 0.0f)
        {
            float s = std::sqrt(trace + 1.0f) * 2.0f;
            q.w = 0.25f * s;
            q.x = (r21 - r12) / s;
            q.y = (r02 - r20) / s;
            q.z = (r10 - r01) / s;
        }
        else if (r00 > r11 && r00 > r22)
        {
            float s = std::sqrt(1.0f + r00 - r11 - r22) * 2.0f;
            q.w = (r21 - r12) / s;
            q.x = 0.25f * s;
            q.y = (r01 + r10) / s;
            q.z = (r02 + r20) / s;
        }
        else if (r11 > r22)
        {
            float s = std::sqrt(1.0f + r11 - r00 - r22) * 2.0f;
            q.w = (r02 - r20) / s;
            q.x = (r01 + r10) / s;
            q.y = 0.25f * s;
            q.z = (r12 + r21) / s;
        }
        else
        {
            float s = std::sqrt(1.0f + r22 - r00 - r11) * 2.0f;
            q.w = (r10 - r01) / s;
            q.x = (r02 + r20) / s;
            q.y = (r12 + r21) / s;
            q.z = 0.25f * s;
        }
        return q;
    }

    Vec3 QuatToBasisRight(const Quat& q)
    {
        return {
            1.0f - 2.0f * (q.y * q.y + q.z * q.z),
            2.0f * (q.x * q.y - q.w * q.z),
            2.0f * (q.x * q.z + q.w * q.y)
        };
    }
    Vec3 QuatToBasisUp(const Quat& q)
    {
        return {
            2.0f * (q.x * q.y + q.w * q.z),
            1.0f - 2.0f * (q.x * q.x + q.z * q.z),
            2.0f * (q.y * q.z - q.w * q.x)
        };
    }
    Vec3 QuatToBasisForward(const Quat& q)
    {
        return {
            2.0f * (q.x * q.z - q.w * q.y),
            2.0f * (q.y * q.z + q.w * q.x),
            1.0f - 2.0f * (q.x * q.x + q.y * q.y)
        };
    }

    inline Quat QuatSlerp(const Quat& a, const Quat& b, float t)
    {
        float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
        Quat bb = b;
        if (d < 0.0f)
        {
            d = -d;
            bb = { -b.x, -b.y, -b.z, -b.w };
        }
        float s0, s1;
        if (d > 0.9995f)
        {
            s0 = 1.0f - t;
            s1 = t;
        }
        else
        {
            float theta = std::acos(std::clamp(d, -1.0f, 1.0f));
            float sinTheta = std::sin(theta);
            s0 = std::sin((1.0f - t) * theta) / sinTheta;
            s1 = std::sin(t * theta) / sinTheta;
        }
        Quat r{
            a.x * s0 + bb.x * s1,
            a.y * s0 + bb.y * s1,
            a.z * s0 + bb.z * s1,
            a.w * s0 + bb.w * s1
        };
        float l = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w);
        return l > 1e-6f ? Quat{ r.x / l, r.y / l, r.z / l, r.w / l } : Quat{};
    }

    // ---------------- Layout helpers ----------------
    // rowMajor: guest memory holds the math matrix row by row.
    inline Mat4 ToMat4(const float* mem, bool rowMajor)
    {
        Mat4 r{};
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                r.m[i][j] = rowMajor ? mem[i * 4 + j] : mem[j * 4 + i];
        return r;
    }

    inline void Mat4ToMem(float* mem, bool rowMajor, const Mat4& a)
    {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                mem[rowMajor ? i * 4 + j : j * 4 + i] = a.m[i][j];
    }

    // D3D9-style view matrix from a camera pose.
    inline void BuildViewMat(Mat4& out, const Vec3& eye, const Vec3& right, const Vec3& up, const Vec3& forward)
    {
        out.m[0][0] = right.x;   out.m[0][1] = right.y;   out.m[0][2] = right.z;   out.m[0][3] = -Dot(right, eye);
        out.m[1][0] = up.x;      out.m[1][1] = up.y;      out.m[1][2] = up.z;      out.m[1][3] = -Dot(up, eye);
        out.m[2][0] = forward.x; out.m[2][1] = forward.y; out.m[2][2] = forward.z; out.m[2][3] = -Dot(forward, eye);
        out.m[3][0] = 0.0f;      out.m[3][1] = 0.0f;      out.m[3][2] = 0.0f;      out.m[3][3] = 1.0f;
    }

    // Try to interpret a math matrix as a pure view matrix.
    bool TryDecomposeView(const Mat4& mm, Vec3& eye, Vec3& right, Vec3& up, Vec3& forward, float eps = 2e-3f)
    {
        for (int j = 0; j < 3; j++)
            if (std::fabs(mm.m[3][j]) > eps)
                return false;
        if (std::fabs(mm.m[3][3] - 1.0f) > eps)
            return false;

        right = { mm.m[0][0], mm.m[0][1], mm.m[0][2] };
        up = { mm.m[1][0], mm.m[1][1], mm.m[1][2] };
        forward = { mm.m[2][0], mm.m[2][1], mm.m[2][2] };

        for (Vec3* r : { &right, &up, &forward })
        {
            float l = Length(*r);
            if (l < 1.0f - 2e-3f || l > 1.0f + 2e-3f)
                return false;
        }
        if (std::fabs(Dot(right, up)) > 5e-3f || std::fabs(Dot(right, forward)) > 5e-3f || std::fabs(Dot(up, forward)) > 5e-3f)
            return false;

        // V = [R | -R*eye] with R's rows = (right, up, forward).
        // eye = -R^T * t, where t is V's translation column.
        Vec3 t{ mm.m[0][3], mm.m[1][3], mm.m[2][3] };
        if (!Finite(t) || Length(t) > 1e5f)
            return false;

        eye.x = -(right.x * t.x + up.x * t.y + forward.x * t.z);
        eye.y = -(right.y * t.x + up.y * t.y + forward.y * t.z);
        eye.z = -(right.z * t.x + up.z * t.y + forward.z * t.z);
        return Finite(eye);
    }

    // Pure projection part (D3D9 style) or the identity.
    inline bool LooksLikeProjectionPart(const Mat4& p, float eps)
    {
        if (CloseToIdentity(p, 1e-3f))
            return true;
        if (std::fabs(p.m[3][2] - 1.0f) > eps || std::fabs(p.m[3][3]) > eps)
            return false;
        if (std::fabs(p.m[3][0]) > eps || std::fabs(p.m[3][1]) > eps)
            return false;
        if (std::fabs(p.m[2][0]) > eps || std::fabs(p.m[2][1]) > eps)
            return false;
        if (std::fabs(p.m[0][1]) > eps || std::fabs(p.m[0][2]) > eps || std::fabs(p.m[0][3]) > eps)
            return false;
        if (std::fabs(p.m[1][0]) > eps || std::fabs(p.m[1][2]) > eps || std::fabs(p.m[1][3]) > eps)
            return false;
        if (std::fabs(p.m[0][0]) < 1e-6f || std::fabs(p.m[1][1]) < 1e-6f)
            return false;
        return true;
    }

    // ---------------- State ----------------
    struct CameraState
    {
        // Calibrated handles.
        uint32_t device = 0;
        uint32_t application = 0;
        uint32_t director = 0;

        bool rowMajor = true;          // guest storage convention of the view matrix
        int32_t viewFloat = -1;        // float index (multiple of 4) of the view matrix in the VS constants
        Mat4 projectionPart = Mat4::Identity();
        // True when the serialized block is P*V (world->clip, the common
        // convention); false when it is V*P. Both are handled.
        bool projBeforeView = true;
        bool havePose = false;
        Mat4 lastView = Mat4::Identity();
        Vec3 lastEye{};
        Vec3 lastRight{}, lastUp{}, lastFwd{};
        bool haveLastEye = false;
        bool loggedCalibration = false;
        bool loggedFailure = false;

        // Per-frame snapshot for the constants diff.
        uint32_t vsSnapshot[1024]{};
        bool snapshotValid = false;

        // Update-list discovery: member base + vector begin/end offsets.
        bool listFound = false;
        uint32_t listBase = 0;
        uint32_t listBeginOff = 0;
        uint32_t listEndOff = 0;
        double lastListScan = 0.0;

        // Sticky player proxy.
        uint32_t proxy = 0;
        Vec3 proxyPos{};
        bool proxyValid = false;

        // Camera-geometry player estimator: sight-line distance D (the player
        // sits on the camera's sight line, a follow cam looks at the player).
        bool estInit = false;
        float estD = 6.3f;
        Vec3 estPlayer{};
        Vec3 estVel{};
        Vec3 estLastEye{};

        // Fused player state.
        bool havePlayer = false;
        Vec3 playerPos{};
        Vec3 lastPlayerPos{};
        Vec3 vel{};
        Vec3 fwd{};
        bool haveFwd = false;
        float blend = 0.0f;
        float wProxy = 0.0f;

        // Last matrix we wrote to the register (self-write detection) and
        // frame counter (object scan throttling).
        float lastWritten[16]{};
        bool lastWrittenValid = false;
        int32_t frameCount = 0;
    };

    CameraState S;
    std::atomic<bool> s_resetRequested{ false };

    void ReadFloats(const uint8_t* base, uint32_t addr, float* out, int count)
    {
        const be<float>* src = reinterpret_cast<const be<float>*>(base + addr);
        for (int i = 0; i < count; i++)
            out[i] = src[i].get();
    }

    bool ReadVec3(const uint8_t* base, uint32_t addr, Vec3& out)
    {
        float v[3];
        ReadFloats(base, addr, v, 3);
        out = { v[0], v[1], v[2] };
        return Finite(out);
    }

    // ---------------- CCamera object scanning ----------------
    // The camera serializes its view matrix to the vertex shader constants
    // every frame; its own world transform (Hedgehog CTransform: quaternion
    // + position + matrix) is stored in the CCamera object. Find a consistent
    // CTransform; the view matrix is the inverse of its rotation/position.
    bool ScanCameraObject(const uint8_t* base, uint32_t camera, bool& rowMajor, Mat4& outView, Vec3& outEye, Vec3& outRight, Vec3& outUp, Vec3& outForward)
    {
        struct Candidate
        {
            Vec3 pos;
            Quat q;
            bool rowMajor;
        };
        std::vector<Candidate> candidates;

        for (uint32_t off = 0; off + 0x60 <= 0x2A0; off += 4)
        {
            float qf[4], pf[4], mf[16];
            ReadFloats(base, camera + off, qf, 4);
            ReadFloats(base, camera + off + 0x10, pf, 4);
            ReadFloats(base, camera + off + 0x20, mf, 16);

            float qlen2 = qf[0] * qf[0] + qf[1] * qf[1] + qf[2] * qf[2] + qf[3] * qf[3];
            if (qlen2 < 0.999f || qlen2 > 1.001f)
                continue;
            for (int i = 0; i < 3; i++)
                if (!std::isfinite(pf[i]) || std::fabs(pf[i]) > 1e4f)
                    continue;

            float qlen = std::sqrt(qlen2);
            Quat q{ qf[0] / qlen, qf[1] / qlen, qf[2] / qlen, qf[3] / qlen };

            Vec3 right = QuatToBasisRight(q);
            Vec3 up = QuatToBasisUp(q);
            Vec3 forward = QuatToBasisForward(q);

            for (int ri = 0; ri < 2; ri++)
            {
                bool rm = ri == 0;
                Mat4 mm = ToMat4(mf, rm);

                if (std::fabs(mm.m[3][3] - 1.0f) > 2e-3f)
                    continue;
                for (int j = 0; j < 3; j++)
                    if (std::fabs(mm.m[3][j]) > 2e-3f)
                        continue;

                Vec3 t{ mm.m[0][3], mm.m[1][3], mm.m[2][3] };
                if (Length(t - Vec3{ pf[0], pf[1], pf[2] }) > 0.25f)
                    continue;

                if (std::fabs(Dot({ mm.m[0][0], mm.m[0][1], mm.m[0][2] }, right) - 1.0f) > 5e-3f ||
                    std::fabs(Dot({ mm.m[1][0], mm.m[1][1], mm.m[1][2] }, up) - 1.0f) > 5e-3f ||
                    std::fabs(Dot({ mm.m[2][0], mm.m[2][1], mm.m[2][2] }, forward) - 1.0f) > 5e-3f)
                    continue;

                candidates.push_back({ { pf[0], pf[1], pf[2] }, q, rm });
            }
        }

        if (candidates.empty())
            return false;

        // Pick the candidate whose view matrix matches the last known view.
        size_t pick = 0;
        if (candidates.size() > 1 && S.havePose)
        {
            for (size_t i = 0; i < candidates.size(); i++)
            {
                Vec3 right = Normalize(QuatToBasisRight(candidates[i].q));
                Vec3 up = Normalize(QuatToBasisUp(candidates[i].q));
                Vec3 forward = Normalize(QuatToBasisForward(candidates[i].q));
                Mat4 v;
                BuildViewMat(v, candidates[i].pos, right, up, forward);
                if (CloseMat(v, S.lastView, 0.1f))
                {
                    pick = i;
                    break;
                }
            }
        }

        const auto& c = candidates[pick];
        Vec3 right = Normalize(QuatToBasisRight(c.q));
        Vec3 up = Normalize(QuatToBasisUp(c.q));
        Vec3 forward = Normalize(QuatToBasisForward(c.q));
        if (std::fabs(Dot(Cross(right, up), forward)) < 0.99f)
            return false;

        rowMajor = c.rowMajor;
        BuildViewMat(outView, c.pos, right, up, forward);
        outEye = c.pos;
        outRight = right;
        outUp = up;
        outForward = forward;
        return true;
    }

    bool IsPlayerProxy(const uint8_t* base, uint32_t proxy)
    {
        if (!ValidGuestPtr(proxy) || !InCodeRange(PPC_LOAD_U32(proxy)))
            return false;
        float p[3];
        ReadFloats(base, proxy + 0x120, p, 3);
        return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
            std::fabs(p[0]) < 1e4f && std::fabs(p[1]) < 1e4f && std::fabs(p[2]) < 1e4f;
    }

    // ---------------- Update list discovery ----------------
    void TryDiscoverUpdateList(uint32_t director, uint8_t* base)
    {
        // The update list is a vector of object pointers (8-byte slots, the
        // same layout the FX job scheduler uses) reachable from a member of
        // the update director. Try member/vector offset pairs and validate:
        // most entries need a code-range vtable and at least one entry must
        // own a character proxy (position at proxy + 0x120).
        const uint32_t vectorOffsets[11][2] = {
            { 0x24, 0x28 }, { 0x20, 0x24 }, { 0x18, 0x1C }, { 0x10, 0x14 },
            { 0x08, 0x0C }, { 0x2C, 0x30 }, { 0x34, 0x38 }, { 0x3C, 0x40 },
            { 0x44, 0x48 }, { 0x4C, 0x50 }, { 0x54, 0x58 },
        };

        std::vector<uint32_t> bases;
        bases.push_back(director);
        for (uint32_t off = 4; off + 4 <= 0x300; off += 4)
        {
            uint32_t b = PPC_LOAD_U32(director + off);
            if (ValidGuestPtr(b))
                bases.push_back(b);
        }

        for (uint32_t b : bases)
        {
            for (auto& vo : vectorOffsets)
            {
                uint32_t begin = PPC_LOAD_U32(b + vo[0]);
                uint32_t end = PPC_LOAD_U32(b + vo[1]);
                if (end <= begin || ((end - begin) & 7) != 0)
                    continue;
                uint32_t count = (end - begin) / 8;
                if (count < 8 || count > 400)
                    continue;

                uint32_t valid = 0, proxyHits = 0;
                bool tooLong = false;
                for (uint32_t it = begin; it != end; it += 8)
                {
                    if (count > 300)
                    {
                        tooLong = true;
                        break;
                    }
                    uint32_t obj = PPC_LOAD_U32(it);
                    if (!ValidGuestPtr(obj) || !InCodeRange(PPC_LOAD_U32(obj)))
                        continue;
                    valid++;
                    if (IsPlayerProxy(base, PPC_LOAD_U32(obj + 0x100)))
                        proxyHits++;
                }
                if (tooLong || valid < count * 3 / 4 || proxyHits == 0)
                    continue;

                S.listFound = true;
                S.listBase = b;
                S.listBeginOff = vo[0];
                S.listEndOff = vo[1];
                LOGFN("HeadCam: update list found (entries={}, proxies={})", valid, proxyHits);
                return;
            }
        }
    }

    // ---------------- Per-frame pieces ----------------
    void CollectProxies(const uint8_t* base, const Vec3& eye, const Vec3& forward)
    {
        S.proxyValid = false;
        if (!S.listFound)
            return;

        uint32_t begin = PPC_LOAD_U32(S.listBase + S.listBeginOff);
        uint32_t end = PPC_LOAD_U32(S.listBase + S.listEndOff);
        if (end <= begin || (end - begin) >= 400 * 8)
            return;

        float bestScore = 1e30f;
        uint32_t bestProxy = 0;
        float currentScore = 1e30f;

        for (uint32_t it = begin; it != end; it += 8)
        {
            uint32_t obj = PPC_LOAD_U32(it);
            if (!ValidGuestPtr(obj))
                continue;
            uint32_t proxy = PPC_LOAD_U32(obj + 0x100);
            if (!IsPlayerProxy(base, proxy))
                continue;

            Vec3 pos;
            if (!ReadVec3(base, proxy + 0x120, pos))
                continue;

            Vec3 d = pos - eye;
            float depth = Dot(d, forward);
            float latLen = Length(d - forward * depth);
            if (depth < 0.5f || depth > 25.0f || latLen > 8.0f)
                continue;

            float score = latLen + 0.3f * std::fabs(depth - 6.0f);
            if (proxy == S.proxy)
                currentScore = score;
            if (score < bestScore)
            {
                bestScore = score;
                bestProxy = proxy;
            }
        }

        // Sticky selection with hysteresis.
        if (S.proxy && currentScore < bestScore * 1.5f + 1.0f)
            S.proxyValid = true;
        else if (bestProxy != 0)
        {
            S.proxy = bestProxy;
            S.proxyValid = true;
        }

        if (S.proxyValid && S.proxy)
        {
            Vec3 pos;
            if (ReadVec3(base, S.proxy + 0x120, pos))
                S.proxyPos = pos;
            else
            {
                S.proxyValid = false;
                S.proxy = 0;
            }
        }
    }
}

namespace HeadCam
{
    void Reset()
    {
        // May be called from the UI thread; the game thread applies it.
        s_resetRequested.store(true);
    }

    void ApplyReset()
    {
        if (!s_resetRequested.exchange(false))
            return;
        // Keep the device handle, the update list and the register calibration
        // (all still valid); clear the transient tracking state so the head
        // cam blends back in fresh from the current camera.
        S.havePose = false;
        S.lastView = Mat4::Identity();
        S.lastEye = {}; S.lastRight = {}; S.lastUp = {}; S.lastFwd = {};
        S.haveLastEye = false;
        S.proxy = 0; S.proxyPos = {}; S.proxyValid = false;
        S.estInit = false; S.estD = 6.3f;
        S.estPlayer = {}; S.estVel = {};
        S.havePlayer = false; S.playerPos = {}; S.lastPlayerPos = {};
        S.vel = {}; S.fwd = {}; S.haveFwd = false;
        S.blend = 0.0f; S.wProxy = 0.0f;
        S.lastWrittenValid = false;
    }

    void OnDeviceCreated(uint32_t device)
    {
        S.device = device;
    }

    void OnGetViewport(void* object)
    {
        if (object && g_memory.IsInMemoryRange(object))
            S.application = g_memory.MapVirtual(object);
    }

    void OnUpdateDirector(uint32_t director, uint8_t* base)
    {
        if (director)
        {
            if (S.director != director)
            {
                S.director = director;
                S.listFound = false;
            }

            if (!S.listFound && App::s_time - S.lastListScan >= 2.0)
            {
                S.lastListScan = App::s_time;
                TryDiscoverUpdateList(director, base);
            }
        }
    }

    void OnCameraUpdateSerialPre(uint32_t, uint8_t* base)
    {
        S.snapshotValid = false;
        if (Config::CameraMode != ECameraMode::Head || !S.device)
        {
            return;
        }

        auto device = reinterpret_cast<GuestDevice*>(base + S.device);
        memcpy(S.vsSnapshot, device->vertexShaderFloatConstants, sizeof(S.vsSnapshot));
        S.snapshotValid = true;
    }

    void OnCameraUpdateSerial(uint32_t camera, uint8_t* base)
    {
        ApplyReset();

        if (Config::CameraMode != ECameraMode::Head)
        {
            S.blend = 0.0f; // the original camera owns the matrix again
            return;
        }
        if (!S.device || !S.snapshotValid)
        {
            return;
        }

        auto device = reinterpret_cast<GuestDevice*>(base + S.device);
        uint32_t* vsBase = device->vertexShaderFloatConstants;

        // The world map keeps its own camera.
        if (PPC_LOAD_U32(camera) == WORLD_MAP_CAMERA_VTABLE)
        {
            return;
        }

        double dt = std::clamp(App::s_deltaTime, 1e-4, 1.0 / 15.0);
        S.frameCount++;

        // --- Which floats did the camera just write? ---
        // Indices are float (u32) offsets into the 1024-float constant block.
        std::vector<int> changed;
        changed.reserve(32);
        for (int i = 0; i < 1024; i++)
            if (vsBase[i] != S.vsSnapshot[i])
                changed.push_back(i);

        auto readBlock = [&](int floatIndex, float* out16)
        {
            const be<float>* src = reinterpret_cast<const be<float>*>(vsBase + floatIndex);
            for (int i = 0; i < 16; i++)
                out16[i] = src[i].get();
        };

        // --- Scan the CCamera object for its world transform. This yields
        // the true view matrix V, which anchors the register calibration. ---
        Mat4 vObj = Mat4::Identity();
        Vec3 objEye{}, objRight{}, objUp{}, objFwd{};
        bool needObjScan = S.viewFloat < 0 ||
            (!CloseToIdentity(S.projectionPart, 1e-3f) && (S.frameCount & 31) == 0) ||
            (S.frameCount & 15) == 0;
        bool haveObj = needObjScan && ScanCameraObject(base, camera, S.rowMajor, vObj, objEye, objRight, objUp, objFwd);

        // --- Calibration: locate the 16 floats the camera serializes. The
        // block is either the view matrix V or V * projection; match it
        // algebraically against the object scan (or, without one, look for a
        // block that is itself a pure view). ---
        Mat4 vOrig = Mat4::Identity();
        Vec3 poseEye{}, poseRight{}, poseUp{}, poseFwd{};
        bool havePose = false;

        if (S.viewFloat < 0)
        {
            // Candidate block starts: anywhere the camera just wrote, plus a
            // full sweep once in a while in case parts of the matrix are
            // unchanged (e.g. zero entries).
            bool sweep = (S.frameCount % 30) == 0;
            for (int f = 0; f + 16 <= 1024; f += 4)
            {
                if (!sweep)
                {
                    bool touched = false;
                    for (int c : changed)
                        if ((c & ~3) == f)
                        {
                            touched = true;
                            break;
                        }
                    if (!touched)
                        continue;
                }

                float cb[16];
                readBlock(f, cb);
                for (int li = 0; li < 2; li++)
                {
                    bool rm = li == 0;
                    Mat4 bm = ToMat4(cb, rm);


                    bool hit = false;
                    if (haveObj)
                    {
                        if (CloseMat(bm, vObj, 2e-3f))
                        {
                            // The block is the bare view matrix.
                            S.projectionPart = Mat4::Identity();
                            hit = true;
                        }
                        else
                        {
                            Mat4 invV = InvertRigid(vObj);
                            // Block is either P*V or V*P; try both.
                            Mat4 pAfter = Mul(bm, invV);    // P when block = P*V
                            if (LooksLikeProjectionPart(pAfter, 2e-3f))
                            {
                                S.projectionPart = pAfter;
                                S.projBeforeView = true;
                                hit = true;
                            }
                            else
                            {
                                Mat4 pBefore = Mul(invV, bm); // P when block = V*P
                                if (LooksLikeProjectionPart(pBefore, 2e-3f))
                                {
                                    S.projectionPart = pBefore;
                                    S.projBeforeView = false;
                                    hit = true;
                                }
                            }
                        }
                    }
                    else
                    {
                        Vec3 e, rgt, up, fwd;
                        if (TryDecomposeView(bm, e, rgt, up, fwd))
                        {
                            S.projectionPart = Mat4::Identity();
                            vOrig = bm;
                            poseEye = e;
                            poseRight = rgt;
                            poseUp = up;
                            poseFwd = fwd;
                            hit = true;
                        }
                    }
                    if (hit)
                    {
                        S.viewFloat = f;
                        S.rowMajor = rm;
                        if (haveObj)
                        {
                            vOrig = vObj;
                            poseEye = objEye;
                            poseRight = objRight;
                            poseUp = objUp;
                            poseFwd = objFwd;
                        }
                        havePose = true;
                        break;
                    }
                }
                if (S.viewFloat >= 0)
                    break;
            }
        }

        // --- This frame's original camera pose. The calibrated block is the
        // ground truth (it defines the rendered camera); the object scan and
        // the previous frame's pose cover gaps. ---
        if (S.viewFloat >= 0)
        {
            float block[16];
            readBlock(S.viewFloat, block);

            bool isOurs = S.lastWrittenValid;
            if (isOurs)
                for (int i = 0; i < 16; i++)
                    if (block[i] != S.lastWritten[i])
                    {
                        isOurs = false;
                        break;
                    }

            if (!isOurs)
            {
                Mat4 reg = ToMat4(block, S.rowMajor);
                bool ok = false;
                if (CloseToIdentity(S.projectionPart, 1e-3f))
                {
                    ok = TryDecomposeView(reg, poseEye, poseRight, poseUp, poseFwd);
                    if (ok)
                        vOrig = reg;
                }
                else
                {
                    Mat4 invP;
                    if (Mat4Invert(S.projectionPart, invP))
                    {
                        // Isolate the view: block is P*V or V*P.
                        vOrig = S.projBeforeView ? Mul(invP, reg) : Mul(reg, invP);
                        ok = TryDecomposeView(vOrig, poseEye, poseRight, poseUp, poseFwd);
                    }
                }
                if (ok)
                    havePose = true;

                // Keep the projection part current (FOV cuts change it).
                if (haveObj)
                {
                    Mat4 invV = InvertRigid(vObj);
                    Mat4 pNew = S.projBeforeView ? Mul(reg, invV) : Mul(invV, reg);
                    if (LooksLikeProjectionPart(pNew, 2e-3f))
                        S.projectionPart = pNew;
                }
            }
        }

        if (!havePose && haveObj)
        {
            vOrig = vObj;
            poseEye = objEye;
            poseRight = objRight;
            poseUp = objUp;
            poseFwd = objFwd;
            havePose = true;
        }
        if (!havePose && S.havePose)
        {
            // Neither source updated this frame: hold the last pose.
            vOrig = S.lastView;
            poseEye = S.lastEye;
            poseRight = S.lastRight;
            poseUp = S.lastUp;
            poseFwd = S.lastFwd;
            havePose = true;
        }

        if (!havePose)
        {
            if (!S.loggedFailure && changed.size() >= 16)
            {
                S.loggedFailure = true;
                LOGFN_WARNING("HeadCam: could not recover the camera matrix yet; the feature stays off until it calibrates");
            }
            return;
        }

        // --- Pose bookkeeping ---
        if (S.haveLastEye && Length(poseEye - S.lastEye) > 50.0f)
        {
            // Teleport (stage transition / respawn): drop transient state so
            // the camera doesn't lerp across the map.
            S.estInit = false;
            S.havePlayer = false;
            S.haveFwd = false;
            S.vel = {};
            S.lastPlayerPos = {};
        }
        S.lastEye = poseEye;
        S.lastRight = poseRight;
        S.lastUp = poseUp;
        S.lastFwd = poseFwd;
        S.haveLastEye = true;
        S.lastView = vOrig;
        S.havePose = true;

        if (S.viewFloat >= 0 && !S.loggedCalibration)
        {
            S.loggedCalibration = true;
            LOGFN("HeadCam: calibrated (float={}, rowMajor={}, eye=({}, {}, {}))",
                S.viewFloat, S.rowMajor ? 1 : 0, poseEye.x, poseEye.y, poseEye.z);
        }

        // --- Player position: character proxy from the update list ---
        CollectProxies(base, poseEye, poseFwd);

        // --- Player position: camera-geometry estimator (fallback) ---
        // A follow camera looks at the player, so the player sits on the
        // sight line a few meters ahead: player ~= eye + D * camFwd. Tracking
        // only the sight-line distance D (not a vertical offset) keeps the
        // estimate stable when the camera rolls with a spin jump — the roll
        // spins camUp but not camFwd, so the player stays put.
        if (!S.estInit || (S.haveLastEye && Length(poseEye - S.estLastEye) > 50.0f))
        {
            S.estInit = true;
            S.estD = 6.3f; // typical follow distance
            S.estVel = {};
            S.estPlayer = poseEye + poseFwd * S.estD;
            S.estLastEye = poseEye;
        }
        else
        {
            Vec3 playerNew = poseEye + poseFwd * S.estD;
            if (Length(playerNew - S.estPlayer) < 4.0f)
            {
                S.estVel = S.estVel * 0.7f + (playerNew - S.estPlayer) * (0.3f / dt);
                S.estPlayer = playerNew;
            }
            S.estLastEye = poseEye;
        }


        // --- Fuse sources ---
        S.wProxy += ((S.proxyValid ? 1.0f : 0.0f) - S.wProxy) * (1.0f - std::exp(-dt / 0.15));

        bool havePlayer = false;
        Vec3 playerPos{};
        if (S.proxyValid)
        {
            playerPos = S.estInit ? S.proxyPos * S.wProxy + S.estPlayer * (1.0f - S.wProxy) : S.proxyPos;
            havePlayer = true;
        }
        else if (S.estInit)
        {
            playerPos = S.estPlayer;
            havePlayer = true;
        }
        S.havePlayer = havePlayer;

        if (havePlayer)
        {
            Vec3 rawVel = (playerPos - S.lastPlayerPos) * (1.0f / dt);
            if (Length(rawVel) < 50.0f)
                S.vel = S.vel * 0.75f + rawVel * 0.25f;
            S.playerPos = playerPos;
        }
        S.lastPlayerPos = playerPos;

        // --- Facing: yaw from the horizontal velocity only. The character's
        // in-air spin is intentionally ignored, so a jump moves the camera
        // without spinning the world. ---
        // World up (Hedgehog is Y-up). Using a fixed horizon instead of the
        // follow camera's up keeps the view level when the camera rolls with
        // a spin jump — the jump reads as a jump, not a spinning world.
        Vec3 up{ 0.0f, 1.0f, 0.0f };
        Vec3 horiz = S.vel - up * Dot(S.vel, up);
        if (Length(horiz) > VELOCITY_THRESHOLD)
        {
            Vec3 target = Normalize(horiz);
            if (!S.haveFwd)
                S.fwd = target;
            else
                S.fwd = Normalize(S.fwd + (target - S.fwd) * (1.0f - std::exp(-dt / YAW_TAU)));
            S.haveFwd = true;
        }
        else if (!S.haveFwd)
        {
            S.fwd = Normalize(poseFwd - up * Dot(poseFwd, up));
            S.haveFwd = Length(S.fwd) > 0.5f;
        }

        // --- Ramp the blend toward the head cam ---
        float blendTarget = (havePlayer && S.haveFwd) ? 1.0f : 0.0f;
        float tau = S.blend < blendTarget ? BLEND_TAU_IN : BLEND_TAU_OUT;
        S.blend += (blendTarget - S.blend) * (1.0f - std::exp(-dt / tau));
        if (S.blend < 0.01f || S.viewFloat < 0)
            return;

        // --- Build the head cam view ---
        Vec3 fwd = Normalize(S.fwd - up * Dot(S.fwd, up));
        Vec3 right = Normalize(Cross(up, fwd));
        if (Length(right) < 0.5f)
            return;

        float pitch = HEAD_PITCH_DEG * 3.14159265358979f / 180.0f;
        float sp = std::sin(pitch), cp = std::cos(pitch);
        Vec3 fwdPitched = fwd * cp - up * sp;
        Vec3 upPitched = up * cp + fwd * sp;

        Vec3 eye = S.playerPos + up * HEAD_HEIGHT + fwd * HEAD_FORWARD;

        Mat4 vHead;
        BuildViewMat(vHead, eye, right, upPitched, fwdPitched);

        // Smoothly blend original and head poses, then re-apply the
        // projection part so the FOV stays whatever the game set.
        Quat q0 = QuatFromBasis(poseRight, poseUp, poseFwd);
        Quat q1 = QuatFromBasis(right, upPitched, fwdPitched);
        Quat q = QuatSlerp(q0, q1, S.blend);

        Mat4 vBlend;
        BuildViewMat(vBlend,
            poseEye * (1.0f - S.blend) + eye * S.blend,
            Normalize(QuatToBasisRight(q)),
            Normalize(QuatToBasisUp(q)),
            Normalize(QuatToBasisForward(q)));

        // Re-serialize in the same convention the engine uses.
        Mat4 finalMat = S.projBeforeView ? Mul(S.projectionPart, vBlend) : Mul(vBlend, S.projectionPart);

        // --- Write into the vertex shader constants ---
        int f0 = S.viewFloat;
        if (f0 < 0 || f0 + 16 > 1024)
            return;
        float mem[16];
        Mat4ToMem(mem, S.rowMajor, finalMat);
        auto* dst = reinterpret_cast<be<float>*>(vsBase + f0);
        for (int i = 0; i < 16; i++)
            dst[i] = mem[i];

        // Remember what we wrote so next frame we can tell whether the game
        // actually updated the register (or we are looking at our own value).
        memcpy(S.lastWritten, mem, sizeof(S.lastWritten));
        S.lastWrittenValid = true;

        // One dirty-flag bit covers 16 floats; mark the groups we touched.
        device->dirtyFlags[0] = device->dirtyFlags[0].get() |
            (0x1ULL << (f0 / 16)) | (0x1ULL << ((f0 + 15) / 16));
    }
}
