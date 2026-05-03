#pragma once

// Spatial vector algebra for Featherstone articulated-body dynamics.
//
// Convention (Featherstone RBDA 2008, §2):
//   SpatialVec stores [angular; linear] — angular part first.
//   For motion vectors: ang = ω, lin = v at the reference point.
//   For force  vectors: ang = τ (torque / couple), lin = f (force).
//
// SpatialInertia is stored as four 3×3 blocks of a symmetric 6×6 matrix:
//
//   [ Ioo     mCross ]
//   [ mCross^T  m*I3 ]
//
// where  Ioo   = rotational inertia about the reference point (parallel-axis shifted from COM)
//        mCross = m * Skew(r),   r = comWorld − refPointWorld
//        mass   = scalar mass
//
// No Mat6 type is used — all operations are expressed with the existing Vec3 and Mat3 primitives.

#include "minphys3d/math/mat3.hpp"
#include "minphys3d/math/vec3.hpp"

namespace minphys3d {

// ---------------------------------------------------------------------------
// SpatialVec — 6D spatial motion or force vector
// ---------------------------------------------------------------------------

struct SpatialVec {
    Vec3 ang{};   // angular component: ω (velocity) or τ (torque)
    Vec3 lin{};   // linear  component: v (velocity at ref) or f (force)
};

inline SpatialVec operator+(const SpatialVec& a, const SpatialVec& b) {
    return {a.ang + b.ang, a.lin + b.lin};
}
inline SpatialVec operator-(const SpatialVec& a, const SpatialVec& b) {
    return {a.ang - b.ang, a.lin - b.lin};
}
inline SpatialVec& operator+=(SpatialVec& a, const SpatialVec& b) {
    a.ang += b.ang;
    a.lin += b.lin;
    return a;
}
inline SpatialVec operator*(float s, const SpatialVec& v) {
    return {s * v.ang, s * v.lin};
}

// ---------------------------------------------------------------------------
// SpatialInertia — 6×6 symmetric spatial inertia (block-3×3 storage)
// ---------------------------------------------------------------------------

struct SpatialInertia {
    Mat3  Ioo{};      // upper-left  3×3: rotational inertia about reference point
    Mat3  mCross{};   // upper-right 3×3: m * Skew(r),  r = COM − ref
                      // lower-left  3×3: Transpose(mCross)  (implicit)
    float mass = 0.0f;
};

inline SpatialInertia operator+(const SpatialInertia& A, const SpatialInertia& B) {
    return {A.Ioo + B.Ioo, A.mCross + B.mCross, A.mass + B.mass};
}
inline SpatialInertia& operator+=(SpatialInertia& A, const SpatialInertia& B) {
    A.Ioo    = A.Ioo    + B.Ioo;
    A.mCross = A.mCross + B.mCross;
    A.mass   = A.mass   + B.mass;
    return A;
}

// ---------------------------------------------------------------------------
// Skew — skew-symmetric matrix of v; satisfies Skew(v)*u == Cross(v, u)
// ---------------------------------------------------------------------------

inline Mat3 Skew(const Vec3& v) {
    Mat3 m{};
    m.m[0][1] = -v.z;  m.m[0][2] =  v.y;
    m.m[1][0] =  v.z;  m.m[1][2] = -v.x;
    m.m[2][0] = -v.y;  m.m[2][1] =  v.x;
    return m;
}

// ---------------------------------------------------------------------------
// SpatialMul — multiply spatial inertia by a spatial vector: f = I * v
// ---------------------------------------------------------------------------

// For a revolute joint with world-space axis s (ang part only, lin = 0):
//   result.ang = Ioo * s          (angular momentum contribution)
//   result.lin = Transpose(mCross) * s  (linear momentum contribution)
inline SpatialVec SpatialMul(const SpatialInertia& I, const SpatialVec& v) {
    return {
        I.Ioo * v.ang + I.mCross * v.lin,
        Transpose(I.mCross) * v.ang + I.mass * v.lin,
    };
}

// ---------------------------------------------------------------------------
// SpatialInertiaFromBody
//   Construct spatial inertia of a rigid body about a reference point in world
//   space using the parallel-axis theorem.
//
//   Iworld : 3×3 forward rotational inertia about the body's COM in world space.
//             (NOT the inverse inertia stored in Body — caller must provide the forward form.)
//   comWorld  : world-space position of the body's center of mass.
//   refWorld  : world-space reference point (typically the joint anchor).
// ---------------------------------------------------------------------------

inline SpatialInertia SpatialInertiaFromBody(
    float mass, const Mat3& Iworld, const Vec3& comWorld, const Vec3& refWorld)
{
    const Vec3  r     = comWorld - refWorld;
    const float rLen2 = Dot(r, r);
    // Parallel-axis theorem: I_ref = I_com + m*(|r|^2*I3 - outer(r,r))
    const Mat3  shiftedI = Iworld + mass * (ScaleIdentity(rLen2) - OuterProduct(r, r));
    const Mat3  mX       = mass * Skew(r);
    return {shiftedI, mX, mass};
}

// ---------------------------------------------------------------------------
// SpatialBias
//   Compute the spatial Coriolis / centrifugal bias force for a body with
//   spatial velocity vel and articulated spatial inertia Ia.
//   Returns p = v ×* (Ia * v), the "force bias" used in ABA pass 1.
//
//   Using the force cross-product operator (Featherstone RBDA eq. 2.24):
//     p.ang = Cross(ω, h.ang) + Cross(v_lin, h.lin)
//     p.lin = Cross(ω, h.lin)
//   where h = Ia * vel (spatial momentum).
// ---------------------------------------------------------------------------

inline SpatialVec SpatialBias(const SpatialInertia& Ia, const SpatialVec& vel) {
    const SpatialVec h = SpatialMul(Ia, vel);
    return {
        Cross(vel.ang, h.ang) + Cross(vel.lin, h.lin),
        Cross(vel.ang, h.lin),
    };
}

// ---------------------------------------------------------------------------
// PropagateInertia
//   Reduce child ABI Ic through a 1-DOF revolute joint with world-space axis s.
//   For a revolute joint the motion subspace is S = [s; 0] (angular only).
//
//   Outputs D = s^T * Ic * s (scalar effective mass at the joint; used to
//   compute 1/D as the body-B angular compliance for PGS invDenomHinge).
//
//   Returns the "apparent inertia" seen at the parent after factoring out
//   this 1-DOF (Schur complement reduction):
//     Ia_reduced = Ic − (Ic·S)(Ic·S)^T / D
//
//   Returns a zero SpatialInertia if D < kEpsilon (degenerate joint).
// ---------------------------------------------------------------------------

inline SpatialInertia PropagateInertia(const SpatialInertia& Ic, const Vec3& s, float& D) {
    // Is = Ic * S = Ic * [s; 0]
    //   ang component: Ioo * s
    //   lin component: Transpose(mCross) * s
    const Vec3 IsAng = Ic.Ioo * s;
    const Vec3 IsLin = Transpose(Ic.mCross) * s;
    D = Dot(s, IsAng);  // s^T * Ic * s  (scalar effective mass)
    if (!(D > kEpsilon)) {
        return {};
    }
    const float invD = 1.0f / D;
    // Ia_reduced = Ic - (Ic*S)(Ic*S)^T / D
    SpatialInertia r = Ic;
    r.Ioo    = r.Ioo    - invD * OuterProduct(IsAng, IsAng);
    r.mCross = r.mCross - invD * OuterProduct(IsAng, IsLin);
    r.mass   = r.mass   - invD * Dot(IsLin, IsLin);
    return r;
}

// ---------------------------------------------------------------------------
// PropagateBias
//   Propagate the child bias force through a 1-DOF revolute joint to the
//   parent, producing the increment to add to the parent's Pa.
//
//   Pc   : child's articulated bias force (pa_child).
//   Ic   : child's full (pre-reduction) articulated spatial inertia.
//   s    : world-space revolute axis.
//   D    : s^T * Ic * s (effective mass at joint, from PropagateInertia).
//   u    : generalised force at joint = tau_applied − s^T * Pc.ang.
//
//   For a revolute joint (Featherstone RBDA eq. 7.42):
//     ΔPa_parent = Pc + (Ic*S) * ((u − s^T * Pc) / D)
// ---------------------------------------------------------------------------

inline SpatialVec PropagateBias(
    const SpatialVec& Pc, const SpatialInertia& Ic,
    const Vec3& s, float D, float u)
{
    if (!(D > kEpsilon)) return {};
    const float sPc   = Dot(s, Pc.ang);         // s^T * Pc  (revolute: angular only)
    const float coeff = (u - sPc) / D;
    const Vec3  IsAng = Ic.Ioo * s;
    const Vec3  IsLin = Transpose(Ic.mCross) * s;
    return {
        Pc.ang + IsAng * coeff,
        Pc.lin + IsLin * coeff,
    };
}

// ---------------------------------------------------------------------------
// SpatialInertiaTransport
//   Move the reference point of a rigid-body spatial inertia from A to B,
//   where offset = B − A in world space.
//
//   Only correct for RIGID-BODY inertias (before any PropagateInertia reduction).
//   For already-reduced (articulated) inertias this gives an approximation;
//   the exact expression would require tracking the COM offset separately.
// ---------------------------------------------------------------------------

inline SpatialInertia SpatialInertiaTransport(const SpatialInertia& Ia, const Vec3& offset) {
    // r_new = r_old - offset  (COM offset from new reference point)
    // mCross_new = m * Skew(r_new) = mCross - m * Skew(offset)
    const Mat3 dX = Ia.mass * Skew(offset);
    const Mat3 mCrossNew = Ia.mCross - dX;
    // Ioo_new = I_com + m*(|r_new|^2 * I - outer(r_new, r_new))
    //         = Ioo_old  +  r_old× * (−dX)^T − dX * r_old×^T  + (−dX) * (−dX)^T / m
    // Simplified via standard parallel-axis:
    //   Ioo_new = Ioo_old − Skew(offset)*Transpose(mCross) + mCross*Skew(offset)^T
    //             − m * Skew(offset) * Skew(offset)^T
    const Mat3 dXSkew    = Skew(offset);
    const float off2     = Dot(offset, offset);
    const Mat3 IooNew    = Ia.Ioo
                          - dXSkew * Transpose(Ia.mCross)
                          + Ia.mCross * Transpose(dXSkew)
                          - Ia.mass * (ScaleIdentity(off2) - OuterProduct(offset, offset));
    return {IooNew, mCrossNew, Ia.mass};
}

} // namespace minphys3d
