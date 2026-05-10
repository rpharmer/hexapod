#pragma once

#include <cmath>
#include <limits>
#include <utility>

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
inline SpatialVec operator*(Real s, const SpatialVec& v) {
    return {s * v.ang, s * v.lin};
}

// ---------------------------------------------------------------------------
// SpatialInertia — 6×6 symmetric spatial inertia (block-3×3 storage)
// ---------------------------------------------------------------------------

struct SpatialInertia {
    Mat3  Ioo{};      // upper-left  3×3: rotational inertia about reference point
    Mat3  mCross{};   // upper-right 3×3: m * Skew(r),  r = COM − ref
                      // lower-left  3×3: Transpose(mCross)  (implicit)
    Real mass = 0.0;
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
    Real mass, const Mat3& Iworld, const Vec3& comWorld, const Vec3& refWorld)
{
    const Vec3  r     = comWorld - refWorld;
    const Real rLen2 = Dot(r, r);
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

inline SpatialInertia PropagateInertiaFromProjection(
    const SpatialInertia& Ic, const Vec3& IsAng, const Vec3& IsLin, Real D)
{
    if (!(D > kEpsilon)) {
        return {};
    }
    const Real invD = 1.0 / D;
    SpatialInertia r = Ic;
    r.Ioo    = r.Ioo    - invD * OuterProduct(IsAng, IsAng);
    r.mCross = r.mCross - invD * OuterProduct(IsAng, IsLin);
    r.mass   = r.mass   - invD * Dot(IsLin, IsLin);
    return r;
}

// Extended form: also returns IsAng = Ic.Ioo*s and IsLin = Transpose(Ic.mCross)*s so the
// caller can pass them to PropagateBias without recomputing them (saves 2 Mat3*Vec3/joint).
inline SpatialInertia PropagateInertia(
    const SpatialInertia& Ic, const Vec3& s, Real& D,
    Vec3& IsAngOut, Vec3& IsLinOut)
{
    // Is = Ic * S = Ic * [s; 0]
    //   ang component: Ioo * s
    //   lin component: Transpose(mCross) * s
    const Vec3 IsAng = Ic.Ioo * s;
    const Vec3 IsLin = Transpose(Ic.mCross) * s;
    IsAngOut = IsAng;
    IsLinOut = IsLin;
    D = Dot(s, IsAng);  // s^T * Ic * s  (scalar effective mass)
    return PropagateInertiaFromProjection(Ic, IsAng, IsLin, D);
}

// Backward-compatible 3-parameter overload (does not return Is vectors).
inline SpatialInertia PropagateInertia(const SpatialInertia& Ic, const Vec3& s, Real& D) {
    Vec3 dum1, dum2;
    return PropagateInertia(Ic, s, D, dum1, dum2);
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
    const Vec3& s, Real D, Real u)
{
    if (!(D > kEpsilon)) return {};
    const Real sPc   = Dot(s, Pc.ang);         // s^T * Pc  (revolute: angular only)
    const Real coeff = (u - sPc) / D;
    const Vec3  IsAng = Ic.Ioo * s;
    const Vec3  IsLin = Transpose(Ic.mCross) * s;
    return {
        Pc.ang + IsAng * coeff,
        Pc.lin + IsLin * coeff,
    };
}

// Overload accepting pre-computed Is vectors from PropagateInertia, avoiding 2 Mat3*Vec3
// products per joint that would otherwise be redundant (Ic and s are the same pair).
inline SpatialVec PropagateBias(
    const SpatialVec& Pc,
    const Vec3& IsAng, const Vec3& IsLin,  // from PropagateInertia(..., IsAngOut, IsLinOut)
    const Vec3& s, Real D, Real u)
{
    if (!(D > kEpsilon)) return {};
    const Real coeff = (u - Dot(s, Pc.ang)) / D;
    return {Pc.ang + IsAng * coeff, Pc.lin + IsLin * coeff};
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
    const Real off2     = Dot(offset, offset);
    const Mat3 IooNew    = Ia.Ioo
                          - dXSkew * Transpose(Ia.mCross)
                          + Ia.mCross * Transpose(dXSkew)
                          - Ia.mass * (ScaleIdentity(off2) - OuterProduct(offset, offset));
    return {IooNew, mCrossNew, Ia.mass};
}

// ---------------------------------------------------------------------------
// SpatialInertiaPluckerTranslateChildToParent
//   Exact Plücker-coordinate change of spatial inertia from the child joint
//   anchor to the parent joint anchor when both frames share the same world
//   orientation.  r = parentAnchor − childAnchor (world).
//
//   Motion map v_parent = X * v_child with X = [I, 0; −Skew(r), I], so
//   v_parent_lin = v_child_lin − Skew(r) * ω.
//   Then I_parent = X^{-T} * I_child * X^{-1} with X^{-1} = [I, 0; Skew(r), I].
//
//   Valid for any symmetric spatial inertia (including articulated reduced
//   tensors).  This does **not** numerically match `SpatialInertiaTransport`
//   on the same offset: transport applies parallel-axis shifts to the stacked
//   `{Ioo, mCross, mass}` representation, while this transform is the Plücker
//   congruence that preserves kinetic energy with `v_parent = X v_child`.
// ---------------------------------------------------------------------------

inline SpatialInertia SpatialInertiaPluckerTranslateChildToParent(
    const SpatialInertia& I,
    const Vec3& r)  // r = parentAnchor − childAnchor (world space)
{
    // Plücker congruence I_parent = X^{-T} I X^{-1} with X^{-1} = [I3 0; Skew(r) I3].
    // Block-analytic derivation (no 6×6 matrix needed, ~120 ops vs ~1620 for Mat6Mul):
    //   mCross_new = mCross − mass · Skew(r)
    //   Ioo_new    = Ioo + mCross·Skew(r) − Skew(r)·mCross^T
    //                    + mass · (|r|²·I3 − r⊗r)
    //   mass_new   = mass  (unchanged)
    // The last term is the standard parallel-axis shift (same as SpatialInertiaFromBody).
    const Mat3 Sr        = Skew(r);
    const Mat3 mCrossNew = I.mCross - I.mass * Sr;
    const Mat3 IooNew    = I.Ioo
                         + I.mCross * Sr
                         - Sr * Transpose(I.mCross)
                         + I.mass * (ScaleIdentity(Dot(r, r)) - OuterProduct(r, r));
    return {IooNew, mCrossNew, I.mass};
}

// ---------------------------------------------------------------------------
// SpatialMotionCompliance
//   Computes h^T * inv(I) * h for a symmetric positive-definite spatial inertia
//   and motion axis h = [ω; v] (same ordering as SpatialVec).  Used for
//   frictionless contact effective mass when replacing rigid-body invMass +
//   angular quadratic form with articulated inertia at a reference point.
//
//   Returns +infinity if the 6×6 system is singular / ill-conditioned.
// ---------------------------------------------------------------------------

inline Real DotSpatial(const SpatialVec& a, const SpatialVec& b) {
    return Dot(a.ang, b.ang) + Dot(a.lin, b.lin);
}

// ---------------------------------------------------------------------------
// SpatialForceTranslateByOffset
//   Shift a spatial force (wrench) reference from point A to point B with the
//   same world orientation (pure translation).  offset = ref_B − ref_A.
//   Moment update: n_B = n_A + offset × f  (Varignon).
// ---------------------------------------------------------------------------

inline SpatialVec SpatialForceTranslateByOffset(const SpatialVec& f, const Vec3& offsetRefBMinusRefA) {
    return {f.ang + Cross(offsetRefBMinusRefA, f.lin), f.lin};
}

struct SpatialSolveDiagnostics {
    bool regularized = false;
    bool failed = false;
    double minPivotAbs = std::numeric_limits<double>::infinity();
};

namespace spatial_internal {

constexpr double kSpatialSolveDiagRidge = 1.0e-10;
constexpr double kSpatialSolvePivotEps = 1.0e-12;

inline void AssembleSpatialSolveSystem(const SpatialInertia& I, double M[6][6]) {
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            M[r][c] = 0.0;
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            M[i][j] = static_cast<double>(I.Ioo.m[i][j]);
            M[i][j + 3] = static_cast<double>(I.mCross.m[i][j]);
        }
    }
    const Mat3 mCt = Transpose(I.mCross);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            M[i + 3][j] = static_cast<double>(mCt.m[i][j]);
            M[i + 3][j + 3] = (i == j) ? static_cast<double>(I.mass) : 0.0;
        }
    }
}

inline bool FactorSpatialSolveSystem(
    const SpatialInertia& I,
    double L[6][6],
    double D[6],
    int perm[6],
    SpatialSolveDiagnostics* diagnostics)
{
    double A[6][6]{};
    AssembleSpatialSolveSystem(I, A);
    for (int i = 0; i < 6; ++i) {
        A[i][i] += kSpatialSolveDiagRidge;
        perm[i] = i;
        D[i] = 0.0;
        for (int j = 0; j < 6; ++j) {
            L[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    if (diagnostics) {
        diagnostics->regularized = true;
        diagnostics->failed = false;
        diagnostics->minPivotAbs = std::numeric_limits<double>::infinity();
    }

    for (int k = 0; k < 6; ++k) {
        int pivot = k;
        double best = std::abs(A[k][k]);
        for (int i = k + 1; i < 6; ++i) {
            const double candidate = std::abs(A[i][i]);
            if (candidate > best) {
                best = candidate;
                pivot = i;
            }
        }
        if (!std::isfinite(best) || best <= kSpatialSolvePivotEps) {
            if (diagnostics) {
                diagnostics->failed = true;
                diagnostics->minPivotAbs = std::min(diagnostics->minPivotAbs, best);
            }
            return false;
        }
        if (pivot != k) {
            for (int c = 0; c < 6; ++c) {
                std::swap(A[k][c], A[pivot][c]);
            }
            for (int r = 0; r < 6; ++r) {
                std::swap(A[r][k], A[r][pivot]);
            }
            for (int r = 0; r < k; ++r) {
                std::swap(L[k][r], L[pivot][r]);
            }
            std::swap(perm[k], perm[pivot]);
        }

        const double diag = A[k][k];
        const double absDiag = std::abs(diag);
        if (!std::isfinite(diag) || absDiag <= kSpatialSolvePivotEps) {
            if (diagnostics) {
                diagnostics->failed = true;
                diagnostics->minPivotAbs = std::min(diagnostics->minPivotAbs, absDiag);
            }
            return false;
        }
        D[k] = diag;
        if (diagnostics) {
            diagnostics->minPivotAbs = std::min(diagnostics->minPivotAbs, absDiag);
        }

        for (int i = k + 1; i < 6; ++i) {
            const double lik = A[i][k] / diag;
            if (!std::isfinite(lik)) {
                if (diagnostics) {
                    diagnostics->failed = true;
                }
                return false;
            }
            L[i][k] = lik;
            for (int j = k + 1; j <= i; ++j) {
                A[i][j] -= lik * diag * L[j][k];
                A[j][i] = A[i][j];
            }
        }
    }

    if (diagnostics && !std::isfinite(diagnostics->minPivotAbs)) {
        diagnostics->failed = true;
        return false;
    }
    return true;
}

inline bool SolveSpatialFactorized(
    const double L[6][6],
    const double D[6],
    const int perm[6],
    const SpatialVec& rhs,
    SpatialVec& out)
{
    double rhsVec[6] = {
        static_cast<double>(rhs.ang.x),
        static_cast<double>(rhs.ang.y),
        static_cast<double>(rhs.ang.z),
        static_cast<double>(rhs.lin.x),
        static_cast<double>(rhs.lin.y),
        static_cast<double>(rhs.lin.z),
    };
    double b[6]{};
    for (int i = 0; i < 6; ++i) {
        b[i] = rhsVec[perm[i]];
    }

    double y[6]{};
    for (int i = 0; i < 6; ++i) {
        double sum = b[i];
        for (int j = 0; j < i; ++j) {
            sum -= L[i][j] * y[j];
        }
        y[i] = sum;
    }

    double z[6]{};
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(D[i]) || std::abs(D[i]) <= kSpatialSolvePivotEps) {
            return false;
        }
        z[i] = y[i] / D[i];
    }

    double xPerm[6]{};
    for (int i = 5; i >= 0; --i) {
        double sum = z[i];
        for (int j = i + 1; j < 6; ++j) {
            sum -= L[j][i] * xPerm[j];
        }
        xPerm[i] = sum;
    }

    double x[6]{};
    for (int i = 0; i < 6; ++i) {
        x[perm[i]] = xPerm[i];
    }
    for (double value : x) {
        if (!std::isfinite(value)) {
            return false;
        }
    }

    out.ang = {
        static_cast<Real>(x[0]),
        static_cast<Real>(x[1]),
        static_cast<Real>(x[2]),
    };
    out.lin = {
        static_cast<Real>(x[3]),
        static_cast<Real>(x[4]),
        static_cast<Real>(x[5]),
    };
    return std::isfinite(out.ang.x) && std::isfinite(out.ang.y) && std::isfinite(out.ang.z)
        && std::isfinite(out.lin.x) && std::isfinite(out.lin.y) && std::isfinite(out.lin.z);
}

inline bool SpatialInertiaSolveMotionDetailed(
    const SpatialInertia& I,
    const SpatialVec& rhs,
    SpatialVec& out,
    SpatialSolveDiagnostics* diagnostics)
{
    double L[6][6]{};
    double D[6]{};
    int perm[6]{};
    if (!FactorSpatialSolveSystem(I, L, D, perm, diagnostics)) {
        return false;
    }
    const bool ok = SolveSpatialFactorized(L, D, perm, rhs, out);
    if (diagnostics && !ok) {
        diagnostics->failed = true;
    }
    return ok;
}

inline Real SpatialMotionComplianceDetailed(
    const SpatialInertia& I,
    const SpatialVec& h,
    SpatialSolveDiagnostics* diagnostics)
{
    SpatialVec x{};
    if (!SpatialInertiaSolveMotionDetailed(I, h, x, diagnostics)) {
        return std::numeric_limits<Real>::infinity();
    }
    const Real compliance = DotSpatial(h, x);
    if (!std::isfinite(compliance) || !(compliance > 0.0)) {
        if (diagnostics) {
            diagnostics->failed = true;
        }
        return std::numeric_limits<Real>::infinity();
    }
    return compliance;
}

} // namespace spatial_internal

// ---------------------------------------------------------------------------
// SpatialInertiaSolveMotion
//   Solve I * x = rhs for the 6×6 symmetric spatial inertia in the same basis
//   as `SpatialMul` / `SpatialMotionCompliance`.  Returns false if singular.
// ---------------------------------------------------------------------------

inline bool SpatialInertiaSolveMotion(const SpatialInertia& I, const SpatialVec& rhs, SpatialVec& out) {
    return spatial_internal::SpatialInertiaSolveMotionDetailed(I, rhs, out, nullptr);
}

inline Real SpatialMotionCompliance(const SpatialInertia& I, const SpatialVec& h) {
    return spatial_internal::SpatialMotionComplianceDetailed(I, h, nullptr);
}

} // namespace minphys3d
