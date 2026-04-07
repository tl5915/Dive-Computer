import numpy as np

def calibrate_ellipsoid(samples, reference_mag):
    n = len(samples)
    if n < 50:
        raise ValueError(f"Need >= 50 samples, got {n}")

    # STEP 1
    S = np.zeros((10, 10), dtype=np.float64)
    for x, y, z in samples:
        d = np.array([x*x, y*y, z*z,
                      2*y*z, 2*x*z, 2*x*y,
                      2*x, 2*y, 2*z, 1.0])
        S += np.outer(d, d)
    S /= n

    # STEP 2
    S11 = S[:6, :6]
    S12 = S[:6, 6:]
    S21 = S[6:, :6]
    S22 = S[6:, 6:]
    S22_inv = np.linalg.inv(S22)
    SS = S11 - S12 @ S22_inv @ S21
    S22a = S22_inv @ S21   # 4x6, used for step 4

    # STEP 3
    C = np.array([
        [ 0,    0.5,  0.5,  0,      0,      0    ],
        [ 0.5,  0,    0.5,  0,      0,      0    ],
        [ 0.5,  0.5,  0,    0,      0,      0    ],
        [ 0,    0,    0,   -0.25,   0,      0    ],
        [ 0,    0,    0,    0,     -0.25,   0    ],
        [ 0,    0,    0,    0,      0,     -0.25 ],
    ], dtype=np.float64)
    C_inv = np.linalg.inv(C)
    E = C_inv @ SS
    import scipy.linalg
    eigvals_g, eigvecs_g = scipy.linalg.eig(SS, C)
    real_mask = np.abs(eigvals_g.imag) < 1e-6
    eigvals_r = eigvals_g[real_mask].real
    eigvecs_r = eigvecs_g[:, real_mask].real
    pos_mask = eigvals_r > 0
    if not np.any(pos_mask):
        raise RuntimeError("No positive generalised eigenvalue")
    best_idx = np.argmax(eigvals_r * pos_mask - 1e18 * (~pos_mask))
    v1 = eigvecs_r[:, best_idx]
    lam = eigvals_r[best_idx]

    # STEP 4
    v2 = -S22a @ v1
    v = np.concatenate([v1, v2])
    v /= np.linalg.norm(v)
    if v[0] + v[1] + v[2] < 0:
        v = -v

    # STEP 5
    Q = np.array([
        [v[0], v[5], v[4]],
        [v[5], v[1], v[3]],
        [v[4], v[3], v[2]]
    ])
    U = np.array([v[6], v[7], v[8]])
    B = -np.linalg.solve(Q, U)   # = -Q^-1 * U

    # STEP 6
    J = v[9]
    field_mag_sq = float(B @ Q @ B) - J
    if field_mag_sq <= 0:
        raise RuntimeError(f"field_mag_sq = {field_mag_sq} <= 0 — calibration degenerate")
    field_mag = np.sqrt(field_mag_sq)

    # STEP 7
    eigvals_Q, eigvecs_Q = np.linalg.eigh(Q)   # eigh = symmetric, always real
    if np.any(eigvals_Q <= 0):
        raise RuntimeError(f"Q not SPD: eigenvalues = {eigvals_Q}")
    sqrt_Q = eigvecs_Q @ np.diag(np.sqrt(eigvals_Q)) @ eigvecs_Q.T
    scale = reference_mag / field_mag
    A = sqrt_Q * scale

    return B, A, field_mag


# Test
data = np.loadtxt(
    r"*\test_data.txt"
)
print(f"{len(data)} samples")

reference_magnitude = 0.569
print(f"\nReference_magnitude = {reference_magnitude}")

B, A, field_mag = calibrate_ellipsoid(data, reference_magnitude)

print(f"\nHard Iron bias (b):")
print(f"Bx = {B[0]:.6f}")
print(f"By = {B[1]:.6f}")
print(f"Bz = {B[2]:.6f}")

print(f"\nSoft Iron matrix (A):")
for r in range(3):
    print(f"[{A[r,0]:10.6f}  {A[r,1]:10.6f}  {A[r,2]:10.6f}]")

print(f"\nMeasured magnitude: {field_mag:.6f}")
print(f"Reference magnitude: {reference_magnitude:.6f}")
print(f"Scale factor: {reference_magnitude/field_mag:.6f}")

# Validation
raw = data
corrected = (raw - B) @ A.T
magnitudes = np.linalg.norm(corrected, axis=1)
print(f"\nValidation:")
print(f"Mean = {magnitudes.mean():.6f}")
print(f"Std = {magnitudes.std():.6f}")