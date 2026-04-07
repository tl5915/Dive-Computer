#pragma once

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <algorithm>


struct MagSample {
    float x, y, z;
};

struct CalibrationCoefficients {
    // Hard Iron (bias offsets): 3 x 1 matrix
    float hard_iron[3];  // [B_x, B_y, B_z]
    // Soft Iron (transformation matrix): 3 x 3 matrix
    float soft_iron[9];  // [A_xx A_xy A_xz; A_yx A_yy A_yz; A_zx A_zy A_zz]
    // Reference field magnitude
    float reference_magnitude;
    // Results
    bool is_valid;
    float field_magnitude;  // Measured field magnitude from calibration
};


// ----- Matrix Operations ----- //
// C = A * B
// A: m x n, B: n x p -> C: m x p
static inline void matmul(const float* A, int m, int n, 
                          const float* B, int p,
                          float* C) {
    memset(C, 0, m * p * sizeof(float));
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            for (int k = 0; k < n; k++) {
                C[i * p + j] += A[i * n + k] * B[k * p + j];
            }
        }
    }
}

// C = A * A'
//A: m x n -> C: m x m
static inline void matmul_transpose(const float* A, int m, int n, float* C) {
    memset(C, 0, m * m * sizeof(float));
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
            for (int k = 0; k < n; k++) {
                C[i * m + j] += A[i * n + k] * A[j * n + k];
            }
        }
    }
}

// Cholesky decomposition: A = L * L'
// In-place: input matrix replaced with lower triangular L
// Return: 0 on success, -1 on failure (not positive definite)
static inline int cholesky_decompose(float* A, int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < i; j++) {
            A[i * n + i] -= A[i * n + j] * A[i * n + j];
        }
        if (A[i * n + i] <= 0) return -1;  // Not positive definite
        A[i * n + i] = sqrtf(A[i * n + i]);
        
        for (int j = i + 1; j < n; j++) {
            for (int k = 0; k < i; k++) {
                A[j * n + i] -= A[j * n + k] * A[i * n + k];
            }
            A[j * n + i] /= A[i * n + i];
        }
    }
    return 0;
}

// Solve lower triangular system: L * x = b
static inline void solve_lower_triangular(float* L, int n, float* x) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < i; j++) {
            x[i] -= L[i * n + j] * x[j];
        }
        x[i] /= L[i * n + i];
    }
}

// Solve upper triangular system: U * x = b
static inline void solve_upper_triangular(float* U, int n, float* x) {
    for (int i = n - 1; i >= 0; i--) {
        for (int j = i + 1; j < n; j++) {
            x[i] -= U[i * n + j] * x[j];
        }
        x[i] /= U[i * n + i];
    }
}

// Matrix inversion with Cholesky decomposition: A = L * L'
// Inverts A in-place
static inline int matrix_inverse_cholesky(float* A, int n) {
    float L[10 * 10];
    memcpy(L, A, n * n * sizeof(float));
    if (cholesky_decompose(L, n) < 0) return -1;
    // L^(-1)
    float L_inv[10 * 10];
    memset(L_inv, 0, n * n * sizeof(float));
    for (int i = 0; i < n; i++) L_inv[i * n + i] = 1.0f / L[i * n + i];
    for (int j = 0; j < n; j++) {
        for (int i = j + 1; i < n; i++) {
            float sum = 0;
            for (int k = j; k < i; k++) {
                sum -= L[i * n + k] * L_inv[k * n + j];
            }
            L_inv[i * n + j] = sum / L[i * n + i];
        }
    }
    // A^(-1) = (L^(-1))' * L^(-1)
    memset(A, 0, n * n * sizeof(float));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < n; k++) {
                A[i * n + j] += L_inv[k * n + i] * L_inv[k * n + j];
            }
        }
    }
    return 0;
}

// Matrix determinant (3 x 3 simplified)
static inline float det3x3(const float* A) {
    return A[0] * (A[4] * A[8] - A[5] * A[7]) -
           A[1] * (A[3] * A[8] - A[5] * A[6]) +
           A[2] * (A[3] * A[7] - A[4] * A[6]);
}

// Invert 3 x 3 matrix
static inline int inv3x3(const float* A, float* A_inv) {
    float det = det3x3(A);
    if (fabsf(det) < 1e-10f) return -1;
    A_inv[0] = (A[4] * A[8] - A[5] * A[7]) / det;
    A_inv[1] = (A[2] * A[7] - A[1] * A[8]) / det;
    A_inv[2] = (A[1] * A[5] - A[2] * A[4]) / det;
    A_inv[3] = (A[5] * A[6] - A[3] * A[8]) / det;
    A_inv[4] = (A[0] * A[8] - A[2] * A[6]) / det;
    A_inv[5] = (A[2] * A[3] - A[0] * A[5]) / det;
    A_inv[6] = (A[3] * A[7] - A[4] * A[6]) / det;
    A_inv[7] = (A[1] * A[6] - A[0] * A[7]) / det;
    A_inv[8] = (A[0] * A[4] - A[1] * A[3]) / det;
    return 0;
}

// Compute eigenvalues and eigenvectors of a 3 x 3 symmetric matrix using Jacobi iteration
static inline int eig3x3_symmetric(const float* A, float* eigenvalues, float* eigenvectors) {
    // Working copies
    float D[9];
    memcpy(D, A, 9 * sizeof(float));
    // V = identity
    float V[9] = { 1,0,0, 0,1,0, 0,0,1 };

    for (int sweep = 0; sweep < 50; sweep++) {
        // Find off-diagonal element with largest absolute value
        float max_val = 0.0f;
        int p = 0, q = 1;
        for (int r = 0; r < 3; r++) {
            for (int c = r + 1; c < 3; c++) {
                float v = fabsf(D[r * 3 + c]);
                if (v > max_val) { max_val = v; p = r; q = c; }
            }
        }
        if (max_val < 1e-10f) break;  // Converged

        // Jacobi rotation to zero D[p][q]
        float Dpp = D[p * 3 + p];
        float Dqq = D[q * 3 + q];
        float Dpq = D[p * 3 + q];
        float theta = 0.5f * (Dqq - Dpp) / Dpq;
        float t = (theta >= 0.0f) ? (1.0f / (theta + sqrtf(1.0f + theta * theta)))
                                  : (1.0f / (theta - sqrtf(1.0f + theta * theta)));
        float c = 1.0f / sqrtf(1.0f + t * t);
        float s = t * c;
        float tau = s / (1.0f + c);

        // Update D (similarity transform)
        D[p * 3 + p] = Dpp - t * Dpq;
        D[q * 3 + q] = Dqq + t * Dpq;
        D[p * 3 + q] = 0.0f;
        D[q * 3 + p] = 0.0f;
        for (int r = 0; r < 3; r++) {
            if (r == p || r == q) continue;
            float Drp = D[r * 3 + p];
            float Drq = D[r * 3 + q];
            D[r * 3 + p] = Drp - s * (Drq + tau * Drp);
            D[p * 3 + r] = D[r * 3 + p];
            D[r * 3 + q] = Drq + s * (Drp - tau * Drq);
            D[q * 3 + r] = D[r * 3 + q];
        }
        // Accumulate rotation in V
        for (int r = 0; r < 3; r++) {
            float Vrp = V[r * 3 + p];
            float Vrq = V[r * 3 + q];
            V[r * 3 + p] = Vrp - s * (Vrq + tau * Vrp);
            V[r * 3 + q] = Vrq + s * (Vrp - tau * Vrq);
        }
    }

    // Extract eigenvalues from diagonal
    float ev[3] = { D[0], D[4], D[8] };
    // Sort descending (bubble sort — only 3 elements)
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2 - i; j++) {
            if (ev[j] < ev[j + 1]) {
                float tmp = ev[j]; ev[j] = ev[j + 1]; ev[j + 1] = tmp;
                // Swap columns j and j+1 of V (columns in V correspond to eigenvectors)
                for (int r = 0; r < 3; r++) {
                    float tmp2 = V[r * 3 + j]; V[r * 3 + j] = V[r * 3 + j + 1]; V[r * 3 + j + 1] = tmp2;
                }
            }
        }
    }
    eigenvalues[0] = ev[0];
    eigenvalues[1] = ev[1];
    eigenvalues[2] = ev[2];
    // Store eigenvectors as rows: eigenvectors[i*3..i*3+2] = column i of V
    for (int i = 0; i < 3; i++) {
        eigenvectors[i * 3 + 0] = V[0 * 3 + i];
        eigenvectors[i * 3 + 1] = V[1 * 3 + i];
        eigenvectors[i * 3 + 2] = V[2 * 3 + i];
    }
    return 0;
}

// Extract hard iron bias from ellipsoid parameters
static inline int extract_hard_iron(const float* v, float* B) {
    // Construct Q matrix from v
    float Q[9] = {
        v[0], v[5], v[4],
        v[5], v[1], v[3],
        v[4], v[3], v[2]
    };
    // U vector
    float U[3] = {v[6], v[7], v[8]};
    // Q^(-1)
    float Q_inv[9];
    if (inv3x3(Q, Q_inv) < 0) return -1;
    // B = -Q^(-1) * U
    B[0] = -(Q_inv[0] * U[0] + Q_inv[1] * U[1] + Q_inv[2] * U[2]);
    B[1] = -(Q_inv[3] * U[0] + Q_inv[4] * U[1] + Q_inv[5] * U[2]);
    B[2] = -(Q_inv[6] * U[0] + Q_inv[7] * U[1] + Q_inv[8] * U[2]);
    return 0;
}

// Compute matrix square root for 3 x 3 symmetric positive definite matrix
// sqrt(A) = V * diag(sqrt(lambda_i)) * V'
static inline int matrix_sqrt_3x3(const float* A, float* sqrt_A) {
    float eigenvalues[3];
    float eigenvectors[9];  // row i = eigenvector i
    if (eig3x3_symmetric(A, eigenvalues, eigenvectors) < 0) return -1;
    // Require all eigenvalues positive (matrix must be SPD)
    if (eigenvalues[0] <= 0.0f || eigenvalues[1] <= 0.0f || eigenvalues[2] <= 0.0f) return -1;
    float sq[3] = { sqrtf(eigenvalues[0]), sqrtf(eigenvalues[1]), sqrtf(eigenvalues[2]) };
    // sqrt_A = V * diag(sq) * V'  where columns of V are the eigenvectors
    // Since eigenvectors[i] is row i (= column i of V), V[r,i] = eigenvectors[i*3+r]
    memset(sqrt_A, 0, 9 * sizeof(float));
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            for (int k = 0; k < 3; k++) {
                // V[r,k] = eigenvectors[k*3+r], V[c,k] = eigenvectors[k*3+c]
                sqrt_A[r * 3 + c] += sq[k] * eigenvectors[k * 3 + r] * eigenvectors[k * 3 + c];
            }
        }
    }
    return 0;
}

// Extract soft iron matrix from Q matrix
static inline int extract_soft_iron(const float* v, const float* B, float* A, 
                                   float reference_magnitude, float measured_magnitude) {
    // Construct Q matrix
    float Q[9] = {
        v[0], v[5], v[4],
        v[5], v[1], v[3],
        v[4], v[3], v[2]
    };
    // Compute matrix square root
    float sqrt_Q[9];
    if (matrix_sqrt_3x3(Q, sqrt_Q) < 0) return -1;
    // Scale by field magnitude ratio
    float scale = measured_magnitude > 1e-6f ? (reference_magnitude / measured_magnitude) : 1.0f;
    for (int i = 0; i < 9; i++) {
        A[i] = sqrt_Q[i] * scale;
    }
    return 0;
}


// ----- Calibration ----- //
static inline int calibrate_ellipsoid(const MagSample* samples, int n_samples,
                                     float reference_mag,
                                     CalibrationCoefficients* cal) {
    if (n_samples < 50) return -1;  // Minimum 50 samples
    
    // --- STEP 1: Build design matrix D (10 x n_samples) --- //
    // [x², y², z², 2yz, 2xz, 2xy, 2x, 2y, 2z, 1]
    float S[10 * 10];
    memset(S, 0, sizeof(S));
    for (int i = 0; i < n_samples; i++) {
        float x = samples[i].x;
        float y = samples[i].y;
        float z = samples[i].z;
        float d[10] = {
            x * x,
            y * y,
            z * z,
            2.0f * y * z,
            2.0f * x * z,
            2.0f * x * y,
            2.0f * x,
            2.0f * y,
            2.0f * z,
            1.0f
        };
        // Accumulate S = S + d * d'
        for (int j = 0; j < 10; j++) {
            for (int k = 0; k < 10; k++) {
                S[j * 10 + k] += d[j] * d[k];
            }
        }
    }
    // Normalise by sample size
    for (int i = 0; i < 100; i++) {
        S[i] /= (float)n_samples;
    }
    
    // --- STEP 2: Schur complement to reduce dimensionality (10D -> 6D) --- //
    // Partition: S = [S11 S12; S21 S22] where S11 is 6 x 6
    float S11[6 * 6], S12[6 * 4], S21[4 * 6], S22[4 * 4];
    float S22_inv[4 * 4], S22a[4 * 6], S22b[6 * 6], SS[6 * 6];
    // Extract submatrices
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) S11[i * 6 + j] = S[i * 10 + j];
        for (int j = 0; j < 4; j++) S12[i * 4 + j] = S[i * 10 + 6 + j];
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 6; j++) S21[i * 6 + j] = S[(6 + i) * 10 + j];
        for (int j = 0; j < 4; j++) S22[i * 4 + j] = S[(6 + i) * 10 + 6 + j];
    }
    // Compute S22_inv via Cholesky
    memcpy(S22_inv, S22, sizeof(S22));
    if (matrix_inverse_cholesky(S22_inv, 4) < 0) return -1;
    // S22a = S22_inv * S21
    matmul(S22_inv, 4, 4, S21, 6, S22a);
    // S22b = S12 * S22a
    matmul(S12, 6, 4, S22a, 6, S22b);
    // SS = S11 - S22b
    for (int i = 0; i < 36; i++) {
        SS[i] = S11[i] - S22b[i];
    }
    
    // --- STEP 3: Apply constraint and solve eigenvalue --- //
    // Constraint matrix C: 4J1*J2 + 4J1*J3 + 4J2*J3 - J4^2 - J5^2 - J6^2 = 1
    // Eigenproblem:  SS * v1 = lambda * C * v1.
    // Reformulate: E = C_inv * SS, find dominant eigenvector
    float C_inv[6 * 6] = {
        -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,
         1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,
         1.0f,  1.0f, -1.0f,  0.0f,  0.0f,  0.0f,
         0.0f,  0.0f,  0.0f, -4.0f,  0.0f,  0.0f,
         0.0f,  0.0f,  0.0f,  0.0f, -4.0f,  0.0f,
         0.0f,  0.0f,  0.0f,  0.0f,  0.0f, -4.0f
    };
    // E = C_inv * SS
    float E[6 * 6];
    matmul(C_inv, 6, 6, SS, 6, E);
    // Power iteration on E to find dominant eigenvector (largest positive eigenvalue)
    float v_cur[6] = {1.0f, 1.0f, 1.0f, 0.1f, 0.1f, 0.1f};
    float Ev[6];
    float lambda = 0;
    for (int iter = 0; iter < 100; iter++) {
        // Normalise
        float norm = 0;
        for (int i = 0; i < 6; i++) norm += v_cur[i] * v_cur[i];
        norm = sqrtf(norm);
        if (norm < 1e-10f) return -1;
        for (int i = 0; i < 6; i++) v_cur[i] /= norm;
        // E * v
        memset(Ev, 0, sizeof(Ev));
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                Ev[i] += E[i * 6 + j] * v_cur[j];
            }
        }
        // lambda = v' * E * v
        lambda = 0;
        for (int i = 0; i < 6; i++) {
            lambda += v_cur[i] * Ev[i];
        }
        // Check convergence
        float delta = 0;
        for (int i = 0; i < 6; i++) {
            delta += (Ev[i] - lambda * v_cur[i]) * (Ev[i] - lambda * v_cur[i]);
        }
        if (delta < 1e-12f) break;
        memcpy(v_cur, Ev, sizeof(Ev));
    }
    // Validate: eigenvalue must be positive
    if (lambda <= 0.0f) return -1;
    
    // --- STEP 4: Reconstruct full solution vector --- //
    // v2 = -S22a * v_cur
    float v2[4];
    memset(v2, 0, sizeof(v2));
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 6; j++) {
            v2[i] -= S22a[i * 6 + j] * v_cur[j];
        }
    }
    // Full vector v = [v_cur; v2]
    float v[10];
    memcpy(v, v_cur, 6 * sizeof(float));
    memcpy(v + 6, v2, sizeof(v2));
    // Normalise v
    float v_norm = 0;
    for (int i = 0; i < 10; i++) v_norm += v[i] * v[i];
    v_norm = sqrtf(v_norm);
    if (v_norm < 1e-10f) return -1;
    for (int i = 0; i < 10; i++) v[i] /= v_norm;
    // Sign correction: Q must be positive definite → trace(Q) = v[0]+v[1]+v[2] > 0.
    if (v[0] + v[1] + v[2] < 0.0f) {
        for (int i = 0; i < 10; i++) v[i] = -v[i];
    }
    
    // --- STEP 5: Extract hard iron bias --- //
    if (extract_hard_iron(v, cal->hard_iron) < 0) return -1;
    
    // --- STEP 6: Compute measured field magnitude and extract soft iron --- //
    float J = v[9];
    // Construct Q matrix and compute field magnitude
    float Q[9] = {
        v[0], v[5], v[4],
        v[5], v[1], v[3],
        v[4], v[3], v[2]
    };
    // B' * Q * B
    float QB[3];
    for (int i = 0; i < 3; i++) {
        QB[i] = 0;
        for (int j = 0; j < 3; j++) {
            QB[i] += Q[i * 3 + j] * cal->hard_iron[j];
        }
    }
    float btqb = 0;
    for (int i = 0; i < 3; i++) {
        btqb += cal->hard_iron[i] * QB[i];
    }
    float field_mag_sq = btqb - J;
    if (field_mag_sq <= 0) return -1;
    cal->field_magnitude = sqrtf(field_mag_sq);
    
    // --- STEP 7: Extract soft iron matrix --- //
    if (extract_soft_iron(v, cal->hard_iron, cal->soft_iron, reference_mag, cal->field_magnitude) < 0) {
        return -1;
    }
    cal->is_valid = true;
    cal->reference_magnitude = reference_mag;
    return 0;
}