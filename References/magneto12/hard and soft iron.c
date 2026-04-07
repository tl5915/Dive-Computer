#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <string.h>
#include <float.h>

// Forward declarations of mymathlib.com routines
void Matrix_x_Its_Transpose(double*, double*, int, int);

void Get_Submatrix(double*, int, int, double*, int, int, int);

int Choleski_LU_Decomposition(double*, int);

int Choleski_LU_Inverse(double*, int);

void Multiply_Matrices(double*, double*, int, int, double*, int);

void Identity_Matrix(double*, int);

 
int Hessenberg_Form_Elementary(double*, double*, int);

static void Hessenberg_Elementary_Transform(double*, double*, int[], int);

 
void Copy_Vector(double*, double*, int);
 

int QR_Hessenberg_Matrix(double*, double*, double[], double[], int, int);

static void One_Real_Eigenvalue(double[], double[], double[], int, double);

static void Two_Eigenvalues(double*, double*, double[], double[], int, int, double);

static void Update_Row(double*, double, double, int, int);

static void Update_Column(double*, double, double, int, int);

static void Update_Transformation(double*, double, double, int, int);

static void Double_QR_Iteration(double*, double*, int, int, int, double*, int);

static void Product_and_Sum_of_Shifts(double*, int, int, double*, double*, double*, int);

static int Two_Consecutive_Small_Subdiagonal(double*, int, int, int, double, double);

static void Double_QR_Step(double*, int, int, int, double, double, double*, int);

static void BackSubstitution(double*, double[], double[], int);

static void BackSubstitute_Real_Vector(double*, double[], double[], int, double, int);

static void BackSubstitute_Complex_Vector(double*, double[], double[], int, double, int);

static void Calculate_Eigenvectors(double*, double*, double[], double[], int);

static void Complex_Division(double, double, double, double, double*, double*);

void Transpose_Square_Matrix(double*, int);

 

int main()

{

 int nlines = 0;

 char buf[120];

 double *D, *S, *C, *S11, *S12, *S12t, *S22, *S22_1, *S22a, *S22b, *SS, *E, *d, *U, *SSS;

 double *eigen_real, *eigen_imag, *v1, *v2, *v, *Q, *Q_1, *B, *QB, J, hmb, *SSSS;

 int *p;

 int i, index;

 double maxval, norm, btqb, *eigen_real3, *eigen_imag3, *Dz, *vdz, *SQ, *A_1, hm, norm1, norm2, norm3;

 double x, y, z;

 

 FILE *fp;

 /* ============================================================================
  * ALGORITHM OVERVIEW: Hard and Soft Iron Calibration via Ellipsoid Fitting
  * ============================================================================
  * 
  * This algorithm solves for magnetometer calibration parameters:
  *   - Hard Iron (B):    3x1 bias vector for each axis
  *   - Soft Iron (A):    3x3 transformation matrix (scale + cross-coupling)
  * 
  * The raw measurement model: True_field = A * (Raw_measurement - B)
  * 
  * The algorithm:
  * 1. Load raw magnetometer samples from file (assumed to be on a sphere)
  * 2. Build design matrix D with polynomial terms (quadratic + linear + constant)
  * 3. Compute Schur complement of normal equations (matrix S)
  * 4. Set up constraint matrix C for sphere constraint
  * 5. Solve constrained eigenvalue problem: C*S*v = lambda*v
  * 6. Extract eigenvector corresponding to largest eigenvalue
  * 7. Compute hard iron bias (B) and soft iron matrix (A)
  * 8. Output calibration coefficients
  * =========================================================================== */

 fp = fopen("c:\\MagCal\\mag.txt", "r");

 while(fgets(buf, 100, fp) != NULL)

   nlines++;

 rewind(fp);

 

 D = (double*)malloc(10 * nlines * sizeof(double));

 for( i = 0; i < nlines; i++)

 {

   fgets(buf, 100, fp);

   sscanf(buf, "%lf\t%lf\t%lf", &x, &y, &z);

   D[i] = x * x;

   D[nlines+i] = y * y;

   D[nlines*2+i] = z * z;

   D[nlines*3+i] = 2.0 * y * z;

   D[nlines*4+i] = 2.0 * x * z;

   D[nlines*5+i] = 2.0 * x * y;

   D[nlines*6+i] = 2.0 * x;

   D[nlines*7+i] = 2.0 * y;

   D[nlines*8+i] = 2.0 * z;

   D[nlines*9+i] = 1.0;

 }

 fclose(fp);

 /* ============================================================================
  * STEP 1: Construct Design Matrix D (10 x nlines)
  * ============================================================================
  * The measurements are fit to the ellipsoid model:
  *   x²*Q11 + y²*Q22 + z²*Q33 + 2*y*z*Q23 + 2*x*z*Q13 + 2*x*y*Q12 + 2*x*U1 + 2*y*U2 + 2*z*U3 + J = 0
  * 
  * Design matrix D has rows for each quadratic coefficient pair:
  * [x², y², z², 2yz, 2xz, 2xy, 2x, 2y, 2z, 1]
  * 
  * See above for D construction with polynomial terms
  * ============================================================================ */

 // allocate memory for matrix S = D' * D (normal equations)

 S = (double*)malloc(10 * 10 * sizeof(double));

 Matrix_x_Its_Transpose(S, D, 10, nlines);

 /* ============================================================================
  * STEP 2: Constraint Matrix C (Sphere Constraint)
  * ============================================================================
  * For the ellipsoid fit, a sphere constraint is used.
  * The constraint matrix ensures the fitted surface is a reasonable ellipsoid:
  *   Q11*Q22 - Q12² >= 0 (positive semidefinite quadratic form)
  *   
  * C is the 6x6 constraint matrix for the quadratic coefficients
  * ============================================================================ */

 // Create pre-inverted constraint matrix C

 C = (double*)malloc(6 * 6 * sizeof(double));

 C[0] = 0.0; C[1] = 0.5; C[2] = 0.5; C[3] = 0.0;  C[4] = 0.0;  C[5] = 0.0;

 C[6] = 0.5;  C[7] = 0.0; C[8] = 0.5; C[9] = 0.0;  C[10] = 0.0;  C[11] = 0.0;

 C[12] = 0.5;  C[13] = 0.5; C[14] = 0.0; C[15] = 0.0;  C[16] = 0.0;  C[17] = 0.0;

 C[18] = 0.0;  C[19] = 0.0;  C[20] = 0.0;  C[21] = -0.25; C[22] = 0.0;  C[23] = 0.0;

 C[24] = 0.0;  C[25] = 0.0; C[26] = 0.0;  C[27] = 0.0;  C[28] = -0.25; C[29] = 0.0;

 C[30] = 0.0;  C[31] = 0.0; C[32] = 0.0;  C[33] = 0.0;  C[34] = 0.0;  C[35] = -0.25;

  
 /* ============================================================================
  * STEP 3: Compute Schur Complement (Reduce dimensionality)
  * ============================================================================
  * The normal equations are partitioned as:
  *   [S11 | S12] [v1]   [0]
  *   [----+----] [--] = [--]
  *   [S21 | S22] [v2]   [0]
  *
  * Schur complement: SS = S11 - S12 * S22^(-1) * S21
  * This reduces the problem from 10D to 6D by eliminating linear terms
  * ============================================================================ */

 S11 = (double*)malloc(6 * 6 * sizeof(double));

 Get_Submatrix(S11, 6, 6, S, 10, 0, 0);

 S12 = (double*)malloc(6 * 4 * sizeof(double));

 Get_Submatrix(S12, 6, 4, S, 10, 0, 6);

 S12t = (double*)malloc(4 * 6 * sizeof(double));

 Get_Submatrix(S12t, 4, 6, S, 10, 6, 0);

 S22 = (double*)malloc(4 * 4 * sizeof(double));

 Get_Submatrix(S22, 4, 4, S, 10, 6, 6);

 

 S22_1 = (double*)malloc(4 * 4 * sizeof(double));

 for(i = 0; i < 16; i++)

   S22_1[i] = S22[i];

 Choleski_LU_Decomposition(S22_1, 4);

 Choleski_LU_Inverse(S22_1, 4);

 

 // Calculate S22a = S22_1 * S12t   4*6 = 4x4 * 4x6   C = AB

 S22a = (double*)malloc(4 * 6 * sizeof(double));

 Multiply_Matrices(S22a, S22_1, 4, 4, S12t, 6);

 

 // Then calculate S22b = S12 * S22a      ( 6x6 = 6x4 * 4x6)

 S22b = (double*)malloc(6 * 6 * sizeof(double));

 Multiply_Matrices(S22b, S12, 6, 4, S22a, 6);

 

 // Calculate SS = S11 - S22b

 SS = (double*)malloc(6 * 6 * sizeof(double));

 for(i = 0; i < 36; i++)

   SS[i] = S11[i] - S22b[i];

 E = (double*)malloc(6 * 6 * sizeof(double));

 Multiply_Matrices(E, C, 6, 6, SS, 6);

 SSS = (double*)malloc(6 * 6 * sizeof(double));

 Hessenberg_Form_Elementary(E, SSS, 6);

 eigen_real = (double*)malloc(6 * sizeof(double));

 eigen_imag = (double*)malloc(6 * sizeof(double));

 QR_Hessenberg_Matrix(E, SSS, eigen_real, eigen_imag, 6, 100);

 

 index = 0;

 maxval = eigen_real[0];

 for(i = 1; i < 6; i++)

 {

   if(eigen_real[i] > maxval)

   {

      maxval = eigen_real[i];

      index = i;

   }

 }

 /* ============================================================================
  * STEP 4: Extract Eigenvector (Maximum Eigenvalue)
  * ============================================================================
  * The eigenvector corresponding to the largest eigenvalue gives the best-fit
  * ellipsoid parameters in vector form: v = [Q11, Q22, Q33, Q23, Q13, Q12, U1, U2, U3, J]
  * ============================================================================ */

 v1 = (double*)malloc(6 * sizeof(double));

 

 v1[0] = SSS[index]; 

 v1[1] = SSS[index+6];

 v1[2] = SSS[index+12];

 v1[3] = SSS[index+18];

 v1[4] = SSS[index+24];

 v1[5] = SSS[index+30];

 // normalize v1

 norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] + v1[3] * v1[3] + v1[4] * v1[4] + v1[5] * v1[5]);

 v1[0] /= norm;

 v1[1] /= norm;

 v1[2] /= norm;

 v1[3] /= norm;

 v1[4] /= norm;

 v1[5] /= norm;

 if(v1[0] < 0.0)

 {

   v1[0] = -v1[0];

   v1[1] = -v1[1];

   v1[2] = -v1[2];

   v1[3] = -v1[3];

   v1[4] = -v1[4];

   v1[5] = -v1[5];

 }

 // Calculate v2 = S22a * v1      ( 4x1 = 4x6 * 6x1)

 v2 = (double*)malloc(4 * sizeof(double));

 Multiply_Matrices(v2, S22a, 4, 6, v1, 1);

 v = (double*)malloc(10 * sizeof(double));

 v[0] = v1[0];

 v[1] = v1[1];

 v[2] = v1[2];

 v[3] = v1[3];

 v[4] = v1[4];

 v[5] = v1[5];

 v[6] = -v2[0];

 v[7] = -v2[1];

 v[8] = -v2[2];

 v[9] = -v2[3];

 /* ============================================================================
  * STEP 5: Extract Hard Iron Bias (B) - DC Offset Calibration
  * ============================================================================
  * The ellipsoid fit parameters are:
  *   v = [Q11, Q22, Q33, Q23, Q13, Q12, U1, U2, U3, J]
  * 
  * The Q matrix (soft iron) is:
  *   Q = [Q11  Q12  Q13]   = [v[0]  v[5]  v[4]]
  *       [Q12  Q22  Q23]     [v[5]  v[1]  v[3]]
  *       [Q13  Q23  Q33]     [v[4]  v[3]  v[2]]
  * 
  * Hard iron bias is extracted via: B = -Q^(-1) * U
  * where U = [U1, U2, U3] = [v[6], v[7], v[8]]
  * ============================================================================ */

 Q = (double*)malloc(3 * 3 * sizeof(double));

 

 Q[0] = v[0];

 Q[1] = v[5];

 Q[2] = v[4];

 Q[3] = v[5];

 Q[4] = v[1];

 Q[5] = v[3];

 Q[6] = v[4];

 Q[7] = v[3];

 Q[8] = v[2];

 U = (double*)malloc(3 * sizeof(double));

 U[0] = v[6];

 U[1] = v[7];

 U[2] = v[8];

 Q_1 = (double*)malloc(3 * 3 * sizeof(double));

 for(i = 0; i < 9; i++)

   Q_1[i] = Q[i];

 Choleski_LU_Decomposition(Q_1, 3);

 Choleski_LU_Inverse(Q_1, 3);

 // Calculate B = Q-1 * U   ( 3x1 = 3x3 * 3x1)

 B = (double*)malloc(3 * sizeof(double));

 Multiply_Matrices(B, Q_1, 3, 3, U, 1);

 B[0] = -B[0];     // x-axis combined bias

 B[1] = -B[1];     // y-axis combined bias

 B[2] = -B[2];     // z-axis combined bias

 

for(i = 0; i < 3; i++)

  printf("%lf\r\n", B[i]);

 // First calculate QB = Q * B   ( 3x1 = 3x3 * 3x1)

 QB = (double*)malloc(3 * sizeof(double));

 Multiply_Matrices(QB, Q, 3, 3, B, 1);

 // Then calculate btqb = BT * QB    ( 1x1 = 1x3 * 3x1)

 Multiply_Matrices(&btqb, B, 1, 3, QB, 1);

 // Calculate hmb = sqrt(btqb - J).

 J = v[9];

 hmb = sqrt(btqb - J);

 

 /* ============================================================================
  * STEP 6: Extract Soft Iron Matrix (A) - Scale and Cross-Coupling Correction
  * ============================================================================
  * The soft iron matrix A comes from the Cholesky decomposition of Q:
  *   A = sqrt(Q) / (magnetic field magnitude)
  * 
  * Algorithm:
  * 1. Compute eigenvalue decomposition of Q
  * 2. Construct diagonal matrix Dz with sqrt(eigenvalues)
  * 3. Compute SQ = V * Dz * V^T (matrix square root)
  * 4. Scale by reference field magnitude: A = (hm/hmb) * SQ
  * 
  * Result: M_calibrated = A * (M_raw - B)
  * ============================================================================ */

 // Calculate SQ, the square root of matrix Q

 SSSS = (double*)malloc(3 * 3 * sizeof(double));

 Hessenberg_Form_Elementary(Q, SSSS, 3);

 eigen_real3 = (double*)malloc(3 * sizeof(double));

 eigen_imag3 = (double*)malloc(3 * sizeof(double));

 QR_Hessenberg_Matrix(Q, SSSS, eigen_real3, eigen_imag3, 3, 100);

 // normalize eigenvectors

 norm1 = sqrt(SSSS[0] * SSSS[0] + SSSS[3] * SSSS[3] + SSSS[6] * SSSS[6]);

 SSSS[0] /= norm1;

 SSSS[3] /= norm1;

 SSSS[6] /= norm1;

 norm2 = sqrt(SSSS[1] * SSSS[1] + SSSS[4] * SSSS[4] + SSSS[7] * SSSS[7]);

 SSSS[1] /= norm2;

 SSSS[4] /= norm2;

 SSSS[7] /= norm2;

 norm3 = sqrt(SSSS[2] * SSSS[2] + SSSS[5] * SSSS[5] + SSSS[8] * SSSS[8]);

 SSSS[2] /= norm3;

 SSSS[5] /= norm3;

 SSSS[8] /= norm3;

 

 Dz = (double*)malloc(3 * 3 * sizeof(double));

 for(i = 0; i < 9; i++)

   Dz[i] = 0.0;

 Dz[0] = sqrt(eigen_real3[0]);

 Dz[4] = sqrt(eigen_real3[1]);

 Dz[8] = sqrt(eigen_real3[2]);

 vdz = (double*)malloc(3 * 3 * sizeof(double));

 Multiply_Matrices(vdz, SSSS, 3, 3, Dz, 3);

 Transpose_Square_Matrix(SSSS, 3);

 SQ = (double*)malloc(3 * 3 * sizeof(double));

 Multiply_Matrices(SQ, vdz, 3, 3, SSSS, 3);

 hm = 0.569;

 A_1 = (double*)malloc(3 * 3 * sizeof(double));

 for(i = 0; i < 9; i++)

   A_1[i] = SQ[i] * hm / hmb;

 

 /* ============================================================================
  * STEP 7: Output Calibration Parameters
  * ============================================================================
  * Hard Iron Bias (B): Printed first - three scalar values
  *   Format: B_x, B_y, B_z (hard iron offsets for each axis)
  * 
  * Soft Iron Matrix (A): Printed second - 3x3 matrix
  *   Format: [A_xx A_xy A_xz]
  *           [A_yx A_yy A_yz]
  *           [A_zx A_zy A_zz]
  * 
  * Calibration Formula: M_calibrated = A * (M_raw - B)
  * ============================================================================ */

 printf("Hard Iron Bias (B - offsets per axis):\r\n");

 for(i = 0; i < 3; i++)

  printf("%lf\r\n", B[i]);

 

 printf("\nSoft Iron Matrix (A - scale & cross-coupling):\r\n");

 for(i = 0; i < 3; i++)

   printf("%lf %lf %lf\r\n", A_1[i*3], A_1[i*3+1], A_1[i*3+2]);

 

 free(D);

 free(S);

 free(C);

 free(S11);

 free(S12);

 free(S12t);

 free(S22);

 free(S22_1);

 free(S22a);

 free(S22b);

 free(SS);

 free(E);

 free(U);

 free(SSS);

 free(eigen_real);

 free(eigen_imag);

 free(v1);

 free(v2);

 free(v);

 free(Q);

 free(Q_1);

 free(B);

 free(QB);

 free(SSSS);

 free(eigen_real3);

 free(eigen_imag3);

 free(Dz);

 free(vdz);

 free(SQ);

 free(A_1);

 return 0;

}