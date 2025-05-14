// include/svd3.h
#ifndef SVD3_H
#define SVD3_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Perform SVD on a 3×3 matrix H:
 *   H = U * diag(S) * Vᵀ
 *
 * @param H    Input 3×3 matrix
 * @param U    Output 3×3 left singular vectors
 * @param S    Output singular values [3]
 * @param Vt   Output 3×3 right singular vectors transposed
 */
void svd3(const float H[3][3], float U[3][3], float S[3], float Vt[3][3]);

#ifdef __cplusplus
}
#endif

#endif // SVD3_H