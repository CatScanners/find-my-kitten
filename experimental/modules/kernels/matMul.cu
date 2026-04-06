#include <cublas_v2.h>

void matMul(float* A, float* B, float* C,
            int ac, int ab, int bc) {
    // A is ac × ab
    // B is ab × bc
    // C is ac × bc

    cublasHandle_t handle;
    cublasCreate(&handle);

    const float alpha = 1.0f;
    const float beta  = 0.0f;

    // cuBLAS uses column-major order.
    // To compute C = A * B in row-major, we compute:
    // Cᵀ = Bᵀ * Aᵀ in column-major.
    cublasSgemm(
        handle,
        CUBLAS_OP_N, CUBLAS_OP_N,
        bc,          // number of columns of Cᵀ (i.e., rows of C)
        ac,          // number of rows of Cᵀ (i.e., columns of C)
        ab,          // shared dimension
        &alpha,
        B, bc,       // Bᵀ treated as column-major B
        A, ab,       // Aᵀ treated as column-major A
        &beta,
        C, bc        // output Cᵀ
    );

    cublasDestroy(handle);
}
