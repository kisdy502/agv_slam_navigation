#include "matrix_demo.hpp"

void eigen_demo() {
    std::cout << "========================================\n";
    std::cout << "        Eigen 矩阵运算演示\n";
    std::cout << "========================================\n\n";
    
    using namespace Eigen;
    
    Matrix3d A;
    A << 1, 2, 3,
         4, 5, 6,
         7, 8, 10;
    
    Vector3d b(1, 2, 3);
    
    std::cout << ">>> 定义 3x3 矩阵 A 和 3x1 向量 b\n";
    std::cout << "矩阵 A:\n" << A << "\n\n";
    std::cout << "向量 b:\n" << b.transpose() << "\n\n";
    
    std::cout << ">>> 【矩阵加法】A + A（对应元素相加）\n";
    std::cout << "A + A =\n" << A + A << "\n\n";
    
    std::cout << ">>> 【数乘】A * 2（每个元素乘以2）\n";
    std::cout << "A * 2 =\n" << A * 2 << "\n\n";
    
    std::cout << ">>> 【矩阵乘法】A * A（矩阵相乘）\n";
    std::cout << "A * A =\n" << A * A << "\n\n";
    
    std::cout << ">>> 【转置】A^T（行变列）\n";
    std::cout << "A^T =\n" << A.transpose() << "\n\n";
    
    std::cout << ">>> 【行列式】det(A)（衡量矩阵(面积、体积)缩放因子）\n";
    std::cout << "det(A) = " << A.determinant() << "\n\n";
    
    std::cout << ">>> 【逆矩阵】A^{-1}（满足 A * A^{-1} = I）\n";
    std::cout << "A^{-1} =\n" << A.inverse() << "\n\n";
    
    std::cout << ">>> 【解线性方程组】Ax = b\n";
    std::cout << "  已知 A 和 b，求 x 使得 Ax = b\n";
    Vector3d x = A.colPivHouseholderQr().solve(b);
    std::cout << "解 x =\n" << x.transpose() << "\n\n";
    
    std::cout << ">>> 【特征值分解】A = V * D * V^{-1}\n";
    std::cout << "  特征值：向量在变换中只伸缩不变的方向\n";
    SelfAdjointEigenSolver<Matrix3d> eigensolver(A);
    if (eigensolver.info() == Success) {
        std::cout << "特征值 (λ1, λ2, λ3):\n" << eigensolver.eigenvalues().transpose() << "\n\n";
        std::cout << "特征向量（每列是一个特征向量）:\n" << eigensolver.eigenvectors() << "\n\n";
        std::cout << "最小特征值 = " << eigensolver.eigenvalues().minCoeff() << " （退化检测用）\n";
        std::cout << "对应特征向量 =\n" << eigensolver.eigenvectors().col(0).transpose() << "\n\n";
    }
    
    std::cout << ">>> 【SVD 分解】A = U * Σ * V^T\n";
    std::cout << "  SVD：更通用的矩阵分解，最小二乘、伪逆用它\n";
    JacobiSVD<Matrix3d> svd(A, ComputeFullU | ComputeFullV);
    std::cout << "奇异值 (Σ 对角线):\n" << svd.singularValues().transpose() << "\n\n";
    
    std::cout << ">>> 【块操作】SLAM 中提取旋转 R 和平移 t\n";
    Matrix4d T = Matrix4d::Identity();
    T.block<3,3>(0,0) = A;           // 左上 3x3 放旋转
    T.block<3,1>(0,3) = b;           // 右上 3x1 放平移
    std::cout << "4x4 变换矩阵 T = [R|t]:\n" << T << "\n\n";
    
    Matrix3d R = T.block<3,3>(0,0);  // 提取旋转部分
    Vector3d t = T.block<3,1>(0,3);  // 提取平移部分
    std::cout << "提取旋转 R:\n" << R << "\n";
    std::cout << "提取平移 t:\n" << t.transpose() << "\n\n";
    
    std::cout << ">>> 【协方差矩阵】描述不确定性\n";
    Matrix3d cov = Matrix3d::Zero();
    cov.diagonal() << 0.1, 0.2, 0.05;
    std::cout << "协方差矩阵（对角线是各维度方差）:\n" << cov << "\n\n";
    std::cout << ">>> 【信息矩阵】协方差的逆\n";
    std::cout << "信息矩阵 = 协方差逆:\n" << cov.inverse() << "\n\n";
}

void slam_transform_demo() {
    std::cout << "========================================\n";
    std::cout << "     SLAM 位姿变换演示\n";
    std::cout << "========================================\n\n";
    
    using namespace Eigen;
    
    double x = 1.0, y = 2.0, theta = M_PI / 6;  // 30度
    
    std::cout << ">>> 【2D 旋转矩阵】从角度构造 SO(2)\n";
    Matrix2d R;
    R << cos(theta), -sin(theta),
         sin(theta),  cos(theta);
    
    std::cout << ">>> 【平移向量】机器人位置\n";
    Vector2d t(x, y);
    
    std::cout << "机器人位姿: x=" << x << ", y=" << y << ", theta=" << theta*180/M_PI << "°\n";
    std::cout << "旋转矩阵 R (SO(2)):\n" << R << "\n";
    std::cout << "平移向量 t: " << t.transpose() << "\n\n";
    
    std::cout << ">>> 【坐标变换】雷达点 → 世界坐标\n";
    std::cout << "  公式: p_world = R * p_laser + t\n";
    Vector2d p_laser(2, 0);
    std::cout << "雷达坐标系下的点: " << p_laser.transpose() << "\n";
    
    Vector2d p_world = R * p_laser + t;
    std::cout << "世界坐标系下的点: " << p_world.transpose() << "\n\n";
    
    std::cout << ">>> 【验证旋转矩阵性质】R^T * R = I, det(R) = 1\n";
    std::cout << "R^T * R (应接近单位矩阵):\n" << R.transpose() * R << "\n";
    std::cout << "det(R) = " << R.determinant() << " (=1 表示纯旋转) \n\n";
}

void simple_matrix_demo() {
    std::cout << "========================================\n";
    std::cout << "      手写矩阵类运算演示\n";
    std::cout << "========================================\n\n";
    
    SimpleMatrix<double> A = {{1, 2}, {3, 4}};
    SimpleMatrix<double> B = {{5, 6}, {7, 8}};
    
    std::cout << ">>> 定义 2x2 矩阵 A 和 B\n";
    A.print("A");
    B.print("B");
    
    std::cout << ">>> 【矩阵加法】C = A + B\n";
    std::cout << "  原理：对应元素相加 C[i][j] = A[i][j] + B[i][j]\n";
    auto C = A + B;
    C.print("C = A + B");
    
    std::cout << ">>> 【矩阵乘法】D = A * B\n";
    std::cout << "  原理：D[i][j] = Σ(A[i][k] * B[k][j])，注意维度要匹配！\n";
    auto D = A * B;
    D.print("D = A * B");
    
    std::cout << ">>> 【转置】E = A^T\n";
    std::cout << "  原理：E[i][j] = A[j][i]，行变列\n";
    auto E = A.transpose();
    E.print("E = A^T");
    
    std::cout << ">>> 【行列式】det(A)\n";
    std::cout << "  原理：2x2: ad - bc，用于判断是否可逆\n";
    std::cout << "det(A) = " << A.determinant() << "\n\n";
    
    std::cout << ">>> 【逆矩阵】A^{-1}\n";
    std::cout << "  原理：A * A^{-1} = I，2x2 逆矩阵公式见代码\n";
    auto A_inv = A.inverse();
    A_inv.print("A^{-1}");
    
    std::cout << ">>> 【验证】A * A^{-1} 应接近单位矩阵\n";
    auto verify = A * A_inv;
    verify.print("A * A^{-1} = I (验证)");
    
    std::cout << ">>> 【单位矩阵】I（对角线为1，其他为0）\n";
    auto I = SimpleMatrix<double>::identity(3);
    I.print("3x3 单位矩阵 I");
}

void MatrixDemo::test() {
    std::cout << std::fixed << std::setprecision(4);
    
    simple_matrix_demo();
    eigen_demo();
    slam_transform_demo();
    
    std::cout << "========================================\n";
    std::cout << "      全部案例执行完成!\n";
    std::cout << "========================================\n";
}
