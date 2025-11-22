#pragma once

#include <eigen3/Eigen/Eigen>

// Eigen版本
template <int Rows, int Cols> class Matrix {
private:
    Eigen::Matrix<float, Rows, Cols> eigen_mat_;

public:
    Matrix() : eigen_mat_(Eigen::Matrix<float, Rows, Cols>::Zero()) {}

    Matrix(const float *_data) : eigen_mat_(Eigen::Map<const Eigen::Matrix<float, Rows, Cols> >(_data)) {}

    // 从Eigen矩阵构造
    Matrix(const Eigen::Matrix<float, Rows, Cols> &mat) : eigen_mat_(mat) {}

    // 初始化列表构造
    Matrix(std::initializer_list<std::initializer_list<float> > init)
    {
        int row = 0;
        for (const auto &row_list : init) {
            if (row >= Rows)
                break;
            int col = 0;
            for (float val : row_list) {
                if (col >= Cols)
                    break;
                eigen_mat_(row, col) = val;
                col++;
            }
            row++;
        }
    }

    const Eigen::Matrix<float, Rows, Cols> &getEigenMatrix() const { return eigen_mat_; }

    Eigen::Matrix<float, Rows, Cols> &getEigenMatrix() { return eigen_mat_; }

    // 数据访问
    float &operator()(int row, int col) { return eigen_mat_(row, col); }
    const float &operator()(int row, int col) const { return eigen_mat_(row, col); }

    // 矩阵运算
    Matrix<Rows, Cols> operator*(const Matrix<Rows, Cols> &other) const
    {
        return Matrix<Rows, Cols>(eigen_mat_ * other.eigen_mat_);
    }

    Matrix<Rows, Cols> operator-(const Matrix<Rows, Cols> &_mat) const
    {
        Matrix<Rows, Cols> res;
        res.eigen_mat_ = this->eigen_mat_ - _mat.eigen_mat_;
        return res;
    }

    template <int OtherCols> Matrix<Rows, OtherCols> operator*(const Matrix<Cols, OtherCols> &other) const
    {
        Matrix<Rows, OtherCols> res;
        res.getEigenMatrix() = this->getEigenMatrix() * other.getEigenMatrix();
        return res;
    }

    bool operator==(const Matrix<Rows, Cols> &other) const
    {
        const float tolerance = 1e-6f;
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                if (std::abs((*this)(i, j) - other(i, j)) > tolerance) {
                    return false;
                }
            }
        }
        return true;
    }

    bool operator!=(const Matrix<Rows, Cols> &other) const { return !(*this == other); }

    template <int rows, int cols> Matrix<rows, cols> block(const int &_start_row, const int &_start_col) const
    {
        Matrix<rows, cols> res;
        for (int i = 0; i < rows; i++) {
            const float *srcPtr = &(*this)(_start_row + i, _start_col);
            float *dstPtr = &res(i, 0);
            memcpy(dstPtr, srcPtr, cols * sizeof(float));
        }
        return res;
    }

    Matrix<Cols, Rows> trans() const { return Matrix<Cols, Rows>(eigen_mat_.transpose()); }

    Matrix<Rows, Cols> inv() const { return Matrix<Rows, Cols>(eigen_mat_.inverse()); }

    // 静态方法
    static Matrix<Rows, Cols> eye() { return Matrix<Rows, Cols>(Eigen::Matrix<float, Rows, Cols>::Identity()); }

    static Matrix<Rows, Cols> zeros() { return Matrix<Rows, Cols>(Eigen::Matrix<float, Rows, Cols>::Zero()); }


    // 转换为Eigen矩阵（用于高级运算）
    Eigen::Matrix<float, Rows, Cols> &eigen() { return eigen_mat_; }
    const Eigen::Matrix<float, Rows, Cols> &eigen() const { return eigen_mat_; }
};
