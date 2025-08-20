#pragma once

#include "dsp/matrix_functions.h"


template <int _rows, int _cols> class Matrix {
public:
    Matrix() : rows_(_rows), cols_(_cols)
    {
        arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
    }

    Matrix(float _data[_rows * _cols]) : Matrix()
    {
        memcpy(this->data_, _data, _rows * _cols * sizeof(float));
        arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
    }

    /**
     * @brief      Copy Constructor
     * @param mat  The copied matrix
     */
    Matrix(const Matrix<_rows, _cols> &_mat) : Matrix()
    {
        memcpy(this->data_, _mat.data_, _rows * _cols * sizeof(float));
        arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
    }

    ~Matrix() {}

    uint32_t rows() const { return _rows; }

    uint32_t cols() const { return _cols; }

    float *operator[](const int &_row) { return &this->data_[_row * _cols]; }

    Matrix<_rows, _cols> &operator=(const Matrix<_rows, _cols> _mat)
    {
        if (this == &_mat)
            return *this; // 检查自赋值
        memcpy(this->data_, _mat.data_, _rows * _cols * sizeof(float));
        return *this;
    }

    Matrix<_rows, _cols> &operator+=(const Matrix<_rows, _cols> _mat)
    {
        arm_mat_add_f32(&this->arm_mat_, &_mat.arm_mat_, &this->arm_mat_);
        return *this;
    }

    Matrix<_rows, _cols> &operator-=(const Matrix<_rows, _cols> _mat)
    {
        arm_mat_sub_f32(&this->arm_mat_, &_mat.arm_mat_, &this->arm_mat_);
        return *this;
    }

    Matrix<_rows, _cols> &operator*=(const float &_val)
    {
        arm_mat_scale_f32(&this->arm_mat_, _val, &this->arm_mat_);
        return *this;
    }

    Matrix<_rows, _cols> &operator/=(const float &_val)
    {
        arm_mat_scale_f32(&this->arm_mat_, 1.f / _val, &this->arm_mat_);
        return *this;
    }

    Matrix<_rows, _cols> operator+(const Matrix<_rows, _cols> &_mat) const
    {
        Matrix<_rows, _cols> res;
        arm_mat_add_f32(&this->arm_mat_, &_mat.arm_mat_, &res.arm_mat_);
        return res;
    }

    Matrix<_rows, _cols> operator-(const Matrix<_rows, _cols> &_mat) const
    {
        Matrix<_rows, _cols> res;
        arm_mat_sub_f32(&this->arm_mat_, &_mat.arm_mat_, &res.arm_mat_);
        return res;
    }

    Matrix<_rows, _cols> operator*(const float &_val) const
    {
        Matrix<_rows, _cols> res;
        arm_mat_scale_f32(&this->arm_mat_, _val, &res.arm_mat_);
        return res;
    }

    friend Matrix<_rows, _cols> operator*(const float &_val,
                                          const Matrix<_rows, _cols> &_mat)
    {
        arm_status s;
        Matrix<_rows, _cols> res;
        s = arm_mat_scale_f32(&_mat.arm_mat_, _val, &res.arm_mat_);
        return res;
    }

    Matrix<_rows, _cols> operator/(const float &_val) const
    {
        Matrix<_rows, _cols> res;
        arm_mat_scale_f32(&this->arm_mat_, 1.f / _val, &res.arm_mat_);
        return res;
    }

    template <int cols2>
    friend Matrix<_rows, cols2> operator*(const Matrix<_rows, _cols> &_mat1,
                                          const Matrix<_cols, cols2> &_mat2)
    {
        Matrix<_rows, cols2> res;
        arm_mat_mult_f32(&_mat1.arm_mat_, &_mat2.arm_mat_, &res.arm_mat_);
        return res;
    }

    bool operator==(const Matrix<_rows, _cols> &_mat) const
    {
        for (int i = 0; i < _rows * _cols; i++) {
            if (this->data_[i] != _mat.data_[i])
                return false;
        }
        return true;
    }

    // Submatrix
    template <int rows, int cols>
    Matrix<rows, cols> block(const int &_start_row, const int &_start_col) const
    {
        Matrix<rows, cols> res;
        for (int row = _start_row; row < _start_row + rows; row++) {
            memcpy((float *)res[0] + ((row - _start_row) * cols),
                   (float *)this->data_ + (row * _cols) + _start_col,
                   cols * sizeof(float));
        }
        return res;
    }

    Matrix<1, _cols> row(const int &_row) const
    {
        return block<1, _cols>(_row, 0);
    } //返回指定列

    Matrix<_rows, 1> col(const int &_col) const
    {
        return block<_rows, 1>(0, _col);
    } //返回指定行

    Matrix<_cols, _rows> trans() const //转置矩阵
    {
        Matrix<_cols, _rows> res;
        arm_mat_trans_f32(&arm_mat_, &res.arm_mat_);
        return res;
    }

    // Trace
    float trace() const
    {
        float res = 0;
        for (int i = 0; i < fmin(_rows, _cols); i++) {
            res += (*this)[i][i];
        }
        return res;
    }

    float norm() const { return sqrtf((this->trans() * *this)[0][0]); }

    Matrix<_cols, _rows> inv() const //求逆矩阵
    {
        if (_cols != _rows)
            return Matrix<_cols, _rows>::zeros();

        Matrix<_cols, _rows> res;
        arm_status status = arm_mat_inverse_f32(&this->arm_mat_, &res.arm_mat_);

        if (status == ARM_MATH_SINGULAR)
            return Matrix<_cols, _rows>::zeros();

        return res;
    }

    static Matrix<_rows, _cols> zeros()
    {
        float data[_rows * _cols] = { 0 };
        return Matrix<_rows, _cols>(data);
    }

    static Matrix<_rows, _cols> ones()
    {
        float data[_rows * _cols] = { 0 };
        for (int i = 0; i < _rows * _cols; i++) {
            data[i] = 1;
        }
        return Matrix<_rows, _cols>(data);
    }

    static Matrix<_rows, _cols> eye() //unit matrix
    {
        float data[_rows * _cols] = { 0 };
        for (int i = 0; i < fmin(_rows, _cols); i++) {
            data[(i * _cols) + i] = 1;
        }
        return Matrix<_rows, _cols>(data);
    }

    static Matrix<_rows, _cols> diag(Matrix<_rows, 1> _vec)
    {
        Matrix<_rows, _cols> res = Matrix<_rows, _cols>::zeros();
        for (int i = 0; i < fmin(_rows, _cols); i++) {
            res[i][i] = _vec[i][0];
        }
        return res;
    }

public:
    arm_matrix_instance_f32 arm_mat_; // The arm math instance

protected:
    int rows_, cols_;
    float data_[_rows * _cols];
};
