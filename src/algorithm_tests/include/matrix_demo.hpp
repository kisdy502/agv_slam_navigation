#ifndef MATRIX_DEMO_HPP
#define MATRIX_DEMO_HPP

#include <iostream>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <iomanip>
#include <Eigen/Dense>

// ============ 手写简化矩阵类 ============
template<typename T>
class SimpleMatrix {
public:
    SimpleMatrix(size_t rows, size_t cols) 
        : rows_(rows), cols_(cols), data_(rows * cols, T(0)) {}
    
    SimpleMatrix(const std::initializer_list<std::initializer_list<T>>& init) {
        rows_ = init.size();
        cols_ = init.begin()->size();
        data_.reserve(rows_ * cols_);
        for (const auto& row : init) {
            if (row.size() != cols_) throw std::invalid_argument("Inconsistent row size");
            for (const auto& val : row) data_.push_back(val);
        }
    }
    
    T& operator()(size_t i, size_t j) { return data_[i * cols_ + j]; }
    const T& operator()(size_t i, size_t j) const { return data_[i * cols_ + j]; }
    
    size_t rows() const { return rows_; }
    size_t cols() const { return cols_; }
    
    void print(const std::string& name = "") const {
        if (!name.empty()) std::cout << name << " =\n";
        for (size_t i = 0; i < rows_; ++i) {
            std::cout << "  [";
            for (size_t j = 0; j < cols_; ++j) {
                std::cout << std::setw(10) << std::fixed << std::setprecision(4) << (*this)(i, j);
                if (j < cols_ - 1) std::cout << ", ";
            }
            std::cout << " ]\n";
        }
        std::cout << "\n";
    }
    
    SimpleMatrix operator+(const SimpleMatrix& other) const {
        if (rows_ != other.rows_ || cols_ != other.cols_)
            throw std::invalid_argument("Dimension mismatch for addition");
        SimpleMatrix result(rows_, cols_);
        for (size_t i = 0; i < data_.size(); ++i)
            result.data_[i] = data_[i] + other.data_[i];
        return result;
    }
    
    SimpleMatrix operator-(const SimpleMatrix& other) const {
        if (rows_ != other.rows_ || cols_ != other.cols_)
            throw std::invalid_argument("Dimension mismatch for subtraction");
        SimpleMatrix result(rows_, cols_);
        for (size_t i = 0; i < data_.size(); ++i)
            result.data_[i] = data_[i] - other.data_[i];
        return result;
    }
    
    SimpleMatrix operator*(T scalar) const {
        SimpleMatrix result(rows_, cols_);
        for (size_t i = 0; i < data_.size(); ++i)
            result.data_[i] = data_[i] * scalar;
        return result;
    }
    
    SimpleMatrix operator*(const SimpleMatrix& other) const {
        if (cols_ != other.rows_)
            throw std::invalid_argument("Dimension mismatch for multiplication");
        SimpleMatrix result(rows_, other.cols_);
        for (size_t i = 0; i < rows_; ++i) {
            for (size_t j = 0; j < other.cols_; ++j) {
                T sum = 0;
                for (size_t k = 0; k < cols_; ++k) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }
    
    SimpleMatrix transpose() const {
        SimpleMatrix result(cols_, rows_);
        for (size_t i = 0; i < rows_; ++i)
            for (size_t j = 0; j < cols_; ++j)
                result(j, i) = (*this)(i, j);
        return result;
    }
    
    T determinant() const {
        if (rows_ != cols_) throw std::invalid_argument("Must be square matrix");
        if (rows_ == 2) {
            return (*this)(0,0) * (*this)(1,1) - (*this)(0,1) * (*this)(1,0);
        }
        if (rows_ == 3) {
            return (*this)(0,0) * ((*this)(1,1)*(*this)(2,2) - (*this)(1,2)*(*this)(2,1))
                 - (*this)(0,1) * ((*this)(1,0)*(*this)(2,2) - (*this)(1,2)*(*this)(2,0))
                 + (*this)(0,2) * ((*this)(1,0)*(*this)(2,1) - (*this)(1,1)*(*this)(2,0));
        }
        throw std::invalid_argument("Only 2x2 and 3x3 supported");
    }
    
    SimpleMatrix inverse() const {
        if (rows_ != cols_) throw std::invalid_argument("Must be square matrix");
        T det = determinant();
        if (std::abs(det) < 1e-10) throw std::runtime_error("Singular matrix, no inverse");
        
        if (rows_ == 2) {
            SimpleMatrix result(2, 2);
            result(0,0) = (*this)(1,1) / det;
            result(0,1) = -(*this)(0,1) / det;
            result(1,0) = -(*this)(1,0) / det;
            result(1,1) = (*this)(0,0) / det;
            return result;
        }
        throw std::invalid_argument("Only 2x2 inverse supported");
    }
    
    static SimpleMatrix identity(size_t n) {
        SimpleMatrix result(n, n);
        for (size_t i = 0; i < n; ++i) result(i, i) = T(1);
        return result;
    }
    
    static SimpleMatrix zero(size_t rows, size_t cols) {
        return SimpleMatrix(rows, cols);
    }

private:
    size_t rows_, cols_;
    std::vector<T> data_;
};

template<typename T>
SimpleMatrix<T> operator*(T scalar, const SimpleMatrix<T>& mat) {
    return mat * scalar;
}

// ============ MatrixDemo 测试类 ============
class MatrixDemo {
public:
    static void test();
};

#endif // MATRIX_DEMO_HPP
