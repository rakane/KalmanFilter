#ifndef MATRIX_H
#define MATRIX_H

#include <stdio.h>
#include <stdlib.h>

template <unsigned int rows, unsigned int cols>
class Matrix
{
public:
    // Default constructor
    Matrix();

    // Constructor with input matrix
    Matrix(const double matrix[rows][cols]);

    // Destructor
    ~Matrix();

    // Subscript operator
    double* operator[](unsigned int index);

    // Constant subscript operator
    const double* operator[](unsigned int index) const;

    // Addition operator
    Matrix<rows, cols> operator+(Matrix<rows, cols> other) const;

    // Subtraction operator
    Matrix<rows, cols> operator-(Matrix<rows, cols> other) const;

    // Multiplication operator
    template<unsigned int rows2, unsigned int cols2>
    Matrix<rows, cols2> operator*(Matrix<rows2, cols2> other) const;

    // Transpose matrix
    Matrix<rows, cols> transpose() const;

    // LU decomposition
    void luDecomposition(Matrix<rows, cols>& LU, unsigned int P[rows + 1]) const;

    // Inverse matrix
    Matrix<rows, cols> inverse() const;

    // Print matrix
    void print() const;

    // Swap two rows
    void swapRows(unsigned int row1, unsigned int row2);

private:

    // Matrix data
    double matrix_[rows][cols];
};

// Default constructor
template <unsigned int rows, unsigned int cols>
Matrix<rows, cols>::Matrix()
{
    // Initialize matrix to 0
    for(unsigned int i = 0; i < rows; i++) 
    {
        for(unsigned int j = 0; j < cols; j++) 
        {
            matrix_[i][j] = 0.0;
        }
    }
}

// Constructor with input matrix
template <unsigned int rows, unsigned int cols>
Matrix<rows, cols>::Matrix(const double matrix[rows][cols])
{
    // Copy values from input matrix
    for(unsigned int i = 0; i < rows; i++) 
    {
        for(unsigned int j = 0; j < cols; j++) 
        {
            matrix_[i][j] = matrix[i][j];
        }
    }
}

// Destructor
template <unsigned int rows, unsigned int cols>
Matrix<rows, cols>::~Matrix()
{
    // Do nothing
}

// Subscript operator
template <unsigned int rows, unsigned int cols>
double* Matrix<rows, cols>::operator[](unsigned int index)
{
    return matrix_[index];
}

// Constant subscript operator
template <unsigned int rows, unsigned int cols>
const double* Matrix<rows, cols>::operator[](unsigned int index) const
{
    return matrix_[index];
}

// Addition operator
template <unsigned int rows, unsigned int cols>
Matrix<rows, cols> Matrix<rows, cols>::operator+(Matrix<rows, cols> other) const
{
    Matrix<rows, cols> result;

    for(unsigned int i = 0; i < rows; i++) 
    {
        for(unsigned int j = 0; j < cols; j++) 
        {
            result[i][j] = matrix_[i][j] + other[i][j];
        }
    }

    return result;
}

// Subtraction operator
template <unsigned int rows, unsigned int cols>
Matrix<rows, cols> Matrix<rows, cols>::operator-(Matrix<rows, cols> other) const
{
    Matrix<rows, cols> result;

    for(unsigned int i = 0; i < rows; i++) 
    {
        for(unsigned int j = 0; j < cols; j++) 
        {
            result[i][j] = matrix_[i][j] - other[i][j];
        }
    }

    return result;
}

// Multiplication operator
template <unsigned int rows, unsigned int cols>
template<unsigned int rows2, unsigned int cols2>
Matrix <rows, cols2> Matrix<rows, cols>::operator*(Matrix<rows2, cols2> other) const
{
    Matrix<rows, cols2> result;

    for(unsigned int i = 0; i < rows; i++) 
    {
        for(unsigned int j = 0; j < cols2; j++) 
        {
            double sum = 0;

            for(unsigned int k = 0; k < cols; k++) 
            {
                sum += matrix_[i][k] * other[k][j];
            }

            result[i][j] = sum;
        }
    }

    return result;
}

// Transpose matrix
template <unsigned int rows, unsigned int cols>
Matrix<rows, cols> Matrix<rows, cols>::transpose() const
{
    Matrix<rows, cols> result;

    for(unsigned int  i = 0; i < rows; i++) 
    {
        for(unsigned int j = 0; j < cols; j++) 
        {
            result[j][i] = matrix_[i][j];
        }
    }

    return result;
}

// LU decomposition using algorithm at https://en.wikipedia.org/wiki/LU_decomposition#C_code_example
template <unsigned int rows, unsigned int cols>
void Matrix<rows, cols>::luDecomposition(Matrix<rows, cols>& LU, unsigned int P[rows + 1]) const
{
    if(rows != cols)
    {
        std::cout << "Error: matrix is not square" << std::endl;
        return;
    }

    // Copy matrix to LU
    for(unsigned int i = 0; i < rows; i++) 
    {
        for(unsigned int j = 0; j < cols; j++) 
        {
            LU[i][j] = matrix_[i][j];
        }
    }

    const unsigned int size = rows;

    unsigned int imax; 
    double maxA, absA;

    for(unsigned int i = 0; i <= size; i++)
    {
        P[i] = i; // Unit permutation matrix, P[i] initialized with i
    }

    for(unsigned int i = 0; i < size; i++) {
        maxA = 0.0;
        imax = i;

        for(unsigned int k = i; k < size; k++)
        {
            absA = std::abs(LU[k][i]);
            if (absA > maxA) { 
                maxA = absA;
                imax = k;
            }
        }

        if(imax != i) {
            // pivoting P
            unsigned int temp = P[i];
            P[i] = P[imax];
            P[imax] = temp;

            // pivoting rows of A
            LU.swapRows(i, imax);

            //counting pivots starting from N (for determinant)
            P[size]++;
        }

        for(unsigned int j = i + 1; j < size; j++) 
        {
            LU[j][i] /= LU[i][i];

            for(unsigned int k = i + 1; k < size; k++)
            {
                LU[j][k] -= LU[j][i] * LU[i][k];
            }
        }
    }
}

// Inverse matrix using LU decomposition algorithm 
// at https://en.wikipedia.org/wiki/LU_decomposition#C_code_example
template <unsigned int rows, unsigned int cols>
Matrix<rows, cols> Matrix<rows, cols>::inverse() const
{
    Matrix<rows, cols> result;

    if(rows != cols)
    {
        std::cout << "Error: matrix is not square" << std::endl;
        return result;
    }

    const unsigned int size = rows;

    // LU decomposition
    Matrix<rows, cols> LU;
    unsigned int P[rows + 1];
    luDecomposition(LU, P);

    // Invert matrix
    for(unsigned int j = 0; j < size; j++) 
    {
        for(unsigned int i = 0; i < size; i++) 
        {
            result[i][j] = P[i] == j ? 1.0 : 0.0;

            for(unsigned int k = 0; k < i; k++)
            {
                result[i][j] -= matrix_[i][k] * result[k][j];
            }
        }

        for(int i = size - 1; i >= 0; i--) 
        {
            for(unsigned int k = i + 1; k < size; k++)
            {
                result[i][j] -= matrix_[i][k] * result[k][j];
            }

            result[i][j] /= matrix_[i][i];
        }
    }
    
    return result;
}

// Print matrix
template <unsigned int rows, unsigned int cols>
void Matrix<rows, cols>::print() const
{
    for(unsigned int i = 0; i < rows; i++) 
    {
        for(unsigned int j = 0; j < cols; j++) 
        {
            std::cout << matrix_[i][j] << "  ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
}

// Swap two rows
template <unsigned int rows, unsigned int cols>
void Matrix<rows, cols>::swapRows(unsigned int row1, unsigned int row2)
{
    double temp[cols];

    for(unsigned int i = 0; i < cols; i++)
    {
        temp[i] = matrix_[row1][i];
        matrix_[row1][i] = matrix_[row2][i];
        matrix_[row2][i] = temp[i];
    }
}

#endif // MATRIX_H
