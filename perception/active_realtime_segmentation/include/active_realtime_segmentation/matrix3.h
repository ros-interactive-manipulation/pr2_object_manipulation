/*
 * Copyright (c) 2011, Mårten Björkman (celle@csc.kth.se) 
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  1.Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  2.Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.  
 *  3.The name of Mårten Björkman may not be used to endorse or
 *    promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MATRIX3_H
#define MATRIX3_H
/** @file matrix3.h
    Contains simple low dimensional vector and matrix classes*/

#include <iostream>

/// Two dimensional Vector Class
class Vector2 {
public:
  /// Storage or the two dimensions
  double x[2];

  /// Constructor
  Vector2() {
    x[0] = x[1] = 0.0;
  }

  /// Access Elements
  /** @param idx Index
   *  @return Value
   */
  inline double &operator[](int idx) {
    return x[idx];
  } 

  /// Access Elements
  /** @param idx Index
   *  @return Value
   */
  inline double operator[](int idx) const {
    return x[idx];
  } 

  /// Access Elements
  /** @param idx Index
   *  @return Value
   */
  inline double &operator()(int idx) {
    return x[idx];
  } 

  /// Vector multiplication with a scalar
  /** @param v Scalar
   *  @return Result Vector
   */
  Vector2 operator*(double v) {
    Vector2 res;
    for (int i=0;i<2;i++)
      res.x[i] = x[i]*v;
    return res;
  }

  /// Set Vector with one scalar value
  /** @param v Scalar
   *  @return Result Vector
   */
  Vector2 &operator=(double v) {
    for (int i=0;i<2;i++)
      x[i] = v;
    return *this;
  }
  
  /// Vector multiplication with a Scalar and output into itself
  /** @param v Scalar
   *  @return Result Vector
   */
  Vector2 &operator*=(double v) {
    for (int i=0;i<2;i++)
      x[i] *= v;
    return *this;
  }
  
  /// Adding a Scalar to a Vector and output into itself
  /** @param v Scalar
   *  @return Result Vector
   */
  Vector2 &operator+=(double v) {
    for (int i=0;i<2;i++)
      x[i] += v;
    return *this;
  }
  
  /// Vector Addition and output into itself
  /** @param v Vector
   *  @return Result Vector
   */
  Vector2 &operator+=(const Vector2 &v) {
    for (int i=0;i<2;i++)
      x[i] += v[i];
    return *this;
  }
  
  /// Print out a vector
  /** @param os Outputstream
   *  @param v Vector
   *  @return print out
   */
  friend std::ostream &operator<<(std::ostream &os, const Vector2 &v) {
    os << "[ " << v[0] << " " << v[1] << " ]";
    return os;
  }
};

/// Three dimensional Vector class
class Vector3 {
public:
  /// Storage for three dimensions
  double x[3];

  /// Constructor
  Vector3() {
    x[0] = x[1] = x[2] = 0.0;
  }

   /// Access Elements
  /** @param idx Index
   *  @return Value
   */
  inline double &operator[](int idx) {
    return x[idx];
  } 
  
  /// Access Elements
  /** @param idx Index
   *  @return Value
   */
  inline double operator[](int idx) const {
    return x[idx];
  } 

  /// Access Elements
  /** @param idx Index
   *  @return Value
   */
  inline double &operator()(int idx) {
    return x[idx];
  } 
  
   /// Vector multiplication with a scalar
  /** @param v Scalar
   *  @return Result Vector
   */
  Vector3 operator*(double v) {
    Vector3 res;
    for (int i=0;i<3;i++)
      res.x[i] = x[i]*v;
    return res;
  }

  /// Set Vector with one scalar value
  /** @param v Scalar
   *  @return Result Vector
   */
  Vector3 &operator=(double v) {
    for (int i=0;i<3;i++)
      x[i] = v;
    return *this;
  }

  /// Vector multiplication with a Scalar and output into itself
  /** @param v Scalar
   *  @return Result Vector
   */
  Vector3 &operator*=(double v) {
    for (int i=0;i<3;i++)
      x[i] *= v;
    return *this;
  }

  /// Adding a Scalar to a Vector and output into itself
  /** @param v Scalar
   *  @return Result Vector
   */
  Vector3 &operator+=(double v) {
    for (int i=0;i<3;i++)
      x[i] += v;
    return *this;
  }

  /// Vector Addition and output into itself
  /** @param v Vector
   *  @return Result Vector
   */
  Vector3 &operator+=(const Vector3 &v) {
    for (int i=0;i<3;i++)
      x[i] += v[i];
    return *this;
  }

  /// Print out a vector
  /** @param os Outputstream
   *  @param v Vector
   *  @return print out
   */
  friend std::ostream &operator<<(std::ostream &os, const Vector3 &v) {
    os << "[ " << v[0] << " " << v[1] << " " << v[2] << " ]";
    return os;
  }
};

/// 2x2 Matrix class
class Matrix2 {
public:
  /// Storage for 2 x 2 Matrix
  Vector2 x[2];

  /// Constructor Initialising to Identity Matrix 
  Matrix2() {  
    identity();
  }

  /// Copy Constructor
  /** @param m 2x2 Matrix
   */
  Matrix2(const Matrix2 &m) {
    for (int i=0;i<2;i++) 
      x[i] = m.x[i];
  }

  /// Copy Matrix values from new matrix
  /** @param m 2x2 Matrix
   */
  Matrix2 &operator=(const Matrix2 &m) {
    for (int i=0;i<2;i++) 
      x[i] = m.x[i];
    return *this;
  }

  /// Set Matrix to Identity Matrix 
  void identity() {
    x[0][0] = x[1][1] = 1.0;
    x[0][1] = x[1][0] = 0.0;
  }

  /// Row Element Access 
  /** @param idx row index
   *  @return row as two dimensional vector
   */
  inline Vector2 &operator[](int idx) {
    return x[idx];
  } 

  /// Row Element Access 
  /** @param idx row index
   *  @return row as two dimensional vector
   */
  inline Vector2 operator[](int idx) const {
    return x[idx];
  } 

  /// Element Access 
  /** @param r row index
   *  @param c column index
   *  @return element value
   */
  inline double &operator()(int r, int c) {
    return x[r][c];
  }

  /// Determinant of matrix
  /** @return Determinant */
  double determinant() {
    return x[0][0]*x[1][1] - x[0][1]*x[1][0];
  }

  /// Invert a matrix
  /** @return Inverted Matrix */
  Matrix2 invert() {
    double det = determinant();
    Matrix2 res;
    if (det!=0.0) {
      double idet = 1.0/det;
      res.x[0][0] =  idet*x[1][1];
      res.x[0][1] = -idet*x[0][1];
      res.x[1][0] = -idet*x[1][0];
      res.x[1][1] =  idet*x[0][0];
    }
    return res;
  }

  /// Matrix Multiplication
  /** @param m Matrix to be multiplied with  
   *  @return Result Matrix */
  Matrix2 operator*(const Matrix2 &m) {
    Matrix2 res;
    for (int j=0;j<2;j++)
      for (int i=0;i<2;i++)
	res.x[j][i] = x[j][0]*m.x[0][i] + x[j][1]*m.x[1][i];
    return res;
  }

  /// Matrix Vector multiplication
  /** @param v Vector to be multiplied with  
   *  @return Result Vector */
  Vector2 operator*(const Vector2 &v) {
    Vector2 res;
    for (int i=0;i<2;i++)
      res.x[i] = x[i][0]*v.x[0] + x[i][1]*v.x[1];
    return res;
  }

  /// Scalar matrix multiplication
  /** @param v Scalar value to be multiplied with  
   *  @return Result Matrix */
  Matrix2 operator*(double v) {
    Matrix2 res;
    for (int i=0;i<2;i++)
      res.x[i] = x[i]*v;
    return res;
  }

  /// Set all Matrix Elements to scalar value
  /** @param v Scalar value to be set to
   *  @return Result Matrix */
  Matrix2 &operator=(double v) {
    for (int i=0;i<2;i++)
      x[i] = v;
    return *this;
  }

  /// Scalar matrix multiplication and output into itself
  /** @param v Scalar value to be multiplied with  
   *  @return Result Matrix */
  Matrix2 &operator*=(double v) {
    for (int i=0;i<2;i++)
      x[i] *= v;
    return *this;
  }
  
  /// Matrix Addition and output into itself
  /** @param m Matrix to be added  
   *  @return Result Matrix */
  Matrix2 &operator+=(const Matrix2 &m) {
    for (int i=0;i<2;i++)
      x[i] += m[i];
    return *this;
  }

  /// Add scalar to every matrix element and output into itself
  /** @param v Scalar value to be added 
   *  @return Result Matrix */
  Matrix2 &operator+=(double v) {
    for (int i=0;i<2;i++)
      x[i] += v;
    return *this;
  }

  /// Print out matrix
  /** @param os output stream
   *  @param m  Matrix to be printed
   *  @return Matrix printed to output stream
   */
  friend std::ostream &operator<<(std::ostream &os, const Matrix2 &m) {
    os << "[ " << m[0][0] << " " << m[0][1] << " ;";
    os << "  " << m[1][0] << " " << m[1][1] << " ]";
    return os;
  }
}; 

/// 3x3 Matrix class
class Matrix3 {
public:
   /// Storage for 3 x 3 Matrix
  Vector3 x[3];
  
  /// Constructor Initialising to Identity Matrix 
  Matrix3() {  
    identity();
  }

  /// Copy Constructor
  /** @param m 3x3 Matrix
   */
  Matrix3(const Matrix3 &m) {
    for (int i=0;i<3;i++) 
      x[i] = m.x[i];
  }
  
  /// Copy Matrix values from new matrix
  /** @param m 3x3 Matrix
   */
  Matrix3 &operator=(const Matrix3 &m) {
    for (int i=0;i<3;i++) 
      x[i] = m.x[i];
    return *this;
  }

  /// Set matrix to identity
  void identity() {
    x[0][0] = x[1][1] = x[2][2] = 1.0;
    x[0][1] = x[0][2] = x[1][0] = 0.0;
    x[1][0] = x[2][0] = x[2][1] = 0.0;
  }
  
  /// Row Element Access 
  /** @param idx row index
   *  @return row as three dimensional vector
   */
  inline Vector3 &operator[](int idx) {
    return x[idx];
  } 

  /// Row Element Access 
  /** @param idx row index
   *  @return row as three dimensional vector
   */
  inline Vector3 operator[](int idx) const {
    return x[idx];
  } 
  
  /// Element Access 
  /** @param r row index
   *  @param c column index
   *  @return element value
   */
  inline double &operator()(int r, int c) {
    return x[r][c];
  }

  /// Determinant
   /** @return Determinant */
  double determinant() {
    return (x[0][0]*(x[1][1]*x[2][2] - x[2][1]*x[1][2]) - 
	    x[1][0]*(x[2][1]*x[0][2] - x[0][1]*x[2][2]) +
	    x[2][0]*(x[0][1]*x[1][2] - x[1][1]*x[0][2]));         
  }

  /// Invert Matrix
  /** @return Inverted Matrix */
  Matrix3 invert() { 
    double m00 = x[1][1]*x[2][2] - x[2][1]*x[1][2];
    double m01 = x[2][1]*x[0][2] - x[0][1]*x[2][2];
    double m02 = x[0][1]*x[1][2] - x[1][1]*x[0][2];
    double det = x[0][0]*m00 + x[1][0]*m01 + x[2][0]*m02;
    Matrix3 res;
    if (det!=0.0) {
      double idet = 1.0/det;
      res.x[0][0] = idet*m00;
      res.x[0][1] = idet*m01;
      res.x[0][2] = idet*m02;         
      res.x[1][0] = idet*(x[2][0]*x[1][2] - x[1][0]*x[2][2]);
      res.x[1][1] = idet*(x[0][0]*x[2][2] - x[2][0]*x[0][2]);        
      res.x[1][2] = idet*(x[1][0]*x[0][2] - x[0][0]*x[1][2]);
      res.x[2][0] = idet*(x[1][0]*x[2][1] - x[2][0]*x[1][1]);
      res.x[2][1] = idet*(x[2][0]*x[0][1] - x[0][0]*x[2][1]);
      res.x[2][2] = idet*(x[0][0]*x[1][1] - x[1][0]*x[0][1]);         
    }
    return res;
  }
  
  /// Matrix Multiplication
  /** @param m Matrix to be multiplied with  
   *  @return Result Matrix */
  Matrix3 operator*(const Matrix3 &m) {
    Matrix3 res;
    for (int j=0;j<3;j++)
      for (int i=0;i<3;i++)
	res.x[j][i] = x[j][0]*m.x[0][i] + x[j][1]*m.x[1][i] + x[j][2]*m.x[2][i];
    return res;
  }
  
  /// Matrix Vector multiplication
  /** @param v Vector to be multiplied with  
   *  @return Result Vector */
  Vector3 operator*(const Vector3 &v) {
    Vector3 res;
    for (int i=0;i<3;i++)
      res.x[i] = x[i][0]*v.x[0] + x[i][1]*v.x[1] + x[i][2]*v.x[2];
    return res;
  }
  
  /// Scalar matrix multiplication
  /** @param v Scalar value to be multiplied with  
   *  @return Result Matrix */
  Matrix3 operator*(double v) {
    Matrix3 res;
    for (int i=0;i<3;i++)
      res.x[i] = x[i]*v;
    return res;
  }
  
  /// Set all Matrix Elements to scalar value
  /** @param v Scalar value to be set to
   *  @return Result Matrix */
  Matrix3 &operator=(double v) {
    for (int i=0;i<3;i++)
      x[i] = v;
    return *this;
  }
  
  /// Scalar matrix multiplication and output into itself
  /** @param v Scalar value to be multiplied with  
   *  @return Result Matrix */
  Matrix3 &operator*=(double v) {
    for (int i=0;i<3;i++)
      x[i] *= v;
    return *this;
  }

  /// Matrix Addition and output into itself
  /** @param m Matrix to be added  
   *  @return Result Matrix */
  Matrix3 &operator+=(const Matrix3 &m) {
    for (int i=0;i<3;i++)
      x[i] += m[i];
    return *this;
  }
  
  /// Add scalar to every matrix element and output into itself
  /** @param v Scalar value to be added 
   *  @return Result Matrix */
  Matrix3 &operator+=(double v) {
    for (int i=0;i<3;i++)
      x[i] += v;
    return *this;
  }

  /// Extract 2x2 submatrix
  /** @param r row index
   *  @param c column index
   *  @return 2x2 matrix
   */
  Matrix2 submatrix(int r, int c) {
    Matrix2 res;
    res.x[0][0] = x[r][c];
    res.x[0][1] = x[r][c+1];
    res.x[1][0] = x[r+1][c];
    res.x[1][1] = x[r+1][c+1];
    return res;
  }


  /// Print out matrix
  /** @param os output stream
   *  @param m  Matrix to be printed
   *  @return Matrix printed to output stream
   */
  friend std::ostream &operator<<(std::ostream &os, const Matrix3 &m) {
    os << "[ " << m[0][0] << " " << m[0][1] << " " << m[0][2] << " ;";
    os << "  " << m[1][0] << " " << m[1][1] << " " << m[1][2] << " ;";
    os << "  " << m[2][0] << " " << m[2][1] << " " << m[2][2] << " ]";
    return os;
  }
}; 

#endif


