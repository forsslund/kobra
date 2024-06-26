//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of H3DUtil.
//
//    H3DUtil is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3DUtil is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3DUtil; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file Matrix3f.h
/// \brief Header file for Matrix3f.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __MATRIX3F_H__
#define __MATRIX3F_H__

#include <H3DUtil/H3DUtil.h>
#include <H3DUtil/H3DBasicTypes.h>
#include <H3DUtil/Vec3f.h>
#include <H3DUtil/Exception.h>

namespace H3DUtil {
  namespace ArithmeticTypes {
    /// forward declarations.
    class Quaternion;
    class Rotation;
    class Matrix3d;

    /// Representation of a 3x3 matrix.
    /// \ingroup H3DUtilBasicTypes
    class H3DUTIL_API Matrix3f {
    public:
      /// Thrown when trying to perform operations on a singular matrix
      /// that are not allowed.
      H3D_API_EXCEPTION( SingularMatrix3f );

      /// Constructor. Creates an identity matrix.
      inline Matrix3f() {
        setToIdentity();
      }

      /// Constructor.
      inline Matrix3f( H3DFloat m00, H3DFloat m01, H3DFloat m02,
                       H3DFloat m10, H3DFloat m11, H3DFloat m12,
                       H3DFloat m20, H3DFloat m21, H3DFloat m22 ) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; 
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; 
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; 
      }

      /// Constructor. From Rotation object.
      explicit Matrix3f( const Rotation &r );

      /// Constructor. From Quaternion object.
      explicit Matrix3f( const Quaternion &q );

      /// Constructor. From Matrix3d.
      explicit Matrix3f( const Matrix3d &v );

      /// Set the Matrix3f to the identity matrix.
      inline void setToIdentity() {
        m[0][0] = 1; m[0][1] = 0; m[0][2] = 0;
        m[1][0] = 0; m[1][1] = 1; m[1][2] = 0;
        m[2][0] = 0; m[2][1] = 0; m[2][2] = 1;
      }

      /// Returns the transpose of the matrix.
      inline Matrix3f transpose() const {
        return Matrix3f( m[0][0], m[1][0], m[2][0],
                         m[0][1], m[1][1], m[2][1],
                         m[0][2], m[1][2], m[2][2] );
      };

      /// Returns the inverse of the matrix.
      Matrix3f inverse() const;

      /// Returns an "inverse" of the matrix regardless of whether the matrix is
      /// a singular matrix or not. When the matrix is not a singular matrix this
      /// function will behave like the inverse function.
      /// When the matrix is a singular matrix and hence not
      /// possible to invert by definition then the zero-value-determinant is replaced with
      /// and epsilon value. This gives an "inverse" of the matrix that can in some cases
      /// be used and get good enough results. It is up to the user of this function to make
      /// sure that this hack is acceptable in any given situation. Use at your own peril.
      Matrix3f inverseSafe() const;

      /// Get a row of the matrix.
      inline H3DFloat* operator[]( int i ) { return m[i]; }

      /// Get a row of the matrix.
      inline const H3DFloat* operator[]( int i ) const { return m[i]; }

      /// Get a row of the matrix.
      inline Vec3f getRow( int i ) const { 
        return Vec3f( m[i][0], m[i][1], m[i][2] ); 
      }

      /// Get an element in the matrix.
      inline H3DFloat getElement( int i, int j ) const { 
        return m[i][j];
      }

      /// Set an element in the matrix.
      inline void setElement( int i, int j, H3DFloat v ) { 
        m[i][j] = v;
      }

      /// Get the scaling part of the matrix for each axis.
      Vec3f getScalePart() const;

      /// Get the euler angles( yaw, pitch, roll ) representation of 
      /// the rotation matrix. The Matrix3f must be a rotation matrix.
      Vec3f toEulerAngles();

      /// Get a column of the matrix.
      inline Vec3f getColumn( int i ) const { 
        return Vec3f( m[0][i], m[1][i], m[2][i] ); 
      }

    private:
      /// The matrix data.
      H3DFloat m[3][3];

    public:
      /// Per-element precision float comparison against an epsilon value.
      inline bool nearEqual( const Matrix3f &rhs, const H3DFloat epsilon = std::numeric_limits< H3DFloat >::epsilon() ) const {
        for( unsigned int i = 0; i < 3; ++i ) {
          for( unsigned int j = 0; j < 3; ++j ) {
            if( !epsilonCompare( m[i][j], rhs[i][j], epsilon ) ) {
              return false;
            }
          }
        }
        return true;
      }
    };
    
    /// \defgroup Matrix3fOperators Matrix3f operators.
    /// \brief Operators on Matrix3f instances. See also the 
    /// \ref TemplateOperators "template operators" for more operators
    /// automatically defined from the explicit ones defined here.
    /// \ingroup H3DUtilBasicTypes
    /// \{

    /// Multiplication between two Matrix3f instances.
    inline Matrix3f operator*( const Matrix3f &m1, const Matrix3f &m2 ) {
      return Matrix3f( 
                      m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0] + m1[0][2]*m2[2][0],
                      m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1] + m1[0][2]*m2[2][1],
                      m1[0][0]*m2[0][2] + m1[0][1]*m2[1][2] + m1[0][2]*m2[2][2],
                      m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0] + m1[1][2]*m2[2][0],
                      m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1] + m1[1][2]*m2[2][1],
                      m1[1][0]*m2[0][2] + m1[1][1]*m2[1][2] + m1[1][2]*m2[2][2],
                      m1[2][0]*m2[0][0] + m1[2][1]*m2[1][0] + m1[2][2]*m2[2][0],
                      m1[2][0]*m2[0][1] + m1[2][1]*m2[1][1] + m1[2][2]*m2[2][1],
                      m1[2][0]*m2[0][2] + m1[2][1]*m2[1][2] + m1[2][2]*m2[2][2] );
    }
    
    /// Addition between two Matrix3f instances.
    inline Matrix3f operator+( const Matrix3f &m1, const Matrix3f &m2 ) {
      return Matrix3f( 
                      m1[0][0]+m2[0][0], m1[0][1]+m2[0][1], m1[0][2]+m2[0][2],
                      m1[1][0]+m2[1][0], m1[1][1]+m2[1][1], m1[1][2]+m2[1][2],
                      m1[2][0]+m2[2][0], m1[2][1]+m2[2][1], m1[2][2]+m2[2][2] );
    }

    /// Muliplication between Matrix3f and float. 
    inline Matrix3f operator*( const Matrix3f &m, const float &f ) {
      return Matrix3f( m[0][0]*f, m[0][1]*f, m[0][2]*f,
                       m[1][0]*f, m[1][1]*f, m[1][2]*f,
                       m[2][0]*f, m[2][1]*f, m[2][2]*f );
    }

    /// Muliplication between Matrix3f and double. 
    inline Matrix3f operator*( const Matrix3f &m, const double &d ) {
      return Matrix3f( 
   (H3DFloat)(m[0][0]*d), (H3DFloat)(m[0][1]*d), (H3DFloat)(m[0][2]*d),
   (H3DFloat)(m[1][0]*d), (H3DFloat)(m[1][1]*d), (H3DFloat)(m[1][2]*d),
   (H3DFloat)(m[2][0]*d), (H3DFloat)(m[2][1]*d), (H3DFloat)(m[2][2]*d ));
    }

    /// Muliplication between Matrix3f and int. 
    inline Matrix3f operator*( const Matrix3f &m, const int &f ) {
      return Matrix3f( m[0][0]*f, m[0][1]*f, m[0][2]*f,
                       m[1][0]*f, m[1][1]*f, m[1][2]*f,
                       m[2][0]*f, m[2][1]*f, m[2][2]*f );
    }

    /// Muliplication between Matrix3f and long. 
    inline Matrix3f operator*( const Matrix3f &m, const long &f ) {
      return Matrix3f( m[0][0]*f, m[0][1]*f, m[0][2]*f,
                       m[1][0]*f, m[1][1]*f, m[1][2]*f,
                       m[2][0]*f, m[2][1]*f, m[2][2]*f );
    }

    /// Equality between two Matrix3f instances.
    inline bool operator==( const Matrix3f &m1, const Matrix3f &m2 ) {
      return (
         m1[0][0]==m2[0][0] && m1[0][1]==m2[0][1] && m1[0][2]==m2[0][2] &&
         m1[1][0]==m2[1][0] && m1[1][1]==m2[1][1] && m1[1][2]==m2[1][2] && 
         m1[2][0]==m2[2][0] && m1[2][1]==m2[2][1] && m1[2][2]==m2[2][2] 
         );
     }

    /// Function for printing a Matrix3f to an ostream.
    H3DUTIL_API std::ostream& operator<<( std::ostream &os, const Matrix3f &m );

    /// Multiplication with float.
    inline Matrix3f operator*( const float &a, const Matrix3f &b ) { return b * a; }

    /// Multiplication with double.
    inline Matrix3f operator*( const double &a, const Matrix3f &b ) { return b * a; }

    /// Multiplication with int.
    inline Matrix3f operator*( const int &a, const Matrix3f &b ) { return b * a; }

    /// Multiplication with long.
    inline Matrix3f operator*( const long &a, const Matrix3f &b ) { return b * a; }
    
    /// Unary minus.
    inline Matrix3f operator-( const Matrix3f &m ) { return m * -1; }
    
    /// Subtraction between two Matrix3f.
    inline Matrix3f operator-( const Matrix3f &a, const Matrix3f &b ) 
        { return a + (-b); }

    /// \}
  }
}

#endif
