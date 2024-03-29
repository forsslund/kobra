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
/// \file Matrix4d.h
/// \brief Header file for Matrix4d.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __MATRIX4D_H__
#define __MATRIX4D_H__

#include <H3DUtil/H3DUtil.h>
#include <H3DUtil/H3DBasicTypes.h>
#include <H3DUtil/Vec4d.h>
#include <H3DUtil/TemplateOperators.h>
#include <H3DUtil/Exception.h>
#include <H3DUtil/Matrix3d.h>
#include <H3DUtil/Rotationd.h>

namespace H3DUtil {
  namespace ArithmeticTypes {
    class Quaternion;
    class Rotation;
    class Quaterniond;
    class Matrix4f;

    /// Representation of a 4x4 matrix.
    /// \ingroup H3DUtilBasicTypes
    class H3DUTIL_API Matrix4d {
    public:
      /// Thrown when trying to perform operations on a singular matrix
      /// that are not allowed.
      H3D_API_EXCEPTION( SingularMatrix4d );

      /// Constructor. Creates an identity matrix.
      inline Matrix4d() { setToIdentity(); }
      
      /// Constructor.
      inline Matrix4d( H3DDouble m00, H3DDouble m01, 
                       H3DDouble m02, H3DDouble m03,
                       H3DDouble m10, H3DDouble m11, 
                       H3DDouble m12, H3DDouble m13,
                       H3DDouble m20, H3DDouble m21, 
                       H3DDouble m22, H3DDouble m23,
                       H3DDouble m30, H3DDouble m31, 
                       H3DDouble m32, H3DDouble m33 ) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03; 
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13; 
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23; 
        m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33; 
      }

      /// Transformation constructor. 
      ///
      /// Creates a transformation matrix from translation, rotation 
      /// and scale parameters.
      ///
      /// \param translation A Vec3d describing the translation.
      /// \param rotation A Rotationd describing the rotation (optional).
      /// \param scale A Vec3d describing the scale (optional).
      ///
      explicit Matrix4d( const Vec3d& translation, 
                         const Rotationd& rotation= Rotationd(), 
                         const Vec3d& scale= Vec3d(1,1,1) );

      /// Constructor. From Rotation object.
      explicit Matrix4d( const Rotation &r );

      /// Constructor. From Quaternion object.
      explicit Matrix4d( const Quaternion &q );

      /// Constructor. From Rotation object.
      explicit Matrix4d( const Rotationd &r );

      /// Constructor. From Quaternion object.
      explicit Matrix4d( const Quaterniond &q );

      /// Constructor. From Matrix3d object. Creates a tranform matrix where
      /// the rotation part is the Matrix3d.
      explicit Matrix4d( const Matrix3d &m );

      /// Constructor. From Matrix4f.
      Matrix4d( const Matrix4f &m );

      /// Set to the identity matrix.
      inline void setToIdentity() {
        m[0][0] = 1; m[0][1] = 0; m[0][2] = 0; m[0][3] = 0;
        m[1][0] = 0; m[1][1] = 1; m[1][2] = 0; m[1][3] = 0;
        m[2][0] = 0; m[2][1] = 0; m[2][2] = 1; m[2][3] = 0;
        m[3][0] = 0; m[3][1] = 0; m[3][2] = 0; m[3][3] = 1;
      }

      /// Returns the inverse of the matrix assuming that it is on 
      /// the form
      ///
      ///  [ m00 m01 m02 m03
      ///    m10 m11 m12 m13
      ///    m20 m21 m22 m23
      ///    0   0   0   1   ]
      ///
      Matrix4d transformInverse() const;

      /// Returns the inverse of the matrix.
      Matrix4d inverse() const;

      /// Returns an "inverse" of the matrix regardless of whether the matrix is
      /// a singular matrix or not. When the matrix is not a singular matrix this
      /// function will behave like the inverse function.
      /// When the matrix is a singular matrix and hence not
      /// possible to invert by definition then the zero-value-determinant is replaced with
      /// and epsilon value. This gives an "inverse" of the matrix that can in some cases
      /// be used and get good enough results. It is up to the user of this function to make
      /// sure that this hack is acceptable in any given situation. Use at your own peril.
      Matrix4d inverseSafe() const;

        /// Returns the transpose of the matrix.
      inline Matrix4d transpose() const {
        return Matrix4d( m[0][0], m[1][0], m[2][0], m[3][0],
                         m[0][1], m[1][1], m[2][1], m[3][1],
                         m[0][2], m[1][2], m[2][2], m[3][2],
                         m[0][3], m[1][3], m[2][3], m[3][3] );
      };
      
      /// Get a row of the matrix.
      inline H3DDouble* operator[]( const int i ) { return m[i]; }

      /// Get a row of the matrix.
      inline const H3DDouble* operator[]( const int i ) const { return m[i]; }

      /// Get a row of the matrix.
      inline Vec4d getRow( int i ) const { 
        return Vec4d( m[i][0], m[i][1], m[i][2], m[i][3] ); 
      }

      /// Get a column of the matrix.
      inline Vec4d getColumn( int i ) const { 
        return Vec4d( m[0][i], m[1][i], m[2][i], m[3][i] ); 
      }

      /// Get an element in the matrix.
      inline H3DDouble getElement( int i, int j ) const { 
        return m[i][j];
      }

      /// Set an element in the matrix.
      inline void setElement( int i, int j, H3DDouble v ) { 
        m[i][j] = v;
      }

      /// Returns the scale and rotation part of the Matrix4d.
      inline Matrix3d getScaleRotationPart() const {
        return Matrix3d( m[0][0], m[0][1], m[0][2],
                         m[1][0], m[1][1], m[1][2],
                         m[2][0], m[2][1], m[2][2] );
      }

      /// Returns the rotation part of the Matrix4d.
      Matrix3d getRotationPart() const;

      /// Get the scaling part of the matrix for each axis.
      inline Vec3d getScalePart() const {
        return getScaleRotationPart().getScalePart();
      }

      /// Returns the translation part of the Matrix4f.
      Vec3d getTranslationPart() const {
        return Vec3d( m[0][3], m[1][3], m[2][3] );
      }

    private:
      /// The matrix data.
      H3DDouble m[4][4];

    public:
      /// Per-element precision double comparison against an epsilon value.
      inline bool nearEqual( const Matrix4d &rhs, const H3DDouble epsilon = std::numeric_limits< H3DDouble >::epsilon() ) const {
        for( unsigned int i = 0; i < 4; ++i ) {
          for( unsigned int j = 0; j < 4; ++j ) {
            if( !epsilonCompare( m[i][j], rhs[i][j], epsilon ) ) {
              return false;
            }
          }
        }
        return true;
      }
    };

    /// \defgroup Matrix4dOperators Matrix4d operators.
    /// \brief Operators on Matrix4d instances. See also the 
    /// \ref TemplateOperators "template operators" for more operators
    /// automatically defined from the explicit ones defined here.
    /// \ingroup H3DUtilBasicTypes
    /// \{

    /// Multiplication between two Matrix4d instances.
    inline Matrix4d operator*( const Matrix4d &m1, const Matrix4d &m2 ) {
      return Matrix4d( 
  m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0] + m1[0][2]*m2[2][0] + m1[0][3]*m2[3][0],
  m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1] + m1[0][2]*m2[2][1] + m1[0][3]*m2[3][1],
  m1[0][0]*m2[0][2] + m1[0][1]*m2[1][2] + m1[0][2]*m2[2][2] + m1[0][3]*m2[3][2],
  m1[0][0]*m2[0][3] + m1[0][1]*m2[1][3] + m1[0][2]*m2[2][3] + m1[0][3]*m2[3][3],
  
  m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0] + m1[1][2]*m2[2][0] + m1[1][3]*m2[3][0],
  m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1] + m1[1][2]*m2[2][1] + m1[1][3]*m2[3][1],
  m1[1][0]*m2[0][2] + m1[1][1]*m2[1][2] + m1[1][2]*m2[2][2] + m1[1][3]*m2[3][2],
  m1[1][0]*m2[0][3] + m1[1][1]*m2[1][3] + m1[1][2]*m2[2][3] + m1[1][3]*m2[3][3],
  
  m1[2][0]*m2[0][0] + m1[2][1]*m2[1][0] + m1[2][2]*m2[2][0] + m1[2][3]*m2[3][0],
  m1[2][0]*m2[0][1] + m1[2][1]*m2[1][1] + m1[2][2]*m2[2][1] + m1[2][3]*m2[3][1],
  m1[2][0]*m2[0][2] + m1[2][1]*m2[1][2] + m1[2][2]*m2[2][2] + m1[2][3]*m2[3][2],
  m1[2][0]*m2[0][3] + m1[2][1]*m2[1][3] + m1[2][2]*m2[2][3] + m1[2][3]*m2[3][3],
  
  m1[3][0]*m2[0][0] + m1[3][1]*m2[1][0] + m1[3][2]*m2[2][0] + m1[3][3]*m2[3][0],
  m1[3][0]*m2[0][1] + m1[3][1]*m2[1][1] + m1[3][2]*m2[2][1] + m1[3][3]*m2[3][1],
  m1[3][0]*m2[0][2] + m1[3][1]*m2[1][2] + m1[3][2]*m2[2][2] + m1[3][3]*m2[3][2],
  m1[3][0]*m2[0][3] + m1[3][1]*m2[1][3] + m1[3][2]*m2[2][3] + m1[3][3]*m2[3][3]
  );
    }

    /// Addition between two Matrix4d instances.
    inline Matrix4d operator+( const Matrix4d &m1, const Matrix4d &m2 ) {
      return Matrix4d( 
  m1[0][0]+m2[0][0], m1[0][1]+m2[0][1], m1[0][2]+m2[0][2], m1[0][3]+m2[0][3],
  m1[1][0]+m2[1][0], m1[1][1]+m2[1][1], m1[1][2]+m2[1][2], m1[1][3]+m2[1][3],
  m1[2][0]+m2[2][0], m1[2][1]+m2[2][1], m1[2][2]+m2[2][2], m1[2][3]+m2[2][3],
  m1[3][0]+m2[3][0], m1[3][1]+m2[3][1], m1[3][2]+m2[3][2], m1[3][3]+m2[3][3] );
    }
    
    /// Muliplication between Matrix4d and float.
    inline Matrix4d operator*( const Matrix4d &m, const float &f ) {
      return Matrix4d( m[0][0]*f, m[0][1]*f, m[0][2]*f, m[0][3]*f,
                       m[1][0]*f, m[1][1]*f, m[1][2]*f, m[1][3]*f,
                       m[2][0]*f, m[2][1]*f, m[2][2]*f, m[2][3]*f,
                       m[3][0]*f, m[3][1]*f, m[3][2]*f, m[3][3]*f );
    }
    /// Muliplication between Matrix4d and double.
    inline Matrix4d operator*( const Matrix4d &m, const double &d ) {
      return Matrix4d( (H3DDouble)(m[0][0]*d), (H3DDouble)(m[0][1]*d), (H3DDouble)(m[0][2]*d), (H3DDouble)(m[0][3]*d),
                       (H3DDouble)(m[1][0]*d), (H3DDouble)(m[1][1]*d), (H3DDouble)(m[1][2]*d), (H3DDouble)(m[1][3]*d),
                       (H3DDouble)(m[2][0]*d), (H3DDouble)(m[2][1]*d), (H3DDouble)(m[2][2]*d), (H3DDouble)(m[2][3]*d),
                       (H3DDouble)(m[3][0]*d), (H3DDouble)(m[3][1]*d), (H3DDouble)(m[3][2]*d), (H3DDouble)(m[3][3]*d) );
    }

    /// Muliplication between Matrix4d and int.
    inline Matrix4d operator*( const Matrix4d &m, const int &f ) {
      return Matrix4d( m[0][0]*f, m[0][1]*f, m[0][2]*f, m[0][3]*f,
                       m[1][0]*f, m[1][1]*f, m[1][2]*f, m[1][3]*f,
                       m[2][0]*f, m[2][1]*f, m[2][2]*f, m[2][3]*f,
                       m[3][0]*f, m[3][1]*f, m[3][2]*f, m[3][3]*f );
    }

    /// Muliplication between Matrix4d and long.
    inline Matrix4d operator*( const Matrix4d &m, const long &f ) {
      return Matrix4d( m[0][0]*f, m[0][1]*f, m[0][2]*f, m[0][3]*f,
                       m[1][0]*f, m[1][1]*f, m[1][2]*f, m[1][3]*f,
                       m[2][0]*f, m[2][1]*f, m[2][2]*f, m[2][3]*f,
                       m[3][0]*f, m[3][1]*f, m[3][2]*f, m[3][3]*f );
    }

    /// Equality between two Matrix4d instances.
    inline bool operator==( const Matrix4d &m1, const Matrix4d &m2 ) {
       return m1[0][0]==m2[0][0] && m1[0][1]==m2[0][1] && m1[0][2]==m2[0][2] && 
   m1[0][3]==m2[0][3] && m1[1][0]==m2[1][0] && m1[1][1]==m2[1][1] && 
   m1[1][2]==m2[1][2] && m1[1][3]==m2[1][3] && m1[2][0]==m2[2][0] && 
   m1[2][1]==m2[2][1] && m1[2][2]==m2[2][2] && m1[2][3]==m2[2][3] &&
   m1[3][0]==m2[3][0] && m1[3][1]==m2[3][1] && m1[3][2]==m2[3][2] && 
   m1[3][3]==m2[3][3];
     }

    /// Function for printing a Matrix4d to an ostream.
    H3DUTIL_API std::ostream& operator<<( std::ostream &os, const Matrix4d &m );

    /// Multiplication with float.
    inline Matrix4d operator*( const float &a, const Matrix4d &b ) { return b * a; }

    /// Multiplication with double.
    inline Matrix4d operator*( const double &a, const Matrix4d &b ) { return b * a; }

    /// Multiplication with int.
    inline Matrix4d operator*( const int &a, const Matrix4d &b ) { return b * a; }

    /// Multiplication with long.
    inline Matrix4d operator*( const long &a, const Matrix4d &b ) { return b * a; }
    
    /// Unary minus.
    inline Matrix4d operator-( const Matrix4d &m ) { return m * -1; }
    
    /// Subtraction between two Matrix4d.
    inline Matrix4d operator-( const Matrix4d &a, const Matrix4d &b ) 
        { return a + (-b); }
    
    /// \}
  }
}

#endif
