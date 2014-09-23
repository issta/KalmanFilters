/** \file
** 
** \brief Define 4x3 matrices of type double and functions to operate on them
** 
** $Id: matrix4x3d.h 1.1 2008/05/21 15:00:21EDT dcmccomas Exp  $
**
** \note
**  -# The matrix is represented as a 4x3 matrix of doubles with
**     subcripts beginning at 0.
**  -# Row Subscripts must range from 0 to 3
**  -# Column Subscripts must range from 0 to 2
**  -# Adequate stack space is a precondition.
**  -# The called functions do not return any error indicators
**  -# All of the functions allow for the result to be one of the
**     input variables.
**
** References:
**  -# Math Library Specifications
**
**  $Date: 2008/05/21 15:00:21EDT $
**
**  $Revision: 1.1 $
**
**  $Log: matrix4x3d.h  $
**  Revision 1.1 2008/05/21 15:00:21EDT dcmccomas 
**  Initial revision
**  Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/inc/project.pj
**  Revision 1.2 2005/11/29 07:49:14EST dcmccomas 
**  New SDO delivery and added doxygen markup
**  Revision 1.4  2004/03/23 15:20:13  ddawson
**  removed DOS line endings
**
**  Revision 1.3  2004/02/24 16:23:28  rbjork
**  Updated Prototypes to have one argument per line
**
**  Revision 1.2  2004/02/04 20:32:44  rbjork
**  Updates resulting from vector code walkthrough
**
**  Revision 1.1  2004/01/23 13:22:59  daviddawsonuser
**  directory reorg
**
**
*/

/** 
** @addtogroup math_matrix
** @{
*/


#ifndef     _matrix4x3d_h
#define     _matrix4x3d_h

/**************************** Include Files  ****************************/

#include "vector3d.h"
#include "vector4d.h"


/************************** Type Definintions ******************************/

typedef struct 
{
  double      Comp[4][3];
} Matrix4x3d;

/******************Exported Function Prototypes ****************************/

/**
** \brief Add two 4x3  matrices of type double.
**        Result[4][3] = Left[4][3] + Right[4][3]
**
** \param[out]  Result   Pointer to matrix containing the addition result
** \param[in]   Left     Pointer to left matrix operand
** \param[in]   Right    Pointer to right matrix operand
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix4x3d_Add (Matrix4x3d       *Result,
                     const Matrix4x3d *Left,
                     const Matrix4x3d *Right);


/**
** \brief Copy a 4x3 Matrix of type double to another 4x3 Matrix.
**        Result[4][3] = Operand[4][3]
**
** \param[out]  Result   Pointer to matrix containing a copy of \c Operand
** \param[in]   Operand  Pointer to source matrix
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix4x3d_Copy (Matrix4x3d       *Result,
                      const Matrix4x3d *Operand);


/**
** \brief Divide each element of a 4x3 Matrix of type double by a scalar.
**        Result[4][3] = Left[4][3] / Scalar
**
** \warning
**    -# Caller must ensure that Scalar is non-zero
**
** \param[out]  Result   Pointer to matrix containing the result
** \param[in]   Left     Pointer to the matrix whose elements will be divided by \c Scalar
** \param[in]   Scalar   Scalar value used for the division
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix4x3d_DivScalar (Matrix4x3d       *Result,
                           const Matrix4x3d *Left,
                           double            Scalar);


/**
** \brief Sets each element of a 4x3 Matrix of type double to zero.
**
** \param[out]  Result   Pointer to matrix containing a zero matrix
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix4x3d_InitZero (Matrix4x3d *Result);


/**
** \brief Multiply each element of a 4x3 Matrix of type double by a scalar.
**        Result[4][3] = Left[4][3] * Scalar
**
** \param[out]  Result   Pointer to matrix containing the result
** \param[in]   Left     Pointer to matrix whose elements will be multiplied by \c Scalar
** \param[in]   Scalar   Scalar value used for the multiplication
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix4x3d_MultScalar (Matrix4x3d       *Result,
                            const Matrix4x3d *Left,
                            double            Scalar);


/**
** \brief Multiply an 4x3 Matrix of type double by a 3x1 vector.
**        Result[4] = Left[4][3] * Right[3]
**
** \param[out]  Result   Pointer to matrix containing the result
** \param[in]   Left     Pointer to left operand matrix
** \param[in]   Right    Pointer to left operand vector
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix4x3d_MultVec (Vector4d         *Result,
                         const Matrix4x3d *Left,
                         const Vector3d   *Right);


/**
** \brief Subtract two 4x3 Matrices of type double.
**        Result[4][3] = Left[4][3] - Right[4][3]
**
** \param[out]  Result   Pointer to matrix containing the subtraction result
** \param[in]   Left     Pointer to left matrix operand
** \param[in]   Right    Pointer to right matrix operand
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix4x3d_Sub (Matrix4x3d       *Result,
                     const Matrix4x3d *Left,
                     const Matrix4x3d *Right);



#endif /* _matrix4x3d_h */
/** @} */

