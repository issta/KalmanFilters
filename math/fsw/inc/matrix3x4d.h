/** \file
**
** \brief Define 3x4 matrices of type double and functions to operate on them
**
** $Id: matrix3x4d.h 1.1 2008/05/21 15:00:21EDT dcmccomas Exp  $
**
** \note
**  -# The matrix is represented as a 3x4 matrix of doubles with
**     subcripts beginning at 0.
**  -# Row Subscripts must range from 0 to 2
**  -# Column Subscripts must range from 0 to 3
**  -# Adequate stack space is a precondition.
**  -# The called features do not return any error indicators
**  -# All of the functions allow for the result to be one of the 
**     input variables.
**
** References:
**  -# Math Library Specifications
**
**
**  $Date: 2008/05/21 15:00:21EDT $
**
**  $Revision: 1.1 $
**
**  $Log: matrix3x4d.h  $
**  Revision 1.1 2008/05/21 15:00:21EDT dcmccomas 
**  Initial revision
**  Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/inc/project.pj
**  Revision 1.2 2005/11/29 07:49:14EST dcmccomas 
**  New SDO delivery and added doxygen markup
**  Revision 1.4.4.1  2004/06/23 15:56:20  ddawson
**  added some extra doxygen examples, namely comments next to parms
**
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

#ifndef     _matrix3x4d_h
#define     _matrix3x4d_h

/****************************  Include Files ******************************/
#include "vector3d.h"
#include "vector4d.h"


/****************************  Type Definintions **************************/

typedef struct 
{
  double      Comp[3][4];
} Matrix3x4d;

/********************** Exported Function Prototypes  ***********************/

/**
** \brief Add two 3x4 matrices of type double.
**        Result[3][4] = Left[3][4] + Right[3][4]
**
** \param[out]  Result   Pointer to matrix containing the addition result
** \param[in]   Left     Pointer to left matrix operand
** \param[in]   Right    Pointer to right matrix operand
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix3x4d_Add (Matrix3x4d       *Result,
                     const Matrix3x4d *Left,
                     const Matrix3x4d *Right);


/**
** \brief Copy a 3x4 Matrix of type double to another 3x4 Matrix.
**        Result[3][4] = Operand[3][4]
**
** \param[out]  Result   Pointer to matrix containing a copy of \c Operand
** \param[in]   Operand  Pointer to source matrix
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix3x4d_Copy (Matrix3x4d       *Result,
                      const Matrix3x4d *Operand);


/**
** \brief Divide each element of a 3x4 Matrix of type double by a scalar.
**        Result[3][4] = Left[3][4] / Scalar
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
void Matrix3x4d_DivScalar (Matrix3x4d       *Result,
                           const Matrix3x4d *Left,
                           double            Scalar);


/**
** \brief Set each element of 3x4 matrix of type double to zero.
**
** \param[out]  Result   Pointer to matrix containing a zero matrix
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix3x4d_InitZero(Matrix3x4d *Result);


/**
** \brief Multiply each element of a 3x4 Matrix of type double  by a scalar.
**        Result[3][4] = Left[3][4] * Scalar
**
** \param[out]  Result   Pointer to matrix containing the result
** \param[in]   Left     Pointer to matrix whose elements will be multiplied by \c Scalar
** \param[in]   Scalar   Scalar value used for the multiplication
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix3x4d_MultScalar (Matrix3x4d       *Result,
                            const Matrix3x4d *Left,
                            double            Scalar);


/**
** \brief Multiply an 3x4 Matrix of type double by a 4x1 vector.
**        Result[3] = Left[3][4] * Right[4]
**
** \param[out]  Result   Pointer to matrix containing the result
** \param[in]   Left     Pointer to left operand matrix
** \param[in]   Right    Pointer to left operand vector
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix3x4d_MultVec (Vector3d         *Result,
                         const Matrix3x4d *Left,
                         const Vector4d   *Right);


/**
** \brief Subtract two 3x4 Matrices of type double.
**          Result[3][4] = Left[3][4] - Right[3][4]
**
** \param[out]  Result   Pointer to matrix containing the subtraction result
** \param[in]   Left     Pointer to left matrix operand
** \param[in]   Right    Pointer to right matrix operand
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Matrix3x4d_Sub (Matrix3x4d       *Result,
                     const Matrix3x4d *Left,
                     const Matrix3x4d *Right);


#endif /* _matrix3x4d_h */
/** @} */ 

