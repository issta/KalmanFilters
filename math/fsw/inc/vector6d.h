/** \file
** 
** \brief Define a 6x1 vector of double snd functions to operate on 6x1 vectors.
** 
** $Id: vector6d.h 1.1 2008/05/21 15:00:25EDT dcmccomas Exp  $
**
** \note
**  -# Vectors are implemented using a structure containing an array to
**     allow type checking.
**  -# Adequate stack space is a precondition.
**  -# The calling function is responsible for ensuring that the calling
**     arguments are consistent with the requested feature.
**  -# The called functions do not return any error indicators
**  -# All of the functions allow the result to be one of the input
**     variables
**
** References:
**  -# Math Library Specifications
**
**  $Date: 2008/05/21 15:00:25EDT $
**  $Revision: 1.1 $
**  $Log: vector6d.h  $
**  Revision 1.1 2008/05/21 15:00:25EDT dcmccomas 
**  Initial revision
**  Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/inc/project.pj
**  Revision 1.1 2006/12/14 13:41:46EST myyang 
**  Initial revision
**  Member added to project d:/mksdata/gnc-fsw/math/project.pj
**
*/

/** 
** @addtogroup math_vector
** @{
*/

#ifndef _vector6d_
#define _vector6d_

/*************************** Type definitions     **************************/

typedef struct
{

	double Comp[6];

}Vector6d;

/********************** Exported Function prototypes *************************/

/**
** \brief Add two 6x1 Vectors of type double.
**        Result[6] = Left[6] + Right[6]
**
** \param[out]  Result  Pointer to vector containing the addition result
** \param[in]   Left    Pointer to left vector operand
** \param[in]   Right   Pointer to right vector operand
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Vector6d_Add (Vector6d       *Result,
                   const Vector6d *Left,
                   const Vector6d *Right);


/**
** \brief Copy 6x1 vector of type double into another 6x1 vector.
**        Result[6] = Operand[6]
**
** \param[out]  Result   Pointer to vector containing a copy of the \c Operand
** \param[in]   Operand  Pointer to source vector
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Vector6d_Copy (Vector6d       *Result,
                    const Vector6d *Operand);


/**
** \brief Divide each element of a 6x1 vector of type double by a scalar.
**        Result[6] = Left[6] / Scalar
**
** \warning
**    Calling program must ensure that scalar is non-zero.
**
** \param[out]  Result  Pointer to vector containing the result
** \param[in]   Left    Pointer to left vector operand
** \param[in]   Scalar  Value of the scalar operand
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Vector6d_DivScalar (Vector6d       *Result,
                         const Vector6d *Left,
                         double Scalar);


/**
** \brief Compute inner product (dot product) of two 6x1 vectors of type double.
**        Result = Left[6] .dot. Right[6]
**
** \param[in]   Left    Pointer to left vector operand
** \param[in]   Right   Pointer to right vector operand
**
** \returns
** \retcode double \retdesc Scalar dot product \endcode
** \endreturns
*/
double Vector6d_Dot(const Vector6d *Left,
                    const Vector6d *Right);


/**
** \brief Sets a 6x1 Vector of type double to zero
**
** \param[out]  Result  Pointer to vector initialized to (0,0,0)
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Vector6d_InitZero(Vector6d *Result);


/**
** \brief Compute the magnitude of a 6x1 vector of type double.
**   
** \param[in]   Operand  Pointer to vector whose magnitude is computed
**
** \returns
** \retcode double \retdesc Vector magnitude \endcode
** \endreturns
*/
double Vector6d_Magnitude (const Vector6d *Operand);


/**
** \brief Multiply each element of a 6x1 vector of type double by a scalar.
**          Result[6] = Left[6] * Scalar
**
** \param[out]  Result  Pointer to vector containing the result
** \param[in]   Left    Pointer to left vector operand
** \param[in]   Scalar  Value of scalar operand
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Vector6d_MultScalar (Vector6d       *Result,
                          const Vector6d *Left,
                          double Scalar);


/**
** \brief Normalize a 6x1 vector of type double.
**   
** \warning
**    -  The calling module must ensure that the magnitude of the vector is 
**       non-zero
**
** \param[out]  Result  Pointer to vector containing the result
** \param[in]   Operand Pointer to vector to be normalized
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Vector6d_Normalize (Vector6d       *Result,
                         const Vector6d *Operand
                         );

/**
** \brief Subtract two 6x1 Vectors of type double.
**          Result[6] = Left[6] - Right[6]
**
** \param[out]  Result  Pointer to vector containing the subtraction result
** \param[in]   Left    Pointer to left vector operand
** \param[in]   Right   Pointer to right vector operand
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Vector6d_Sub (Vector6d       *Result,
                   const Vector6d *Left,
                   const Vector6d *Right);

#endif /* end _vector6d_ */
/** @} */
/****************** end of file vec6d.h ********************************/
