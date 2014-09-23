/** \file
**
** \brief Define quaternion type and functions to operate quaternions
**
** $Id: quaternion.h 1.2 2008/09/10 10:01:02EDT gwelter Exp  $
**
** \note
**   -#  All functions that calculate nonscalar objects return
**       their results in the first parameter.  These function have
**       a return value of void.
**   -#  All functions that calculate a scalar return their result
**       as the return value of the function itself.
**   -#  All functions are guaranteed not to modify their input
**       parameters.
**   -#  All functions are capable of using the same variable
**       as an input and output parameter.
**   -#  The first three elements of Quaternion structures are
**       the vector components and the 4th is the scalar 
**       component.
**   -#  Functions (except Quaternion_MakeScalarPositive) may
**       output quaternions with negative scalar components.
**       If the user requires positive scalar components
**       he/she should use MakeScalarPositive.
**   -#  Functions do not perform any error reporting.  It is the
**       user's responsibility to check parameters before passing
**       them in.  Each function lists the conditions that will
**       cause errors to occur.
**   -#  Functions do not guard against floating point exceptions.
**       As above it is the user's responsibility to pass valid
**       parameters into the functions.
**
** \sa
**   gnc_fsw_math_spec.doc
**
**
** $Date: 2008/09/10 10:01:02EDT $
**
** Revision 1.1  2004/01/23 13:22:59  daviddawsonuser
** directory reorg
** Revision 1.1  2004/10/04 18:26:02  ddawson
** added mathlib files to SDO repository
** Revision 1.1 2004/12/16 13:16:19EST rperera 
** Initial revision
** Member added to project /home/mksdata/MKS-SDO-REPOSITORY/fsw/shared/inc/project.pj
** Revision 1.2  2004/03/23 20:56:30  ddawson
** renumbered design notes comments
** Revision 1.2 2005/11/29 07:49:32EST dcmccomas 
** New SDO delivery and added doxygen markup
** Revision 1.2  2003/12/10 20:40:07  daviddawsonuser
** clean up based on Bob's review
** Revision 1.2.4.3.2.1  2005/11/17 03:41:20  ddawson
** Imported SDO changes into main trunk and then merged with doxygen branch
** $Revision: 1.2 $
** $Log: quaternion.h  $
** Revision 1.2 2008/09/10 10:01:02EDT gwelter 
** pre-gpm updates
** Revision 1.3  2003/12/12 16:21:01  daviddawsonuser
** alphabetized functions
** Revision 1.3  2005/11/16 20:44:14  ddawson
** update from SDO, propagate quaternion function modified to not perform
** propagation if input vector magnitude is below MATH_ZERO_DOUBLE_TOLERANCE
** rather than 0.0
** Revision 1.3 2006/07/27 13:29:25EDT jjhageman 
** Added end line to avoid compilation warnings
** Revision 1.4  2003/12/12 18:05:20  daviddawsonuser
** added comments
** Revision 1.5  2003/12/12 21:06:47  daviddawsonuser
** *** empty log message ***
** Revision 1.6  2004/01/20 19:13:29  daviddawsonuser
** Made changes based on code review
**
** Revision 2.0  2008/03/24 07:10 gary welter
** modified Quaternion_Normalize()
**           for more efficient division 
** added    Quaternion_IsNormalized()
**          Quaternion_Standardize()
**          Quaternion_ToEulerEigenVectorAngle()
**          Quaternion_FromEulerEigenVectorAngle()
**          Quaternion_ToEulerVector()
**          Quaternion_FromEulerector()
**
**
*/

/** 
** @addtogroup mathlib_quat
** @{
*/

#ifndef _quaternion_h_
#define _quaternion_h_


/************************ Includes ***********************************/

#include "mathconstants.h"
#include "vector3d.h"
#include "matrix3x3d.h"

/******************** Type Definitions *******************************/

/**
**
** Basic mathlib quaternion structure. First 3 elements of array are x, y, z
** components of quaternion respectively and the fourth element is
** the scalar component
*/
typedef struct              
{

   double Comp[4]; /**< Quaternion components in the order of (x,y,z,scalar) */

} Quaternion;


/*************** Exported Function Prototypes ************************/


/**
** \brief Calculates the rotation angle that a quaternion represents.
**
** \note
**    -# Returned angle is in radians and between -PI and +PI
**    -# Function will fail if the scalar component of the 
**       input quaternion is greater than 1 or less than -1.
**
** \param[in] InputQuat Pointer to input quaternion
**
** \returns
** \retcode double \retdesc  Angle in radians that quaternion represents \endcode
** \endreturns
*/
double Quaternion_Angle (const Quaternion *InputQuat);



/**
** \brief Passes back the conjugate of the input quaternion.
**
** \par Description
**        The quaternion is conjugated by negating the first three
**        elements (vector portion) of the input quaternion.  This
**        preserves the sign of the scalar element.
**
** \param[out] ConjugateQuat  Pointer to quaternion containing the conjugate of the input quaternion
** \param[in]  InputQuat      Pointer to input quaternion
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Quaternion_Conjugate (Quaternion       *ConjugateQuat,
                           const Quaternion *InputQuat);



/**
** \brief Copies one quaternion to another.
**
** \par Description
**        Merely does a structure copy which users may want to simply
**        do themselves.  Using this function however will protect the
**        user against future changes in how quaternions are stored.
**
** \param[out] DuplicateQuat  Pointer to duplicate of InputQuat
** \param[in]  InputQuat      Pointer to input quaternion
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Quaternion_Copy (Quaternion       *DuplicateQuat,
                      const Quaternion *InputQuat);



/**
** \brief Calculates a delta quaternion which describes a rotation
**        between two input quaternions.
**
** \par Description    
** \verbatim
**           QuatAB *   QuatBC  =  QuatAC
**             ^          ^          ^
**           input1     output     input2
** \endverbatim
**
** \param[out]  QuatBC      Pointer to quaternion describing rotation from B to C
** \param[in]   QuatAB      Pointer to input quaternion describing rotation from A to B
** \param[in]   QuatAC      Pointer to input quaternion describing rotation from A to C
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Quaternion_Delta (Quaternion       *QuatBC,
                       const Quaternion *QuatAB,
                       const Quaternion *QuatAC);



/*
** \brief Transforms an Euler (vector, angle) set to a quaternion
**
**  \note 
**    -# See Equation 12-14 in Wertz
**
**  \param[out]  Q    quaternion
**  \param[in]   Eva  Euler eigen vector & angle (in radians)
*/
void Quaternion_FromEulerEigenVectorAngle( Quaternion    *Q,
                                           const double   Eva[4] );



/*
** \brief Transforms a rotation vector to a quaternion
**
**  \param[out] Q  corresponding quaternion
**  \param[in]  V  rotation Euler vector (units: radians)
*/
void Quaternion_FromEulerVector( Quaternion     *Q,
                                 const Vector3d *V  );




/**
** \brief Creates a quaternion from a direction cosine matrix.  
**
** \par Description
**        Algorithm is taken from Spacecraft Attitude Determination and
**        Control by James R. Wertz, Page 415, Eqns. 12-14.  The algorithm
**        minimizes numerical inaccuracies by using the one of four equivalent
**        algorithms that will yield the best results for the particular
**        input matrix.
**
** \warning
**   Will divide by zero: 
**       -# if trace > maximum diagonal component
**          and trace = 1 + 2 * maximum diagonal component
**       -# if trace > maximum diagonal component
**          and trace = -1
**
**
** \param[out]  Result      Pointer to quaternion derived from the direction cosine matrix
** \param[in]   Operand     Pointer to direction cosine matrix
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Quaternion_FromMatrix (Quaternion       *Result,
                            const Matrix3x3d *Operand);



/**
** \brief Set a quaternion to the identity quaternion (0, 0, 0, 1).
**
** \param[out]  IdentityQuat     Pointer to quaternion which will be set to (0,0,0,1)
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Quaternion_Identity (Quaternion *IdentityQuat);


/**
** \brief Determines whether a quaternion is normalized
**
** \par Description
**         Determines whether a quaternion is normalized 
**         to within specified tolerance.
**
** \param[in]  InputQuat           Pointer to input quaternion to be checked
** \param[in]  NormalizationLimit  normalization limit (e.g., ~ 1.0e-5)
**
** \returns
** \retcode ing \retdesc  1 if input quaternion is normalized,
**                        0 otherwise
**
** \endreturns
*/
int Quaternion_IsNormalized (const Quaternion *InputQuat,
						     const double      NormalizationLimit);


/**
** \brief Returns the magnitude of a quaternion.
**
** \param[in]  InputQuat     Pointer to input quaternion
**
** \returns
** \retcode double \retdesc Magnitude of quaternion \endcode
** \endreturns
*/
double Quaternion_Magnitude (const Quaternion *InputQuat);



/**
** \brief Generate a quaternion equivalent to input quaternion with a positive scalar component.
**
** \param[out]  OutputQuat  Pointer to output quaternion guaranteed to have positive scalar component
** \param[in]   InputQuat   Pointer to input quaternion
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Quaternion_MakeScalarPositive (Quaternion       *OutputQuat,
                                    const Quaternion *InputQuat);



/**
** \brief Multiplies two quaternions together.
**    
** \par Description
**    -# Quaternions are multiplied such that if QuatXY is a rotation from
**       X to Y and QuatYZ is a rotation from Y to Z, the QuatXZ will 
**       represent a rotation from X to Z.
**
**       The quaternion multiply algorithm was taken from Spacecraft
**       Attitude Determination and Control by James R. Wertz, Page 416
**       Eqn. (12-15b).
**
** \note
**    -# Always returns a normalized quaternion
**    -# Will cause a floating point exception (divide by zero in
**       Quaternion_Normalize) if either quaternion contains
**       all zeros (invalid quaterion)
**
** \param[out]  QuatXZ   Pointer to quaternion representing rotation from X to Z
** \param[in]   QuatXY   Pointer to quaternion representing rotation from X to Y
** \param[in]   QuatYZ   Pointer to quaternion represinting rotation form Y to Z
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Quaternion_Mult (Quaternion       *QuatXZ,
                      const Quaternion *QuatXY,
                      const Quaternion *QuatYZ);



/**
** \brief Normalizes a quaternion
**
** \par Description
**         Normalizes a quaternion by dividing each element by the magnitude of
**         the quaternion.
**
** \param[out]  NormalizedQuat   Pointer to the normalized input quaternion
** \param[in]   InputQuat        Pointer to quaternion to be normalized
**
** \returns
** \retcode void \
** \endreturns
*/
void Quaternion_Normalize (Quaternion       *NormalizedQuat,
                           const Quaternion *InputQuat);




/*
** \brief Rotates an input quaternion through a rotation described
**        by a three element vector.
**
** \par Description
**    The function basically creates a delta quaternion from the input
**    rotation vector and then multiplies the input quaternion by 
**    this delta quaternion.
**
** \warning
**    Function will fail if a (0,0,0,0) quaternion is input.  
**    Quaternion_Normalize called from Quaternion_Mulitply will
**    cause a floating point exception.
**
** \note
**   -# Function is capable of accepting a (0, 0, 0) vector which will
**      cause the output PropagatedQuat to equal the InputQuat.
**
** \param[out]  PropagatedQuat  Pointer to quaternion offset from input quaternion by RotationVec
** \param[in]   InputQuat       Pointer to quaternion to be rotated
** \param[in]   RotationVec     Pointer to rotation vector in radians
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Quaternion_Propagate (Quaternion       *PropagatedQuat,
                           const Quaternion *InputQuat,
                           const Vector3d   *RotationVec);

/*
** \brief Standardizes a quaternion
**
** \par Description
**         Standardizes a quaternion by 
**         (1) negating it if scalar element is negative, and 
**         (2) dividing each element by the magnitude of the quaternion.
**
** \param[out]  StandardizedQuat  Pointer to standardized quaternion 
** \param[in]   InputQuat         Pointer to input quaternion
**
** \retcode int \retdesc  1 if input quaternion was 
**                           sufficiently normalized
**                        0 otherwise
** \endreturns
*/
int Quaternion_Standardize (Quaternion       *StandardizedQuat,  
                            const Quaternion *InputQuat,
							const double      NormalizationLimit );


/*
** \brief Transforms a quaternion to an Euler (vector, angle) set
**
**  \note 
**    -# direction cosine matrix from quaternion,
**        See Equation 12-13a in Wertz
**
**  \param[out]  Eva  Euler eigen vector and angle 
**  \param[in]   Q    quaternion
*/
void Quaternion_ToEulerEigenVectorAngle( double            Eva[4],
                                         const Quaternion *Q  );


/*
** \brief Transforms a quaternion to a rotation vector
**
**  \note 
**    -# convert quaternion to rotation vector [radians]
**        e.g., rotation rate times time
**
**  \param[out] V  Euler rotation vector (units: radians)
**  \param[in]  Q  quaternion
*/
void Quaternion_ToEulerVector( Vector3d         *V,
                               const Quaternion *Q  );



/*
** \brief Calculates the equivalent direction cosine matrix of
**        an input quaternion.
**
** \par Description
**        Algorithm was taken from Spacecraft Attitude Determination and
**        Control by James R. Wertz, Page 414, Eqn. (12-13a)
**
** \param[out]  Result   Pointer to 3 by 3 direction cosine matrix
** \param[in]   Operand  Pointer to input quaternion
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Quaternion_ToMatrix (Matrix3x3d       *Result,
                          const Quaternion *Operand);



/*
** \brief Transforms a vector using an input quaternion
**
** \param[out]  Result   Pointer to rotated 3 element vector
** \param[in]   QuatOp   Pointer to quaternion representing rotation to be applied
** \param[in]   VecOp    Pointer to 3 dimensional vector to be rotated
**
** \returns
** \retcode void \endcode
** \endreturns
*/
void Quaternion_VectorTransform (Vector3d         *Result,
                                 const Quaternion *QuatOp,
                                 const Vector3d   *VecOp );




#endif  /* #ifndef _quaternion_h_ */
/** @} */
