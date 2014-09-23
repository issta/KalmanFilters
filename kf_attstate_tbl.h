/*
** $Id: kf_attstate_tbl.h 1.3 2011/07/07 22:11:04EDT dkobe Exp  $
**
** Purpose:  Attitude State Kalman Filter Table Data Definition
**
** References:
**    - Kalman Filter design derived from LRO Kalman Filter and modified for both
**      GPM and with a goal of making it a component of the FSB Re-use library
**    - GPM GN&C Kalman Filter Algorithm Specification describes the Kalman Filter
**      in Algorithm format
** 
** Notes:
**    - Suggested table structures for containing Attitude State Kalman Filter parameters
**    - This module requires the Code 582 Flight Software Re-use Library (FSRL)
**      Math Library
** 
*/

/*************************************************************************
**
** Ensure that "this" header is included only once
**
**************************************************************************/

#ifndef _kf_attstate_tbl_
#define _kf_attstate_tbl_

#include "kf_attstate_cfg.h"
#include "vector3d.h"
#include "matrix3x3d.h"

/** 
**   \brief Double Precision 3x6 element matrix
**/
typedef struct
{
    double  Comp[3][6];
} KF_Matrix3x6d;

/**
**   \brief Suggested data structure for holding 3-Axis Attitude State Kalman Filter parameters
**
**   This data structure contains all the parameters necessary to initialize
**   the data structures required for using the 3-Axis version of the Attitude 
**   State Kalman Filter.
**
**/
typedef struct
{
    double         SigmaUSquared;        /**< \brief IRU Drift Rate Bias Random Walk Variance
                                              \details The drift Rate Bias Random Walk Variance
                                               in units of rad<sup>2</sup>/sec<sup>3</sup>.<BR>  
                                               <BR><B>Alg. Ref:</B> sigma<sup>2</sup><sub>U</sub> */ 
    double         SigmaVSquared;        /**< \brief IRU Angle Measurement Variance
                                              \details The Angle Measurement Variance (White Noise)
                                               in units of rad<sup>2</sup>/sec.  <BR>
                                               <BR><B>Alg. Ref:</B> sigma<sup>2</sup><sub>V</sub> */ 
    double         InitBiasCov;          /**< \brief Initial IRU Drift Rate Bias Estimate Variance
                                              \details The Drift Rate Bias Estimate Variance used to
                                               initialize the diagonal terms of the Error Covariance
                                               Matrix expressed in units of rad<sup>2</sup>/sec<sup>2</sup><BR>
                                               <BR><B>Alg. Ref:</B> sigma<sup>2</sup><sub>B</sub> */ 
    double         AttCovDivTol;         /**< \brief Attitude Covariance Divergence Tolerance
                                              \details This constant defines the covariance limit 
                                               for the Attitude portion of the Estimation Error 
                                               Covariance Matrix. (Units are in rad^2)  This value 
                                               is obtained by recognizing that the derivation of 
                                               the non-linear algorithm required a small error 
                                               approximation.  Once an error is no longer considered
                                               small, the results of the algorithm are somewhat 
                                               unpredictable. */
    double         BiasCovDivTol;        /**< \brief Bias Covariance Divergence Tolerance
                                              \details This constant defines the covariance limit 
                                               for the Attitude portion of the Estimation Error 
                                               Covariance Matrix. (Units are in rad^2/cycle^2) 
                                               This value is obtained by recognizing that the 
                                               derivation of the non-linear algorithm required 
                                               a small error approximation.  Once an error is 
                                               no longer considered small, the results of the 
                                               algorithm are somewhat unpredictable. */
    double         AttCovConTol;         /**< \brief Attitude Covariance Convergence Tolerance
                                              \details Once the Attitude Error Covariance in all three
                                               axes drops below this limit (Units are in rad^2), the 
                                               Attitude error is considered to be "converged". */
    double         BiasCovConTol;        /**< \brief Bias Covariance Convergence Tolerance
                                              \details Once the Bias Error Covariance in all three
                                               axes drops below this limit (Units are in rad^2/cycle^2),
                                               the Bias error is considered to be "converged". */
    double         RsSqRatioLimit;       /**< \brief KF Adjusted squared residuals validity limit 
                                              \details Factor for Kalman Filter adjusted squared 
                                               residuals tolerance testing. <BR>
                                               <BR><B>Alg. Ref:</B> RsSqRatioFactor */
    double         MaxSqrdResTol;        /**< \brief Coarse Residual Tolerance
                                              \details This constant defines the maximum reasonable 
                                               squared residual of an attitude sensor. 
                                               (Units are in rad^2)  This value is obtained by 
                                               recognizing that the derivation of the non-linear 
                                               algorithm required a small error approximation. Once 
                                               an error is no longer considered small, the results 
                                               of the algorithm are somewhat unpredictable. */
#if KF_ATTSTATE_VEC_INPUT == 1
    double         SigmaMSquared;        /**< \brief 2-Axis Attitude Sensor Measurement Variance
                                              \details The 2-Axis Attitude Sensor Measurement Variance 
                                               is only used when a 2-Axis Attitude Sensor is being used
                                               in conjuction with the #KF_AttState_KalmanFilter2 function.<BR>
                                               <BR><B>Alg. Ref:</B> sigma<sup>2</sup><sub>M</sub> */
#endif /* KF_ATTSTATE_VEC_INPUT == 1 */
#if KF_ATTSTATE_QUAT_INPUT == 1
    Matrix3x3d     MeasurementNoiseCov;  /**< \brief 3-Axis Attitude Sensor Measurement Noise Covariance Matrix 
                                              \details Star Tracker Measurement Noise Covariance Matrix
                                               in the body frame.  This term is only necessary when a 3-Axis
                                               Attitude Sensor (i.e. a quaternion star tracker) is being used.
                                               Terms CANNOT equal zero. <BR>
                                               <BR><B>Alg. Ref: G</B> */
#endif /* KF_ATTSTATE_QUAT_INPUT == 1 */
} KF_AttState_Tbl;

/*************************************************************************/

#endif /* _kf_attstate_tbl_ */

/************************/
/*  End of File Comment */
/************************/
