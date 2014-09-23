/*
** $Id: kf_attstate.h 1.5 2012/03/23 14:19:22EDT dkobe Exp  $
**
** Purpose:  Attitude State Kalman Filter Interface Definition
**
** References:
**    - Kalman Filter design derived from LRO Kalman Filter and modified for both
**      GPM and with a goal of making it a component of the FSB Re-use library
**    - GPM GN&C Kalman Filter Algorithm Specification describes the Kalman Filter
**      in Algorithm format
** 
** Notes:
**    - This module requires the Code 582 Flight Software Re-use Library (FSRL)
**    - This module assumes attitude measurement is in the form of a quaternion
**    - This module assumes the spacecraft body rate is obtained from an IRU
** 
*/

/*************************************************************************
**
** Ensure that "this" header is included only once
**
**************************************************************************/

#ifndef _kf_attstate_
#define _kf_attstate_

/*************************************************************************
**
** Include section
**
**************************************************************************/
#include "common_types.h"
#include "matrix6x6d.h"
#include "quaternion.h"
#include "vector6d.h"
#include "kf_attstate_tbl.h"

/******************** Macro Definitions **********************************/

/** 
**
**  The called function detected no errors.  Calculations were performed successfully.
**
*/  
#define KF_ATTSTATE_SUCCESS               (0)

/** 
**  \brief Covariance Matrix diagonal term is not converged
**
**  The updated Covariance Matrix diagonal term has not converged to within the
**  table specified limit. 
**  See \link #KF_AttState_Tbl.AttCovConTol Attitude Convergence tolerance. \endlink
**  and \link #KF_AttState_Tbl.BiasCovConTol Bias Convergence tolerance. \endlink
**
**  Note that the return value is a positive integer, thus
**  is not considered an error but rather a status.  Also note that the value is
**  shifted for each element in the covariance matrix diagonal.  Therefore, the
**  following bit masks are implied and any combination is possible:
**
**      0x00000001 = Attitude X-Axis Covariance has not converged
**      0x00000002 = Attitude Y-Axis Covariance has not converged
**      0x00000004 = Attitude Z-Axis Covariance has not converged
**      0x00000008 = Bias X-Axis Covariance has not converged
**      0x00000010 = Bias Y-Axis Covariance has not converged
**      0x00000020 = Bias Z-Axis Covariance has not converged
**
*/  
#define KF_ATTSTATE_NOT_CONVERGED         (0x00000001)
#define KF_ATTSTATE_NOT_CONVERGED_MASK    (0x0000003F)

/** 
**  \brief Return code error bit 
**
**  Errors detected in the KF library code are indicated with a combination of
**  the following error bit (which forces the resulting return code to be a negative
**  value) and a bit identifying the particular error in question.
*/  
#define KF_ATTSTATE_ERROR_BIT             (int32)(0x80000000)

/** 
**  \brief Covariance Matrix has at least one attitude diagonal term that is diverging
**
**  The update of the Covariance Matrix caused at least one of the diagonal terms for
**  the Attitude portion of the Covariance Matrix to exceed the 
**  \link #KF_AttState_Tbl.AttCovDivTol Attitude Divergence tolerance. \endlink
**  Note that these bits are only set along with the #KF_ATTSTATE_ERROR_BIT.
**
**  The axes in question can be determined by checking the following bits:
**
**      0x00000040 = Attitude X-Axis Covariance has diverged
**      0x00000080 = Attitude Y-Axis Covariance has diverged
**      0x00000100 = Attitude Z-Axis Covariance has diverged
*/  
#define KF_ATTSTATE_ATT_DIVERGENT_VARIANCE_X    (int32)(0x00000040)  
#define KF_ATTSTATE_ATT_DIVERGENT_VARIANCE_Y    (int32)(0x00000080)  
#define KF_ATTSTATE_ATT_DIVERGENT_VARIANCE_Z    (int32)(0x00000100)  

/** 
**  \brief Covariance Matrix has at least one attitude diagonal term that is diverging
**
**  The update of the Covariance Matrix caused at least one of the diagonal terms for
**  the Bias portion of the Covariance Matrix to exceed the 
**  \link #KF_AttState_Tbl.BiasCovDivTol Bias Divergence tolerance. \endlink
**  Note that these bits are only set along with the #KF_ATTSTATE_ERROR_BIT.
**
**  The axes in question can be determined by checking the following bits:
**
**      0x00000200 = Bias X-Axis Covariance has diverged
**      0x00000400 = Bias Y-Axis Covariance has diverged
**      0x00000800 = Bias Z-Axis Covariance has diverged
*/  
#define KF_ATTSTATE_BIAS_DIVERGENT_VARIANCE_X   (int32)(0x00000200)  
#define KF_ATTSTATE_BIAS_DIVERGENT_VARIANCE_Y   (int32)(0x00000400)  
#define KF_ATTSTATE_BIAS_DIVERGENT_VARIANCE_Z   (int32)(0x00000800)  

/** 
**  \brief Covariance Matrix Diagonal has at least one negative term
**
**  The update of the Covariance Matrix caused at least one of the diagonal terms of
**  the Covariance Matrix to become negative.
**  Note that these bits are only set along with the #KF_ATTSTATE_ERROR_BIT.
**
**  The axes in question can be determined by checking the following bits:
**
**      0x00001000 = Attitude X-Axis Covariance is negative
**      0x00002000 = Attitude Y-Axis Covariance is negative
**      0x00004000 = Attitude Z-Axis Covariance is negative
**      0x00008000 = Bias X-Axis Covariance is negative
**      0x00010000 = Bias Y-Axis Covariance is negative
**      0x00020000 = Bias Z-Axis Covariance is negative
*/  
#define KF_ATTSTATE_NEGATIVE_VARIANCE_ATT_X         (int32)(0x00001000)  
#define KF_ATTSTATE_NEGATIVE_VARIANCE_ATT_Y         (int32)(0x00002000)  
#define KF_ATTSTATE_NEGATIVE_VARIANCE_ATT_Z         (int32)(0x00004000)  
#define KF_ATTSTATE_NEGATIVE_VARIANCE_BIAS_X        (int32)(0x00008000)  
#define KF_ATTSTATE_NEGATIVE_VARIANCE_BIAS_Y        (int32)(0x00010000)  
#define KF_ATTSTATE_NEGATIVE_VARIANCE_BIAS_Z        (int32)(0x00020000)  

/** 
**  \brief The Residuals passed into Kalman Filter are too large relative to expected
**
**  The residuals provided as an input to the Kalman Filter exceeded the tolerance
**  specified by the table parameter #KF_AttState_Tbl.RsSqRatioLimit .
**  Note that this bit is only set along with the #KF_ATTSTATE_ERROR_BIT.
**
*/  
#define KF_ATTSTATE_LARGE_RELATIVE_RESIDUAL   (int32)(0x80040000)  

/** 
**  \brief The Residuals passed into Kalman Filter are too large, absolutely
**
**  The residuals provided as an input to the Kalman Filter exceeded the small error
**  tolerance specified by the configuration parameter #KF_AttState_Tbl.MaxSqrdResTol.
**  Note that this bit is only set along with the #KF_ATTSTATE_ERROR_BIT.
**
*/  
#define KF_ATTSTATE_LARGE_ABSOLUTE_RESIDUAL   (int32)(0x80080000)  

/** 
**  \brief The Attitude Covariance with Measurement Noise Covariance Matrix is not invertable
**
**  The left half of the Error Covariance Matrix, with the addition of the Star
**  Tracker Measurement Noise Covariance, resulted in a matrix that cannot be
**  inverted.
**  Note that this bit is only set along with the #KF_ATTSTATE_ERROR_BIT.
**
*/  
#define KF_ATTSTATE_SINGULAR_MATRIX           (int32)(0x80100000)  

/** 
**  \brief Variance is equal to zero
**
**  The maximum expected variance was equal to zero.
**  Note that this bit is only set along with the #KF_ATTSTATE_ERROR_BIT.
**
*/  
#define KF_ATTSTATE_ZERO_VARIANCE             (int32)(0x80200000)  

/** 
**  \brief Bad Vector Variance
**
**  The vector variance passed to #KF_AttState_KalmanFilter2 is bad.  The Kalman Gain
**  could not be computed because the denominator would have been zero or less than
**  zero.
**  Note that this bit is only set along with the #KF_ATTSTATE_ERROR_BIT.
**
*/  
#define KF_ATTSTATE_BAD_VEC_VARIANCE          (int32)(0x80400000)  

/** 
**  \brief All Covariances have converged
**
**  The attitude and bias covariances have all been reduced to
**  values less than the table specified convergence limits.
**
*/  
#define KF_ATTSTATE_CONVERGED                 (int32)(0x00800000)  


/******************** Type Definitions ***********************************/

/**
**   \brief Transpose of 3x6 matrix used in several places
**
**   This data structure is the appropriate shape to contain the transpose of
**   the State-to-Measurement matrix
*/
typedef struct
{
    double Comp[6][3];
} KF_Matrix6x3d;

/**
**   \brief Data structure containing Attitude State Kalman Filter Constants
**
**   This data structure contains the Attitude State Kalman Filter data that is nominally constant throughout
**   a Kalman Filter execution.
**
**   \sa #KF_AttState_CurrentState;
*/
typedef struct
{
    Matrix6x6d         StateTransNoiseCov;  /**< \brief State Transition Noise Covariance Matrix
                                                 \details State Transition Noise Covariance Matrix
                                                  contains noise on propagated solution due to gyro
                                                  noise.<BR>
                                                  <BR><B>Alg. Ref: Q</B> */
    Matrix3x3d         MeasurementNoiseCov; /**< \brief Measurement Noise Covariance Matrix
                                                 \details Star Tracker Measurement Noise Covariance Matrix
                                                  in the body frame.  Terms CANNOT equal zero. <BR>
                                                  <BR><B>Alg. Ref: G</B> */
    double             InitBiasCov;         /**< \brief Initial IRU Drift Rate Bias Estimate Variance
                                                 \details The Drift Rate Bias Estimate Variance used to
                                                  initialize the diagonal terms of the Error Covariance
                                                  Matrix expressed in units of rad<sup>2</sup>/sec<sup>2</sup><BR>
                                                  <BR><B>Alg. Ref:</B> sigma<sup>2</sup><sub>B</sub> */ 
    double             AttCovDivTol;         /**< \brief Attitude Covariance Divergence Tolerance
                                                  \details This constant defines the covariance limit 
                                                   for the Attitude portion of the Estimation Error 
                                                   Covariance Matrix. (Units are in rad^2)  This value 
                                                   is obtained by recognizing that the derivation of 
                                                   the non-linear algorithm required a small error 
                                                   approximation.  Once an error is no longer considered
                                                   small, the results of the algorithm are somewhat 
                                                   unpredictable. */
    double             BiasCovDivTol;        /**< \brief Bias Covariance Divergence Tolerance
                                                  \details This constant defines the covariance limit 
                                                   for the Attitude portion of the Estimation Error 
                                                   Covariance Matrix. (Units are in rad^2/cycle^2) 
                                                   This value is obtained by recognizing that the 
                                                   derivation of the non-linear algorithm required 
                                                   a small error approximation.  Once an error is 
                                                   no longer considered small, the results of the 
                                                   algorithm are somewhat unpredictable. */
    double             AttCovConTol;         /**< \brief Attitude Covariance Convergence Tolerance
                                                  \details Once the Attitude Error Covariance in all three
                                                   axes drops below this limit (Units are in rad^2), the 
                                                   Attitude error is considered to be "converged". */
    double             BiasCovConTol;        /**< \brief Bias Covariance Convergence Tolerance
                                                  \details Once the Bias Error Covariance in all three
                                                   axes drops below this limit (Units are in rad^2/cycle^2),
                                                   the Bias error is considered to be "converged". */
    double             RsSqRatioLimit;       /**< \brief KF Adjusted squared residuals validity limit 
                                                  \details Factor for Kalman Filter adjusted squared 
                                                   residuals tolerance testing. <BR>
                                                   <BR><B>Alg. Ref:</B> RsSqRatioFactor */
    double             MaxSqrdResTol;        /**< \brief Coarse Residual Tolerance
                                                  \details This constant defines the maximum reasonable 
                                                   squared residual of an attitude sensor. 
                                                   (Units are in rad^2)  This value is obtained by 
                                                   recognizing that the derivation of the non-linear 
                                                   algorithm required a small error approximation. Once 
                                                   an error is no longer considered small, the results 
                                                   of the algorithm are somewhat unpredictable. */
#if KF_ATTSTATE_VEC_INPUT == 1
    double             MeasuredVecVariance; /**< \brief Single-Axis Measurement Vector Variance
                                                 \details Single-Axis Measurement Vector Variance
                                                 which is <B>ONLY</B> relevant when using the
                                                 #KF_AttState_KalmanFilter2 function for computing
                                                 an attitude estimate using a 2-axis attitude
                                                 sensor. <BR>
                                                 <BR><B>Alg. Ref: sigma<sup>2</sup><sub>M</sub> */
#endif /* KF_ATTSTATE_VEC_INPUT == 1 */
} KF_AttState_Constants;

/**
**   \brief Data structure containing Attitude State Kalman Filter State Data
**
**   This data structure contains the Attitude State Kalman Filter data that is updated on every cycle
**   the Kalman Filter is run.
**
**   \sa #KF_AttState_Constants;
*/
typedef struct
{
    Matrix6x6d         ErrCov;              /**< \brief Current Error Covariance Matrix
                                                 \details Current Error Covariance Matrix. <BR>
                                                  <BR><B>Alg. Ref: P </B>*/
    Matrix6x6d         StateTransition;     /**< \brief Current State Transition Matrix
                                                 \details Current State Transition Matrix. <BR>
                                                  <BR><B>Alg. Ref: Phi </B>*/
    Vector6d           OptimalState;        /**< \brief Current Optimal Kalman Filter State
                                                 \details Current Optimal Kalman Filter State. <BR>
                                                  <BR><B>Alg. Ref: X </B>*/
    Vector3d           AdjustedResidual;    /**< \brief Residual relative to revised KF State Vector */
} KF_AttState_CurrentState;

/******************* Function Prototypes *********************************/

/**
** \brief Attitude State Kalman Filter Initialization
**
** \par Description
**          This function initializes the constants and the current state variables for the Kalman Filter.<BR>
**          The following constants are initialized by this function using Table Data as the input: <BR><BR>
**              -# State Transition Noise Covariance (<I>KF_AttState_Constants.StateTransNoiseCov</I>) - 
**                 Generated from KF_AttState_Tbl.SigmaUSquared and KF_AttState_Tbl.SigmaVSquared <BR>
**              -# Measurement Noise Covariance Matrix - Set equal to KF_AttState_Tbl.MeasurementNoiseCov <BR>
**              -# KF Adjusted Squared Residuals Validity Limit - Set equal to KF_AttState_Tbl.RsSqRatioLimit <BR><BR>
**          The following current state variables are initialized by this function: <BR><BR>
**              -# Current Error Covariance Matrix - Diagonal set to 
**                 [(PI/3)<sup>2</sup>, (PI/3)<sup>2</sup>, (PI/3)<sup>2</sup>, 
**                  KF_AttState_Tbl.InitBiasCov, KF_AttState_Tbl.InitBiasCov, KF_AttState_Tbl.InitBiasCov] <BR>
**              -# Current Optimal Kalman Filter State - All terms are set to zero <BR>
**
** \par Assumptions, External Events, and Notes:
**          This function clears the contents of the current state of the Kalman Filter and re-initializes all
**          relevant data.  If the state of the Kalman Filter needs to be retained, then the user should call
**          #KF_AttState_InitConstants instead.<BR>
**          The Attitude Sensor is assumed to have a variance less than (PI/3)<sup>2</sup>.<BR>
**
** \param[in] StateConst                  A pointer to Kalman Filter constants.  These will be initialized by this function.
**
** \param[in] CurrentState                A pointer to structure containg the current state of the Kalman Filter.
**                                        Some of these values will be initialized by this function.
**
** \param[in] TableData                   A pointer to Kalman Filter table parameters.
**
** \param[in] Period                      The nominal time period between Kalman Filter updates. (secs)
**
** \param[out] *StateConst                Kalman Filter constants initialized by content of Kalman Filter Table parameters
**
** \param[out] *CurrentState              Kalman Filter current state initialized by content of Kalman Filter Table parameters
**
** \sa #KF_AttState_InitConstants
**/
void KF_AttState_Init(
         KF_AttState_Constants     *StateConst,   /* Ptr to KF State Constants      */
         KF_AttState_CurrentState  *CurrentState, /* Ptr to KF Current State        */
         const KF_AttState_Tbl     *TableData,    /* Ptr to KF Table Data           */
         const double               Period);      /* Time between KF updates (secs) */


/**
** \brief Attitude State Kalman Filter Constants Initialization
**
** \par Description
**          This function initializes the constants for the Kalman Filter.
**          The following constants are initialized by this function using Table Data as the input: <BR><BR>
**              -# State Transition Noise Covariance (<I>KF_AttState_Constants.StateTransNoiseCov</I>) - 
**                 Generated from KF_AttState_Tbl.SigmaUSquared and KF_AttState_Tbl.SigmaVSquared <BR>
**              -# Measurement Noise Covariance Matrix - Set equal to KF_AttState_Tbl.MeasurementNoiseCov <BR>
**              -# KF Adjusted Squared Residuals Validity Limit - Set equal to KF_AttState_Tbl.RsSqRatioLimit <BR><BR>
**
** \par Assumptions, External Events, and Notes:
**          This function is provided as a means of updating the constants without losing the current state of
**          the Kalman Filter (which would happen if #KF_AttState_Init were to be called).  If all aspects of
**          the Kalman Filter need to be initialized, then #KF_AttState_Init should be called instead.
**
** \param[in] StateConst                  A pointer to Kalman Filter constants.  These will be initialized by this function.
**
** \param[in] TableData                   A pointer to Kalman Filter table parameters.
**
** \param[in] Period                      The nominal time period between Kalman Filter updates. (secs)
**
** \param[out] *StateConst                Kalman Filter constants initialized by content of Kalman Filter Table parameters
**
** \sa #KF_AttState_Init
**/
void KF_AttState_InitConstants(
         KF_AttState_Constants     *StateConst,   /* Ptr to KF State Constants      */
         const KF_AttState_Tbl     *TableData,    /* Ptr to KF Table Data           */
         const double               Period);      /* Time between KF updates (secs) */


#if KF_ATTSTATE_QUAT_INPUT == 1
/**
** \brief Attitude State Kalman Filter 3-axis Attitude Sensor Processing
**
** \par Description
**          This function processes the latest 3-axis Attitude Sensor readings and computes the residuals and
**          adjusted residuals.
**
** \par Assumptions, External Events, and Notes:
**          None
**
** \param[in]  AttitudeSensorResidual  A pointer to vector containing residuals associated with
**                                     a specific attitude sensor
**
** \param[in]  PropQuatEciToBcs        A pointer to the Kalman Filter estimate of the ECI to BCS quaternion
**                                     propagated to the current time using the IRU rate data that has been
**                                     corrected with the Kalman Filter estimate of the IRU drift bias
**
** \param[in]  MeasuredQuatEciToBcs    A pointer to the ECI to BCS quaternion obtained from the quaternion
**                                     star tracker
**
** \param[out] *AttitudeSensorResidual The star tracker residuals are updated with the differences between
**                                     the expected attitude and the measured attitude
**
**
** \sa #KF_AttState_PropagateStateErrCov, #KF_AttState_KalmanFilter3, #KF_AttState_AttUpdate
**/
void KF_AttState_AttSensorProc3(
         Vector3d            *AttitudeSensorResidual,
         const Quaternion    *PropQuatEciToBcs,
         const Quaternion    *MeasuredQuatEciToBcs);
#endif /* KF_ATTSTATE_QUAT_INPUT == 1 */

/**
** \brief Attitude State Kalman Filter Propagate State Error Covariance
**
** \par Description
**          This function processes the latest Attitude Sensor readings and computes the residuals and
**          adjusted residuals.
**
** \par Assumptions, External Events, and Notes:
**          None
**
** \param[in]  CurrentState              A pointer to structure containing current state of Kalman Filter
**
** \param[in]  StateConst                A pointer to structure containing constants used by Kalman Filter
**
** \param[in]  BodyRate                  A pointer to a vector containing body rates previously corrected 
**                                       by the current best estimate of the IRU drift bias
**
** \param[in]  Period                    Time between Kalman Filter updates (secs)
**
** \returns
** \retcode #KF_ATTSTATE_SUCCESS             \retdesc \copydoc KF_ATTSTATE_SUCCESS            \endcode
** \retcode #KF_ATTSTATE_DIVERGENT_VARIANCE  \retdesc \copydoc KF_ATTSTATE_DIVERGENT_VARIANCE \endcode
** \retcode #KF_ATTSTATE_NEGATIVE_VARIANCE   \retdesc \copydoc KF_ATTSTATE_NEGATIVE_VARIANCE  \endcode
** \endreturns
**
** \sa #KF_AttState_AttSensorProc2, #KF_AttState_AttSensorProc3, #KF_AttState_KalmanFilter2, 
**     #KF_AttState_KalmanFilter3, #KF_AttState_AttUpdate
**/
int32 KF_AttState_PropagateStateErrCov(
         KF_AttState_CurrentState     *CurrentState, /* Ptr to KF Current State        */
         const KF_AttState_Constants  *StateConst,   /* Ptr to KF State Constants      */
         const Vector3d               *BodyRate,     /* IRU rates corrected by IRU drift bias */
         const double                  Period);      /* Time between KF updates (secs) */


#if KF_ATTSTATE_QUAT_INPUT == 1
/**
** \brief Attitude State Kalman Filter Processing for 3-axis Attitude Sensors
**
** \par Description
**          This function performs the Kalman Filter algorithm on the given 3-axis attitude state data.
**
** \par Assumptions, External Events, and Notes:
**          None
**
** \param[in]  CurrentState            A pointer to structure containing current state of Kalman Filter
**
** \param[in]  AttitudeSensorResidual  A pointer to a vector containing current residuals of attitude sensor
**
** \param[in]  StateConst              A pointer to structure containing constants used by Kalman Filter
**
** \param[out] *CurrentState           The Current Optimal Kalman Filter State and its associated
**                                     covariance are updated   
**
** \returns
** \retcode #KF_ATTSTATE_SUCCESS                  \retdesc \copydoc KF_ATTSTATE_SUCCESS                 \endcode
** \retcode #KF_ATTSTATE_LARGE_RELATIVE_RESIDUAL  \retdesc \copydoc KF_ATTSTATE_LARGE_RELATIVE_RESIDUAL \endcode
** \retcode #KF_ATTSTATE_LARGE_ABSOLUTE_RESIDUAL  \retdesc \copydoc KF_ATTSTATE_LARGE_ABSOLUTE_RESIDUAL \endcode
** \retcode #KF_ATTSTATE_SINGULAR_MATRIX          \retdesc \copydoc KF_ATTSTATE_SINGULAR_MATRIX         \endcode
** \retcode #KF_ATTSTATE_NEGATIVE_VARIANCE        \retdesc \copydoc KF_ATTSTATE_NEGATIVE_VARIANCE       \endcode
** \retcode #KF_ATTSTATE_ZERO_VARIANCE            \retdesc \copydoc KF_ATTSTATE_ZERO_VARIANCE           \endcode
** \endreturns
**
** \sa #KF_AttState_PropagateStateErrCov, #KF_AttState_AttSensorProc3, #KF_AttState_AttUpdate
**/
int32 KF_AttState_KalmanFilter3(
            KF_AttState_CurrentState       *CurrentState,            /* Ptr to KF Current State    */
            const Vector3d                 *AttitudeSensorResidual,  /* Ptr to Att Sensor Residual */
            const KF_AttState_Constants    *StateConst);             /* Ptr to KF State Constants  */
#endif /* KF_ATTSTATE_QUAT_INPUT == 1 */

/**
** \brief Attitude State Kalman Filter Attitude Update
**
** \par Description
**          This function updates the best estimate of the ECI to BCS quaternion using data obtained
**          previously from the Kalman Filter.
**
** \par Assumptions, External Events, and Notes:
**          None
**
** \param[in]  KFQuatEciToBcs      A pointer to quaternion to be populated with Kalman Filter's best
**                                 estimate of the ECI to BCS quaternion
**
** \param[in]  IruEstDriftBias     A pointer to a vector to be populated with Kalman Filter's best
**                                 estimate of the IRU drift bias
**
** \param[in]  PropQuatEciToBcs    A pointer to the measured ECI to BCS quaternion that had been
**                                 propagated to the current time using the current IRU data and the
**                                 previous Kalman Filter's best estimate of the IRU drift bias
**
** \param[in]  CurrentState        A pointer to the current state of the Kalman Filter
**
** \param[out] *KFQuatEciToBcs     The Kalman Filter's best estimate of the ECI to BCS quaternion
**
** \param[out] *IruEstDriftBias    The Kalman Filter's best estimate of the IRU's drift bias      
**
**
** \sa #KF_AttState_PropagateStateErrCov, #KF_AttState_AttSensorProc2, #KF_AttState_AttSensorProc3, #KF_AttState_AttUpdate
**/
void KF_AttState_AttUpdate(
         Quaternion                      *KFQuatEciToBcs,   /* Resulting best estimate of ECI to BCS Quaternion  */
         Vector3d                        *IruEstDriftBias,  /* Resulting best estimate of IRU Drift Bias         */
         const Quaternion                *PropQuatEciToBcs, /* Measured ECI to BCS Quat propagated with IRU data */
         const KF_AttState_CurrentState  *CurrentState);    /* Current state of Kalman Filter                    */


#if KF_ATTSTATE_VEC_INPUT == 1
/**
** \brief Attitude State Kalman Filter 2-axis Attitude Sensor Processing
**
** \par Description
**          This function processes the latest 2-axis Attitude Sensor readings and computes the residuals and
**          adjusted residuals.
**
** \par Assumptions, External Events, and Notes:
**          None
**
** \param[in]  AttitudeSensorResidual  A pointer to value containing residual associated with
**                                     a specific attitude sensor
**
** \param[in]  AttitudeSensorSensitivity  A pointer to vector containing first three terms of the
**                                        Attitude Sensor's Sensitivity Matrix
**
** \param[in]  ReferenceVector         A pointer to the reference vector in the estimated body frame
**
** \param[in]  MeasuredVector          A pointer to the 2-axis Attitude Sensor measurement vector
**
** \param[in]  StateConst              A pointer to structure containing constants used by Kalman Filter
**
** \param[out] *AttitudeSensorResidual The attitude sensor residual is updated with the differences between
**                                     the expected attitude and the measured attitude
**
** \param[out] *AttitudeSensorSensitivity  The attitude sensor sensitivity is updated
**
** \returns
** \retcode #KF_ATTSTATE_SUCCESS                  \retdesc \copydoc KF_ATTSTATE_SUCCESS                  \endcode
** \retcode #KF_ATTSTATE_LARGE_ABSOLUTE_RESIDUAL  \retdesc \copydoc KF_ATTSTATE_LARGE_ABSOLUTE_RESIDUAL  \endcode
** \endreturns
**
** \sa #KF_AttState_PropagateStateErrCov, #KF_AttState_KalmanFilter2, #KF_AttState_AttUpdate
**/
int32 KF_AttState_AttSensorProc2(
         double                      *AttitudeSensorResidual,
         Vector3d                    *AttitudeSensorSensitivity,
         const Vector3d              *ReferenceVector,
         const Vector3d              *MeasuredVector,
         const KF_AttState_Constants *StateConst);
#endif /* KF_ATTSTATE_VEC_INPUT == 1 */

#if KF_ATTSTATE_VEC_INPUT == 1
/**
** \brief Attitude State Kalman Filter Processing for 2-axis Attitude Sensors
**
** \par Description
**          This function performs the Kalman Filter algorithm on the given 2-axis attitude state data.
**
** \par Assumptions, External Events, and Notes:
**          None
**
** \param[in]  CurrentState            A pointer to structure containing current state of Kalman Filter
**
** \param[in]  AttitudeSensorResidual  A pointer to a vector containing current residuals of attitude sensor
**
** \param[in]  StateConst              A pointer to structure containing constants used by Kalman Filter
**
** \param[out] *CurrentState           The Current Optimal Kalman Filter State and its associated
**                                     covariance are updated   
**
** \returns
** \retcode #KF_ATTSTATE_SUCCESS            \retdesc \copydoc KF_ATTSTATE_SUCCESS           \endcode
** \retcode #KF_ATTSTATE_LARGE_RESIDUALS    \retdesc \copydoc KF_ATTSTATE_LARGE_RESIDUALS   \endcode
** \retcode #KF_ATTSTATE_SINGULAR_MATRIX    \retdesc \copydoc KF_ATTSTATE_SINGULAR_MATRIX   \endcode
** \retcode #KF_ATTSTATE_NEGATIVE_VARIANCE  \retdesc \copydoc KF_ATTSTATE_NEGATIVE_VARIANCE \endcode
** \retcode #KF_ATTSTATE_ZERO_VARIANCE      \retdesc \copydoc KF_ATTSTATE_ZERO_VARIANCE     \endcode
** \endreturns
**
** \sa #KF_AttState_PropagateStateErrCov, #KF_AttState_AttSensorProc2, #KF_AttState_AttUpdate
**/
int32 KF_AttState_KalmanFilter2(
            KF_AttState_CurrentState       *CurrentState,            /* Ptr to KF Current State    */
            const Vector3d                 *AttitudeSensorResidual,  /* Ptr to Att Sensor Residual */
            const KF_AttState_Constants    *StateConst);             /* Ptr to KF State Constants  */
#endif /* KF_ATTSTATE_VEC_INPUT == 1 */


/*************************************************************************/

#endif /* _kf_attstate_ */

/************************/
/*  End of File Comment */
/************************/
