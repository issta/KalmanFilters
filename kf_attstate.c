/*
** $Id: kf_attstate.c 1.9 2012/03/28 12:53:22EDT dkobe Exp  $
**
** Purpose:  Attitude State Kalman Filter Implementation
**
** References:
**    - Kalman Filter design derived from LRO Kalman Filter and modified for both
**      GPM and with a goal of making it a component of the FSB Re-use library
**    - GPM GN&C Kalman Filter Algorithm Specification describes the Kalman Filter
**      in Algorithm format
** 
** Notes:
**    - This module requires the Code 582 Flight Software Re-use Library (FSRL)
**      Math Library
**    - This module assumes attitude measurement is made in the form of a quaternion
**    - This module assumes the spacecraft body rate is obtained from an IRU
** 
*/

/*************************************************************************
**
** Include section
**
**************************************************************************/

#include <math.h>
#include <string.h> /* For memset */
//#include "cfe_psp.h"
#include "kf_attstate.h"
#include "kf_attstate_cfg.h"
#include "matrix3x3d.h"
#include "matrixmxnd.h"
#include "mathconstants.h"

/******************** Macro Definitions **********************************/
#define KF_ATTSTATE_LARGE_RESIDUALS 1

/******************** Type Definitions ***********************************/

/******************** Local Prototypes ***********************************/

/**
** \brief Attitude State Kalman Filter State Transition Matrix Generation
**
** \par Description
**          This function creates the state transition matrix using exact
**          form for a given constant non-zero body rate vector and time
**          period.
**
** \par Assumptions, External Events, and Notes:
**          None
**
** \param[in] BodyRate        Spacecraft body rates as measured by the IRU
**                            and updated with drift correction [rad/sec]
**
** \param[in] Period          Time between Kalman Filter updates [secs]
**
** \returns
** \retstmt
**        This function returns a state transition matrix that can be used
**        to propagate the estimation error covariance matrix.  \endcode
** \endreturns
**
**/
Matrix6x6d KF_AttState_TransitionMatrix(const Vector3d   BodyRate,
                                        const double     Period);

/**
** \brief Attitude State Kalman Filter Initialize Error Covariance
**
** \par Description
**          This function sets the Error Covariance Matrix to the
**          "No Knowledge" state.  This includes setting the attitude
**          error covariance to (pi/3)^2 and the IRU drift bias to
**          a table specified value.
**
** \par Assumptions, External Events, and Notes:
**          None
**
** \param[in]  ErrCovMatrix     A pointer to a 6x6 Error Covariance Matrix to be initialized
**
** \param[in]  StateConst       A pointer to the standard AttState constants data structure
**                              which is typically initialized with the #KF_AttState_InitConstants
**                              function.
**
** \param[out] *ErrCovMatrix    The Err Covariance Matrix is set equal to "no knowledge" state.
**
**/
void KF_AttState_InitErrCov(Matrix6x6d                  *ErrCovMatrix,
                            const KF_AttState_Constants *StateConst);


/*******************************************************************
**
** KF_AttState_Init
**
** NOTE: For complete prolog information, see 'kf_attstate.h'
********************************************************************/

void KF_AttState_Init(
         KF_AttState_Constants     *StateConst,
         KF_AttState_CurrentState  *CurrentState,
         const KF_AttState_Tbl     *TableData,
         const double               Period)
{
    /* Initialize Constants */
    KF_AttState_InitConstants(StateConst, TableData, Period);
    
    /* Clear memory of any residual data */
    memset(CurrentState, 0, sizeof(KF_AttState_CurrentState));
    
    /* Set Error Covariance Matrix to "No Knowledge" State */
    KF_AttState_InitErrCov(&CurrentState->ErrCov, StateConst);
}


/*******************************************************************
**
** KF_AttState_InitConstants
**
** NOTE: For complete prolog information, see 'kf_attstate.h'
********************************************************************/

void KF_AttState_InitConstants(
         KF_AttState_Constants    *StateConst,
         const KF_AttState_Tbl    *TableData,
         const double              Period)
{
    double      SNC1, SNC23, SNC4;
    double      Period2, Period3;
    Matrix3x3d  MNC_Transpose;
    
    /* Clear all constants of any garbage */
    memset(StateConst, 0, sizeof(KF_AttState_Constants));

    /* Initialize the Measurement Noise Covariance Matrix */
    /* by making sure it is as symmetric as possible      */
    Matrix3x3d_Transpose(&MNC_Transpose, &TableData->MeasurementNoiseCov);
    Matrix3x3d_Add(&StateConst->MeasurementNoiseCov, &MNC_Transpose, &TableData->MeasurementNoiseCov);
    Matrix3x3d_MultScalar(&StateConst->MeasurementNoiseCov, &StateConst->MeasurementNoiseCov, 0.5);
    
    /* Initialize the State Transition Noise Covariance Matrix */
    Period2 = Period * Period;
    Period3 = Period2 * Period;
    
    SNC1  = (TableData->SigmaVSquared * Period) + 
            ((TableData->SigmaUSquared * Period3) / 3.0);
    SNC23 = -0.5 * (TableData->SigmaUSquared * Period2);
    SNC4  = TableData->SigmaUSquared * Period;
    
    /* Initialize the appropriate diagonals in each quadrant */
    StateConst->StateTransNoiseCov.Comp[0][0] = SNC1;
    StateConst->StateTransNoiseCov.Comp[1][1] = SNC1;
    StateConst->StateTransNoiseCov.Comp[2][2] = SNC1;
    
    StateConst->StateTransNoiseCov.Comp[0][3] = SNC23;
    StateConst->StateTransNoiseCov.Comp[1][4] = SNC23;
    StateConst->StateTransNoiseCov.Comp[2][5] = SNC23;
    
    StateConst->StateTransNoiseCov.Comp[3][0] = SNC23;
    StateConst->StateTransNoiseCov.Comp[4][1] = SNC23;
    StateConst->StateTransNoiseCov.Comp[5][2] = SNC23;
    
    StateConst->StateTransNoiseCov.Comp[3][3] = SNC4;
    StateConst->StateTransNoiseCov.Comp[4][4] = SNC4;
    StateConst->StateTransNoiseCov.Comp[5][5] = SNC4;
    
    /* Make a copy of other limits specified in table */
    StateConst->RsSqRatioLimit = TableData->RsSqRatioLimit;
    StateConst->InitBiasCov    = TableData->InitBiasCov;
    StateConst->AttCovDivTol   = TableData->AttCovDivTol;
    StateConst->BiasCovDivTol  = TableData->BiasCovDivTol;
    StateConst->AttCovConTol   = TableData->AttCovConTol;
    StateConst->BiasCovConTol  = TableData->BiasCovConTol;
    StateConst->RsSqRatioLimit = TableData->RsSqRatioLimit;
    StateConst->MaxSqrdResTol  = TableData->MaxSqrdResTol;
    
#if KF_ATTSTATE_VEC_INPUT == 1
    StateConst->MeasuredVecVariance = TableData->SigmaMSquared;
#endif /* KF_ATTSTATE_VEC_INPUT */
    
}


/*******************************************************************
**
** KF_AttState_InitErrCov
**
** NOTE: For complete prolog information, see above
********************************************************************/
void KF_AttState_InitErrCov(Matrix6x6d                  *ErrCovMatrix,
                            const KF_AttState_Constants *StateConst)
{
    uint32  i, j;
    
    /* All non-diagonal terms are set to zero */
    Matrix6x6d_InitZero(ErrCovMatrix);
    
    /* Current Attitude Error Covariance is set to "no knowledge" value to 
     * ensure it is larger than variance of attitude sensors */
    for (i=0; i<3; i++)
    {
        for (j=0; j<3; j++)
        {
            ErrCovMatrix->Comp[i][j] = StateConst->MeasurementNoiseCov.Comp[i][j];
        }
    }
    
    /* Current Bias Covariance is initialized with diagonal elements 
     * set to table specified parameter */
    for (i=3; i<6; i++)
    {
        ErrCovMatrix->Comp[i][i] = StateConst->InitBiasCov;
    }
}

#if KF_ATTSTATE_QUAT_INPUT == 1         
/*******************************************************************
**
** KF_AttState_AttSensorProc3
**
** NOTE: For complete prolog information, see 'kf_attstate.h'
********************************************************************/

void KF_AttState_AttSensorProc3(
         Vector3d           *AttitudeSensorResidual,
         const Quaternion   *PropQuatEciToBcs,
         const Quaternion   *MeasuredQuatEciToBcs)
{
    uint32     Axis;
    Quaternion DeltaQuat, ExpectedQuatBcsToEci;
    double     HalfTheta, SinHalfTheta, Factor;

    /* Calculate quaternion difference between the expected and measured quaternions */
    Quaternion_Conjugate(&ExpectedQuatBcsToEci, PropQuatEciToBcs);
    Quaternion_Mult(&DeltaQuat, &ExpectedQuatBcsToEci, MeasuredQuatEciToBcs);


    /* Computes the magnitude of the quaternion errors.  Check that HalfTheta is not
        greater than 1 before taking the arccosine */
    if ((1.0 - fabs(DeltaQuat.Comp[3])) < MTH_DOUBLE_ZERO_TOLERANCE)
    {
        HalfTheta = 0;
    }
    else
    {
        HalfTheta = acos(DeltaQuat.Comp[3]);
    }

    SinHalfTheta = sin(HalfTheta);

    /* Calculate the Star Tracker Residuals and the H matrix */
    if(fabs(SinHalfTheta) > MTH_DOUBLE_ZERO_TOLERANCE)
    {
        Factor = 2.0 * (HalfTheta / SinHalfTheta);
    }
    else
    {
        Factor = 2.0;
    }

    for (Axis = 0; Axis < 3; Axis++ )
    {       
        AttitudeSensorResidual->Comp[Axis] = Factor * DeltaQuat.Comp[Axis];
    }
}
#endif /* KF_ATTSTATE_QUAT_INPUT == 1 */

#if KF_ATTSTATE_VEC_INPUT == 1
/*******************************************************************
**
** KF_AttState_AttSensorProc2
**
** NOTE: For complete prolog information, see 'kf_attstate.h'
********************************************************************/

int32 KF_AttState_AttSensorProc2(
         double                      *AttitudeSensorResidual,
         Vector3d                    *AttitudeSensorSensitivity,
         const Vector3d              *ReferenceVector,
         const Vector3d              *MeasuredVector,
         const KF_AttState_Constants *StateConst)
{
    Vector3d   DifferenceVector;
    Vector3d   CrossProdVector;
    double     Residual;
    int32      ReturnCode = KF_ATTSTATE_SUCCESS;
    
    Vector3d_Sub(&DifferenceVector, MeasuredVector, ReferenceVector);
    
    Residual = Vector3d_Magnitude(&DifferenceVector);
    
    /* Return the calculated Residual */
    *AttitudeSensorResidual = Residual;
    
    /* Check Residual for Reasonableness */
    if ((Residual*Residual) > StateConst->MaxSqrdResTol)
    {
        ReturnCode = KF_ATTSTATE_LARGE_RESIDUALS;
    }
    else
    {
         if (Residual > MTH_DOUBLE_ZERO_TOLERANCE)
         {
             Vector3d_Cross(&CrossProdVector, &DifferenceVector, MeasuredVector);
             Vector3d_DivScalar(AttitudeSensorSensitivity, CrossProdVector, Residual);
         }
         else if (sqrt((MeasuredVector->Comp[1]*MeasuredVector->Comp[1]) +
                       (MeasuredVector->Comp[2]*MeasuredVector->Comp[2])) > MTH_DOUBLE_ZERO_TOLERANCE)
         {
             /* If the Measurement Vector is non-zero and Residual is near zero */
             AttitudeSensorSensitivity->Comp[0] = 0.0;
             AttitudeSensorSensitivity->Comp[1] = -MeasuredVector->Comp[2];
             AttitudeSensorSensitivity->Comp[2] = MeasuredVector->Comp[1];
         }
         else /* Residuals and Measurement Vector are near zero */
         {
             AttitudeSensorSensitivity->Comp[0] = 0.0;
             AttitudeSensorSensitivity->Comp[1] = 1.0;
             AttitudeSensorSensitivity->Comp[2] = 0.0;
         }
    }
    
    return ReturnCode;
}
#endif /* KF_ATTSTATE_VEC_INPUT == 1 */

/*******************************************************************
**
** KF_AttState_PropagateStateErrCov
**
** NOTE: For complete prolog information, see 'kf_attstate.h'
********************************************************************/

int32 KF_AttState_PropagateStateErrCov(
        KF_AttState_CurrentState     *CurrentState,
        const KF_AttState_Constants  *StateConst,
        const Vector3d               *BodyRate,
        const double                  Period)
{
    int32      Status = KF_ATTSTATE_SUCCESS;
    Matrix6x6d StateTransition_Transpose;
    Matrix6x6d Temp1;
    Matrix6x6d ErrCov_Transpose;
    uint32     i;
    
    /* Create the State Transition Matrix */
    CurrentState->StateTransition = KF_AttState_TransitionMatrix(*BodyRate, Period);

    Matrix6x6d_Transpose(&StateTransition_Transpose, &CurrentState->StateTransition);

    /* Propagate estimated error covariance matrix */
    Matrix6x6d_Mult(&Temp1, &CurrentState->ErrCov, &StateTransition_Transpose);
    Matrix6x6d_Mult(&CurrentState->ErrCov, &CurrentState->StateTransition,  &Temp1);
    Matrix6x6d_Add(&CurrentState->ErrCov, &CurrentState->ErrCov, &StateConst->StateTransNoiseCov);

    /* Force Error Covariance matrix to be symmetric (round-off error mitigation) */
    Matrix6x6d_Transpose (&ErrCov_Transpose, &CurrentState->ErrCov);
    Matrix6x6d_Add(&CurrentState->ErrCov, &CurrentState->ErrCov, &ErrCov_Transpose);
    Matrix6x6d_DivScalar(&CurrentState->ErrCov, &CurrentState->ErrCov, 2.0);
    
    i=0;
    while (i<3)
    {
        /* Check the Attitude Covariances for validity */
        if (CurrentState->ErrCov.Comp[i][i] > StateConst->AttCovDivTol)
        {
            Status |= (KF_ATTSTATE_ATT_DIVERGENT_VARIANCE_X << i);
            Status |= KF_ATTSTATE_ERROR_BIT;
        }
        else if (CurrentState->ErrCov.Comp[i][i] < 0.0)
        {
            Status |= (KF_ATTSTATE_NEGATIVE_VARIANCE_ATT_X << i);
            Status |= KF_ATTSTATE_ERROR_BIT;
        }
        
        /* Check the Drift Bias Covariances for validity */
        if (CurrentState->ErrCov.Comp[i+3][i+3] > StateConst->BiasCovDivTol)
        {
            Status |= (KF_ATTSTATE_BIAS_DIVERGENT_VARIANCE_X << i);
            Status |= KF_ATTSTATE_ERROR_BIT;
        }
        else if (CurrentState->ErrCov.Comp[i+3][i+3] < 0.0)
        {
            Status |= (KF_ATTSTATE_NEGATIVE_VARIANCE_BIAS_X << i);
            Status |= KF_ATTSTATE_ERROR_BIT;
        }

        /* Check the Attitude Covariances for convergence */
        if (CurrentState->ErrCov.Comp[i][i] > StateConst->AttCovConTol)
        {
            Status |= (KF_ATTSTATE_NOT_CONVERGED << i);
        }
        
        /* Check the Drift Bias Covariances for convergence */
        if (CurrentState->ErrCov.Comp[i+3][i+3] > StateConst->BiasCovConTol)
        {
            Status |= (KF_ATTSTATE_NOT_CONVERGED << (i+3));
        }
        
        i++;
    }

    /* Determine if total convergence has been achieved */
    if ((Status & KF_ATTSTATE_NOT_CONVERGED_MASK) == 0)
    {
        Status |= KF_ATTSTATE_CONVERGED;
    }

    /* Zero Current Optimal Kalman Filter State because it is defined to be
     * relative to the propagated prior solution.   */
    Vector6d_InitZero(&CurrentState->OptimalState);
    
    return (Status);
}


/*******************************************************************
**
** KF_AttState_TransitionMatrix
**
** NOTE: For complete prolog information, see above
********************************************************************/

Matrix6x6d KF_AttState_TransitionMatrix(const Vector3d   BodyRate,
                                        const double     Period)
{
    double      RotationAngle;     /* Rotation Angle (alpha) */
    double      RotationAngle2;    /* Rotation Angle Squared */
    double      C;                 /* cos(RotationAngle)     */
    double      OmC;               /* 1 - cos(RotationAngle) */
    double      OmCdRA2;           /* OmC / (RotationAngle)^2 */ 
    double      S;                 /* sin(RotationAngle)     */
    double      SdRA;              /* sin(RotationAngle) / RotationAngle */
    double      OmSdRA;            /* (1 - sin(RotationAngle) / RotationAngle */
    double      RateMag;           /* Body Rate Magnitude */
    double      NegPeriod;
    Matrix6x6d  STM;               /* State Transition Matrix */
    Matrix3x3d  Skew, EigenDyad;
    Vector3d    EigenVector;
    
    /* Compute magnitude of spacecraft rate */
    RateMag = Vector3d_Magnitude(&BodyRate);
    
    /* Compute the Rotation Angle and the Eigen Vector */
    RotationAngle = RateMag * Period;
    
    if (fabs(RateMag) < MTH_DOUBLE_ZERO_TOLERANCE)
    {
        EigenVector.Comp[0] = 0.0;
        EigenVector.Comp[1] = 0.0;
        EigenVector.Comp[2] = 0.0;
    }
    else
    {
        Vector3d_DivScalar (&EigenVector, &BodyRate, RateMag);
    }

    Skew.Comp[0][0] =  0.0;
    Skew.Comp[0][1] =  EigenVector.Comp[2];
    Skew.Comp[0][2] = -EigenVector.Comp[1];
    
    Skew.Comp[1][0] = -EigenVector.Comp[2];
    Skew.Comp[1][1] =  0.0;
    Skew.Comp[1][2] =  EigenVector.Comp[0];
    
    Skew.Comp[2][0] =  EigenVector.Comp[1];
    Skew.Comp[2][1] = -EigenVector.Comp[0];
    Skew.Comp[2][2] =  0.0;
    
    /* Perform an Outer Multiplication to get the Eigen Dyad */
    MatrixMxNd_Mult( &EigenDyad.Comp[0][0], 
                     &EigenVector.Comp[0], 3, 1, 
                     &EigenVector.Comp[0], 1, 3);

    RotationAngle2 = RotationAngle * RotationAngle;
    
    /* Determine if a small angle approximation is necessary */
    if (RotationAngle > KF_ATTSTATE_ROTATION_ANGLE_THRESHOLD)
    {
        C = cos(RotationAngle);
        OmC = 1.0 - C;
        OmCdRA2 = OmC / RotationAngle2;
        S = sin(RotationAngle);
        SdRA = S / RotationAngle;
        OmSdRA = 1.0 - SdRA;
    }
    else /* Perform a small angle approximation to eliminate round-off errors */
    {
        OmCdRA2 = (1.0 - (RotationAngle2 / 12.0)) / 2.0;
        OmC = OmCdRA2 * RotationAngle2;
        C = 1.0 - OmC;
        OmSdRA = RotationAngle2 / 6.0;
        SdRA = 1.0 - OmSdRA;
        S = RotationAngle * SdRA;
    }

    /* Form the State Transition Matrix one quadrant at a time */
    
    /* Upper Left Quadrant */
    STM.Comp[0][0] = C + (OmC * EigenDyad.Comp[0][0]);
    STM.Comp[0][1] =     (OmC * EigenDyad.Comp[0][1]) + (S * Skew.Comp[0][1]);
    STM.Comp[0][2] =     (OmC * EigenDyad.Comp[0][2]) + (S * Skew.Comp[0][2]);
    
    STM.Comp[1][0] =     (OmC * EigenDyad.Comp[1][0]) + (S * Skew.Comp[1][0]);
    STM.Comp[1][1] = C + (OmC * EigenDyad.Comp[1][1]);
    STM.Comp[1][2] =     (OmC * EigenDyad.Comp[1][2]) + (S * Skew.Comp[1][2]);
    
    STM.Comp[2][0] =     (OmC * EigenDyad.Comp[2][0]) + (S * Skew.Comp[2][0]);
    STM.Comp[2][1] =     (OmC * EigenDyad.Comp[2][1]) + (S * Skew.Comp[2][1]);
    STM.Comp[2][2] = C + (OmC * EigenDyad.Comp[2][2]);
    
    /* Upper Right Quadrant */
    NegPeriod = -Period;
    STM.Comp[0][3] = NegPeriod * (SdRA + (OmSdRA * EigenDyad.Comp[0][0]));
    STM.Comp[0][4] = NegPeriod * (       (OmSdRA * EigenDyad.Comp[0][1]) + 
                                  (OmCdRA2 * RotationAngle * Skew.Comp[0][1]));
    STM.Comp[0][5] = NegPeriod * (       (OmSdRA * EigenDyad.Comp[0][2]) + 
                                  (OmCdRA2 * RotationAngle * Skew.Comp[0][2]));
    
    STM.Comp[1][3] = NegPeriod * (       (OmSdRA * EigenDyad.Comp[1][0]) + 
                                  (OmCdRA2 * RotationAngle * Skew.Comp[1][0]));
    STM.Comp[1][4] = NegPeriod * (SdRA + (OmSdRA * EigenDyad.Comp[1][1]));
    STM.Comp[1][5] = NegPeriod * (       (OmSdRA * EigenDyad.Comp[1][2]) + 
                                  (OmCdRA2 * RotationAngle * Skew.Comp[1][2]));
    
    STM.Comp[2][3] = NegPeriod * (       (OmSdRA * EigenDyad.Comp[2][0]) + 
                                  (OmCdRA2 * RotationAngle * Skew.Comp[2][0]));
    STM.Comp[2][4] = NegPeriod * (       (OmSdRA * EigenDyad.Comp[2][1]) + 
                                  (OmCdRA2 * RotationAngle * Skew.Comp[2][1]));
    STM.Comp[2][5] = NegPeriod * (SdRA + (OmSdRA * EigenDyad.Comp[2][2]));
    
    /* Lower Left Quadrant (Zero Matrix) */
    STM.Comp[3][0] = 0.0;
    STM.Comp[3][1] = 0.0;
    STM.Comp[3][2] = 0.0;
    
    STM.Comp[4][0] = 0.0;
    STM.Comp[4][1] = 0.0;
    STM.Comp[4][2] = 0.0;
    
    STM.Comp[5][0] = 0.0;
    STM.Comp[5][1] = 0.0;
    STM.Comp[5][2] = 0.0;
    
    /* Lower Right Quadrant (Identity Matrix) */
    STM.Comp[3][3] = 1.0;
    STM.Comp[3][4] = 0.0;
    STM.Comp[3][5] = 0.0;
    
    STM.Comp[4][3] = 0.0;
    STM.Comp[4][4] = 1.0;
    STM.Comp[4][5] = 0.0;
    
    STM.Comp[5][3] = 0.0;
    STM.Comp[5][4] = 0.0;
    STM.Comp[5][5] = 1.0;
    
    return STM;

} /* end of TransitionMatrix() */


#if KF_ATTSTATE_QUAT_INPUT == 1
/*******************************************************************
**
** KF_AttState_KalmanFilter3
**
** NOTE: For complete prolog information, see 'kf_attstate.h'
********************************************************************/

int32 KF_AttState_KalmanFilter3(
        KF_AttState_CurrentState       *CurrentState,
        const Vector3d                 *AttitudeSensorResidual,
        const KF_AttState_Constants    *StateConst)
{
    KF_Matrix6x3d   KalmanGain;
    Matrix3x3d      CurrentAttErrCov, AttErrCovWithMeasNoise, AttErrCovWithMeasNoise_Inverted;
    Matrix6x6d      I_minus_KalmanGainH, NewErrCov;
    double          SumSqResiduals, ExpectedVariance;
    Vector6d        TempOptimalState, ZTemp, NewOptimalState;
    uint32          i, j;
    int32           Status = KF_ATTSTATE_SUCCESS;
   
    /* Compute the Sum of the Squares of the Input Residuals to test for reasonableness */
    SumSqResiduals = Vector3d_Dot(AttitudeSensorResidual, AttitudeSensorResidual);
    
    ExpectedVariance = 0.0;
    for (i=0; i<3; i++)
    {
        ExpectedVariance += CurrentState->ErrCov.Comp[i][i];
        ExpectedVariance += StateConst->MeasurementNoiseCov.Comp[i][i];
        
        /* Extract upper left corner of current Error Covariance matrix
         * to obtain just the Error Covariance associated with Attitude  */
        for (j=0; j<3; j++)
        {
            CurrentAttErrCov.Comp[i][j] = CurrentState->ErrCov.Comp[i][j];
        }
    }

    if ((fabs(ExpectedVariance) < MTH_DOUBLE_ZERO_TOLERANCE) || 
        ((SumSqResiduals / ExpectedVariance) > StateConst->RsSqRatioLimit))
    {
        Status = KF_ATTSTATE_LARGE_RELATIVE_RESIDUAL;
    }
    else if (SumSqResiduals > StateConst->MaxSqrdResTol)
    {
        Status = KF_ATTSTATE_LARGE_ABSOLUTE_RESIDUAL;
    }
    else
    {
        Matrix3x3d_Add(&AttErrCovWithMeasNoise, &CurrentAttErrCov, &StateConst->MeasurementNoiseCov);

        if (Matrix3x3d_Invert(&AttErrCovWithMeasNoise_Inverted, &AttErrCovWithMeasNoise) == 1)
        {
            /* Initialize the 6x6 Identity Matrix used to compute ( I - KalmanGain * StateToMeasurement ) */
            Matrix6x6d_InitIdentity(&I_minus_KalmanGainH);
        
            /* Loop through each column of the Inverted Attitude Error Covariance 
             * with Measurement Noise Covariance Matrix */
            for (j=0; j<3; j++)
            {
                /* Loop through each row of the Error Covariance Matrix */
                for (i=0; i<6; i++)
                {
                    KalmanGain.Comp[i][j] = CurrentState->ErrCov.Comp[i][0] * AttErrCovWithMeasNoise_Inverted.Comp[0][j] + 
                                            CurrentState->ErrCov.Comp[i][1] * AttErrCovWithMeasNoise_Inverted.Comp[1][j] +
                                            CurrentState->ErrCov.Comp[i][2] * AttErrCovWithMeasNoise_Inverted.Comp[2][j];

                    I_minus_KalmanGainH.Comp[i][j] -= KalmanGain.Comp[i][j];
                }
            }
        
            /* Compute the new optimal state */
            MatrixMxNd_Mult(&TempOptimalState.Comp[0], 
                            &I_minus_KalmanGainH.Comp[0][0], 6, 6, 
                            &CurrentState->OptimalState.Comp[0], 6, 1);
        
            MatrixMxNd_Mult(&ZTemp.Comp[0], 
                            &KalmanGain.Comp[0][0], 6, 3, 
                            &AttitudeSensorResidual->Comp[0], 3, 1);
        
            Vector6d_Add(&NewOptimalState, &TempOptimalState, &ZTemp);

            /* Compute the new Error Covariance Matrix */
            Matrix6x6d_Mult(&NewErrCov, &I_minus_KalmanGainH, &CurrentState->ErrCov);
            
            /* Compute Adjusted Residuals for telemetry */
            for (i=0; i<3; i++)
            {
                CurrentState->AdjustedResidual.Comp[i] = AttitudeSensorResidual->Comp[i] - 
                                                         NewOptimalState.Comp[i];
            }
        
            /* Validate the new Error Covariance Matrix before using the new Optimal State */
            i=0;
            while (i<3)
            {
                if (NewErrCov.Comp[i][i] < 0.0)
                {
                    Status |= (KF_ATTSTATE_NEGATIVE_VARIANCE_ATT_X << i);
                    Status |= KF_ATTSTATE_ERROR_BIT;
                }
                if (NewErrCov.Comp[i+3][i+3] < 0.0)
                {
                    Status |= (KF_ATTSTATE_NEGATIVE_VARIANCE_BIAS_X << i);
                    Status |= KF_ATTSTATE_ERROR_BIT;
                }
                
                if (NewErrCov.Comp[i][i] > StateConst->AttCovConTol)
                {
                    Status |= (KF_ATTSTATE_NOT_CONVERGED << i);
                }
                
                if (NewErrCov.Comp[i+3][i+3] > StateConst->BiasCovConTol)
                {
                    Status |= (KF_ATTSTATE_NOT_CONVERGED << (i+3));
                }
                i++;
            }

            /* Determine if total convergence has been achieved */
            if ((Status & KF_ATTSTATE_NOT_CONVERGED_MASK) == 0)
            {
                Status |= KF_ATTSTATE_CONVERGED;
            }
        
            if (Status >= KF_ATTSTATE_SUCCESS)
            {
                /* Update current state to newly computed values */
                Matrix6x6d_Copy(&CurrentState->ErrCov, &NewErrCov);
                Vector6d_Copy(&CurrentState->OptimalState, &NewOptimalState);
            }
        }
        else  /* Matrix is not invertable */
        {
            Status = KF_ATTSTATE_SINGULAR_MATRIX;
        }
    }
    
    return Status;
}
#endif /* KF_ATTSTATE_QUAT_INPUT == 1 */


#if KF_ATTSTATE_VEC_INPUT == 1
/*******************************************************************
**
** KF_AttState_KalmanFilter2
**
** NOTE: For complete prolog information, see 'kf_attstate.h'
********************************************************************/

int32 KF_AttState_KalmanFilter2(
        KF_AttState_CurrentState       *CurrentState,
        const double                    AttitudeSensorResidual,
        const Vector3d                 *AttitudeSensorSensitivity,
        const KF_AttState_Constants    *StateConst)
{
    KF_Matrix6x3d   CurrentErrCov;
    Matrix3x3d      GainTimesSensitivity;
    double          SumSqResiduals, ExpectedVariance, KalmanGainDivisor;
    uint32          i, j;
    Matrix6x6d      I_minus_GainTimesSensitivity, NewErrCov;
    Vector6d        AttErrCovTimesSensitivity;
    Vector6d        KalmanGain;
    Vector6d        TempOptimalState, ZTemp, NewOptimalState;
    int32           Status = KF_ATTSTATE_SUCCESS;
   
    /* Compute the Sum of the Squares of the Input Residuals to test for reasonableness */
    SumSqResiduals = AttitudeSensorResidual * AttitudeSensorResidual;
    
    ExpectedVariance = 0.0;
    for (i=0; i<3; i++)
    {
        ExpectedVariance += CurrentState->ErrCov.Comp[i][i];
        ExpectedVariance += StateConst->MeasurementNoiseCov.Comp[i][i];
        
        /* Extract left side of current Error Covariance matrix
         * to obtain just the Error Covariance associated with Attitude  */
        for (j=0; j<3; j++)
        {
            CurrentErrCov.Comp[i][j] = CurrentState->ErrCov.Comp[i][j];
            CurrentErrCov.Comp[i+3][j] = CurrentState->ErrCov.Comp[i+3][j];
        }
    }
    ExpectedVariance = ExpectedVariance + (2 * StateConst->MeasuredVecVariance);
    
    /* Determine if the residual is reasonable */
    if ((SumSqResiduals / ExpectedVariance) > StateConst->RsSqRatioLimit)
    {
        Status = KF_ATTSTATE_LARGE_RELATIVE_RESIDUAL;
    }
    else if (SumSqResiduals > StateConst->MaxSqrdResTol)
    {
        Status = KF_ATTSTATE_LARGE_ABSOLUTE_RESIDUAL;
    }
    else
    {
        MatrixMxNd_Mult(&AttErrCovTimesSensitivity, &CurrentErrCov, 6, 3, AttitudeSensorSensitivity, 3, 1);
        KalmanGainDivisor = (AttitudeSensorSensitivity.Comp[0] * AttErrCovTimesSensitivity.Comp[0]) +
                            (AttitudeSensorSensitivity.Comp[1] * AttErrCovTimesSensitivity.Comp[1]) +
                            (AttitudeSensorSensitivity.Comp[2] * AttErrCovTimesSensitivity.Comp[2]);
        
        if (KalmanGainDivisor < MTH_DOUBLE_ZERO_TOLERANCE)
        {
            Status = KF_ATTSTATE_BAD_VEC_VARIANCE;
        }
        else
        {
            Vector6d_DivScalar(&KalmanGain, &AttErrCovTimesSensitivity, KalmanGainDivisor);
            Matrix6x6d_InitIdentity(&I_minus_GainTimesSensitivity);
        
            for (i=0; i<3; i++)
            {
                for (j=0; j<3; j++)
                {
                    GainTimesSensitivity.Comp[i][j] = KalmanGain.Comp[i] * AttitudeSensorSensitivity->Comp[j];
                    I_minus_GainTimesSensitivity.Comp[i][j] -= GainTimesSensitivity.Comp[i][j];
                }
            }
        
            /* Compute the new optimal state */
            MatrixMxNd_Mult(&TempOptimalState.Comp[0], 
                            &I_minus_GainTimesSensitivity.Comp[0][0], 6, 6, 
                            &CurrentState->OptimalState.Comp[0], 6, 1);
        
            Vector6d_MultScalar(&ZTemp, &KalmanGain, AttitudeSensorResidual);
            Vector6d_Add(&NewOptimalState, &TempOptimalState, &ZTemp);
            
            /* Compute the new Error Covariance Matrix */
            Matrix6x6d_Mult(&NewErrCov, &I_minus_KalmanGain, &CurrentState->ErrCov);

            /* Validate the new Error Covariance Matrix before using the new Optimal State */
            i=0;
            while (i<3)
            {
                if (NewErrCov.Comp[i][i] < 0.0)
                {
                    Status |= (KF_ATTSTATE_NEGATIVE_VARIANCE_ATT_X << i);
                    Status |= KF_ATTSTATE_ERROR_BIT;
                }
                if (NewErrCov.Comp[i+3][i+3] < 0.0)
                {
                    Status |= (KF_ATTSTATE_NEGATIVE_VARIANCE_BIAS_X << i);
                    Status |= KF_ATTSTATE_ERROR_BIT;
                }
                
                if (NewErrCov.Comp[i][i] > StateConst->AttCovConTol)
                {
                    Status |= (KF_ATTSTATE_NOT_CONVERGED << i);
                }
                
                if (NewErrCov.Comp[i+3][i+3] > StateConst->BiasCovConTol)
                {
                    Status |= (KF_ATTSTATE_NOT_CONVERGED << (i+3));
                }
                i++;
            }

            /* Determine if total convergence has been achieved */
            if ((Status & KF_ATTSTATE_NOT_CONVERGED_MASK) == 0)
            {
                Status |= KF_ATTSTATE_CONVERGED;
            }
        
            if (Status >= KF_ATTSTATE_SUCCESS)
            {
                /* Update current state to newly computed values */
                Matrix6x6d_Copy(&CurrentState->ErrCov, &NewErrCov);
                Vector6d_Copy(&CurrentState->OptimalState, &NewOptimalState);
            }
        }
    }
    else  /* Matrix is not invertable */
    {
        Status = KF_ATTSTATE_SINGULAR_MATRIX;
    }
    
    return Status;
}
#endif /* KF_ATTSTATE_VEC_INPUT == 1 */

/*******************************************************************
**
** KF_AttState_AttUpdate
**
** NOTE: For complete prolog information, see 'kf_attstate.h'
********************************************************************/

void KF_AttState_AttUpdate(
         Quaternion                      *KFQuatEciToBcs,
         Vector3d                        *IruEstDriftBias,
         const Quaternion                *PropQuatEciToBcs,
         const KF_AttState_CurrentState  *CurrentState)
{
    uint32      i;
    Quaternion  QuatKFState, ProductQuat;
    double      SumOfSquares = 0.0;

    /* Update inertial to body quaternion with optimal state estimate, XF */
    for (i = 0; i < 3; i++)
    {
        QuatKFState.Comp[i]  = 0.5 * CurrentState->OptimalState.Comp[i];
        SumOfSquares += QuatKFState.Comp[i] * QuatKFState.Comp[i];
    }

    QuatKFState.Comp[3]  = sqrt(1.0 - SumOfSquares);

    Quaternion_Mult(&ProductQuat, PropQuatEciToBcs, &QuatKFState);
    Quaternion_Normalize(KFQuatEciToBcs, &ProductQuat);

    /* Update estimated IRU drift bias in body frame */
    for (i = 0; i < 3; i++)
    {
        IruEstDriftBias->Comp[i] += CurrentState->OptimalState.Comp[i + 3];
    }
}


/************************/
/*  End of File Comment */
/************************/

