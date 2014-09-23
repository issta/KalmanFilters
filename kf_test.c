/* KF library test */

#include <stdio.h>
#include "kf_attstate.h"
#include "kf_attstate_cfg.h"
#include "kf_attstate_tbl.h"

int main(void)
{

int i, j;

//    Matrix6x6d STNCovMat;
 //   Matrix3x3d MNCovMat;
    double     InitBiasCov   = 1.0e-5;              // Initial IRU Drift Rate Bias Estimate Variance
    double     AttCovDivTol  = 2.8e-8;   // Attitude Covariance Divergence Tolerance
    double     BiasCovDivTol = 1.0;      // Bias Covariance Divergence Tolerance
    double     AttCovConTol  = 3.6e-4;   // Attitude Covariance Convergence Tolerance
    double     BiasCovConTol = 2.0;      // Bias Covariance Convergence Tolerance
    double     RsSqRatioLimit = 3.0e-4;  // KF Adjusted squared residuals validity limit 
    double     MaxSqrdResTol = 1.0;      // Coarse Residual Tolerance
    double     MeasuredVecVariance = 1.0;      // single-axis measured Vector Variance

    double   SigmaUSquared = 1.0e-5;        // IRU Drift Rate Bias Random Walk Variance
    double   SigmaVSquared = 1.0e-5;        // IRU Angle Measurement Variance
    //#if KF_ATTSTATE_VEC_INPUT == 1
    double         SigmaMSquared;        // 2-Axis Attitude Sensor Measurement Variance
    //#if KF_ATTSTATE_QUAT_INPUT == 1
    Matrix3x3d     MeasurementNoiseCov;  

    KF_AttState_Constants KFAttConst;

    for (i=0; i<=5; i++)
     for (j=0; j<=5; j++)
       KFAttConst.StateTransNoiseCov.Comp[i][j] = (float)(j+1);

    for (i=0; i<=2; i++)
     for (j=0; j<=2; j++)
       KFAttConst.MeasurementNoiseCov.Comp[i][j] = (float)(j+1);


    // initial values KF_AttState_Constants 


    KFAttConst.InitBiasCov = InitBiasCov;
    KFAttConst.AttCovDivTol = AttCovDivTol;
    KFAttConst.BiasCovConTol = BiasCovConTol;
    KFAttConst.AttCovConTol = AttCovConTol;
    KFAttConst.BiasCovConTol = BiasCovConTol;
    KFAttConst.RsSqRatioLimit = RsSqRatioLimit;
    KFAttConst.MaxSqrdResTol = MaxSqrdResTol;
//    KFAttConst.MeasuredVecVariance = MeasuredVecVariance;



    KF_AttState_CurrentState KFAttCState; 

    for (i=0; i<=5; i++)
     for (j=0; j<=5; j++)
       KFAttCState.ErrCov.Comp[i][j] = (float)(j+1);

    for (i=0; i<=5; i++)
     for (j=0; j<=5; j++)
       KFAttCState.StateTransition.Comp[i][j] = (float)(j+1);
 
    const Vector6d  OptimalState = {5.0, 9.0, 3.3, 4.0, 5.0, 6.0};        
    const Vector3d  AdjustedResidual = {1.0, 2.0, 3.0};    
    const Vector3d  BodyRate = {1.0, 2.0, 3.0};    
    const Quaternion PropQuatEciToBcs = {0.25, 0.50, 1.0, 0.3};
    const Quaternion MeasuredQuatEciToBcs = {0.25, 0.50, 1.0, 0.3};

    KFAttCState.OptimalState = OptimalState;
    KFAttCState.AdjustedResidual = AdjustedResidual;

    KF_AttState_Tbl KFAttStateTbl; 
    KFAttStateTbl.SigmaUSquared = 1.0;
    KFAttStateTbl.SigmaVSquared = 1.0;
    KFAttStateTbl.InitBiasCov = InitBiasCov;
    KFAttStateTbl.AttCovDivTol = AttCovDivTol;
    KFAttStateTbl.BiasCovDivTol = BiasCovDivTol;
    KFAttStateTbl.AttCovConTol = AttCovConTol; 
    KFAttStateTbl.BiasCovConTol = BiasCovConTol; 
    KFAttStateTbl.RsSqRatioLimit = RsSqRatioLimit; 
/*     KFAttStateTbl.SigmaMSquared = 1.0; */

    KF_AttState_Init(&KFAttConst, &KFAttCState, &KFAttStateTbl, 1);


    for (i=0; i<=5; i++)
       { 
       for (j=0; j<=5; j++)
       printf("%f ",  KFAttCState.ErrCov.Comp[i][j]);
       //printf("%f ",  KFAttCState.StateTransition.Comp[i][j]);
       printf("\n");
       }

    
       printf("\n");
    
     for (j=0; j<=5; j++)
       printf("%f ",  KFAttCState.OptimalState.Comp[j]);

    KF_AttState_InitConstants( &KFAttConst, &KFAttStateTbl, 1);

    for (i=0; i<=5; i++)
      {
      for (j=0; j<=5; j++)
       printf("%f ",  KFAttConst.StateTransNoiseCov.Comp[i][j]);
       printf("\n");
       }

    KF_AttState_AttSensorProc3( &KFAttCState.AdjustedResidual, &PropQuatEciToBcs, &MeasuredQuatEciToBcs);
    KF_AttState_PropagateStateErrCov( &KFAttCState , &KFAttConst, &BodyRate, 1.0);
    KF_AttState_KalmanFilter3( &KFAttCState, &KFAttCState.AdjustedResidual, &KFAttConst);

}
