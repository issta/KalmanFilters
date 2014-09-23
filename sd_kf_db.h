/**
** @File:
**
** Purpose:
**   Specification for State Determination Kalman Filter Parameter Table
**   from the GPM GNC Parameter database
**
** Notes:
**   - This file is automatically generated from the GPM database
**
** $Id: sd_kf_db.h 1.15 2013/08/29 13:44:12EDT jwu1 Exp  $
** $Date: 2013/08/29 13:44:12EDT $
** $Revision: 1.15 $
**
** $Log: sd_kf_db.h  $
** Revision 1.15 2013/08/29 13:44:12EDT jwu1 
** Regenerated from table script with the updated table values.
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GPM-REPOSITORY/apps/sd/fsw/src/sd_kf/project.pj
**
*/

#ifndef _sd_kf_db_
#define _sd_kf_db_

/*
** Macro Definitions
*/
#define SD_KF_TABLE_NAME    "SD_KF_Table"

#define SD_KF_SIGMA_U_SQUARED            (  6.000000000000000e-19)    /**< GPM.ACS.KF.IruSigmaU2_a */
#define SD_KF_SIGMA_V_SQUARED            (  4.000000000000000e-21)    /**< GPM.ACS.KF.IruSigmaV2_a */
#define SD_KF_INIT_ATT_COVARIANCE        (  1.000000000000000e+00)    /**< GPM.ACS.KF.InitAttVar_rad2 */
#define SD_KF_INIT_BIAS_COVARIANCE       (  1.000000000000000e-08)    /**< GPM.ACS.KF.InitBiasVar_rps2 */
#define SD_KF_ATT_COV_DIVERGENCE_LIMIT   (  8.376979044134487e-05)    /**< GPM.ACS.KF.AttCovDivLimit_a */
#define SD_KF_BIAS_COV_DIVERGENCE_LIMIT  (  1.100000000000000e-08)    /**< GPM.ACS.KF.BiasCovDivLimit_a */
#define SD_KF_ATT_COV_CONVERGENCE_LIMIT  (  8.499999999999999e-08)    /**< GPM.ACS.KF.AttCovCnvLimit_a */
#define SD_KF_BIAS_COV_CONVERGENCE_LIMIT (  1.700000000000000e-10)    /**< GPM.ACS.KF.BiasCovCnvLimit_a */
#define SD_KF_ADJ_SQUARED_RES_LIMIT      (  3.600000000000000e+01)    /**< GPM.ACS.KF.RsSqRatioLimit_a */
#define SD_KF_MAX_SQUARED_RES_LIMIT      (  3.046174197867086e-04)    /**< GPM.ACS.KF.MaxSquaredResidual_a */
#define SD_KF_MEAS_NOISE_COV_11          (  2.400000000000000e-08)    /**< GPM.ACS.KF.StCov_M3x3_B(1,1) */
#define SD_KF_MEAS_NOISE_COV_12          (  0.000000000000000e+00)    /**< GPM.ACS.KF.StCov_M3x3_B(1,2) */
#define SD_KF_MEAS_NOISE_COV_13          (  0.000000000000000e+00)    /**< GPM.ACS.KF.StCov_M3x3_B(1,3) */
#define SD_KF_MEAS_NOISE_COV_21          (  0.000000000000000e+00)    /**< GPM.ACS.KF.StCov_M3x3_B(2,1) */
#define SD_KF_MEAS_NOISE_COV_22          (  1.200000000000000e-07)    /**< GPM.ACS.KF.StCov_M3x3_B(2,2) */
#define SD_KF_MEAS_NOISE_COV_23          ( -9.400000000000000e-08)    /**< GPM.ACS.KF.StCov_M3x3_B(2,3) */
#define SD_KF_MEAS_NOISE_COV_31          (  0.000000000000000e+00)    /**< GPM.ACS.KF.StCov_M3x3_B(3,1) */
#define SD_KF_MEAS_NOISE_COV_32          ( -9.400000000000000e-08)    /**< GPM.ACS.KF.StCov_M3x3_B(3,2) */
#define SD_KF_MEAS_NOISE_COV_33          (  1.200000000000000e-07)    /**< GPM.ACS.KF.StCov_M3x3_B(3,3) */

#define SD_KF_MAX_RATE_CORRECTION        (  2.424000000000000e-06)    /**< GPM.ACS.KF.MaxRateCorChange_a */
#define SD_KF_CYCLE_PERIOD               (1)                          /**< GPM.ACS.KF.NumAcsPeriods_c  [cycles] */

#endif /* end of _sd_kf_db_ */

/* end of file */
