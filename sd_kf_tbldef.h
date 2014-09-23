/**
** \file
**
** Definition of the State Determination Kalman Filter Object Table
**
** $Id: sd_kf_tbldef.h 1.6 2012/07/10 09:19:18EDT dkobe Exp  $
**
** References:
**
** $Date: 2012/07/10 09:19:18EDT $
**
** $Revision: 1.6 $
**
** $Log: sd_kf_tbldef.h  $
** Revision 1.6 2012/07/10 09:19:18EDT dkobe 
** SD User's Guide updates
** Revision 1.6 2012/06/29 16:14:05EDT dkobe 
** Updates to SD User's Guide
** Revision 1.5 2011/03/25 14:56:37EDT dkobe 
** Baseline table validation code (post code walkthrough)
** Revision 1.4 2011/02/04 14:57:37EST dkobe 
** Updates to comments to update User's Guide for Build 3.1
** Revision 1.3 2010/11/04 16:20:23EDT dkobe 
** Modified to  match algorithm more closely
** Revision 1.2 2010/07/16 14:06:37EDT dkobe 
** Updated to match algorithm and requirements for Build 3
** Revision 1.1 2009/10/06 14:29:13EDT dkobe 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GPM-REPOSITORY/apps/sd/fsw/src/sd_kf/project.pj
**
*/

#ifndef _sd_kf_tbldef_
#define _sd_kf_tbldef_

#define KF_ATTSTATE_QUAT_INPUT  (1)

#include "common_types.h"
#include "kf_attstate_tbl.h"

/*
** Type Definitions
*/

/**
** @addtogroup SD_TBL State Determination Table Specifications
** @{
*/

/**
** @addtogroup SD_TBL_KF State Determination Kalman Filter Table Specification
** @{
*/
/** 
**  \sdtbl KF Processing Table structure
**
**  \par Description
**      Contains Kalman Filter Parameters used for Processing Attitude Data
**
**  \par Table Usage
**       This table is used to update Kalman Filter initial parameters and
**       also for specifying limits used to flag the current state of the
**       filter in telemetry.
** 
**       \b NOTE: The contents of this table does \b NOT become active during
**       Kalman Filter processing until a \link #SD_KF_ENABLE_CC KF Enable Command \endlink
**       has been sent and processed.
**
**  \par Table Validation
**      \arg \c AttState.SigmaUSquared       - must be greater than #SD_KF_SIGMAUSQUARED_TBL_MIN
**      \arg \c AttState.SigmaVSquared       - must be greater than #SD_KF_SIGMAVSQUARED_TBL_MIN
**      \arg \c AttState.InitBiasCov         - must be greater than #SD_KF_INITBIASCOV_TBL_MIN
**      \arg \c AttState.AttCovDivTol        - must be greater than #SD_KF_ATTCOVDIVTOL_TBL_MIN
**      \arg \c AttState.BiasCovDivTol       - must be greater than #SD_KF_BIASCOVDIVTOL_TBL_MIN
**      \arg \c AttState.AttCovConTol        - must be greater than #SD_KF_ATTCOVCONTOL_TBL_MIN
**      \arg \c AttState.BiasCovConTol       - must be greater than #SD_KF_BIASCOVCONTOL_TBL_MIN
**      \arg \c AttState.RsSqRatioLimit      - must be greater than #SD_KF_RSSQRATIOLIMIT_TBL_MIN
**      \arg \c AttState.MaxSqrdResTol       - must be greater than #SD_KF_MAXSQRDRESTOL_TBL_MIN
**      \arg \c AttState.MeasurementNoiseCov - trace of matrix must be greater than #MTH_DOUBLE_ZERO_TOLERANCE and 
**                                             matrix must be symmetric to within #SD_KF_MEASUREMENTNOISECOV_TBL_SYM_TOL
**      \arg \c MaxRateCorChange             - must be greater than #SD_KF_MAXRATECORCHANGE_TBL_MIN
**      \arg \c ProcessingCyclePeriod        - must be greater than zero
**
**  \par Field Definitions
**      \arg \c AttState              - Set of Parameters used to initialize the Kalman Filter.
**                                      see #KF_AttState_Tbl for details on each field.
**      \arg \c MaxRateCorChange      - Maximum allowed drift rate correction allowed in a single update.
**      \arg \c ProcessingCyclePeriod - Number of ACS cycles, inclusive, between Kalman Filter executions (1=every cycle).
*/
typedef struct
{
    KF_AttState_Tbl     AttState;
    double              MaxRateCorChange;
    uint32              ProcessingCyclePeriod;
} SD_KF_Table_t;


/** @}*/
/** @}*/

#endif /* end of _sd_kf_tbldef_ */

/* end of file */
