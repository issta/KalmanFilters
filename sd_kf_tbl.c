/**
** \file
**
** Implementation of State Determination Kalman Filter Object Parameter Table
**
** $Id: sd_kf_tbl.c 1.8 2012/01/23 16:00:55EST sstrege Exp  $
**
** References:
**      -# GPM GNC-FSW Algorithms Document
**
** $Date: 2012/01/23 16:00:55EST $
**
** $Revision: 1.8 $
**
** $Log: sd_kf_tbl.c  $
** Revision 1.8 2012/01/23 16:00:55EST sstrege 
** Added __attribute statement to Table File Header definition to fix compiler warning
** Revision 1.7 2011/07/08 16:44:09EDT dkobe 
** Removed unnecessary table parameter.
** Revision 1.6 2010/12/15 15:39:48EST dkobe 
** Corrected order of Initial Attitude and Bias covariances in table
** Revision 1.5 2010/11/04 16:20:25EDT dkobe 
** Modified to  match algorithm more closely
** Revision 1.4 2010/07/29 12:17:00EDT dkobe 
** Updates following Kalman Filter Code Walkthrough
** Revision 1.3 2010/07/16 14:06:42EDT dkobe 
** Updated to match algorithm and requirements for Build 3
** Revision 1.2 2009/10/09 10:45:15EDT dkobe 
** Corrected table names, added telemetry generation for SD to CL
** Revision 1.1 2009/10/06 14:29:12EDT dkobe 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GPM-REPOSITORY/apps/sd/fsw/src/sd_kf/project.pj
**
*/

/*
** Includes
*/

#include "sd_kf_tbldef.h"
#include "sd_kf_db.h"

//#include "cfe_tbl_filedef.h"
//
///*
//** Table file header
//*/
//
//static CFE_TBL_FileDef_t CFE_TBL_FileDef __attribute__((__used__)) =
//{
//    "SD_KF_TableDef", "SD.SD_KF_Table", "SD_KF DefParamTbl",
//    "sd_kf_tbl.tbl", sizeof(SD_KF_Table_t)
//};

SD_KF_Table_t SD_KF_TableDef = 
{
    {
        SD_KF_SIGMA_U_SQUARED,
        SD_KF_SIGMA_V_SQUARED,
        SD_KF_INIT_BIAS_COVARIANCE,
        SD_KF_ATT_COV_DIVERGENCE_LIMIT,
        SD_KF_BIAS_COV_DIVERGENCE_LIMIT,
        SD_KF_ATT_COV_CONVERGENCE_LIMIT,
        SD_KF_BIAS_COV_CONVERGENCE_LIMIT,
        SD_KF_ADJ_SQUARED_RES_LIMIT,
        SD_KF_MAX_SQUARED_RES_LIMIT,
        {
            {
                {
                    SD_KF_MEAS_NOISE_COV_11,
                    SD_KF_MEAS_NOISE_COV_12,
                    SD_KF_MEAS_NOISE_COV_13
                },
                {
                    SD_KF_MEAS_NOISE_COV_21,
                    SD_KF_MEAS_NOISE_COV_22,
                    SD_KF_MEAS_NOISE_COV_23
                },
                
                {
                    SD_KF_MEAS_NOISE_COV_31,
                    SD_KF_MEAS_NOISE_COV_32,
                    SD_KF_MEAS_NOISE_COV_33
                }
            }
        }
    },
    
    SD_KF_MAX_RATE_CORRECTION,
    
    SD_KF_CYCLE_PERIOD

}; /* end of SD_KF_TableDef */

/* end of file */
