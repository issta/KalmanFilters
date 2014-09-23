/*
** $Id: kf_attstate_cfg.h 1.2 2010/07/29 12:23:08EDT dkobe Exp  $
**
** Purpose:  Attitude State Kalman Filter Configuration
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
**    - This module assumes attitude measurement is in the form of a quaternion
**    - This module assumes the spacecraft body rate is obtained from an IRU
** 
*/

/*************************************************************************
**
** Ensure that "this" header is included only once
**
**************************************************************************/

#ifndef _kf_attstate_cfg_
#define _kf_attstate_cfg_

#include "mathconstants.h"

/** 
**  \brief Enable/Disable the use of the Vector Measurement Input version of the 
**         Kalman Filter Library
**
**  The Kalman Filter Library implements two versions of the library depending upon the
**  type of Attitude Sensor being employed.  To enable the use of the Vector Measurement
**  Input version, this macro should be set to one (1).  To disable its use and remove it
**  from the library during compilation, this macro should be set to zero (0).
**
**  NOTE: There is no restriction on whether both versions are used simultaneously.
**
*/  
#define KF_ATTSTATE_VEC_INPUT               (0)

/** 
**  \brief Enable/Disable the use of the Quaternion Input version of the 
**         Kalman Filter Library
**
**  The Kalman Filter Library implements two versions of the library depending upon the
**  type of Attitude Sensor being employed.  To enable the use of the Quaternion
**  Input version, this macro should be set to one (1).  To disable its use and remove it
**  from the library during compilation, this macro should be set to zero (0).
**
**  NOTE: There is no restriction on whether both versions are used simultaneously.
**
*/  
#define KF_ATTSTATE_QUAT_INPUT              (1)

/**
**   \brief Small Angle Approximation Transition Matrix Threshold
**
**   This angle determines the maximum rotation where a small angle approximation should
**   be used to create the State Transition Matrix in the #KF_AttState_TransitionMatrix
**   function.  Rotation angles less than this can't use the full formula for generating
**   the transition matrix because round-off error would be a problem.
**
**   This threshold would only need to be revisited if the processor's floating point
**   accuracy should improve upon the typical accuracy obtained by a 32-bit processor
**   with 64-bit double precision floating point values. 
**/
#define KF_ATTSTATE_ROTATION_ANGLE_THRESHOLD   (0.0001)


/*************************************************************************/

/* Verify configuration settings are valid */
#if KF_ATTSTATE_VEC_INPUT != 0
  #if KF_ATTSTATE_VEC_INPUT != 1
    #error "Kalman Filter Library Config Err: KF_ATTSTATE_VEC_INPUT must equal either (0) or (1)"
  #endif
#endif

#if KF_ATTSTATE_QUAT_INPUT != 0
  #if KF_ATTSTATE_QUAT_INPUT != 1
    #error "Kalman Filter Library Config Err: KF_ATTSTATE_QUAT_INPUT must equal either (0) or (1)"
  #endif
#endif

#endif /* _kf_attstate_cfg_ */

/************************/
/*  End of File Comment */
/************************/
