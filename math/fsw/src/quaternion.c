/* 
** File:
**   $Id: quaternion.c 1.2 2008/09/10 10:02:06EDT gwelter Exp  $
**
** Purpose: Quaternion math library function implementation.
**
** $Log: quaternion.c  $
** Revision 1.2 2008/09/10 10:02:06EDT gwelter 
** pre-gpm updates
** Revision 1.1  2004/01/23 13:22:59  daviddawsonuser
** directory reorg
** Revision 1.1  2004/10/04 18:23:35  ddawson
** imported mathlib to SDO repository
** Revision 1.1  2004/12/16 13:16:34EST rperera 
** Member added to project /home/mksdata/MKS-SDO-REPOSITORY/fsw/shared/math/project.pj
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/src/project.pj
** Revision 1.1.1.1  2003/12/08 20:00:06  daviddawsonuser
** Imported Sources
** Revision 1.1.4.2.2.1  2005/11/17 03:41:20  ddawson
** Imported SDO changes into main trunk and then merged with doxygen branch
**
** Revision 1.2  2003/12/10 20:40:07  daviddawsonuser
** clean up based on Bob's review
** Revision 1.2  2005(?)/11/16 20:44:14  ddawson
** update from SDO, propagate quaternion function modified to not perform
** propagation if input vector magnitude is below MATH_ZERO_DOUBLE_TOLERANCE
** rather than 0.0
** Revision 1.2  2005/11/29 07:49:32EST dcmccomas 
** New SDO delivery and added doxygen markup
**
** Revision 1.3  2003/12/12 16:48:08  daviddawsonuser
** alphabetized functions
** Revision 1.3 2007/06/11 11:28:40EDT dcmccomas 
** Quaternion_Angle() - Add check for Comp[3] > 1.0 and return 0.0
** Quaternon_Delta() - Ensure posiitve Comp[3] delta
**
** Revision 1.4  2003/12/12 21:06:46  daviddawsonuser
** *** empty log message ***
** Revision 1.5  2004/01/20 19:13:29  daviddawsonuser
** Made changes based on code review
** Revision 1.6  2004/01/20 19:20:30  daviddawsonuser
** miscellaneous cleanup
**
** Revision 2.0 2008/05/21 15:00:28EDT dcmccomas 
** Revision for new MKS
** Revision 2.0  2008/06/23 07:10:00 gary welter
** modified Quaternion_Normalize() - one division rather than four
** added    Quaternion_Standardize()
**          Quaternion_ToToEulerEigenVectorAngle()
**          Quaternion_FromToEulerEigenVectorAngle()
**          Quaternion_ToEulerVector()
**          Quaternion_FromEulerVector()
**
**
**
*/

/*
** Include Files
*/

#include <math.h>
#include "quaternion.h"
#include "mathconstants.h"


/*
** Exported Functions
*/


/****************************************************************************
** 
** Function: Quaternion_Angle
**
*/
double Quaternion_Angle (const Quaternion *InputQuat)
{

  double Angle;    /* angle in radians */

  /* 
  ** Guard against the scalar component being slightly bigger than 1. If it
  ** is greater than 1 assume that it is 1 which would make the resulting 
  ** angle equal to 0
  */
  if (fabs(InputQuat->Comp[3]) < 1.0)
  {
     Angle = 2 * acos(InputQuat->Comp[3]);       /* calculate angle which will be between
                                                    0 and 2PI */
  }
  else
  {
     Angle = 0.0;
  }

  if (Angle > MTH_PI)                    /* return angle between -PI and + PI */
  {
    Angle -= MTH_TWO_PI;
  }

  return Angle;

}  /* End Quaternion_Angle */



/****************************************************************************
** 
** Function: Quaternion_Conjugate
**
*/
void Quaternion_Conjugate (Quaternion       *ConjugateQuat, 
                           const Quaternion *InputQuat)
{
  ConjugateQuat->Comp[0] = -InputQuat->Comp[0];
  ConjugateQuat->Comp[1] = -InputQuat->Comp[1];
  ConjugateQuat->Comp[2] = -InputQuat->Comp[2];
  ConjugateQuat->Comp[3] =  InputQuat->Comp[3];

}  /* End of Quaternion_Conjugate */


/****************************************************************************
** 
** Function: Quaternion_Copy
**
*/
void Quaternion_Copy (Quaternion       *DuplicateQuat,
                      const Quaternion *InputQuat)
{
  *DuplicateQuat = *InputQuat;
}


/****************************************************************************
** 
** Function: Quaternion_Delta
**
*/
void Quaternion_Delta (Quaternion       *QuatBC,
                       const Quaternion *QuatAB,
                       const Quaternion *QuatAC)
{
  Quaternion QuatBA;

  Quaternion_Conjugate(&QuatBA, QuatAB);

  Quaternion_Mult(QuatBC, &QuatBA, QuatAC);

  /* Ensure that scalar component is positive */
  Quaternion_MakeScalarPositive(QuatBC, QuatBC);

}  /* Quaternion_DeltaQuat */

/****************************************************************************
** 
** Function: Quaternion_FromEulerEigenVectorAngle
**
*/
void Quaternion_FromEulerEigenVectorAngle( Quaternion   *Q,
                                           const double  Eva[4]  ) 
{

    int i;
    double HalfAngle;
    double SinHalfAngle;

    HalfAngle = 0.5 * Eva[3];

    SinHalfAngle = sin( HalfAngle );

    for ( i=0; i<3; i++ ) {
      Q->Comp[i] = Eva[i] * SinHalfAngle;
    }

    Q->Comp[3] = cos( HalfAngle );

}  /* End Quaternion_FromEulerEigenVectorAngle() */


/****************************************************************************
** 
** Function: Quaternion_FromEulerVector
**
*/
void Quaternion_FromEulerVector( Quaternion     *Q,
                                 const Vector3d *V  ) 
{

    int i;
    double Eva[4], VNormInverse;

    Eva[3] = sqrt( V->Comp[0] * V->Comp[0]  + 
		           V->Comp[1] * V->Comp[1]  + 
				   V->Comp[2] * V->Comp[2]    );

    if ( Eva[3] > 0.0 ) {

      VNormInverse = 1.0 / Eva[3];

      for ( i=0; i<3; i++ ) {
        Eva[i] = V->Comp[i] * VNormInverse;
      }

      Quaternion_FromEulerEigenVectorAngle( Q, Eva );

    } else {

      for ( i=0; i<3; i++ ) {
        Q->Comp[i] = 0.0;
      }
      Q->Comp[3] = 1.0;

    }

}  /* End Quaternion_FromEulerVector() */


/****************************************************************************
** 
** Function: Quaternion_FromMatrix
**
*/
void Quaternion_FromMatrix (Quaternion       *Result, 
                            const Matrix3x3d *Operand) 
{

   double       Trace;            /* Sum of Matrix diagonals                    */
   double       MaxDiag;          /* Value of maximal diagonal matrix element   */
   unsigned int FirstQuatElement; /* index of the 1st quat element calculated   */
   short        DiagIndex;        /* Diagonal index for looping through matrix  */
   double       Divisor;          /* 4 times the 1st calculated quaternion 
				     element.  Will be used to calculate the 
				     other three quat elements.                 */

   /* Algorithm taken from Spacecraft Attitude Determination and Control
      by James Wertz, page 415, Eqns. 12-14.  This function will use one
      of four mathematically equivalent methods to calculate the quaternion.
      It chooses the correct method for each input matrix by choosing the
      method that  maximizes the value of Divisor (see below).  If Divisor were 
      allowed to be close to zero,  numerical errors stemming from dividing
      by it  would degrade the accuracy of the resulting quaternion.        */

   /*
   ** Step 1 - Compute Trace of A and find the maximal diagonal value and the
   **          column/row that it is in.  This will tell us which quaternion
   **          element to calculate first.  
   */

   Trace = 0.0;
   MaxDiag = Operand->Comp[0][0];            /* Init Trace to 1st diagonal element */
   FirstQuatElement = 0;                     /* Init to zero                       */

   for (DiagIndex=0; DiagIndex < 3; DiagIndex++) 
   {
     Trace += Operand->Comp[DiagIndex][DiagIndex];
     if (Operand->Comp[DiagIndex][DiagIndex] > MaxDiag ) 
     {
       MaxDiag = Operand->Comp[DiagIndex][DiagIndex];
       FirstQuatElement = DiagIndex;
     }
   } /* End diagonal loop */

   if ( Trace > MaxDiag ) 
   {
     /* Calculate the 4th (scalar) quat element first */
     FirstQuatElement = 3;
   }

   /*
   ** Step 2 - Compute the 1st quaternion element.  We could start with
   **          any of the elements but we want to compute the biggest one
   **          first.  This will reduce numerical inaccuracy by ensuring
   **          that the divisor in the ensuing divisions is not close
   **          to zero.
   */
   if (FirstQuatElement < 3)
   {
     Result->Comp[FirstQuatElement] = sqrt( ((2.0 * MaxDiag + 1.0 - Trace) / 4.0) );
   }
   else
   {
     /* equation for cacluating 4th element is slightly different */
     Result->Comp[FirstQuatElement] = sqrt( ((Trace + 1.0) / 4.0) );
   }
   Divisor = ( 4.0 * Result->Comp[FirstQuatElement]);


   /*
   ** Step 3 - Calculate other three elements based on the first element
   **          that has just been calculated.
   */
   switch ( FirstQuatElement ) 
   {
     
   case 0:
     /* Calculate based on 1st quat element */
     Result->Comp[1] = (Operand->Comp[0][1] + Operand->Comp[1][0]) / Divisor;
     Result->Comp[2] = (Operand->Comp[0][2] + Operand->Comp[2][0]) / Divisor;
     Result->Comp[3] = (Operand->Comp[1][2] - Operand->Comp[2][1]) / Divisor;
     break;

   case 1:
     /* Calculate based on 2nd quat element */
     Result->Comp[0] = (Operand->Comp[0][1] + Operand->Comp[1][0]) / Divisor;
     Result->Comp[2] = (Operand->Comp[2][1] + Operand->Comp[1][2]) / Divisor;
     Result->Comp[3] = (Operand->Comp[2][0] - Operand->Comp[0][2]) / Divisor;
     break ;

   case 2:
     /* Calculate based on 3rd quat element */
     Result->Comp[0] = (Operand->Comp[0][2] + Operand->Comp[2][0]) / Divisor;
     Result->Comp[1] = (Operand->Comp[2][1] + Operand->Comp[1][2]) / Divisor;
     Result->Comp[3] = (Operand->Comp[0][1] - Operand->Comp[1][0]) / Divisor;
     break ;

   case 3:
     /* Calculate based on 4th quat element */
     Result->Comp[0] = (Operand->Comp[1][2] - Operand->Comp[2][1]) / Divisor;
     Result->Comp[1] = (Operand->Comp[2][0] - Operand->Comp[0][2]) / Divisor;
     Result->Comp[2] = (Operand->Comp[0][1] - Operand->Comp[1][0]) / Divisor;
     break ;
      
   }  /* End switch on FirstQuatElement */
   
} /* End  Quaternion_FromMatrix */



/****************************************************************************
** 
** Function: Quaternion_Identity
**
*/
void Quaternion_Identity (Quaternion *IdentityQuat)
{
  IdentityQuat->Comp[0] = 0.0;
  IdentityQuat->Comp[1] = 0.0;
  IdentityQuat->Comp[2] = 0.0;
  IdentityQuat->Comp[3] = 1.0;
}

/****************************************************************************
** 
** Function: Quaternion_IsNormalized
**
*/
int Quaternion_IsNormalized (const Quaternion *InputQuat,
						     const double      NormalizationLimit )
{
  double Magnitude;
  double Error;
  int    NormalizationCheck;

  Magnitude = Quaternion_Magnitude(InputQuat);

  Error = fabs( Magnitude - 1.0 );

  if ( Error < NormalizationLimit ) NormalizationCheck = 1;
  else                              NormalizationCheck = 0;

  return NormalizationCheck;
}


/****************************************************************************
** 
** Function: Quaternion_Magnitude
**
*/
double Quaternion_Magnitude (const Quaternion *InputQuat)
{
  double Magnitude;
  
  Magnitude = sqrt(InputQuat->Comp[0] * InputQuat->Comp[0] +
                   InputQuat->Comp[1] * InputQuat->Comp[1] +
                   InputQuat->Comp[2] * InputQuat->Comp[2] +
                   InputQuat->Comp[3] * InputQuat->Comp[3]);

  return (Magnitude);
}



/****************************************************************************
** 
** Function: Quaternion_MakeScalarPositive
**
*/
void Quaternion_MakeScalarPositive (Quaternion       *OutputQuat,
                                    const Quaternion *InputQuat)
{
  if (InputQuat->Comp[3] < 0.0)
  {
    OutputQuat->Comp[0] = -InputQuat->Comp[0];
    OutputQuat->Comp[1] = -InputQuat->Comp[1];
    OutputQuat->Comp[2] = -InputQuat->Comp[2];
    OutputQuat->Comp[3] = -InputQuat->Comp[3];
  }
  else
  {
    *OutputQuat = *InputQuat;
  }

}  /* End Quaternion_MakeScalarPositive */



/****************************************************************************
** 
** Function: Quaternion_Mult
**
*/
void Quaternion_Mult (Quaternion       *QuatXZ, 
		      const Quaternion *QuatXY, 
		      const Quaternion *QuatYZ) 
{
  Quaternion TempQuatXZ;

  /* Taken from Wertz, Page 416, Eqn. 12-15b */
  TempQuatXZ.Comp[0] =   (QuatXY->Comp[0] * QuatYZ->Comp[3])
                       + (QuatXY->Comp[1] * QuatYZ->Comp[2])
                       - (QuatXY->Comp[2] * QuatYZ->Comp[1])
                       + (QuatXY->Comp[3] * QuatYZ->Comp[0]);
   
  TempQuatXZ.Comp[1] = - (QuatXY->Comp[0] * QuatYZ->Comp[2]) 
                       + (QuatXY->Comp[1] * QuatYZ->Comp[3]) 
                       + (QuatXY->Comp[2] * QuatYZ->Comp[0]) 
                       + (QuatXY->Comp[3] * QuatYZ->Comp[1]);

  TempQuatXZ.Comp[2] =   (QuatXY->Comp[0] * QuatYZ->Comp[1])
                       - (QuatXY->Comp[1] * QuatYZ->Comp[0])
                       + (QuatXY->Comp[2] * QuatYZ->Comp[3])
                       + (QuatXY->Comp[3] * QuatYZ->Comp[2]);

  TempQuatXZ.Comp[3] = - (QuatXY->Comp[0] * QuatYZ->Comp[0])
                       - (QuatXY->Comp[1] * QuatYZ->Comp[1])
                       - (QuatXY->Comp[2] * QuatYZ->Comp[2])
                       + (QuatXY->Comp[3] * QuatYZ->Comp[3]);

   Quaternion_Normalize(QuatXZ, &TempQuatXZ);

} /* End Quaternion_Mult */



/****************************************************************************
** 
** Function: Quaternion_Normalize
**
*/
void Quaternion_Normalize (Quaternion       *NormalizedQuat, 
                           const Quaternion *InputQuat)
{
  double Magnitude;
  double MagnitudeInverse;

  Magnitude = Quaternion_Magnitude(InputQuat);

  MagnitudeInverse = 1.0 / Magnitude;

  NormalizedQuat->Comp[0] = InputQuat->Comp[0] * MagnitudeInverse;
  NormalizedQuat->Comp[1] = InputQuat->Comp[1] * MagnitudeInverse;
  NormalizedQuat->Comp[2] = InputQuat->Comp[2] * MagnitudeInverse;
  NormalizedQuat->Comp[3] = InputQuat->Comp[3] * MagnitudeInverse;
}



/****************************************************************************
** 
** Function: Quaternion_Propagate
**
*/
void Quaternion_Propagate (Quaternion       *PropagatedQuat,
                           const Quaternion *InputQuat,
                           const Vector3d   *RotationVec)
{
  Quaternion DeltaQuat;
  double     AngleMag;
  double     Temp;
  Vector3d   AxisUnitVec;

  /* Construct delta quaternion */
  AngleMag = Vector3d_Magnitude(RotationVec);

  if (AngleMag < MTH_DOUBLE_ZERO_TOLERANCE) 
  {
    *PropagatedQuat = *InputQuat;            /* RotationVec is {0,0,0} so simply return
						InputQuat                               */
  }
  else
  {                                        
    Vector3d_Normalize(&AxisUnitVec, RotationVec);
      
    Temp = sin(0.5 * AngleMag);
    
    DeltaQuat.Comp[0] = AxisUnitVec.Comp[0] * Temp;
    DeltaQuat.Comp[1] = AxisUnitVec.Comp[1] * Temp;
    DeltaQuat.Comp[2] = AxisUnitVec.Comp[2] * Temp;
    DeltaQuat.Comp[3] = cos(0.5 * AngleMag);
    
    /* Rotate input quaternion by delta quaternion */
    Quaternion_Mult(PropagatedQuat, InputQuat, &DeltaQuat);
  }
  
}  /* Quaternion_Propagate */

/****************************************************************************
** 
** Function: Quaternion_Standardize
**
*/
int Quaternion_Standardize (Quaternion       *StandardizedQuat, 
                            const Quaternion *InputQuat,
							const double      NormalizationLimit )
{

  int i;

  int NormalizationCheck;

  double Dif, Magnitude, MagnitudeInverse;

  Magnitude = Quaternion_Magnitude(InputQuat);

  Dif = fabs( 1.0 - Magnitude );

  if ( Dif <= NormalizationLimit ) {

    NormalizationCheck = 1;

    Quaternion_MakeScalarPositive( StandardizedQuat, InputQuat );

    MagnitudeInverse = 1.0 / Magnitude;
    for ( i=0; i<4; i++ ) {
      StandardizedQuat->Comp[i] *= MagnitudeInverse;
    } 

  } else {

    NormalizationCheck = 0;

    Quaternion_Identity( StandardizedQuat );

  }

  return NormalizationCheck;

}

/****************************************************************************
** 
** Function: Quaternion_ToEulerEigenVectorAngle
**
*/
void Quaternion_ToEulerEigenVectorAngle( double             Eva[4],
                                         const Quaternion  *Q       ) 
{

    int i;
    double SinHalfAngle;
    double SinHalfAngleInverse;

    SinHalfAngle = sqrt( Q->Comp[0] * Q->Comp[0] + 
                         Q->Comp[1] * Q->Comp[1] + 
                         Q->Comp[2] * Q->Comp[2]   );

    if ( SinHalfAngle > 0.0 ) {

      SinHalfAngleInverse = 1.0 / SinHalfAngle;

      for ( i=0; i<3; i++ ) Eva[i] = Q->Comp[i] * SinHalfAngleInverse;

      if ( Q->Comp[3] < 0.0 ) {
        for ( i=0; i<3; i++ ) Eva[i] = - Eva[i];
      } 

      if ( SinHalfAngle < 1.0 ) Eva[3] = 2.0 * asin( SinHalfAngle );
      else                      Eva[3] = MTH_PI;

    } else {

      for ( i=0; i<4; i++ ) Eva[i] = 0.0;

    }

}  /* End Quaternion_ToEulerEigenVectorAngle() */

/****************************************************************************
** 
** Function: Quaternion_ToEulerVector
**
*/
void Quaternion_ToEulerVector( Vector3d         *V,
                               const Quaternion *Q      ) 
{

    int    i;
    double Eva[4];

    Quaternion_ToEulerEigenVectorAngle( Eva, Q );

    for ( i=0; i<3; i++ ) {
      V->Comp[i] = Eva[i] * Eva[3];
    }

}  /* End Quaternion_ToEulerVector() */

/****************************************************************************
** 
** Function: Quaternion_ToMatrix
**
*/
void Quaternion_ToMatrix (Matrix3x3d       *Result, 
			  const Quaternion *Operand) 
{

   double Temp0;
   double Temp1;
   double Temp2;
   double Temp3;
   double Temp4;
   double Temp5;

   /* Algorithm taken from Wertz, Eqn. (12-13a).  Temp variables were used
      improve efficiency */

   /*
   ** Diagonal elements
   */
   Temp0 = Operand->Comp[0] * Operand->Comp[0];
   Temp1 = Operand->Comp[1] * Operand->Comp[1];
   Temp2 = Operand->Comp[2] * Operand->Comp[2];
   Temp3 = Operand->Comp[3] * Operand->Comp[3];

   Result->Comp[0][0] =  Temp0 - Temp1 - Temp2 + Temp3;
   Result->Comp[1][1] = -Temp0 + Temp1 - Temp2 + Temp3;
   Result->Comp[2][2] = -Temp0 - Temp1 + Temp2 + Temp3;

   /*
   ** Off-diagonal elements
   */
   Temp0 = Operand->Comp[0] * Operand->Comp[1];
   Temp1 = Operand->Comp[1] * Operand->Comp[2];
   Temp2 = Operand->Comp[0] * Operand->Comp[2];
   Temp3 = Operand->Comp[0] * Operand->Comp[3];
   Temp4 = Operand->Comp[1] * Operand->Comp[3];
   Temp5 = Operand->Comp[2] * Operand->Comp[3];

   Result->Comp[1][0] = 2.0 * (Temp0 - Temp5);
   Result->Comp[2][0] = 2.0 * (Temp2 + Temp4);
   Result->Comp[0][1] = 2.0 * (Temp0 + Temp5);
   Result->Comp[2][1] = 2.0 * (Temp1 - Temp3);
   Result->Comp[0][2] = 2.0 * (Temp2 - Temp4);
   Result->Comp[1][2] = 2.0 * (Temp1 + Temp3);

} /* End Quaternion_ToMatrix */

/****************************************************************************
** 
** Function: Quaternion_VectorTransform
**
*/
void Quaternion_VectorTransform (Vector3d         *Result, 
                                 const Quaternion *QuatOp,
                                 const Vector3d   *VecOp   )
{

   unsigned int  i;
   double        Qangle;
   Vector3d      Cross1;
   Vector3d      Cross2;
   Vector3d      Qvec;

   Qvec.Comp[0] = QuatOp->Comp[0];
   Qvec.Comp[1] = QuatOp->Comp[1];
   Qvec.Comp[2] = QuatOp->Comp[2];
   Qangle = QuatOp->Comp[3];

   Vector3d_Cross (&Cross1, &Qvec, VecOp);
   Vector3d_Cross (&Cross2, &Qvec, &Cross1);

   for (i = 0; i < 3; i++)
   {   
     Result->Comp[i] = VecOp->Comp[i] +
                       2.0 * (Cross2.Comp[i] - Qangle * Cross1.Comp[i]);
   }

}  /* End VectorTransform() */




/* end of file */
