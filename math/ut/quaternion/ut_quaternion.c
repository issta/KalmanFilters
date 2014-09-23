/*
** File:
** $Id: ut_quaternion.c 1.2 2008/09/10 10:12:01EDT gwelter Exp  $
**
** Purpose: Unit test the quaternion component of the math library.
**
** References:
**   1. GN&C FSW Math Library Specification
**   2. GN&C FSW Developer Guide
**
** $Date: 2008/09/10 10:12:01EDT $
** $Revision: 1.2 $
** $Log: ut_quaternion.c  $
** Revision 1.2 2008/09/10 10:12:01EDT gwelter 
** pre-gpm updates
** Revision 2.0 2008/05/21 15:00:39EDT dcmccomas 
** Revision for new MKS
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/ut/quaternion/project.pj
** Revision 2.0 2008/06/23 07:00:00EDT gwelter
** Added tests for new quaternion functions
*/

/*
** Include Files
*/


#include "quaternion.h"
#include "mathconstants.h"
#include <stdio.h>
#include <math.h>

#include "utdriver.h"
#include "utdat.h"
#include "utvector3d.h"
#include "utmatrix3x3d.h"
#include "utquaternion.h"

/*
** Macro Definitions
*/

#define UT_FILE_NAME  "ut_quaternion"

/* 
** This is the tolerance that calculated data must match the input test data
** for those functions which use test data from a file. This value can be made
** smaller if higher precision test data is used
*/

#define TEST_DATA_TOLERANCE 1e-7


/******************************************************************************
** Function: TestAngle
**
** Notes:
**  1. Generates a series of unit eigenvectors that span the entire 
**     unit sphere at given intervals (8000 vectors generated). 
**  2. For each eigenvector a delta quaternion is constructed 
**     which represents a rotation of 3.6 degrees. 
**  3. Starts off with a (0,0,0,1) quaternion and successively 
**     multiplies it by the delta quaternion until a full 
**     360 rotation has been accomplished (100 steps). 
**  4. At each step, involving a 3.6 degree, Quaternion_Angle is 
**     called to calculate the angle the quaternion represents. 
**     It should be 3.6 * the step number. 
**  5. This test depends on Quaternion_Mult, which is also tested 
**     by this test suite.  Deficiencies in Quaternion_Mult may 
**     invalidate this test. 
*/
void TestAngle (void)
{		
   boolean TestPassed = TRUE;
   int     Step;	
   double  i, j, k;	
   double  EigenVec[3];	
   double  Magnitude;
   double  AngleStepTotal = 100;	
   double  Theta;	
   double  VectorStep = 0.1;		
   double  Angle;	
   double  AngleDelta;
   Quaternion DeltaQuat;	
   Quaternion Quat;	
   Quaternion DuplicateQuat;		

   for (i = -1.0; i <= 1.0; i+= VectorStep)
   {		
      for (j = -1.0; j <= 1.0; j += VectorStep)
      {			
         for (k = -1.0; k <= 1.0; k += VectorStep)
         {								
            Magnitude = sqrt(i * i + j * j + k * k);
            EigenVec[0] = i / Magnitude;				
            EigenVec[1] = j / Magnitude;				
            EigenVec[2] = k / Magnitude;				
            /* Construct delta quat */					
            Theta = 2 * MTH_PI / AngleStepTotal;	
            DeltaQuat.Comp[0] = EigenVec[0] * sin(Theta / 2);		
            DeltaQuat.Comp[1] = EigenVec[1] * sin(Theta / 2);		
            DeltaQuat.Comp[2] = EigenVec[2] * sin(Theta / 2);		
            DeltaQuat.Comp[3] = cos(Theta / 2);						
            Quat.Comp[0] = 0.0;
            Quat.Comp[1] = 0.0;
            Quat.Comp[2] = 0.0;
            Quat.Comp[3] = 1.0;

            for (Step = 0; Step < 100; Step++)
            {					
               DuplicateQuat = Quat;
               Angle = Quaternion_Angle(&Quat);			
               
               /* Verify that input quaternion was not modified by function */
               if (UtQuaternion_IdenticalComp(&Quat, &DuplicateQuat) == FALSE)
               {						
                  UtLog_VarLine(UtDriver->Log, "TestAngle() Error: Input quaternion was modified for (i,j,k)=(%f,%f,%f) at step %d",i,j,k,Step);
                  TestPassed = FALSE;	
               }										

               /* 
               ** Compute difference between calculated and predicted value and 
               ** make AngleDelta between -pi and pi.
               */
               AngleDelta = fabs(Angle - Theta * Step);
               if (AngleDelta > MTH_PI)
               { 
                  AngleDelta -= (2 * MTH_PI);		
               }

#ifdef DEBUG
               UtLog_VarLine(UtDriver->Log, "TestAngle() Debug: Step %d, Q = (%f,%f,%f,%f)", Step, Quat.Comp[0], Quat.Comp[1], Quat.Comp[2], Quat.Comp[3]);
#endif
					if (AngleDelta > 1e-14)
               {
                  UtLog_VarLine(UtDriver->Log, "TestAngle() Error: Expected versus Computed miscompare by %f for (i,j,k)=(%f,%f,%f) at step %d",AngleDelta, i, j, k, Step);
                  TestPassed = FALSE;
               }
               Quaternion_Mult(&Quat, &Quat, &DeltaQuat);

            }  /* End Step loop */
         }  /* End k loop */
      }  /* End j loop */
   } /* End i loop */


   /*
   ** Special test for check that was added to return exactly 0.0
   ** if 4'th component was >= 1.0
   */
   Angle = 1.0;
   Quat.Comp[0] = 0.0;
   Quat.Comp[1] = 0.0;
   Quat.Comp[2] = 0.0;
   Quat.Comp[3] = 1.001;
   Angle = Quaternion_Angle(&Quat);			
   UTASSERT_EQUAL(Angle,0.0,"Quaternion_Angle","Verify 0.0 when scalar > 1.0");

   UTASSERT_BOOL(TestPassed,"Quaternion_Angle","Complete function");

} /* End TestAngle() */


/******************************************************************************
** Function: TestConjugate
**
** Notes:
**  1. Verifies conjugate calculated correctly by manually calculating 
**     conjugate and comparing to function output. 
**  2. Checks that input parameters aren't modified. 
**  3. Checks that same variable can be uses for output and input 
**
*/
void TestConjugate (void)
{	

   boolean    TestPassed = TRUE;
   int        Step = 0;
   double     StepSize = 0.5;
   double     i, j, k, l;
   Quaternion InputQuat;
   Quaternion DuplicateInputQuat;
   Quaternion ResultQuat;

   for (i = -1; i <= 1; i += StepSize)
   {
      for (j = -1; j <= 1; j += StepSize)
      {
         for (k = -1; k <= 1; k += StepSize)
         {
            for (l = -1; l <= 1; l += StepSize)
            {
               Step++;
               UtQuaternion_GenerateQuat(&InputQuat, i, j, k, l);
               DuplicateInputQuat = InputQuat;  /* save off for later check */
               Quaternion_Conjugate(&ResultQuat, &InputQuat);
               
               /* Check that conjuage is correctly done */
               if (ResultQuat.Comp[0] != -InputQuat.Comp[0] ||	
                   ResultQuat.Comp[1] != -InputQuat.Comp[1] ||	
                   ResultQuat.Comp[2] != -InputQuat.Comp[2] ||	
                   ResultQuat.Comp[3] !=  InputQuat.Comp[3])
               {
                  UtLog_VarLine(UtDriver->Log, "TestConjugate() Error: Expected versus Computed miscompare at step %d InputQuat=(%f,%f,%f,%f)",Step,InputQuat.Comp[0],InputQuat.Comp[1],InputQuat.Comp[2],InputQuat.Comp[3]);
                  TestPassed = FALSE;
               }  

               /* Check that input quaternion was not modified */
               if (UtQuaternion_IdenticalComp(&InputQuat, &DuplicateInputQuat) == FALSE)
               {
                  UtLog_VarLine(UtDriver->Log, "TestConjugate() Error: Input quaternion was modified at step %d",Step);
               }

               /* Check that same variable can be used for input and output */
               Quaternion_Conjugate(&InputQuat, &InputQuat);
               if (UtQuaternion_IdenticalComp(&InputQuat, &ResultQuat) == FALSE)
               {
                  UtLog_VarLine(UtDriver->Log, "TestConjugate() Error: Step %d, couldn't use same variable for input and output",Step);
                  TestPassed = FALSE;		
               }
 
            }  /* End l loop */
         }  /* End k loop */
      }  /* End j loop */
   } /* End i loop */

   UTASSERT_BOOL(TestPassed,"Quaternion_Conjugate","Complete function");

} /* End TestConjugate() */


/******************************************************************************
** Function: TestCopy
**
** Notes:
**  1. Verifies copy done correctly across range of quaternions 
**  2. Checks that input quaternion isn't modified by call. 
**  3. Checks that same variable can be used for output and input 
**
*/
void TestCopy(void)
{		
   
   boolean    TestPassed = TRUE;
   int        Step = 0;
   double     StepSize = 0.25;
   double     i, j, k, l;
   Quaternion InputQuat;
   Quaternion DuplicateInputQuat;
   Quaternion OutputQuat;

   for (i = -1; i <= 1; i += StepSize)
   {
      for (j = -1; j <= 1; j += StepSize)
      {
         for (k = -1; k <= 1; k += StepSize)
         {
            for (l = -1; l <= 1; l += StepSize)
            {
               Step++;					
               
               if (i != 0 && j != 0 && k != 0 && l != 0)
               {						
                  UtQuaternion_GenerateQuat(&InputQuat, i, j, k, l);
                  DuplicateInputQuat = InputQuat;  /* save off for later check */
                  Quaternion_Copy(&OutputQuat, &InputQuat);	
                  
                  if (UtQuaternion_IdenticalComp(&OutputQuat, &InputQuat) == FALSE)
                  {
                     UtLog_VarLine(UtDriver->Log, "TestCopy() Error: Step %d, Input not equal to output",Step);
                     TestPassed = FALSE;
                  }			
                  
                  if (UtQuaternion_IdenticalComp(&InputQuat,&DuplicateInputQuat) == FALSE)
                  {
                     UtLog_VarLine(UtDriver->Log, "TestCopy() Error: Step %d, Input quaternion was modified",Step);
                     TestPassed = FALSE;
                  }

                  /* check that we can use same variable as input and output */
                  Quaternion_Copy(&InputQuat, &InputQuat);
                  if (UtQuaternion_IdenticalComp(&InputQuat,&DuplicateInputQuat) == FALSE)
                  {
                     UtLog_VarLine(UtDriver->Log, "TestCopy() Error: Step %d, Not able to use variable as input and output",Step);
                     TestPassed = FALSE;
                  }
               
               } /* End if not zero */

            }  /* End l loop */
         }  /* End k loop */
      }  /* End j loop */
   } /* End i loop */

   UTASSERT_BOOL(TestPassed,"Quaternion_Copy","Complete function");

} /* End TestCopy() */


/*************************************************************************************
** Function: TestDelta
**
** Notes:
**  1. Generates two quaternions across quaternion space, and 
**     calls Delta using all possible combinations 
**  2. Verifies that multiplying the 1st input quaternion by
**     the output delta quaternion gives you the second
**     input quaternion.
**  3. Verifies that input quaternions are not modified by function. 
**  4. Veririfes that function is capable of using the same 
**     variable for either input and output.
**  5. Quaternion_Delta() was modified to ensure that the the result would have a 
**     positive angle (scalar component). "DeltaScalarPositive" was added to verify this.
**
*/
void TestDelta(void)
{		
   
   boolean    TestPassed = TRUE, DeltaScalarPositive = TRUE;
   int        Step = 0;
   double     StepSize = 0.5;
   double     i, j, k, l, ii, jj, kk, ll;
   Quaternion Input1Quat, DupInput1Quat;
   Quaternion Input2Quat, DupInput2Quat;
   Quaternion OutputDeltaQuat;
   Quaternion OutputMultiplyQuat;

   for (i = -1; i <= 1; i += StepSize)
   {		
      for (j = -1; j <= 1; j += StepSize)
      {		
         for (k = -1; k <= 1; k += StepSize)
         {		
            for (l = -1; l <= 1; l += StepSize)
            {		
               UtQuaternion_GenerateQuat(&Input1Quat, i, j, k, l);
               DupInput1Quat = Input1Quat;

               for (ii = -1; ii <= 1; ii += StepSize)
               {				
                  for (jj = -1; jj <= 1; jj += StepSize)
                  {		
                     for (kk = -1; kk <= 1; kk += StepSize)
                     {				
                        for (ll = -1; ll <= 1; ll += StepSize)
                        {		
                           Step++;
                           
                           UtQuaternion_GenerateQuat(&Input2Quat, ii, jj, kk, ll);
                           DupInput2Quat = Input2Quat;
                           
                           Quaternion_Delta(&OutputDeltaQuat, &Input1Quat, &Input2Quat);
                           if (OutputDeltaQuat.Comp[3] < 0.0)
                              DeltaScalarPositive = FALSE;
                           Quaternion_Mult(&OutputMultiplyQuat, &Input1Quat, &OutputDeltaQuat);	
                           
                           /*
                           ** Check that multipling 1st input by the 
                           ** deltaquaternion gives you the second input back
                           */																		
                           if (UtQuaternion_EqualComp(&Input2Quat, &OutputMultiplyQuat, 1e-15) == FALSE)
                           {										
                              UtLog_VarLine(UtDriver->Log, "TestDelta() Error: Step %d, Expected result not euqal to computed",Step);
                              TestPassed = FALSE;						
                           }

                           /* check that both inputs weren't modfied by function */	
                           if ((UtQuaternion_IdenticalComp(&Input1Quat, &DupInput1Quat) == FALSE) ||	
                               (UtQuaternion_IdenticalComp(&Input2Quat, &DupInput2Quat) == FALSE))
                           {																				
                              UtLog_VarLine(UtDriver->Log, "TestDelta() Error: Step %d, Inputs were modified",Step);
                              TestPassed = FALSE;									
                           }

                           /* check that function is capable of using 1st input as output */
                           Quaternion_Delta(&DupInput1Quat, &DupInput1Quat, &Input2Quat);	
                           if (UtQuaternion_IdenticalComp(&DupInput1Quat, &OutputDeltaQuat) == FALSE)
                           {										
                              UtLog_VarLine(UtDriver->Log, "TestDelta() Error: Step %d, Unable to use 1st input as output",Step);
                              TestPassed = FALSE;	
                           }

                           /* check that function is capable of using 2nd input as output */	
                           Quaternion_Delta(&Input2Quat, &Input1Quat, &Input2Quat);
                           if (UtQuaternion_IdenticalComp(&Input2Quat, &OutputDeltaQuat) == FALSE)
                           {								
                              UtLog_VarLine(UtDriver->Log, "TestDelta() Error: Step %d, Unable to use 2nd input as output",Step);
                              TestPassed = FALSE;	
                           }

                           /* set dupInput1Quat back to input1Quat for next loop */	
                           DupInput1Quat = Input1Quat;			
            
                        }  /* End ll loop */
                     }  /* End kk loop */
                  }  /* End jj loop */
               } /* End ii loop */

            }  /* End l loop */
         }  /* End k loop */
      }  /* End j loop */
   } /* End i loop */

   UTASSERT_BOOL(DeltaScalarPositive,"Quaternion_Delta","Verify positive delta scalar component");

   UTASSERT_BOOL(TestPassed,"Quaternion_Delta","Complete function");

} /* End TestDelta() */


/*************************************************************************************
** Function: TestIdentity
**
** Notes:
**  1. Verifies that function returns quaternion (0,0,0,1)
**
*/
void TestIdentity(void)
{		
   
   boolean    TestPassed = TRUE;
   Quaternion Quat;
   
   Quaternion_Identity(&Quat);
   
   if (Quat.Comp[0] != 0.0 ||	
       Quat.Comp[1] != 0.0 ||	
       Quat.Comp[2] != 0.0 ||	
       Quat.Comp[3] != 1.0)
   {					
      TestPassed = FALSE;	
   }
   
   UTASSERT_BOOL(TestPassed,"Quaternion_Identity","Complete function");

} /* End TestIdentity() */



/*************************************************************************************
** Function: TestMagnitude
**
** Notes:
**  1. Verifies magnitude calculated correctly by independently 
**     calculating magnitude and comparing it to function output. 
**  2. Checks that input parameter isn't modified
*/
void TestMagnitude(void)
{		
   
   boolean    TestPassed = TRUE;
   int        Step = 0;
   double     StepSize = 0.25;
   double     i, j, k, l;
   double     Magnitude;	
   double     Difference;
   double     CalculatedMagnitude;
   Quaternion InputQuat;
   Quaternion DuplicateInputQuat;


   for (i = -1; i <= 1; i += StepSize)
   {
      for (j = -1; j <= 1; j += StepSize)
      {
         for (k = -1; k <= 1; k += StepSize)
         {
            for (l = -1; l <= 1; l += StepSize)
            {
               Step++;
               
               if (i != 0 && j != 0 && k != 0 && l != 0)
               {

                  UtQuaternion_GenerateQuat(&InputQuat, i, j, k, l);
                  DuplicateInputQuat = InputQuat; /* save off for later check */

                  Magnitude = Quaternion_Magnitude(&InputQuat);
                  CalculatedMagnitude = sqrt(InputQuat.Comp[0] * InputQuat.Comp[0] +
                                             InputQuat.Comp[1] * InputQuat.Comp[1] +	
                                             InputQuat.Comp[2] * InputQuat.Comp[2] +
                                             InputQuat.Comp[3] * InputQuat.Comp[3]);

                  /* Calulate difference between expected and returned value */	
                  Difference = fabs(Magnitude - CalculatedMagnitude);		
                  if (Difference != 0.0)
                  {								
                     UtLog_VarLine(UtDriver->Log, "TestMagintude() Error: Step %d, InputQuat = (%f,%f,%f,%f), Magnitude=%f", 
                                   Step, InputQuat.Comp[0], InputQuat.Comp[1], InputQuat.Comp[2], InputQuat.Comp[3],Magnitude);
                     TestPassed = FALSE;
                  }

                  if (UtQuaternion_IdenticalComp(&InputQuat, &DuplicateInputQuat) == FALSE)
                  {								
                     UtLog_VarLine(UtDriver->Log, "TestMagnitude() Error: Step %d, Modified input parameter",Step);
                     TestPassed = FALSE;	
                  }

               } /* End if not zero */

            }  /* End l loop */
         }  /* End k loop */
      }  /* End j loop */
   } /* End i loop */

   UTASSERT_BOOL(TestPassed,"Quaternion_Magnitude","Complete function");

} /* End TestMagnitude() */


/*************************************************************************************
** Function: TestNormalize
**
** Notes:
**  1. Generates quaternions at set intervals across quaternion space
**     - Multiplies all elements by 1.1 to create non normalized quats 
**     - Calls Quaternion_Normalize to normalize them 
**     - checks that normalized quats have magnitude within 1e-15 of 1 
**     - checks that ratio between corresponding elements of unnormalized 
**       and normalized quats are the same for all four elements 
**  2. Checks that input parameter isn't modified by function 
**  3. Checks that we can use the same variable for input and output
**  4. Checks that zero input does not result in a divide-by-zero exception
**
*/
#define QUATTEST_NORMALIZATION_LIMIT 1e-15

void TestNormalize(void)
{		
   
   boolean    TestPassed = TRUE;
   int        Step = 0;
   int        QuaternionIsNormalizd;
   double     StepSize = 0.25;  /* step size through quaternion component space */
   double     i, j, k, l;       /* index used for generating quaternions */
   double     Magnitude;        /* magnitude of quaternion */
   Quaternion OutputQuat;	
   Quaternion InputQuat;
   Quaternion DuplicateInputQuat;  /* save inputQuat to verify that its not modified by normalize  */
   double     Ratio1 = 0.0,        /* ratios between corresponding elements of input and output    */
              Ratio2 = 0.0,        /* quaternions */
              Ratio3 = 0.0,
              Ratio4 = 0.0;
   int        Index;               /* used for looping through quaternion */

   for (i = -1; i <= 1; i += StepSize)
   {
      for (j = -1; j <= 1; j += StepSize)
      {
         for (k = -1; k <= 1; k += StepSize)
         {
            for (l = -1; l <= 1; l += StepSize)
            {
               
               Step++;			

               UtQuaternion_GenerateQuat(&InputQuat, i, j, k, l); /* Create non-normalized quaternion */
               for (Index = 0; Index < 4; Index++)
               {	
                  InputQuat.Comp[Index] *= 1.1;
               }

               DuplicateInputQuat = InputQuat; /* save inputQuat to verify that its not modified by Quaternion_Normalize */
               
			   QuaternionIsNormalizd = 
				 Quaternion_IsNormalized( &InputQuat, 0.000001 );

			   if ( QuaternionIsNormalizd == 1 )
               {								
                 UtLog_VarLine(UtDriver->Log, "TestNormalize() Error: Step %d, unnormalized quaternion found to be normalized",Step);
                 TestPassed = FALSE;			 
               }

#ifdef DEBUG
               Magnitude = sqrt(InputQuat.Comp[0] * InputQuat.Comp[0] +	
                                InputQuat.Comp[1] * InputQuat.Comp[1] +	
                                InputQuat.Comp[2] * InputQuat.Comp[2] +		
                                InputQuat.Comp[3] * InputQuat.Comp[3]);

               UtLog_VarLine(UtDriver->Log, "TestNormalize() Debug: Step %d, InputQuat = (%f,%f,%f,%f), Magnitude=%f", 
                             Step, InputQuat.Comp[0], InputQuat.Comp[1], InputQuat.Comp[2], InputQuat.Comp[3],Magnitude);
#endif

               Magnitude = sqrt(InputQuat.Comp[0] * InputQuat.Comp[0] +	
                                InputQuat.Comp[1] * InputQuat.Comp[1] +	
                                InputQuat.Comp[2] * InputQuat.Comp[2] +		
                                InputQuat.Comp[3] * InputQuat.Comp[3]);

			   if ( Magnitude > 0.000001 ) {
               
			     Quaternion_Normalize(&OutputQuat, &InputQuat);
                 Magnitude = sqrt(OutputQuat.Comp[0] * OutputQuat.Comp[0] +
                                  OutputQuat.Comp[1] * OutputQuat.Comp[1] +	
                                  OutputQuat.Comp[2] * OutputQuat.Comp[2] +
                                  OutputQuat.Comp[3] * OutputQuat.Comp[3]);

#ifdef DEBUG
               UtLog_VarLine(UtDriver->Log, "TestNormalize() Debug: Step %d, OutputQuat = (%f,%f,%f,%f), Magnitude=%f", 
                             Step, OutputQuat.Comp[0], OutputQuat.Comp[1], OutputQuat.Comp[2], OutputQuat.Comp[3],Magnitude);
#endif		  					

			     /* Check that quaternion was normalized */
                 if (fabs(Magnitude - 1.0) > QUATTEST_NORMALIZATION_LIMIT)
                 {					
                  UtLog_VarLine(UtDriver->Log, "TestNormalize() Error: Step %d, OutputQuat = (%f,%f,%f,%f), Magnitude=%20.15g", 
                                Step, InputQuat.Comp[0], InputQuat.Comp[1], InputQuat.Comp[2], InputQuat.Comp[3],Magnitude);
                  TestPassed = FALSE;
                 }

				 QuaternionIsNormalizd = 
				  Quaternion_IsNormalized( &OutputQuat, 0.000001 );

			     if ( QuaternionIsNormalizd == 0 )
                 {								
                   UtLog_VarLine(UtDriver->Log, "TestNormalized() Error: Step %d, normalized quaternion found to be unnormalized",Step);
                   TestPassed = FALSE;			 
                 }


                 /* Ensure that ratios between all elements were preserved */
                 if   (OutputQuat.Comp[0] != 0.0) Ratio1 = InputQuat.Comp[0] / OutputQuat.Comp[0];
			     else Ratio1 = 1.1;

                 if   (OutputQuat.Comp[1] != 0.0) Ratio2 = InputQuat.Comp[1] / OutputQuat.Comp[1];
			     else Ratio2 = 1.1;

                 if   (OutputQuat.Comp[2] != 0.0) Ratio3 = InputQuat.Comp[2] / OutputQuat.Comp[2];
			     else Ratio3 = 1.1;

                 if   (OutputQuat.Comp[3] != 0.0) Ratio4 = InputQuat.Comp[3] / OutputQuat.Comp[3];
			     else Ratio4 = 1.1;

                 if (fabs(Ratio1 - Ratio2) > QUATTEST_NORMALIZATION_LIMIT ||
                     fabs(Ratio1 - Ratio3) > QUATTEST_NORMALIZATION_LIMIT ||
                     fabs(Ratio1 - Ratio4) > QUATTEST_NORMALIZATION_LIMIT)
                 {
                  /* the ratios between elements were not preserved */	
                  UtLog_VarLine(UtDriver->Log, "TestNormalize() Error: Step %d, Ratios aren't preserved",Step); 
                  TestPassed = FALSE;
                 }

                 /* verifiy that inputQuat wasn't modfied by Normalize */
                 if (UtQuaternion_IdenticalComp(&InputQuat, &DuplicateInputQuat) == FALSE)
                 {								
                  UtLog_VarLine(UtDriver->Log, "TestNormalized() Error: Step %d, Modified input parameter",Step);
                  TestPassed = FALSE;	
                 }

                 /* 
                 ** Verify that we can use same paramenter for input and output.
                 ** Check is done by normalize the InputQuat used in previous call
                 ** to Quaternion_Normalize and verifying that it is the same as
                 ** OutputQuat
                 */
                 Quaternion_Normalize(&InputQuat, &InputQuat);	
                 if (UtQuaternion_IdenticalComp(&InputQuat, &OutputQuat) == FALSE)
                 {								
                  UtLog_VarLine(UtDriver->Log, "TestNormalized() Error: Step %d, Couldn't use same quaternion for input and output",Step);
                  TestPassed = FALSE;			 
                 }

			   } else {

			     QuaternionIsNormalizd = 
					 Quaternion_IsNormalized( &InputQuat, 0.000001 );

			     if ( QuaternionIsNormalizd == 1 )
                 {								
                  UtLog_VarLine(UtDriver->Log, "TestIsNormalizedd() Error: Step %d, zero quaternion found to be normalized",Step);
                  TestPassed = FALSE;			 
                 }


			   } /* quaternion is normalizable */

            }  /* End l loop */
         }  /* End k loop */
      }  /* End j loop */
   } /* End i loop */

   UTASSERT_BOOL(TestPassed,"Quaternion_Normalize","Complete function");

} /* End TestNormalize() */


/*************************************************************************************
** Function: TestStandardize
**
** Notes:
**  1. Generates quaternions at set intervals across quaternion space
**     - Multiplies all elements by -1.1 to create non normalized quats
**     - Calls Quaternion_Stabdardize to standardize them 
**     - checks that standardized quats have magnitude within 1e-15 of 1 
**     - checks that scalar element is 
**     - checks that ratio between corresponding elements of unnormalized 
**       and normalized quats are the same for all four elements 
**  2. Checks that input parameter isn't modified by function 
**  3. Checks that we can use the same variable for input and output
**  4. Checks that zero input produces an error flag
**
*/
#define QUATTEST_NORMALIZATION_LIMIT 1e-15

void TestStandardize(void)
{		
   
   boolean    TestPassed = TRUE;
   int        Step = 0;
   int        QuaternionIsNormalized;
   double     StepSize = 0.25;  /* step size through quaternion component space */
   double     i, j, k, l;       /* index used for generating quaternions */
   double     Magnitude;        /* magnitude of quaternion */
   Quaternion OutputQuat;	
   Quaternion InputQuat;
   Quaternion DuplicateInputQuat;  /* save inputQuat to verify that its not modified by normalize  */
   double     Ratio1 = 0.0,        /* ratios between corresponding elements of input and output    */
              Ratio2 = 0.0,        /* quaternions */
              Ratio3 = 0.0,
              Ratio4 = 0.0;
   int        Index;               /* used for looping through quaternion */
   int        m;
   double     scaleRatio = 1.000001;

   for (i = -1; i <= 1; i += StepSize)
   {
      for (j = -1; j <= 1; j += StepSize)
      {
         for (k = -1; k <= 1; k += StepSize)
         {
            for (l = -1; l <= 1; l += StepSize)
            {
               
               Step++;			

               UtQuaternion_GenerateQuat(&InputQuat, i, j, k, l); /* Create non-normalized quaternion */
               for (Index = 0; Index < 4; Index++)
               {	
                  InputQuat.Comp[Index] *= scaleRatio;
               }

               DuplicateInputQuat = InputQuat; /* save inputQuat to verify that its not modified by Quaternion_Normalize */
               

#ifdef DEBUG
               Magnitude = sqrt(InputQuat.Comp[0] * InputQuat.Comp[0] +	
                                InputQuat.Comp[1] * InputQuat.Comp[1] +	
                                InputQuat.Comp[2] * InputQuat.Comp[2] +		
                                InputQuat.Comp[3] * InputQuat.Comp[3]);

               UtLog_VarLine(UtDriver->Log, "TestStandardize() Debug: Step %d, InputQuat = (%f,%f,%f,%f), Magnitude=%f", 
                             Step, InputQuat.Comp[0], InputQuat.Comp[1], InputQuat.Comp[2], InputQuat.Comp[3],Magnitude);
#endif
               QuaternionIsNormalized = Quaternion_Standardize(&OutputQuat, &InputQuat, 0.00001 );

			   if ( QuaternionIsNormalized ) {

                 Magnitude = sqrt(OutputQuat.Comp[0] * OutputQuat.Comp[0] +
                                  OutputQuat.Comp[1] * OutputQuat.Comp[1] +	
                                  OutputQuat.Comp[2] * OutputQuat.Comp[2] +
                                  OutputQuat.Comp[3] * OutputQuat.Comp[3]);

#ifdef DEBUG
               UtLog_VarLine(UtDriver->Log, "TestStandardize() Debug: Step %d, OutputQuat = (%f,%f,%f,%f), Magnitude=%f", 
                             Step, OutputQuat.Comp[0], OutputQuat.Comp[1], OutputQuat.Comp[2], OutputQuat.Comp[3],Magnitude);
#endif		  					
                 /* Check that quaternion was normalized */
                 if (fabs(Magnitude - 1.0) > QUATTEST_NORMALIZATION_LIMIT)
                 {					
                  UtLog_VarLine(UtDriver->Log, "TestStandardize() Error: Step %d, OutputQuat = (%f,%f,%f,%f), Magnitude=%20.15g", 
                                Step, InputQuat.Comp[0], InputQuat.Comp[1], InputQuat.Comp[2], InputQuat.Comp[3],Magnitude);
                  TestPassed = FALSE;
			     }

                 /* Ensure that ratios between all elements were preserved */
                 if      (OutputQuat.Comp[0] != 0.0) Ratio1 = InputQuat.Comp[0] / OutputQuat.Comp[0];
			     else if ( InputQuat.Comp[3] < 0.0 ) Ratio1 = - scaleRatio;
			     else                                Ratio1 =   scaleRatio;

                 if      (OutputQuat.Comp[1] != 0.0) Ratio2 = InputQuat.Comp[1] / OutputQuat.Comp[1];
			     else if ( InputQuat.Comp[3] < 0.0 ) Ratio2 = - scaleRatio;
			     else                                Ratio2 =   scaleRatio;

                 if      (OutputQuat.Comp[2] != 0.0) Ratio3 = InputQuat.Comp[2] / OutputQuat.Comp[2];
			     else if ( InputQuat.Comp[3] < 0.0 ) Ratio3 = - scaleRatio;
			     else                                Ratio3 =   scaleRatio;

                 if      (OutputQuat.Comp[3] != 0.0) Ratio4 = InputQuat.Comp[3] / OutputQuat.Comp[3];
			     else if ( InputQuat.Comp[3] < 0.0 ) Ratio4 = - scaleRatio;
			     else                                Ratio4 =   scaleRatio;

                 if (fabs(Ratio1 - Ratio2) > QUATTEST_NORMALIZATION_LIMIT ||
                     fabs(Ratio1 - Ratio3) > QUATTEST_NORMALIZATION_LIMIT ||
                     fabs(Ratio1 - Ratio4) > QUATTEST_NORMALIZATION_LIMIT )
                 {
                  /* the ratios between elements were not preserved */	
                  UtLog_VarLine(UtDriver->Log, "TestStandardize() Error: Step %d, Ratios aren't preserved",Step); 
                  TestPassed = FALSE;
                  UtLog_VarLine(UtDriver->Log, "  Ratios: %f, %f, %f, %f",Ratio1,Ratio2,Ratio3,Ratio4); 
                  UtLog_VarLine(UtDriver->Log, "  IQuat:  %f, %f, %f, %f",InputQuat.Comp[0],InputQuat.Comp[1],InputQuat.Comp[2],InputQuat.Comp[3]); 
                  UtLog_VarLine(UtDriver->Log, "  OQuat:  %f, %f, %f, %f",OutputQuat.Comp[0],OutputQuat.Comp[1],OutputQuat.Comp[2],OutputQuat.Comp[3]); 
                  UtLog_VarLine(UtDriver->Log, "  GenIn:  %f, %f, %f, %f",i,j,k,l); 
                 }  

                 /* 
                 ** Verify that scalar element is non negative
                 */
                 if ( OutputQuat.Comp[3] < 0.0 ) {			  			
                  /* the scalar element is negative */	
                  UtLog_VarLine(UtDriver->Log, "TestStandardize() Error: Step %d, scalar term negative",Step); 
                  TestPassed = FALSE;
                 }

                 /* verifiy that inputQuat wasn't modfied by Normalize */
                 if (UtQuaternion_IdenticalComp(&InputQuat, &DuplicateInputQuat) == FALSE)
                 {								
                  UtLog_VarLine(UtDriver->Log, "TestStandardize() Error: Step %d, Modified input parameter",Step);
                  TestPassed = FALSE;	
                 }

                 /* 
                 ** Verify that we can use same paramenter for input and output.
                 ** Check is done by Standardizing the InputQuat used in previous call
                 ** to Quaternion_Standardize and verifying that it is the same as
                 ** OutputQuat
                 */
                 Quaternion_Normalize(&InputQuat, &InputQuat);	
                 if (UtQuaternion_IdenticalComp(&InputQuat, &OutputQuat) == FALSE)
                 {								
                  UtLog_VarLine(UtDriver->Log, "TestStandardize() Error: Step %d, Couldn't use same quaternion for input and output",Step);
                  TestPassed = FALSE;			 
                 }

                 /* verify that a seriously non-normalized quaternion is caught */
                 for ( m=0; m<4; m++ ) InputQuat.Comp[m] = 1.1 * DuplicateInputQuat.Comp[m];
                 QuaternionIsNormalized = Quaternion_Standardize(&OutputQuat, &InputQuat, 0.00001 );
                 if ( QuaternionIsNormalized == TRUE ) {
                  UtLog_VarLine(UtDriver->Log, "TestStandardize() Error: Step %d, seriously non-normal quaternion not caught",Step);
                  TestPassed = FALSE;			 
                 }

			   } /* quaternion is normalized */

            }  /* End l loop */
         }  /* End k loop */
      }  /* End j loop */
   } /* End i loop */

   /* comfirm that a zero quaternion is trapped with an error flag */
   for ( m=0; m<4; m++ ) InputQuat.Comp[m] = 0.0;
   m = Quaternion_Standardize(&OutputQuat, &InputQuat, 0.00001 );

   if ( m == TRUE )
   {
     UtLog_VarLine(UtDriver->Log, "TestStandardize() Error: seriously non-normal quaternion not flagged");
     TestPassed = FALSE;			 
   }			  			


   UTASSERT_BOOL(TestPassed,"Quaternion_Standardize","Complete function");

} /* End TestStandardize() */



/*************************************************************************************
** Function: TestToAndFromMatrix
**
** Notes:
**  1. Generates quaternions across quaternion space 
**  2. Converts each quaternion to a DCM using Quaternion_ToMatrix and then 
**     converts that DCM back to a quaternion with Quaternion_FromMatrix. 
**     Verifies that output quaternion is equivalent to input 
**     quaternion within a tolerance.
**  3. Verifies that input quaternion and input matrix to both
**     functions is not modified by the functions.
**
*/
void TestToAndFromMatrix(void)
{		
   
   boolean    TestPassed = TRUE; 
   int        Step = 0;	
   double     StepSize = 0.25;
   double     i, j, k, l; /* indices for generating quaternions */
   double     Angle;	     /* angle between input output quaternions */
   Quaternion InputQuat;	
   Quaternion DuplicateInputQuat;	
   Quaternion OutputQuat;	
   Matrix3x3d Dcm;	
   Matrix3x3d DuplicateDcm;	

   /*
   ** This section generates quaternions, converts them to DCMs, converts the DCMs
   ** back to quaternions and verifies that the final and starting quaternions are equal.
   ** Also verifies that input parameters are not affected.
   */
   
   for (i = -1; i <= 1; i += StepSize)
   {	
      for (j = -1; j <= 1; j += StepSize)
      {
         for (k = -1; k <= 1; k += StepSize)
         {
            for (l = -1; l <= 1; l += StepSize)
            {
               Step++;

               UtQuaternion_GenerateQuat(&InputQuat, i, j, k, l);
               DuplicateInputQuat = InputQuat; /* Save off input quat for later test */

               Quaternion_ToMatrix(&Dcm, &InputQuat);	
               DuplicateDcm = Dcm;	
               Quaternion_FromMatrix(&OutputQuat, &Dcm);	
               
               /* UtQuaternion_EqualDir() checks for < Tolerance and not equal so perform own check */
               UtQuaternion_EqualDir(&InputQuat, &OutputQuat, 0.0, &Angle);
               if (fabs(Angle) > 0.0)
               {
                  UtLog_VarLine(UtDriver->Log, "TestToAndFromMatrix() Error: Step %d, Input quat and generated quat don't agree, Difference = %g",Step,Angle);
                  TestPassed = FALSE;	

               }

               if (UtQuaternion_EqualComp(&InputQuat, &OutputQuat, 1e-15) == FALSE)
               {
                  UtLog_VarLine(UtDriver->Log, "TestToAndFromMatrix() Error: Step %d, Input quat and generated quat don't agree",Step);
                  TestPassed = FALSE;	
               }

               if (UtQuaternion_IdenticalComp(&InputQuat, &DuplicateInputQuat) == FALSE)
               {
                  UtLog_VarLine(UtDriver->Log, "TestToAndFromMatrix() Error: Step %d, inputQuat was modfied by Quaternion_QuatToMatrix()",Step);
                  TestPassed = FALSE;	
               }

               if ( !UtMatrix3x3d_IdenticalComp(&Dcm, &DuplicateDcm) )
               {
                  UtLog_VarLine(UtDriver->Log, "TestToAndFromMatrix() Error: Step %d, Input DCM was modfied by Quaternion_MatrixToQuaternion()",Step);
                  TestPassed = FALSE;	
               }


            }  /* End l loop */
         }  /* End k loop */
      }  /* End j loop */
   } /* End i loop */

   UTASSERT_BOOL(TestPassed,"Quaternion_ToMatrix,Quaternion_FromMatrix","Complete function");

} /* End TestToAndFromMatrix() */



/******************************************************************************
** Function: TestToAndFromMatrixUsingFile
**
** Notes:
**  1. Uses independently generated data to verify both functions.
**     1000 pairs of quaternions and DCMs were generated independently. 
**     The DCM's were fed into Quaternion_FromMatrix and the output quaternion 
**     was compared with independent quaternions.  And for 
**     Quaternion_ToMatrix the reverse was done. 
**  2. Expected tolerance depends upon the precision of the input test data. 
**     Test was only run with single precision input data.
*/

#define  TD_QUATDCM_FILE           "QuatDcmColumnwise.inp"
#define  TD_QUATDCM_QUAT_COMP_0    0
#define  TD_QUATDCM_DCM_COMP_0_0   4
#define  TD_QUATDCM_REC_LEN       13

void TestToAndFromMatrixUsingFile(void)
{		
   
   boolean      TestPassed = TRUE; 
   int          Step = 0;
   int          i, j;        /* matrix indices */
   double       FileData[TD_QUATDCM_REC_LEN];
   UtDat_Class  UtDatObj;
   Quaternion   TruthQuat;	
   Quaternion   CalculatedQuat;
   Matrix3x3d   TruthDcm;
   Matrix3x3d   CalculatedDcm;

   if (UtDat_Open(&UtDatObj, TD_QUATDCM_FILE, 0) == UTDAT_VALID)
   {
	   while( UtDat_ReadDataSample(&UtDatObj, FileData, TD_QUATDCM_REC_LEN) == TD_QUATDCM_REC_LEN)
      {

         UtQuaternion_InitQuat(&TruthQuat,&FileData[TD_QUATDCM_QUAT_COMP_0]);
         /*UtMatrix_InitMatColMaj(&TruthVec,&TruthData[TD_QUATDCM_DCM_COMP_0_0]);*/
         /*
         for (j = 0; j < 3; j++)
         {			
            TruthDcm.Comp[0][j] = FileData[TD_QUATDCM_DCM_COMP_0_0+j*3];
            TruthDcm.Comp[1][j] = FileData[TD_QUATDCM_DCM_COMP_0_0+j*3+1];
            TruthDcm.Comp[2][j] = FileData[TD_QUATDCM_DCM_COMP_0_0+j*3+2];
         }
         */
         UtMatrix3x3d_InitColMajMat (&TruthDcm,&FileData[TD_QUATDCM_DCM_COMP_0_0]);
         Step++;

         /******************* Quaternion to DCM Test *******************/
         
         Quaternion_ToMatrix(&CalculatedDcm,&TruthQuat);	
         
         for (i = 0; i < 3; i++)
         {			
            for (j = 0; j < 3; j++)
            {				

               if (fabs(TruthDcm.Comp[i][j] - CalculatedDcm.Comp[i][j]) > TEST_DATA_TOLERANCE)
               {						   
                  UtLog_VarLine(UtDriver->Log, "TestToAndFromMatrixUsingFile() Error: Expected Quaternion_ToMatrix() result not equal to computed result at step %d",Step);
                  TestPassed = FALSE;
               }

            } /* End j loop */ 
         } /* End i loop */ 
      
         /******************* DCM to Quaternion Test *******************/
         
         Quaternion_FromMatrix(&CalculatedQuat,&TruthDcm);
         
         if (UtQuaternion_EqualComp(&TruthQuat, &CalculatedQuat, TEST_DATA_TOLERANCE) != TRUE)
         {				
            UtLog_VarLine(UtDriver->Log, "TestToAndFromMatrixUsingFile() Error: Expected Quaternion_FromMatrix() result not equal to computed result at step %d",Step);
            TestPassed = FALSE;	
         }

      } /* End while file data */
   
      UtDat_Close(&UtDatObj);

   } /* End if data file opened */
   else
   {
      UtLog_VarLine(UtDriver->Log, "Couldn't open test data: %s",TD_QUATDCM_FILE);
      TestPassed = FALSE;
   }

   UTASSERT_BOOL(TestPassed,"Quaternion_ToMatrix,Quaternion_FromMatrix","Complete function");

} /* End TestToAndFromMatrixUsingFile() */



/*************************************************************************************
** Function: TestMult
**
** Notes:
**  1. Uses 1000 sets of independently generated quaternion 
**     multiply operations to verify function.  Reads in test
**     data from text file and compares to output from Quaternion_Mult. 
**  2. Verifies that both input parameters are not modfied by function
**  3. Verifies that either input parameter can be the same 
**     variable as the output parameter.
**
*/

#define  TD_MULT_FILE     "QuatQuatMult.inp"
#define  TD_MULT_QUAT_LEFT_COMP_0     0
#define  TD_MULT_QUAT_RIGHT_COMP_0    4
#define  TD_MULT_QUAT_RESULT_COMP_0   8
#define  TD_MULT_REC_LEN             12

void TestMult(void)
{		
   
   boolean      TestPassed = TRUE; 
   int          Step = 0;
   double       FileData[TD_QUATDCM_REC_LEN];
   UtDat_Class  UtDatObj;
   Quaternion   QuatLeft;
   Quaternion   QuatRight;
   Quaternion   DuplicateQuatLeft;
   Quaternion   DuplicateQuatRight;
   Quaternion   ExpectedQuatResult;
   Quaternion   CalculatedQuatResult;

   if (UtDat_Open(&UtDatObj, TD_MULT_FILE, 0) == UTDAT_VALID)
   {

	   while( UtDat_ReadDataSample(&UtDatObj, FileData, TD_MULT_REC_LEN) == TD_MULT_REC_LEN)
      {
         UtQuaternion_InitQuat(&QuatLeft,&FileData[TD_MULT_QUAT_LEFT_COMP_0]);
         UtQuaternion_InitQuat(&QuatRight,&FileData[TD_MULT_QUAT_RIGHT_COMP_0]);
         UtQuaternion_InitQuat(&ExpectedQuatResult,&FileData[TD_MULT_QUAT_RESULT_COMP_0]);

         Step++;
         DuplicateQuatLeft  = QuatLeft;
         DuplicateQuatRight = QuatRight;
         Quaternion_Mult(&CalculatedQuatResult, &QuatLeft, &QuatRight);	
     
         /* Verify correct answer is calculated */
         if (UtQuaternion_EqualComp(&ExpectedQuatResult, &CalculatedQuatResult, TEST_DATA_TOLERANCE) != TRUE)
         {
            UtLog_VarLine(UtDriver->Log, "TestMult() Error: Expected result not equal to computed result at step %d",Step);
            TestPassed = FALSE;
         }			
    
         if ((UtQuaternion_IdenticalComp(&QuatLeft,  &DuplicateQuatLeft)  != TRUE) ||
             (UtQuaternion_IdenticalComp(&QuatRight, &DuplicateQuatRight) != TRUE))
         {
            UtLog_VarLine(UtDriver->Log, "TestMult() Error: Input quaternion modified at step = %d",Step);
            TestPassed = FALSE;
         }				
   
         Quaternion_Mult(&QuatLeft, &QuatLeft, &QuatRight);
         Quaternion_Mult(&DuplicateQuatRight, &DuplicateQuatLeft, &DuplicateQuatRight);

         if ((UtQuaternion_IdenticalComp(&QuatLeft, &CalculatedQuatResult) != TRUE) ||	
             (UtQuaternion_IdenticalComp(&DuplicateQuatRight, &CalculatedQuatResult) != TRUE))
         {
            UtLog_VarLine(UtDriver->Log, "TestMult() Error: Unable to use same quaternion for input and result at step = %d",Step);
            TestPassed = FALSE;
         }

      } /* End while file data */

      UtDat_Close(&UtDatObj);

   } /* End if data file opened */
   else
   {
      UtLog_VarLine(UtDriver->Log, "Couldn't open test data: %s",TD_QUATDCM_FILE);
      TestPassed = FALSE;
   }

   UTASSERT_BOOL(TestPassed,"Quaternion_Mult","Complete function");

} /* End TestMult() */


/*************************************************************************************
** Function: TestMakeScalarPositive
**
** Notes:
**  1. Generates quaternions across quaternion space and feeds 
**     them into function.
**  2. Verifies that output and input quaternions are equivalent 
**     (all components either equal or all components are equal 
**     to the negative of the corresponding input component) 
**  3. Verifies that all output quaternions have positive
**     scalar components
**  4. Verifies that input quaternion is not modified by function.
**  5. Veririfes that function is capable of using same variable 
**     for input and output. 
*/
void TestMakeScalarPositive(void)
{		
   
   boolean    TestPassed = TRUE; 
   int        Step = 0;	
   double     StepSize = 0.5;	
   double     i, j, k, l;	 /* indices for generating quaternions */	
   Quaternion InputQuat;
   Quaternion DuplicateInputQuat;
   Quaternion OutputQuat;

   for (i = -1; i <= 1; i += StepSize)
   {
      for (j = -1; j <= 1; j += StepSize)
      {
         for (k = -1; k <= 1; k += StepSize)
         {
            for (l = -1; l <= 1; l += StepSize)
            {

               Step++;

               UtQuaternion_GenerateQuat(&InputQuat, i, j, k, l);
               DuplicateInputQuat = InputQuat;  /* Save off input quat for later test */

               Quaternion_MakeScalarPositive(&OutputQuat, &InputQuat);
               
               /* Check that input and output are equivalent quaternions */
               if (UtQuaternion_IdenticalComp(&OutputQuat, &InputQuat) == FALSE)
               {						
                  UtLog_VarLine(UtDriver->Log, "TestMakeScalarPositive() Error: Step %d, Output not equal to input",Step);
                  TestPassed = FALSE;
               }

               /* Check that scalar component of output is positive */
               if (OutputQuat.Comp[3] < 0.0)
               {			
                  UtLog_VarLine(UtDriver->Log, "TestMakeScalarPositive() Error: Step %d, Scalar not negative",Step);
                  TestPassed = FALSE;
               }				

               /* Check that input parameter was not modified by function */	
               if (UtQuaternion_IdenticalComp(&DuplicateInputQuat, &InputQuat) == FALSE)
               {					
                  UtLog_VarLine(UtDriver->Log, "TestMakeScalarPositive() Error: Step %d, Input modified",Step);
                  TestPassed = FALSE;
               }

               /* Verify that function is capable of using same parameter for input and output */	
               Quaternion_MakeScalarPositive(&InputQuat, &InputQuat);	
               if (UtQuaternion_IdenticalComp(&InputQuat, &OutputQuat) == FALSE)
               {					
                  UtLog_VarLine(UtDriver->Log, "TestMakeScalarPositive() Error: Step %d, Unable to use input as output",Step);
                  TestPassed = FALSE;	
               }

            }  /* End l loop */
         }  /* End k loop */
      }  /* End j loop */
   } /* End i loop */

   UTASSERT_BOOL(TestPassed,"Quaternion_MakeScalarPositive","Complete function");

} /* End TestMakeScalarPositive() */


/*************************************************************************************
** Function: TestPropagate
**
** Notes:
**   1. Generates large number of unit vectors spanning the 
**      unit sphere. 
**   2. For each unit vector generate a range of vectors 
**      varrying in magnitude (2Pi, Pi, Pi/2, Pi/4, 
**      Pi/8, Pi/16, Pi/32, and Pi/64) that are parallel to the 
**      unit vector.  These will be used as the input vector 
**      in the series of tests. 
**   3. For each vector generated in step 2, we start with a 
**      quaternion of (0,0,0,1) and propagate it successively 
**      until we have propagated it all the way through 360 degrees. 
**      For the 2Pi rate vector this only takes one step.  For 
**      the Pi/64 rate vector it will take 128 steps. 
**   4. At each propagation step we check that the eigenvector 
**      of the quaternion at that step is same as the rate 
**      unit vector that was used to generate the rate vector 
**      in step 2.  The eigenvector should always equal the 
**      rate unit vector. 
**   5. At each step we also check that the angle that the 
**      quaternion represents is equal to the number of steps 
**      multiplied by the magnitude of the rate vector. 
**   6. Check that input quaternion and vector are not modified 
**      by function. 
**   7. Verify that we can use the same quaternion for input 
**      and output.
*/
void TestPropagate(void)
{		
   
   boolean      TestPassed = TRUE; 
   double       Step;		
   double       VectorStep = .1;	
   double       i, j, k;	
   double       Magnitude;		/* mag used to create unit vector from i,j,k */	
   double       VecMag;	
   double       VecMagStepPower;	
   double       VecMagStep;	
   Vector3d     RateUnitVec;
   Vector3d     Rate;	
   Vector3d     DuplicateRate;	
   Quaternion   Quat;	
   Quaternion   DuplicateQuat;
   Quaternion   PropagatedQuat;
   Vector3d     EigenVec;		
   double       DeltaX;	
   double       DeltaY;	
   double       DeltaZ;	
   double       Theta;	
   double       DeltaAngle;	

   for (i = -1.0; i <= 1.0; i+= VectorStep)
   {
      for (j = -1.0; j <= 1.0; j += VectorStep)
      {
         for (k = -1.0; k <= 1.0; k += VectorStep)
         {
            for (VecMagStepPower = 0; VecMagStepPower < 8; VecMagStepPower++)
            {

               VecMagStep = pow(2, VecMagStepPower);
               VecMag = 2 * MTH_PI / VecMagStep;

					/* initialize quat to origin */
               Quat.Comp[0] = 0.0;
               Quat.Comp[1] = 0.0;	
               Quat.Comp[2] = 0.0;	
               Quat.Comp[3] = 1.0;
               /* Construct rate vector */
               Magnitude = sqrt(i * i + j * j + k * k);
               if (Magnitude > 1e-12)
               {
                  /* Only divide by magnitude if nonzero */
                  RateUnitVec.Comp[0] = i / Magnitude;
                  RateUnitVec.Comp[1] = j / Magnitude;
                  RateUnitVec.Comp[2] = k / Magnitude;
               }
               else
               {	
                  /* Set rate to zero if indexes are zero */
                  RateUnitVec.Comp[0] = 0;
                  RateUnitVec.Comp[1] = 0;
                  RateUnitVec.Comp[2] = 0;
               }
               Rate.Comp[0] = RateUnitVec.Comp[0] * VecMag;	
               Rate.Comp[1] = RateUnitVec.Comp[1] * VecMag;	
               Rate.Comp[2] = RateUnitVec.Comp[2] * VecMag;


#ifdef DEBUG
               UtLog_VarLine(UtDriver->Log, "TestPropagate() Debug: Step %f, VecMagStepPower=%f, (i,j,k)=(%f,%f,%f), Rate=(%f,%f,%f)",
                             Step,VecMagStepPower, i, j, k, Rate.Comp[0], Rate.Comp[1], Rate.Comp[2]);
#endif
					for (Step = 1; Step < (VecMagStep + 1); Step++)
               {
#ifdef DEBUG
               UtLog_VarLine(UtDriver->Log, "TestPropagate() Debug: Step-1 %f, Quat=(%f,%f,%f,%f)",
                             Step, Quat.Comp[0], Quat.Comp[1], Quat.Comp[2], Quat.Comp[3]);
#endif
					DuplicateQuat = Quat;
               DuplicateRate = Rate;
               Quaternion_Propagate(&PropagatedQuat, &Quat, &Rate);	
               if (Rate.Comp[0] == 0.0 &&
                   Rate.Comp[1] == 0.0 &&
                   Rate.Comp[2] == 0.0)
               {
                  /*
                  ** Check that propagated quat is equal to input quat for zero rates.	
                  ** Can't do the normal checks done below because of various divide	
                  ** by zero problems.
                  */
                  if (UtQuaternion_IdenticalComp(&PropagatedQuat, &Quat) == FALSE)
                  {					
                     UtLog_VarLine(UtDriver->Log, "TestPropagate() Error: Step %f, Failed to process zero rates. (i,j,k)=(%f,%f,%f)",Step, i, j, k);
                     TestPassed = FALSE;		
                  }					
               }					
               else
               {	
                  /************* Normal checks done for nonzero rates *************/	
                  /* 
                  ** Check that eigen vector of the Quaternion equals the rate vector.	
                  ** The eigen vector should remain constant since we are rotating		
                  ** on one axis.
                  */										
                  Theta = 2 * acos(PropagatedQuat.Comp[3]);
                  EigenVec.Comp[0] = PropagatedQuat.Comp[0] / sin(Theta/2);
                  EigenVec.Comp[1] = PropagatedQuat.Comp[1] / sin(Theta/2);
                  EigenVec.Comp[2] = PropagatedQuat.Comp[2] / sin(Theta/2);	
                  DeltaX = fabs(RateUnitVec.Comp[0] - EigenVec.Comp[0]);	
                  DeltaY = fabs(RateUnitVec.Comp[1] - EigenVec.Comp[1]);	
                  DeltaZ = fabs(RateUnitVec.Comp[2] - EigenVec.Comp[2]);
                  if (DeltaX > 1e-12 || DeltaY > 1e-12 || DeltaZ > 1e-12)
                  {													
                     /*
                     ** Eigenvector check is only valid if all of the following values are not close to zero.
                     ** All of these values will be close to zero for quaternions close to the origin (0,0,0,1).	
                     ** We get quaternions close to the origin at the end of each test series when we've propagated	
                     ** the quaternion through 360 degrees.  The calculated eigen vectors can vary wildly in
                     ** this range because we are dividing extremely small numbers (1st 3 quaternion elements)			
                     ** by another extremely small number, sin(theta/2).  So we don't want to check the eigen			
                     ** vector here.  We do still perform all the other checks including the angle check below
                     */
                     if (PropagatedQuat.Comp[0] > 1e-14 || PropagatedQuat.Comp[1] > 1e-14 || PropagatedQuat.Comp[2] > 1e-14 || sin(Theta/2) > 1e-15)
                     {								
                        UtLog_VarLine(UtDriver->Log, "TestPropagate() Error: Step %f, Eigen vector check failed. (i,j,k)=(%f,%f,%f)", Step, i, j, k);
                        TestPassed = FALSE;	
                     }						
                  }				
                  /*
                  ** Check that the angle of the quaternion is correct. It should
                  ** equal the number of steps times the magnitude of the rate vector 
                  */			
                  if (Theta < 0.0)
                  {
                     Theta += 2 * MTH_PI;
                  }
                  DeltaAngle = fabs(Theta - ((double) Step) * VecMag);	
                  if (DeltaAngle > 1e-13)
                  {							
                     UtLog_VarLine(UtDriver->Log, "TestPropagate() Error: Step %f, Angle check failed. (i,j,k)=(%f,%f,%f)", Step, i, j, k);
                     TestPassed = FALSE;			
                  }

               }  /* End if Rate == 0.0 */

               /* Check that input quaternion was not modified */	
               if (UtQuaternion_IdenticalComp(&Quat, &DuplicateQuat) == FALSE)
               {									
                  UtLog_VarLine(UtDriver->Log, "TestPropagate() Error: Step %f, Input quaternion was modified. (i,j,k)=(%f,%f,%f)", Step, i, j, k);
                  TestPassed = FALSE;				
               }									
               /* Check that input rate was not modified */	
               if ( !UtVector3d_IdenticalComp(&Rate, &DuplicateRate) )
               {											
                  UtLog_VarLine(UtDriver->Log, "TestPropagate() Error: Step %f, Input rate was modified. (i,j,k)=(%f,%f,%f)", Step, i, j, k);
                  TestPassed = FALSE;	
               }									
               /* 
               ** Check that function can use same quaternion for input and output.  Also
               ** serves to propagate quat so that it is updated for the next step in the loop. 
               */									
               Quaternion_Propagate(&Quat, &Quat, &Rate);
               if (UtQuaternion_IdenticalComp(&Quat, &PropagatedQuat) == FALSE)
               {									
                  UtLog_VarLine(UtDriver->Log, "TestPropagate() Error: Step %f, Unable to use same quaternion as input and output. (i,j,k)=(%f,%f,%f)", Step, i, j, k);
                  TestPassed = FALSE;			
               }
               
            } /* End Step loop */

         } /* End VecMagStepPower loop */

         }  /* End k loop */
      }  /* End j loop */
   } /* End i loop */

   UTASSERT_BOOL(TestPassed,"Quaternion_Propagate","Complete function");

} /* End TestPropagate() */


/*************************************************************************************
** Function: TestVectorTransform
**
** Notes:
**  1. Uses independently generated data to use a quaternion
**     to rotate a vector and compare the output to previously
**     calculated values. 
**  2. Verifies that input vector and quaternion are not modified
**     by function.
**  3. Verifies that function is capable of using the same
**     vector for input and output.
**  4. The last three columns in the file contain a residual vector that 	
**     is the residual between rotating the vector	with a quaternion and
**     using an alternate method involving DCMs. This data is not used.
**
*/
#define  TD_VEC_XFORM_FILE     "QuatVecMult.inp"
#define  TD_VEC_XFORM_QUAT_COMP_0         0
#define  TD_VEC_XFORM_VEC_COMP_0          4
#define  TD_VEC_XFORM_VEC_RESULT_COMP_0   7
#define  TD_VEC_XFORM_REC_LEN            13

void TestVectorTransform(void)
{		
   
   boolean      TestPassed = TRUE;
   int          Step = 0;
   int          i;  /* matrix index */
   double       FileData[TD_VEC_XFORM_REC_LEN];
   UtDat_Class  UtDatObj;
   Quaternion   InputQuat;    /* Qaternion read from data text file */
   Quaternion   DuplicateInputQuat;
   Vector3d     InputVec;     /* Vector read from file that will be rotated */
   Vector3d     DuplicateInputVec;
   Vector3d     ExpectedResultVec;    /* Vector that is inputVec rotated by inputQuat */
   Vector3d     CalculatedResultVec;  /* output from VectorTransform that will be compared to rotatedVec */

	
   if (UtDat_Open(&UtDatObj, TD_MULT_FILE, 0) == UTDAT_VALID)
   {

	   while( UtDat_ReadDataSample(&UtDatObj, FileData, TD_VEC_XFORM_REC_LEN) == TD_VEC_XFORM_REC_LEN)
      {
         UtQuaternion_InitQuat(&InputQuat,&FileData[TD_VEC_XFORM_QUAT_COMP_0]);
         UtVector3d_InitVec(&InputVec,&FileData[TD_VEC_XFORM_VEC_COMP_0]);
         UtVector3d_InitVec(&ExpectedResultVec,&FileData[TD_VEC_XFORM_VEC_RESULT_COMP_0]);
 
         Step++;
         DuplicateInputQuat = InputQuat;
         DuplicateInputVec  = InputVec;
      
         /********************** Quaternion Vector Transform Test ************************************/
         Quaternion_VectorTransform(&CalculatedResultVec, &InputQuat, &InputVec);
         for (i = 0; i < 3; i++)
         {
            /* Verify calculated vectors agree with expected results from input file */
            if (fabs(ExpectedResultVec.Comp[i] - CalculatedResultVec.Comp[i]) > TEST_DATA_TOLERANCE)
            {
               UtLog_VarLine(UtDriver->Log, "TestVectorTransform Error: Step &d, Expected result not equal to computed result",Step);
               TestPassed = FALSE;		
            }
         }
         
         /* Verify input quaternion was not modified by function */
         if (UtQuaternion_IdenticalComp(&DuplicateInputQuat, &InputQuat) != TRUE)
         {
            UtLog_VarLine(UtDriver->Log, "TestVectorTransform Error: Step %d, Input quaternion was modified",Step);
            TestPassed = FALSE;	
         }

         /* Verify input vector was not modified by function */
         if ( !UtVector3d_IdenticalComp(&DuplicateInputVec, &InputVec) )
         {	
            UtLog_VarLine(UtDriver->Log, "TestVectorTransform Error: Step %d, Input vector was modified",Step);
            TestPassed = FALSE;	
         }

         /* Verify that function is capable of using same vector for input and output */
         Quaternion_VectorTransform(&InputVec, &InputQuat, &InputVec);
         if ( !UtVector3d_IdenticalComp(&InputVec, &CalculatedResultVec) )
         {
            UtLog_VarLine(UtDriver->Log, "TestVectorTransform Error: Step %d, Unable to use input vector as output",Step);
            TestPassed = FALSE;		
         }

      }  /* End while file data */

   } /* End if data file opened */
   else
   {
      UtLog_VarLine(UtDriver->Log, "Couldn't open test data: %s",TD_QUATDCM_FILE);
      TestPassed = FALSE;
   }

   UTASSERT_BOOL(TestPassed,"Quaternion_VectorTransform","Complete function");

} /* End TestVectorTransform() */


/*************************************************************************************
** Function: TestToAndFromEva
** Notes:
**  1. Generates a number of special case EVA sets and known associated quaternions
**  2. Converts EVA to quaternion with Quaternion_FromEva(); compares with expected result
**  3. Converts Quaternion to EVA with Quaternion_ToEva(); compares with expected result
**  4. In each case, verifies that input is not changed
**
*/
void TestToAndFromEva(void)
{		
   
   boolean    TestPassed = TRUE; 
   int        i, k;
   double     f, pi;
   double     InputEva[4];
   double     DuplcEva[4];
   double     OutputEva[4];
   Quaternion ExpectedQuat;	
   Quaternion OutputQuat;
   Quaternion DuplcQuat;
   Quaternion ExpectedQuat_Conjugate;
   Quaternion QDelta;
   
   pi = acos(-1.0);
   f = 1.0 / sqrt(2.0);

   k = 0;
   while ( k < 4 )
   {	
	  if ( k == 0 ) {
	      /* zero rotation */
          InputEva[0] = 1.0; 
          InputEva[1] = 0.0; 
          InputEva[2] = 0.0; 
          InputEva[3] = 0.0;
          ExpectedQuat.Comp[0] = 0.0;
          ExpectedQuat.Comp[1] = 0.0;
          ExpectedQuat.Comp[2] = 0.0;
          ExpectedQuat.Comp[3] = 1.0;

	  } else if ( k == 1 ) {
          /* pi/2 radians about x */
          InputEva[0] = 1.0; 
          InputEva[1] = 0.0; 
          InputEva[2] = 0.0; 
          InputEva[3] = 0.5 * pi;
          ExpectedQuat.Comp[0] = f;
          ExpectedQuat.Comp[1] = 0.0;
          ExpectedQuat.Comp[2] = 0.0;
          ExpectedQuat.Comp[3] = f;

	  } else if ( k == 2 ) {
          /* pi/2 radians about y */
          InputEva[0] = 0.0; 
          InputEva[1] = 1.0; 
          InputEva[2] = 0.0; 
          InputEva[3] = 0.5 * pi;
          ExpectedQuat.Comp[0] = 0.0;
          ExpectedQuat.Comp[1] = f;
          ExpectedQuat.Comp[2] = 0.0;
          ExpectedQuat.Comp[3] = f;

	  } else if ( k == 3 ) {
          /* pi/2 radians about z */
          InputEva[0] = 0.0; 
          InputEva[1] = 0.0; 
          InputEva[2] = 1.0; 
          InputEva[3] = 0.5 * pi;
          ExpectedQuat.Comp[0] = 0.0;
          ExpectedQuat.Comp[1] = 0.0;
          ExpectedQuat.Comp[2] = f;
          ExpectedQuat.Comp[3] = f;

	  }

	  k++;

	  for ( i=0; i<4; i++ ) DuplcEva[i] = InputEva[i];

	  Quaternion_Conjugate ( &ExpectedQuat_Conjugate, &ExpectedQuat ); 

	  Quaternion_FromEulerEigenVectorAngle( &OutputQuat, InputEva );

	  Quaternion_Mult ( &QDelta, &ExpectedQuat_Conjugate, &OutputQuat );

        /* QDelta should be identity quaternion */
        if ( fabs(QDelta.Comp[0])     > 1e-15 ||
             fabs(QDelta.Comp[1])     > 1e-15 ||
             fabs(QDelta.Comp[2])     > 1e-15 ||
             fabs(QDelta.Comp[3]-1.0) > 1e-15    )
        {
          UtLog_VarLine(UtDriver->Log, "TestFromEva() Error: Step %d",k);
          TestPassed = FALSE;
	  }
	  
	  /* input should remain unchanged */
        if ( fabs( DuplcEva[0] - InputEva[0] ) > 1e-15 ||
             fabs( DuplcEva[1] - InputEva[1] ) > 1e-15 ||
             fabs( DuplcEva[2] - InputEva[2] ) > 1e-15 ||
             fabs( DuplcEva[3] - InputEva[3] ) > 1e-15   )
        {
          UtLog_VarLine(UtDriver->Log, "TestFromEva() Error: Input changed at Step %d",k);
          TestPassed = FALSE;
	  }

	  for ( i=0; i<4; i++ ) DuplcQuat.Comp[i] = OutputQuat.Comp[i];

	  Quaternion_ToEulerEigenVectorAngle( OutputEva, &OutputQuat );

	  /* vector and angle should agree with original input,
	  ** unless angle is zero */
	  if ( InputEva[3] == 0.0 ) 
        {
          if ( OutputEva[3] != 0.0 ) 
          {
            UtLog_VarLine(UtDriver->Log, "TestToEva() Error: Step %d",k);
            TestPassed = FALSE;
          }
	  } 
	  else if ( fabs(OutputEva[0]-InputEva[0]) > 1e-15 ||
                fabs(OutputEva[1]-InputEva[1]) > 1e-15 ||
                fabs(OutputEva[2]-InputEva[2]) > 1e-15 ||
                fabs(OutputEva[3]-InputEva[3]) > 1e-15    )
        {
          UtLog_VarLine(UtDriver->Log, "TestToEva() Error: Step %d",k);
          TestPassed = FALSE;
	  }

	  /* input quaternion should remain unchanged */
        if ( fabs( DuplcQuat.Comp[0] - OutputQuat.Comp[0] ) > 1e-15 ||
             fabs( DuplcQuat.Comp[1] - OutputQuat.Comp[1] ) > 1e-15 ||
             fabs( DuplcQuat.Comp[2] - OutputQuat.Comp[2] ) > 1e-15 ||
             fabs( DuplcQuat.Comp[3] - OutputQuat.Comp[3] ) > 1e-15   )
        {
          UtLog_VarLine(UtDriver->Log, "TestToEva() Error: Input changed at Step %d",k);
          TestPassed = FALSE;
	  }

   }

   UTASSERT_BOOL(TestPassed,"Quaternion_FromAndToEva","Complete function");


} /* End TestToAndFromEulerEigenVectorAngle() */

/*************************************************************************************
** Function: TestToAndFromVector
** Notes:
**  1. Generates a number of special case rotation vectors and known associated quaternions
**  2. Converts vector to quaternion with Quaternion_FromVector(); compares with expected result
**  3. Converts Quaternion to vector with Quaternion_ToVector(); compares with expected result
**  4. In each case, verifies that input is not changed
**
*/
void TestToAndFromVector(void)
{		
   
   boolean    TestPassed = TRUE; 
   int        i, k;
   double     f, pi;
   Vector3d   InputVector;
   Vector3d   DuplcVector;
   Vector3d   OutputVector;
   Quaternion ExpectedQuat;	
   Quaternion OutputQuat;
   Quaternion DuplcQuat;
   Quaternion ExpectedQuat_Conjugate;
   Quaternion QDelta;
   
   pi = acos(-1.0);
   f = 1.0 / sqrt(2.0);

   k = 0;
   while ( k < 4 )
   {	
	  if ( k == 0 ) {
	      /* zero rotation */
          InputVector.Comp[0] = 0.0; 
		  InputVector.Comp[1] = 0.0; 
		  InputVector.Comp[2] = 0.0; 
          ExpectedQuat.Comp[0] = 0.0;
          ExpectedQuat.Comp[1] = 0.0;
          ExpectedQuat.Comp[2] = 0.0;
          ExpectedQuat.Comp[3] = 1.0;

	  } else if ( k == 1 ) {
          /* pi/2 radians about x */
          InputVector.Comp[0] = 0.5 * pi; 
		  InputVector.Comp[1] = 0.0; 
		  InputVector.Comp[2] = 0.0; 
          ExpectedQuat.Comp[0] = f;
          ExpectedQuat.Comp[1] = 0.0;
          ExpectedQuat.Comp[2] = 0.0;
          ExpectedQuat.Comp[3] = f;

	  } else if ( k == 2 ) {
          /* pi/2 radians about y */
          InputVector.Comp[0] = 0.0; 
		  InputVector.Comp[1] = 0.5 * pi;; 
		  InputVector.Comp[2] = 0.0; 
          ExpectedQuat.Comp[0] = 0.0;
          ExpectedQuat.Comp[1] = f;
          ExpectedQuat.Comp[2] = 0.0;
          ExpectedQuat.Comp[3] = f;

	  } else if ( k == 3 ) {
          /* pi/2 radians about z */
          InputVector.Comp[0] = 0.0; 
		  InputVector.Comp[1] = 0.0; 
		  InputVector.Comp[2] = 0.5 * pi; 
          ExpectedQuat.Comp[0] = 0.0;
          ExpectedQuat.Comp[1] = 0.0;
          ExpectedQuat.Comp[2] = f;
          ExpectedQuat.Comp[3] = f;

	  }

	  k++;

	  for ( i=0; i<3; i++ ) DuplcVector.Comp[i] = InputVector.Comp[i];

	  Quaternion_Conjugate ( &ExpectedQuat_Conjugate, &ExpectedQuat ); 

	  Quaternion_FromEulerVector( &OutputQuat, &InputVector );

	  Quaternion_Mult ( &QDelta, &ExpectedQuat_Conjugate, &OutputQuat );

      /* QDelta should be identity quaternion */
	  if ( fabs(QDelta.Comp[0])     > 1e-15 ||
		   fabs(QDelta.Comp[1])     > 1e-15 ||
		   fabs(QDelta.Comp[2])     > 1e-15 ||
		   fabs(QDelta.Comp[3]-1.0) > 1e-15    )
      {
        UtLog_VarLine(UtDriver->Log, "TestFromVector() Error: Step %d",k);
        TestPassed = FALSE;
	  }
	  
	  /* input should remain unchanged */
      if ( fabs( DuplcVector.Comp[0] - InputVector.Comp[0] ) > 1e-15 ||
		   fabs( DuplcVector.Comp[1] - InputVector.Comp[1] ) > 1e-15 ||
		   fabs( DuplcVector.Comp[2] - InputVector.Comp[2] ) > 1e-15    )
      {
        UtLog_VarLine(UtDriver->Log, "TestFromVector() Error: Input changed at Step %d",k);
        TestPassed = FALSE;
	  }

	  for ( i=0; i<4; i++ ) DuplcQuat.Comp[i] = OutputQuat.Comp[i];

	  Quaternion_ToEulerVector( &OutputVector, &OutputQuat );

	  /* vector should agree with original input */
	  if ( fabs(OutputVector.Comp[0]-InputVector.Comp[0]) > 1e-15 ||
		   fabs(OutputVector.Comp[1]-InputVector.Comp[1]) > 1e-15 ||
		   fabs(OutputVector.Comp[2]-InputVector.Comp[2]) > 1e-15   )
      {
        UtLog_VarLine(UtDriver->Log, "TestToVector() Error: Step %d",k);
        TestPassed = FALSE;
		UtLog_VarLine(UtDriver->Log, "IV: %f, %f, %f",InputVector.Comp[0],InputVector.Comp[1],InputVector.Comp[2]); 
		UtLog_VarLine(UtDriver->Log, "OV: %f, %f, %f",OutputVector.Comp[0],OutputVector.Comp[1],OutputVector.Comp[2]); 
	  }

	  /* input quaternion should remain unchanged */
      if ( fabs( DuplcQuat.Comp[0] - OutputQuat.Comp[0] ) > 1e-15 ||
		   fabs( DuplcQuat.Comp[1] - OutputQuat.Comp[1] ) > 1e-15 ||
		   fabs( DuplcQuat.Comp[2] - OutputQuat.Comp[2] ) > 1e-15 ||
		   fabs( DuplcQuat.Comp[3] - OutputQuat.Comp[3] ) > 1e-15   )
      {
        UtLog_VarLine(UtDriver->Log, "TestToVector() Error: Input changed at Step %d",k);
        TestPassed = FALSE;
	  }

   }

   UTASSERT_BOOL(TestPassed,"Quaternion_FromAndToVector","Complete function");


} /* End TestToAndFromEulerVector() */


/***************************************************************************
** Unit Test Function Database
*/

static UtDriver_Test TestArray[] = 
{

   { "Quaternion_Angle",               TestAngle                     },
   { "Quaternion_Conjugate",           TestConjugate                 },
   { "Quaternion_Copy",                TestCopy                      },
   { "Quaternion_Delta",               TestDelta                     },
   { "Quaternion_Identity",            TestIdentity                  },
   { "Quaternion_Magnitude",           TestMagnitude                 },
   { "Quaternion_Normalize",           TestNormalize                 },
   { "Quaternion_Standardize",         TestStandardize               },
   { "Quaternion_ToAndFromMatrix",     TestToAndFromMatrix           },
   { "Quaternion_ToAndFromMatrix",     TestToAndFromMatrixUsingFile  },
   { "Quaternion_Mult",                TestMult                      },
   { "Quaternion_MakeScalarPositive",  TestMakeScalarPositive        },
   { "Quaternion_Propagate",           TestPropagate                 },
   { "Quaternion_VectorTransform",     TestVectorTransform           },
   { "Quaternion_TestToAndFromEva",    TestToAndFromEva              },
   { "Quaternion_TestToAndFromVector", TestToAndFromVector           }

};

#define UT_TEST_CNT (sizeof(TestArray)/sizeof(UtDriver_Test))

/******************************************************************************
** Function: main
*/

int main (void)
{

 	int RetStatus = UTDRIVER_FAIL;  /* Assume failure */

   if ( UtDriver_ConstructorWithTests(UT_FILE_NAME, NULL, NULL, TestArray, UT_TEST_CNT) )
   {

      UtDriver_Execute();

      RetStatus = UtDriver_Destructor();

   } /* End if constructed */


   return RetStatus;

} /* End main() */

/* end of file */
