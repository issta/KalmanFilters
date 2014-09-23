/*
** File:
** $Id: ut_mathconstants.c 1.1 2008/05/21 15:00:32EDT dcmccomas Exp  $
**
** Purpose: Unit test the mathconstants component of the math library.
**
** References:
**   1. GN&C FSW Math Library Specification
**   2. GN&C FSW Developer Guide
**
** $Date: 2008/05/21 15:00:32EDT $
** $Revision: 1.1 $
** $Log: ut_mathconstants.c  $
** Revision 1.1 2008/05/21 15:00:32EDT dcmccomas 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/ut/mathconstants/project.pj
*/

/*
** Include Files
*/

#include <math.h>
#include "mathconstants.h"

#include "utdriver.h"

/*
** Macro Definitions
*/

#define UT_FILE_NAME "ut_mathconstants"


/******************************************************************************
** Function: TestMacros 
*/
void TestMacros(void)
{

	double   LocalPi = acos(-1.0);
   double   TestValue;
   double   Tolerance = 1e-15;

   UTASSERT_EQUAL(cos(MTH_PI), -1.0,"MTH_PI","cos(MTH_PI) == -1.0");

   UTASSERT_EQUAL(cos(MTH_TWO_PI), 1.0,"MTH_TWO_PI","cos(MTH_TWO_PI) == 1.0");


   UTASSERT_EQUAL(sin(MTH_HALF_PI), 1.0,"MTH_HALF_PI","sin(MTH_HALF_PI) == 1.0");

	TestValue = 180.0 * MTH_RADS_PER_DEG;
   UTASSERT_EQUAL_TOL(TestValue,LocalPi,Tolerance,"MTH_RADS_PER_DEG","(MTH_RADS_PER_DEG*180) == pi");
	
	
	TestValue = LocalPi * MTH_DEGS_PER_RAD;
   UTASSERT_EQUAL_TOL(TestValue,180.0,Tolerance,"MTH_DEGS_PER_RAD","(MTH_DEGS_PER_RAD*pi) == 180.0");

	TestValue = MTH_RADS_PER_ARCSEC * 3600 * 180;
   UTASSERT_EQUAL_TOL(TestValue,LocalPi,Tolerance,"MTH_RADS_PER_ARCSEC","(MTH_RADS_PER_ARCSEC * 3600 * 180) == pi");


	TestValue     = MTH_ARCSECS_PER_RAD * LocalPi / 180;
   UTASSERT_EQUAL_TOL(TestValue,3600,Tolerance,"MTH_ARCSECS_PER_RAD","(MTH_ARCSECS_PER_RAD * pi / 180) == 3600");


   TestValue     = MTH_SECS_PER_DAY * MTH_DAYS_PER_CENTURY;
   UTASSERT_EQUAL_TOL(TestValue,MTH_SECS_PER_CENTURY,Tolerance,"MTH_SECS_PER_DAY, MTH_DAYS_PER_CENTURY, MTH_SECS_PER_CENTURY","(MTH_SECS_PER_DAY * MTH_DAYS_PER_CENTURY) == MTH_SECS_PER_CENTURY");

	
} /* TestMacros () */

/***************************************************************************
** Unit Test Function Database
*/

static UtDriver_Test TestArray[] = 
{

   { "Macro Definitions", TestMacros }

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
