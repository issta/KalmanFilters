/*
** File:
** $Id: ut_scalar.c 1.1 2008/05/21 15:00:40EDT dcmccomas Exp  $
**
** Purpose: Unit test the scalar component of the math library.
**
** References:
**   1. GN&C FSW Math Library Specification
**   2. GN&C FSW Developer Guide
**
** $Date: 2008/05/21 15:00:40EDT $
** $Revision: 1.1 $
** $Log: ut_scalar.c  $
** Revision 1.1 2008/05/21 15:00:40EDT dcmccomas 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/ut/scalar/project.pj
*/
/*
** Include Files
*/

#include <math.h>
#include "scalar.h"

#include "utdriver.h"

/*
** Macro Definitions
*/

#define UT_FILE_NAME "ut_scalar"  /* No extension required */

typedef enum
{
   LOWER = -1,
   EQUAL =  0,
   UPPER =  1

} LimitStatusType;


/******************************************************************************
** Function: TestMacros 
*/
void TestMacros(void)
{

   UTASSERT_EQUAL(SCALAR_MAX(2,5), 5,"SCALAR_MAX","SCALAR_MAX( 2,5) == 5");
   UTASSERT_EQUAL(SCALAR_MAX(-5,5), 5,"SCALAR_MAX","SCALAR_MAX(-5,5) == 5");
   UTASSERT_EQUAL(SCALAR_MAX(-6.6,-2.2), -2.2,"SCALAR_MAX","SCALAR_MAX(-6.6,-2.2) == -2.2");

   UTASSERT_EQUAL(SCALAR_MIN(-10.1,-10.0), -10.1,"SCALAR_MIN","SCALAR_MIN(-10.1,-10.0) == -10.1");
   UTASSERT_EQUAL(SCALAR_MIN(-.1,.3), -.1,"SCALAR_MIN","SCALAR_MIN(-.1,.3) == -.1");
   UTASSERT_EQUAL(SCALAR_MIN(1000.1,1001), 1000.1,"SCALAR_MIN","SCALAR_MIN(1000.1,1001) == 1000.1");	

} /* End TestMacros() */

/******************************************************************************
** Function: TestLimit 
*/
void TestLimit(void){
	
	int    LimitStatus;
	double LimitValue;
	
	/* Both limits negative */
	
	LimitStatus = Scalar_Limit(&LimitValue, -20, -10, -5);

   UTASSERT_EQUAL(LimitStatus, LOWER,"Scalar_Limit()","LimitStatus == LOWER");
   UTASSERT_EQUAL(LimitValue, -10,"Scalar_Limit()","LimitValue == -10");
	

	LimitStatus = Scalar_Limit(&LimitValue, -7, -10, -5);

   UTASSERT_EQUAL(LimitStatus, EQUAL,"Scalar_Limit()","LimitStatus == EQUAL");
   UTASSERT_EQUAL(LimitValue, -7,"Scalar_Limit()","LimitValue == -7");	
	

   LimitStatus = Scalar_Limit(&LimitValue, -4.9, -10, -5);

   UTASSERT_EQUAL(LimitStatus, UPPER,"Scalar_Limit()","LimitStatus == UPPER");
   UTASSERT_EQUAL(LimitValue, -5,"Scalar_Limit()","LimitValue == 5");	
	
	/* Negative lower limit and Positive Upper Limit */
	
	LimitStatus = Scalar_Limit(&LimitValue, -6, -2, 4.5);
	
   UTASSERT_EQUAL(LimitStatus, LOWER,"Scalar_Limit()","LimitStatus == LOWER");
   UTASSERT_EQUAL(LimitValue, -2,"Scalar_Limit()","LimitValue == -2");


   LimitStatus = Scalar_Limit(&LimitValue, -1.9999, -2, 4.5);
	
   UTASSERT_EQUAL(LimitStatus, EQUAL,"Scalar_Limit()","LimitStatus == EQUAL");
   UTASSERT_EQUAL(LimitValue, -1.9999,"Scalar_Limit()","LimitValue == -1.9999");
	
	LimitStatus = Scalar_Limit(&LimitValue, 4.55, -2, 4.5);
	
   UTASSERT_EQUAL(LimitStatus, UPPER,"Scalar_Limit()","LimitStatus == UPPER");
   UTASSERT_EQUAL(LimitValue, 4.5,"Scalar_Limit()","LimitValue == 4.5");	
	
	/* both limits positive */
	
	LimitStatus = Scalar_Limit(&LimitValue, -5, 100, 110);
	
   UTASSERT_EQUAL(LimitStatus, LOWER,"Scalar_Limit()","LimitStatus == LOWER");
   UTASSERT_EQUAL(LimitValue, 100,"Scalar_Limit()","LimitValue == 100");


	LimitStatus = Scalar_Limit(&LimitValue, 101, 100, 110);
	
   UTASSERT_EQUAL(LimitStatus, EQUAL,"Scalar_Limit()","LimitStatus == EQUAL");
   UTASSERT_EQUAL(LimitValue, 101,"Scalar_Limit()","LimitValue == 101");


	LimitStatus = Scalar_Limit(&LimitValue, 1000, 100, 110);
	
   UTASSERT_EQUAL(LimitStatus, UPPER,"Scalar_Limit()","LimitStatus == UPPER");
   UTASSERT_EQUAL(LimitValue, 110,"Scalar_Limit()","LimitValue == 110");
	
	/* input equal to limits */
	
	LimitStatus = Scalar_Limit(&LimitValue, 100, 100, 110);
	
   UTASSERT_EQUAL(LimitStatus, EQUAL,"Scalar_Limit()","LimitStatus == EQUAL");
   UTASSERT_EQUAL(LimitValue, 100,"Scalar_Limit()","LimitValue == 100");


   LimitStatus = Scalar_Limit(&LimitValue, 110, 100, 110);
	
   UTASSERT_EQUAL(LimitStatus, EQUAL,"Scalar_Limit()","LimitStatus == EQUAL");
   UTASSERT_EQUAL(LimitValue, 110,"Scalar_Limit()","LimitValue == 110");
	
} /* End TestLimit() */

/***************************************************************************
** Unit Test Function Database
*/

static UtDriver_Test TestArray[] = 
{

   { "Macro Definitions", TestMacros },
   { "Scalar_Limit",      TestLimit  }

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

      RetStatus = UtDriver_Destructor();   /* Returns assertion status */

   } /* End if constructed */


   return RetStatus;

} /* End main() */

/* end of file */
