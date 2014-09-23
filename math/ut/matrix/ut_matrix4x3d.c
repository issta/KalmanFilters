/*
** File:
** $Id: ut_matrix4x3d.c 1.1 2008/05/21 15:00:35EDT dcmccomas Exp  $
**
** Purpose: Unit test the Matrix4x3d component of the math library.
**
** References:
**   1. GN&C FSW Math Library Specification
**   2. GN&C FSW Developer Guide
**
** $Date: 2008/05/21 15:00:35EDT $
** $Revision: 1.1 $
** $Log: ut_matrix4x3d.c  $
** Revision 1.1 2008/05/21 15:00:35EDT dcmccomas 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/ut/matrix/project.pj
*/
/*
** Include Files
*/

#include <math.h>
#include <string.h>
#include "utdriver.h"
#include "utmatrix4x3d.h"
#include "utvector4d.h"

/*
** Macro Definitions
*/

#define UT_FILE_NAME   "ut_matrix4x3d"
#define UT_EQ_TOL      1.0e-10
/*
** File Data
*/

static UtLog_Class      UtLogObj;
static UtAssert_Class   UtAssertObj;

/******************************************************************************
** Function: TestAdd
*/
static void TestAdd(void)
{

   Matrix4x3d AMat, BMat, CMat, Expected;

   AMat.Comp[0][0] = 1.0e0;
   AMat.Comp[0][1] = 2.0e0;
   AMat.Comp[0][2] = 3.0e0;  
   AMat.Comp[1][0] = 5.0e0;
   AMat.Comp[1][1] = 6.0e0;
   AMat.Comp[1][2] = 7.0e0;
   AMat.Comp[2][0] = 9.0e0;
   AMat.Comp[2][1] = 1.0e1;
   AMat.Comp[2][2] = 1.1e1;
   AMat.Comp[3][0] = 4.0e0;
   AMat.Comp[3][1] = 8.0e0;
   AMat.Comp[3][2] = 1.2e1;

   BMat.Comp[0][0] = 1.0e0;
   BMat.Comp[0][1] = 2.0e0;
   BMat.Comp[0][2] = 3.0e0;  
   BMat.Comp[1][0] = 5.0e0;
   BMat.Comp[1][1] = 6.0e0;
   BMat.Comp[1][2] = 7.0e0;
   BMat.Comp[2][0] = 9.0e0;
   BMat.Comp[2][1] = 1.0e1;
   BMat.Comp[2][2] = 1.1e1;
   BMat.Comp[3][0] = 4.0e0;
   BMat.Comp[3][1] = 8.0e0;
   BMat.Comp[3][2] = 1.2e1;

   Expected.Comp[0][0] = 2.0e0;
   Expected.Comp[0][1] = 4.0e0;
   Expected.Comp[0][2] = 6.0e0;  
   Expected.Comp[1][0] = 1.0e1;
   Expected.Comp[1][1] = 1.2e1;
   Expected.Comp[1][2] = 1.4e1;
   Expected.Comp[2][0] = 1.8e1;
   Expected.Comp[2][1] = 2.0e1;
   Expected.Comp[2][2] = 2.2e1;
   Expected.Comp[3][0] = 8.0e0;
   Expected.Comp[3][1] = 1.6e1;
   Expected.Comp[3][2] = 2.4e1;

   memset(&CMat, 0, sizeof(Matrix4x3d));

   Matrix4x3d_Add(&CMat, &AMat, &BMat);

   UTASSERT_BOOL(UtMatrix4x3d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix4x3d_Add","Add");

} /* End TestAdd() */



/******************************************************************************
** Function: TestCopy
*/
static void TestCopy(void)
{

   Matrix4x3d AMat, CMat;

   AMat.Comp[0][0] = -2.22222e0;
   AMat.Comp[0][1] = -4.44444e0;
   AMat.Comp[0][2] = -6.66666e0;  
   AMat.Comp[1][0] = -8.88888e0;
   AMat.Comp[1][1] = -1.11111e1;
   AMat.Comp[1][2] = -1.333332e1;
   AMat.Comp[2][0] = -1.555554e1;
   AMat.Comp[2][1] = -1.777776e1;
   AMat.Comp[2][2] = -1.999998e1;
   AMat.Comp[3][0] = 22.2222e0;
   AMat.Comp[3][1] = 24.4444e0;
   AMat.Comp[3][2] = 26.6666e0;

   memset(&CMat, 0, sizeof(Matrix4x3d));

   Matrix4x3d_Copy(&CMat, &AMat);

   UTASSERT_BOOL(UtMatrix4x3d_IdenticalComp(&AMat, &CMat),"Matrix4x3d_Copy","Copy");

} /* End TestCopy() */



/******************************************************************************
** Function: TestDivScalar
*/
static void TestDivScalar(void)
{

   Matrix4x3d AMat, CMat, Expected;
   double scalar_factor = -2.0e0;

   AMat.Comp[0][0] = -2.22222e0;
   AMat.Comp[0][1] = -4.44444e0;
   AMat.Comp[0][2] = -6.66666e0;  
   AMat.Comp[1][0] = -8.88888e0;
   AMat.Comp[1][1] = -1.11111e1;
   AMat.Comp[1][2] = -1.333332e1;
   AMat.Comp[2][0] = -1.555554e1;
   AMat.Comp[2][1] = -1.777776e1;
   AMat.Comp[2][2] = -1.999998e1;
   AMat.Comp[3][0] = 22.2222e0;
   AMat.Comp[3][1] = 24.4444e0;
   AMat.Comp[3][2] = 26.6666e0;

   Expected.Comp[0][0] = 1.11111e0;
   Expected.Comp[0][1] = 2.22222e0;
   Expected.Comp[0][2] = 3.33333e0;  
   Expected.Comp[1][0] = 4.44444e0;
   Expected.Comp[1][1] = 5.55555e0;
   Expected.Comp[1][2] = 6.66666e0;
   Expected.Comp[2][0] = 7.77777e0;
   Expected.Comp[2][1] = 8.88888e0;
   Expected.Comp[2][2] = 9.99999e0;
   Expected.Comp[3][0] = -11.1111e0;
   Expected.Comp[3][1] = -12.2222e0;
   Expected.Comp[3][2] = -13.3333e0;

   memset(&CMat, 0, sizeof(Matrix4x3d));

   Matrix4x3d_DivScalar(&CMat, &AMat, scalar_factor);

   UTASSERT_BOOL(UtMatrix4x3d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix4x3d_DivScalar","DivScalar");

} /* End TestDivScalar() */



/******************************************************************************
** Function: TestInitZero
*/
static void TestInitZero(void)
{

   Matrix4x3d AMat, Expected;

   AMat.Comp[0][0] = -2.22222e0;
   AMat.Comp[0][1] = -4.44444e0;
   AMat.Comp[0][2] = -6.66666e0;  
   AMat.Comp[1][0] = -8.88888e0;
   AMat.Comp[1][1] = -1.11111e1;
   AMat.Comp[1][2] = -1.333332e1;
   AMat.Comp[2][0] = -1.555554e1;
   AMat.Comp[2][1] = -1.777776e1;
   AMat.Comp[2][2] = -1.999998e1;
   AMat.Comp[3][0] = 22.2222e0;
   AMat.Comp[3][1] = 24.4444e0;
   AMat.Comp[3][2] = 26.6666e0;

   memset(&Expected, 0, sizeof(Matrix4x3d));
   
   Matrix4x3d_InitZero(&AMat);

   UTASSERT_BOOL(UtMatrix4x3d_IdenticalComp(&AMat, &Expected),"Matrix4x3d_InitZero","InitZero");

} /* End TestInitZero() */



/******************************************************************************
** Function: TestMultScalar
*/
static void TestMultScalar(void)
{

   Matrix4x3d AMat, BMat, Expected;
   double scalar_factor = -2.0e0;

   AMat.Comp[0][0] = 1.11111e0;
   AMat.Comp[0][1] = 2.22222e0;
   AMat.Comp[0][2] = 3.33333e0;  
   AMat.Comp[1][0] = 4.44444e0;
   AMat.Comp[1][1] = 5.55555e0;
   AMat.Comp[1][2] = 6.66666e0;
   AMat.Comp[2][0] = 7.77777e0;
   AMat.Comp[2][1] = 8.88888e0;
   AMat.Comp[2][2] = 9.99999e0;
   AMat.Comp[3][0] = -11.1111e0;
   AMat.Comp[3][1] = -12.2222e0;
   AMat.Comp[3][2] = -13.3333e0;

   Expected.Comp[0][0] = -2.22222e0;
   Expected.Comp[0][1] = -4.44444e0;
   Expected.Comp[0][2] = -6.66666e0;
   Expected.Comp[1][0] = -8.88888e0;
   Expected.Comp[1][1] = -1.11111e1;
   Expected.Comp[1][2] = -1.333332e1;
   Expected.Comp[2][0] = -1.555554e1;
   Expected.Comp[2][1] = -1.777776e1;
   Expected.Comp[2][2] = -1.999998e1;
   Expected.Comp[3][0] = 22.2222e0;
   Expected.Comp[3][1] = 24.4444e0;
   Expected.Comp[3][2] = 26.6666e0;

   memset(&BMat, 0, sizeof(Matrix4x3d));

   Matrix4x3d_MultScalar(&BMat, &AMat, scalar_factor);

   UTASSERT_BOOL(UtMatrix4x3d_EqualComp(&BMat, &Expected, UT_EQ_TOL),"Matrix4x3d_MultScalar","MultScalar");

} /* End TestMultScalar() */




/******************************************************************************
** Function: TestMultVec
*/
static void TestMultVec(void)
{

   Matrix4x3d BMat;
   Vector3d BVec;
   Vector4d CVec, Expected;

   BMat.Comp[0][0] = 1.0e0;
   BMat.Comp[0][1] = 2.0e0;
   BMat.Comp[0][2] = 3.0e0;  
   BMat.Comp[1][0] = 5.0e0;
   BMat.Comp[1][1] = 6.0e0;
   BMat.Comp[1][2] = 7.0e0;
   BMat.Comp[2][0] = 9.0e0;
   BMat.Comp[2][1] = 1.0e1;
   BMat.Comp[2][2] = 1.1e1;
   BMat.Comp[3][0] = 4.0e0;
   BMat.Comp[3][1] = 8.0e0;
   BMat.Comp[3][2] = 1.2e1;

   BVec.Comp[0] = 1.0e0;
   BVec.Comp[1] = 3.0e0;
   BVec.Comp[2] = 5.0e0;

   Expected.Comp[0] = 2.2e1;
   Expected.Comp[1] = 5.8e1;
   Expected.Comp[2] = 9.4e1;
   Expected.Comp[3] = 8.8e1;

   memset(&CVec, 0, sizeof(Vector4d));

   Matrix4x3d_MultVec(&CVec, &BMat, &BVec);
   
   UTASSERT_BOOL(UtVector4d_EqualComp(&CVec, &Expected, UT_EQ_TOL),"Matrix4x3d_MultVec","MultVec");

} /* End TestMultVec() */



/******************************************************************************
** Function: TestSub
*/
static void TestSub(void)
{

   Matrix4x3d AMat, BMat, CMat, Expected;

   AMat.Comp[0][0] = 2.0e0;
   AMat.Comp[0][1] = 4.0e0;
   AMat.Comp[0][2] = 6.0e0;  
   AMat.Comp[1][0] = 1.0e1;
   AMat.Comp[1][1] = 1.2e1;
   AMat.Comp[1][2] = 1.4e1;
   AMat.Comp[2][0] = 1.8e1;
   AMat.Comp[2][1] = 2.0e1;
   AMat.Comp[2][2] = 2.2e1;
   AMat.Comp[3][0] = 8.0e0;
   AMat.Comp[3][1] = 1.6e1;
   AMat.Comp[3][2] = 2.4e1;

   BMat.Comp[0][0] = 1.0e0;
   BMat.Comp[0][1] = 2.0e0;
   BMat.Comp[0][2] = 3.0e0;  
   BMat.Comp[1][0] = 5.0e0;
   BMat.Comp[1][1] = 6.0e0;
   BMat.Comp[1][2] = 7.0e0;
   BMat.Comp[2][0] = 9.0e0;
   BMat.Comp[2][1] = 1.0e1;
   BMat.Comp[2][2] = 1.1e1;	
   BMat.Comp[3][0] = 4.0e0;
   BMat.Comp[3][1] = 8.0e0;
   BMat.Comp[3][2] = 1.2e1;

   Expected.Comp[0][0] = 1.0e0;
   Expected.Comp[0][1] = 2.0e0;
   Expected.Comp[0][2] = 3.0e0;  
   Expected.Comp[1][0] = 5.0e0;
   Expected.Comp[1][1] = 6.0e0;
   Expected.Comp[1][2] = 7.0e0;
   Expected.Comp[2][0] = 9.0e0;
   Expected.Comp[2][1] = 1.0e1;
   Expected.Comp[2][2] = 1.1e1;
   Expected.Comp[3][0] = 4.0e0;
   Expected.Comp[3][1] = 8.0e0;
   Expected.Comp[3][2] = 1.2e1;

   memset(&CMat, 0, sizeof(Matrix4x3d));

   Matrix4x3d_Sub(&CMat, &AMat, &BMat);

   UTASSERT_BOOL(UtMatrix4x3d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix4x3d_Sub","Sub");

} /* End TestSub() */


/***************************************************************************
** Unit Test Function Database
*/
static UtDriver_Test TestArray[] = 
{
   { "Matrix4x3d_Add",        TestAdd        },
   { "Matrix4x3d_Copy",       TestCopy       },
   { "Matrix4x3d_DivScalar",  TestDivScalar  },
   { "Matrix4x3d_InitZero",   TestInitZero   },
   { "Matrix4x3d_MultScalar", TestMultScalar },
   { "Matrix4x3d_MultVec",    TestMultVec    },
   { "Matrix4x3d_Sub",        TestSub        }
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
