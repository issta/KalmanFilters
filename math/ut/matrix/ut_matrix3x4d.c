/*
** File:
** $Id: ut_matrix3x4d.c 1.1 2008/05/21 15:00:34EDT dcmccomas Exp  $
**
** Purpose: Unit test the Matrix3x4d component of the math library.
**
** References:
**   1. GN&C FSW Math Library Specification
**   2. GN&C FSW Developer Guide
**
** $Date: 2008/05/21 15:00:34EDT $
** $Revision: 1.1 $
** $Log: ut_matrix3x4d.c  $
** Revision 1.1 2008/05/21 15:00:34EDT dcmccomas 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/ut/matrix/project.pj
*/
/*
** Include Files
*/

#include <math.h>
#include <string.h>
#include "utdriver.h"
#include "utmatrix3x4d.h"
#include "utvector3d.h"

/*
** Macro Definitions
*/

#define UT_FILE_NAME   "ut_matrix3x4d"
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

   Matrix3x4d AMat, BMat, CMat, Expected;

   AMat.Comp[0][0] = 1.0e0;
   AMat.Comp[0][1] = 2.0e0;
   AMat.Comp[0][2] = 3.0e0;
   AMat.Comp[0][3] = 4.0e0;
   AMat.Comp[1][0] = 5.0e0;
   AMat.Comp[1][1] = 6.0e0;
   AMat.Comp[1][2] = 7.0e0;
   AMat.Comp[1][3] = 8.0e0;
   AMat.Comp[2][0] = 9.0e0;
   AMat.Comp[2][1] = 1.0e1;
   AMat.Comp[2][2] = 1.1e1;
   AMat.Comp[2][3] = 1.2e1;

   BMat.Comp[0][0] = 1.0e0;
   BMat.Comp[0][1] = 2.0e0;
   BMat.Comp[0][2] = 3.0e0;
   BMat.Comp[0][3] = 4.0e0;
   BMat.Comp[1][0] = 5.0e0;
   BMat.Comp[1][1] = 6.0e0;
   BMat.Comp[1][2] = 7.0e0;
   BMat.Comp[1][3] = 8.0e0;
   BMat.Comp[2][0] = 9.0e0;
   BMat.Comp[2][1] = 1.0e1;
   BMat.Comp[2][2] = 1.1e1;
   BMat.Comp[2][3] = 1.2e1;

   Expected.Comp[0][0] = 2.0e0;
   Expected.Comp[0][1] = 4.0e0;
   Expected.Comp[0][2] = 6.0e0;
   Expected.Comp[0][3] = 8.0e0;
   Expected.Comp[1][0] = 1.0e1;
   Expected.Comp[1][1] = 1.2e1;
   Expected.Comp[1][2] = 1.4e1;
   Expected.Comp[1][3] = 1.6e1;
   Expected.Comp[2][0] = 1.8e1;
   Expected.Comp[2][1] = 2.0e1;
   Expected.Comp[2][2] = 2.2e1;
   Expected.Comp[2][3] = 2.4e1;

   memset(&CMat, 0, sizeof(Matrix3x4d));

   Matrix3x4d_Add(&CMat, &AMat, &BMat);

   UTASSERT_BOOL(UtMatrix3x4d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix3x4d_Add","Add");

} /* End TestAdd() */

/******************************************************************************
** Function: TestCopy
*/
static void TestCopy(void)
{

   Matrix3x4d AMat, BMat;

   AMat.Comp[0][0] = 2.0e0;
   AMat.Comp[0][1] = 4.0e0;
   AMat.Comp[0][2] = 6.0e0;
   AMat.Comp[0][3] = 8.0e0;
   AMat.Comp[1][0] = 10.0e0;
   AMat.Comp[1][1] = 12.0e0;
   AMat.Comp[1][2] = 14.0e0;
   AMat.Comp[1][3] = 16.0e0;
   AMat.Comp[2][0] = 18.0e0;
   AMat.Comp[2][1] = 2.0e1;
   AMat.Comp[2][2] = 2.2e1;
   AMat.Comp[2][3] = 2.4e1;

   memset(&BMat, 0, sizeof(Matrix3x4d));

   Matrix3x4d_Copy(&BMat, &AMat);

   UTASSERT_BOOL(UtMatrix3x4d_IdenticalComp(&AMat, &BMat),"Matrix3x4d_Copy","Copy");

} /* End TestCopy() */


/******************************************************************************
** Function: TestDivScalar
*/
static void TestDivScalar(void)
{

   Matrix3x4d AMat, BMat, Expected;

   double scalar_factor = -2.0e0;

   AMat.Comp[0][0] = -2.22222e0;
   AMat.Comp[0][1] = -4.44444e0;
   AMat.Comp[0][2] = -6.66666e0;
   AMat.Comp[0][3] = 22.2222e0;
   AMat.Comp[1][0] = -8.88888e0;
   AMat.Comp[1][1] = -1.11111e1;
   AMat.Comp[1][2] = -1.333332e1;
   AMat.Comp[1][3] = 2.44444e1;
   AMat.Comp[2][0] = -1.555554e1;
   AMat.Comp[2][1] = -1.777776e1;
   AMat.Comp[2][2] = -1.999998e1;
   AMat.Comp[2][3] = 2.66666e1;

   Expected.Comp[0][0] = 1.11111e0;
   Expected.Comp[0][1] = 2.22222e0;
   Expected.Comp[0][2] = 3.33333e0;
   Expected.Comp[0][3] = -11.1111e0;
   Expected.Comp[1][0] = 4.44444e0;
   Expected.Comp[1][1] = 5.55555e0;
   Expected.Comp[1][2] = 6.66666e0;
   Expected.Comp[1][3] = -12.2222e0;
   Expected.Comp[2][0] = 7.77777e0;
   Expected.Comp[2][1] = 8.88888e0;
   Expected.Comp[2][2] = 9.99999e0;
   Expected.Comp[2][3] = -13.3333e0;

   memset(&BMat, 0, sizeof(Matrix3x4d));

   Matrix3x4d_DivScalar(&BMat, &AMat, scalar_factor);

   UTASSERT_BOOL(UtMatrix3x4d_EqualComp(&BMat, &Expected, UT_EQ_TOL),"Matrix3x4d_DivScalar","DivScalar");

} /* End TestDivScalar() */



/******************************************************************************
** Function: TestInitZero()
*/
static void TestInitZero(void)
{

   Matrix3x4d AMat, Expected;

   AMat.Comp[0][0] = -2.22222e0;
   AMat.Comp[0][1] = -4.44444e0;
   AMat.Comp[0][2] = -6.66666e0;
   AMat.Comp[0][3] = 22.2222e0;
   AMat.Comp[1][0] = -8.88888e0;
   AMat.Comp[1][1] = -1.11111e1;
   AMat.Comp[1][2] = -1.333332e1;
   AMat.Comp[1][3] = 2.44444e1;
   AMat.Comp[2][0] = -1.555554e1;
   AMat.Comp[2][1] = -1.777776e1;
   AMat.Comp[2][2] = -1.999998e1;
   AMat.Comp[2][3] = 2.66666e1;

   memset(&Expected, 0, sizeof(Matrix3x4d));

   Matrix3x4d_InitZero(&AMat);

   UTASSERT_BOOL(UtMatrix3x4d_IdenticalComp(&AMat, &Expected),"Matrix3x4d_InitZero","InitZero");

} /* End TestInitZero() */



/******************************************************************************
** Function: TestMultScalar
*/
static void TestMultScalar(void)
{

   Matrix3x4d AMat, BMat, Expected;
   double scalar_factor = -2.0e0;

   AMat.Comp[0][0] = 1.11111e0;
   AMat.Comp[0][1] = 2.22222e0;
   AMat.Comp[0][2] = 3.33333e0;
   AMat.Comp[0][3] = -11.1111e0;
   AMat.Comp[1][0] = 4.44444e0;
   AMat.Comp[1][1] = 5.55555e0;
   AMat.Comp[1][2] = 6.66666e0;
   AMat.Comp[1][3] = -12.2222e0;
   AMat.Comp[2][0] = 7.77777e0;
   AMat.Comp[2][1] = 8.88888e0;
   AMat.Comp[2][2] = 9.99999e0;
   AMat.Comp[2][3] = -13.3333e0;

   Expected.Comp[0][0] = -2.22222e0;
   Expected.Comp[0][1] = -4.44444e0;
   Expected.Comp[0][2] = -6.66666e0;
   Expected.Comp[0][3] = 22.2222e0;
   Expected.Comp[1][0] = -8.88888e0;
   Expected.Comp[1][1] = -1.11111e1;
   Expected.Comp[1][2] = -1.333332e1;
   Expected.Comp[1][3] = 2.44444e1;
   Expected.Comp[2][0] = -1.555554e1;
   Expected.Comp[2][1] = -1.777776e1;
   Expected.Comp[2][2] = -1.999998e1;
   Expected.Comp[2][3] = 2.66666e1;

   memset(&BMat, 0, sizeof(Matrix3x4d));

   Matrix3x4d_MultScalar(&BMat, &AMat, scalar_factor);

   UTASSERT_BOOL(UtMatrix3x4d_EqualComp(&BMat, &Expected, UT_EQ_TOL),"Matrix3x4d_MultScalar","MultScalar");

} /* End TestMultScalar() */


/******************************************************************************
** Function: TestMultVec
*/
static void TestMultVec(void)
{

   Matrix3x4d AMat;
   Vector4d BVec;
   Vector3d CVec, Expected;

   AMat.Comp[0][0] = 1.0e0;
   AMat.Comp[0][1] = 2.0e0;
   AMat.Comp[0][2] = 3.0e0;
   AMat.Comp[0][3] = 4.0e0;
   AMat.Comp[1][0] = 5.0e0;
   AMat.Comp[1][1] = 6.0e0;
   AMat.Comp[1][2] = 7.0e0;
   AMat.Comp[1][3] = 8.0e0;
   AMat.Comp[2][0] = 9.0e0;
   AMat.Comp[2][1] = 1.0e1;
   AMat.Comp[2][2] = 1.1e1;
   AMat.Comp[2][3] = 1.2e1;

   BVec.Comp[0] = 1.0e0;
   BVec.Comp[1] = 3.0e0;
   BVec.Comp[2] = 5.0e0;
   BVec.Comp[3] = 7.0e0;

   Expected.Comp[0] = 5.0e1;
   Expected.Comp[1] = 1.14e2;
   Expected.Comp[2] = 1.78e2;

   memset(&CVec, 0, sizeof(Vector3d));

   Matrix3x4d_MultVec(&CVec, &AMat, &BVec);

   UTASSERT_BOOL(UtVector3d_EqualComp(&CVec, &Expected, UT_EQ_TOL),"Matrix3x4d_MultVec","MultVec");

} /* End TestMultVec() */



/******************************************************************************
** Function: TestSub
*/
static void TestSub(void)
{

   Matrix3x4d AMat, BMat, CMat, Expected;

   AMat.Comp[0][0] = 2.0e0;
   AMat.Comp[0][1] = 4.0e0;
   AMat.Comp[0][2] = 6.0e0;
   AMat.Comp[0][3] = 8.0e0;
   AMat.Comp[1][0] = 1.0e1;
   AMat.Comp[1][1] = 1.2e1;
   AMat.Comp[1][2] = 1.4e1;
   AMat.Comp[1][3] = 1.6e1;
   AMat.Comp[2][0] = 1.8e1;
   AMat.Comp[2][1] = 2.0e1;
   AMat.Comp[2][2] = 2.2e1;
   AMat.Comp[2][3] = 2.4e1;

   BMat.Comp[0][0] = 1.0e0;
   BMat.Comp[0][1] = 2.0e0;
   BMat.Comp[0][2] = 3.0e0;
   BMat.Comp[0][3] = 4.0e0;
   BMat.Comp[1][0] = 5.0e0;
   BMat.Comp[1][1] = 6.0e0;
   BMat.Comp[1][2] = 7.0e0;
   BMat.Comp[1][3] = 8.0e0;
   BMat.Comp[2][0] = 9.0e0;
   BMat.Comp[2][1] = 1.0e1;
   BMat.Comp[2][2] = 1.1e1;
   BMat.Comp[2][3] = 1.2e1;

   Expected.Comp[0][0] = 1.0e0;
   Expected.Comp[0][1] = 2.0e0;
   Expected.Comp[0][2] = 3.0e0;
   Expected.Comp[0][3] = 4.0e0;
   Expected.Comp[1][0] = 5.0e0;
   Expected.Comp[1][1] = 6.0e0;
   Expected.Comp[1][2] = 7.0e0;
   Expected.Comp[1][3] = 8.0e0;
   Expected.Comp[2][0] = 9.0e0;
   Expected.Comp[2][1] = 1.0e1;
   Expected.Comp[2][2] = 1.1e1;
   Expected.Comp[2][3] = 1.2e1;

   memset(&CMat, 0, sizeof(Matrix3x4d));

   Matrix3x4d_Sub(&CMat, &AMat, &BMat);

   UTASSERT_BOOL(UtMatrix3x4d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix3x4d_Sub","Sub");

} /* End TestSub() */

/***************************************************************************
** Unit Test Function Database
*/

static UtDriver_Test TestArray[] = 
{
   { "Matrix3x4d_Add",        TestAdd        },
   { "Matrix3x4d_Copy",       TestCopy       },
   { "Matrix3x4d_DivScalar",  TestDivScalar  },
   { "Matrix3x4d_InitZero",   TestInitZero   },
   { "Matrix3x4d_MultScalar", TestMultScalar },
   { "Matrix3x4d_MultVec",    TestMultVec    },
   { "Matrix3x4d_Sub",        TestSub        }
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
