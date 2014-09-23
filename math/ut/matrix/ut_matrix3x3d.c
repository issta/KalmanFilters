/*
** File:
** $Id: ut_matrix3x3d.c 1.2 2008/09/10 10:11:30EDT gwelter Exp  $
**
** Purpose: Unit test the Matrix3x3d component of the math library.
**
** References:
**   1. GN&C FSW Math Library Specification
**   2. GN&C FSW Developer Guide
**
** $Date: 2008/09/10 10:11:30EDT $
** $Revision: 1.2 $
** $Log: ut_matrix3x3d.c  $.pj
** Revision 1.2 2008/09/10 10:11:30EDT gwelter .pj
** pre-gpm updates.pj
**
** Revision 2.0 2008/05/21 15:00:33EDT dcmccomas 
** Revision for new MKS
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/ut/matrix/project 
** Revision 2.0 2008/06/23 07:00:00EDT gwelter
**  incorporated tests for new matrix functions
*/
/*
** Include Files
*/

#include <math.h>
#include <string.h>
#include "quaternion.h"
#include "utdriver.h"
#include "utmatrix3x3d.h"
#include "utvector3d.h"

/*
** Macro Definitions
*/

#define UT_FILE_NAME   "ut_matrix3x3d"
#define UT_EQ_TOL      1.0e-10

/******************************************************************************
** Function: TestAdd
*/
static void TestAdd(void)
{

   Matrix3x3d AMat, BMat, CMat, Expected;

   AMat.Comp[0][0] = 1.0e0;
   AMat.Comp[0][1] = 2.0e0;
   AMat.Comp[0][2] = 3.0e0;
   AMat.Comp[1][0] = 4.0e0;
   AMat.Comp[1][1] = 5.0e0;
   AMat.Comp[1][2] = 6.0e0;
   AMat.Comp[2][0] = 7.0e0;
   AMat.Comp[2][1] = 8.0e0;
   AMat.Comp[2][2] = 9.0e0;

   BMat.Comp[0][0] = 10.0e0;
   BMat.Comp[0][1] = 11.0e0;
   BMat.Comp[0][2] = 12.0e0;
   BMat.Comp[1][0] = 13.0e0;
   BMat.Comp[1][1] = 14.0e0;
   BMat.Comp[1][2] = 15.0e0;
   BMat.Comp[2][0] = 16.0e0;
   BMat.Comp[2][1] = 17.0e0;
   BMat.Comp[2][2] = 19.0e0;

   Expected.Comp[0][0] = 11.0e0;
   Expected.Comp[0][1] = 13.0e0;
   Expected.Comp[0][2] = 15.0e0;
   Expected.Comp[1][0] = 17.0e0;
   Expected.Comp[1][1] = 19.0e0;
   Expected.Comp[1][2] = 21.0e0;
   Expected.Comp[2][0] = 23.0e0;
   Expected.Comp[2][1] = 25.0e0;
   Expected.Comp[2][2] = 28.0e0;
	
   memset(&CMat,0,sizeof(Matrix3x3d));

   Matrix3x3d_Add(&CMat, &BMat, &AMat);

   UTASSERT_BOOL(UtMatrix3x3d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix3x3d_Add","Add");

} /* End TestAdd() */


/******************************************************************************
** Function: TestColVecToMatrix
*/
static void TestColVecToMatrix(void)
{

   Vector3d   V1, V2, V3;
   Matrix3x3d Result, Expected;

   V1.Comp[0] = 1.1;
   V1.Comp[1] = 1.2;
   V1.Comp[2] = 1.3;
   V2.Comp[0] = 2.1;
   V2.Comp[1] = 2.2;
   V2.Comp[2] = 2.3;
   V3.Comp[0] = 3.1;
   V3.Comp[1] = 3.2;
   V3.Comp[2] = 3.3;

   Expected.Comp[0][0] = 1.1;
   Expected.Comp[0][1] = 2.1;
   Expected.Comp[0][2] = 3.1;
   Expected.Comp[1][0] = 1.2;
   Expected.Comp[1][1] = 2.2;
   Expected.Comp[1][2] = 3.2;
   Expected.Comp[2][0] = 1.3;
   Expected.Comp[2][1] = 2.3;
   Expected.Comp[2][2] = 3.3;
	
	memset(&Result,0,sizeof(Matrix3x3d));

   Matrix3x3d_ColVecToMatrix(&Result, &V1, &V2, &V3);

   UTASSERT_BOOL(UtMatrix3x3d_EqualComp(&Result, &Expected, UT_EQ_TOL),"Matrix3x3d_ColVecToMatrix","ColVecToMatrix");

} /* End TestColVecToMatrix() */


/******************************************************************************
** Function: TestCopy
*/
static void TestCopy(void)
{

	Matrix3x3d AMat, BMat;

   AMat.Comp[0][0] = 1.0e0;
   AMat.Comp[0][1] = 2.0e0;
   AMat.Comp[0][2] = 3.0e0;
   AMat.Comp[1][0] = 4.0e0;
   AMat.Comp[1][1] = 5.0e0;
   AMat.Comp[1][2] = 6.0e0;
   AMat.Comp[2][0] = 7.0e0;
   AMat.Comp[2][1] = 8.0e0;
   AMat.Comp[2][2] = 9.0e0;
	
	memset(&BMat,0,sizeof(Matrix3x3d));

	Matrix3x3d_Copy(&BMat, &AMat);

   UTASSERT_BOOL(UtMatrix3x3d_IdenticalComp(&AMat, &BMat),"Matrix3x3d_Copy","Copy");

} /* End TestCopy() */

/******************************************************************************
** Function: TestDivScalar
*/
static void TestDivScalar(void)
{

   Matrix3x3d AMat, CMat, Expected;	
   double scalar_factor = -2.0e2;	

   AMat.Comp[0][0] = 1.11111e0;	
   AMat.Comp[0][1] = 2.22222e0;	
   AMat.Comp[0][2] = 3.33333e0;	
   AMat.Comp[1][0] = 7.77777e0;	
   AMat.Comp[1][1] = 8.88888e0;	
   AMat.Comp[1][2] = 9.99999e0;	
   AMat.Comp[2][0] = 4.44444e1;	
   AMat.Comp[2][1] = 5.55555e1;	
   AMat.Comp[2][2] = 6.66666e1;	
   Expected.Comp[0][0] = -5.55555e-3;
   Expected.Comp[0][1] = -1.11111e-2;	
   Expected.Comp[0][2] = -1.666665e-2;
   Expected.Comp[1][0] = -3.888885e-2;	
   Expected.Comp[1][1] = -4.44444e-2;	
   Expected.Comp[1][2] = -4.999995e-2;	
   Expected.Comp[2][0] = -2.22222e-1;	
   Expected.Comp[2][1] = -2.777775e-1;	
   Expected.Comp[2][2] = -3.33333e-1;	
   
   memset(&CMat, 0, sizeof(Matrix3x3d));
   
   Matrix3x3d_DivScalar(&CMat, &AMat, scalar_factor);	
   
   UTASSERT_BOOL(UtMatrix3x3d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix3x3d_DivScalar","DivScalar");

} /* End TestDivScalar() */


/******************************************************************************
** Function: TestInitIdentity
*/
static void TestInitIdentity(void)
{

   Matrix3x3d CMat, Expected;	
   
   Expected.Comp[0][0] = 1.0e0;	
   Expected.Comp[0][1] = 0.0e0;	
   Expected.Comp[0][2] = 0.0e0;	
   Expected.Comp[1][0] = 0.0e0;	
   Expected.Comp[1][1] = 1.0e0;	
   Expected.Comp[1][2] = 0.0e0;	
   Expected.Comp[2][0] = 0.0e0;	
   Expected.Comp[2][1] = 0.0e0;	
   Expected.Comp[2][2] = 1.0e0;	
   
   memset(&CMat, 0xFF, sizeof(Matrix3x3d));
   
   Matrix3x3d_InitIdentity(&CMat);
   
   UTASSERT_BOOL(UtMatrix3x3d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix3x3d_InitIdentity","InitIdentity");

} /* End TestInitIdentity() */


/******************************************************************************
** Function: TestInitZero
*/
static void TestInitZero(void)
{

   Matrix3x3d CMat, Expected;	

   Expected.Comp[0][0] = 0.0e0;
   Expected.Comp[0][1] = 0.0e0;
   Expected.Comp[0][2] = 0.0e0;
   Expected.Comp[1][0] = 0.0e0;	
   Expected.Comp[1][1] = 0.0e0;	
   Expected.Comp[1][2] = 0.0e0;	
   Expected.Comp[2][0] = 0.0e0;	
   Expected.Comp[2][1] = 0.0e0;	
   Expected.Comp[2][2] = 0.0e0;	
   
   memset(&CMat, 0xFF, sizeof(Matrix3x3d));
   
   Matrix3x3d_InitZero(&CMat);
   
   UTASSERT_BOOL(UtMatrix3x3d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix3x3d_InitZero","InitZero");

} /* End TestInitZero() */


/******************************************************************************
** Function: TestMult
*/
static void TestMult(void)
{

   Matrix3x3d AMat, BMat, CMat, Expected;

   AMat.Comp[0][0] = 1.111111e0;
   AMat.Comp[0][1] = 2.222222e0;
   AMat.Comp[0][2] = 3.333333e0;
   AMat.Comp[1][0] = 4.444444e0;
   AMat.Comp[1][1] = 5.555555e0;
   AMat.Comp[1][2] = 6.666666e0;
   AMat.Comp[2][0] = 7.777777e0;
   AMat.Comp[2][1] = 8.888888e0;
   AMat.Comp[2][2] = 9.999999e0;

   BMat.Comp[0][0] = 1.11111e0;
   BMat.Comp[0][1] = 0.00000e0;
   BMat.Comp[0][2] = 0.00000e0;
   BMat.Comp[1][0] = 0.00000e0;
   BMat.Comp[1][1] = 1.11111e0;
   BMat.Comp[1][2] = 0.00000e0;
   BMat.Comp[2][0] = 0.00000e0;
   BMat.Comp[2][1] = 0.00000e0;
   BMat.Comp[2][2] = 1.11111e0;

   Expected.Comp[0][0] = 1.2345665432099999e0;
   Expected.Comp[0][1] = 2.4691330864199998e0;
   Expected.Comp[0][2] = 3.7036996296300004e0;
   Expected.Comp[1][0] = 4.9382661728399997e0;
   Expected.Comp[1][1] = 6.1728327160500003e0;
   Expected.Comp[1][2] = 7.4073992592600009e0;
   Expected.Comp[2][0] = 8.6419658024700006e0;
   Expected.Comp[2][1] = 9.8765323456799994e0;
   Expected.Comp[2][2] = 11.111098888900002e0;
	
   memset(&CMat,0,sizeof(Matrix3x3d));

   Matrix3x3d_Mult(&CMat, &BMat, &AMat);

   UTASSERT_BOOL(UtMatrix3x3d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix3x3d_Mult","Mult");

} /* End TestMult() */


/******************************************************************************
** Function: TestMultScalar
*/
static void TestMultScalar(void){

   Matrix3x3d AMat, CMat, Expected;

   double scalar_factor = -1.0e0;

   AMat.Comp[0][0] = 1.0e0;
   AMat.Comp[0][1] = 2.0e0;
   AMat.Comp[0][2] = 3.0e0;
   AMat.Comp[1][0] = 4.0e0;
   AMat.Comp[1][1] = 5.0e0;
   AMat.Comp[1][2] = 6.0e0;
   AMat.Comp[2][0] = 7.0e0;
   AMat.Comp[2][1] = 8.0e0;
   AMat.Comp[2][2] = 9.0e0;

   Expected.Comp[0][0] = -1.0e0;
   Expected.Comp[0][1] = -2.0e0;
   Expected.Comp[0][2] = -3.0e0;
   Expected.Comp[1][0] = -4.0e0;
   Expected.Comp[1][1] = -5.0e0;
   Expected.Comp[1][2] = -6.0e0;
   Expected.Comp[2][0] = -7.0e0;
   Expected.Comp[2][1] = -8.0e0;
   Expected.Comp[2][2] = -9.0e0;
	
   memset(&CMat,0,sizeof(Matrix3x3d));

   Matrix3x3d_MultScalar(&CMat, &AMat, scalar_factor);

   UTASSERT_BOOL(UtMatrix3x3d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix3x3d_MultScalar","MultScalar");

} /* End TestMultScalar() */


/******************************************************************************
** Function: TesttMultVec
*/
static void TestMultVec(void)
{

   Matrix3x3d AMat;
   Vector3d   BVec, CVec, Expected;

   AMat.Comp[0][0] = 1.0e0;
   AMat.Comp[0][1] = 2.0e0;
   AMat.Comp[0][2] = 3.0e0;
   AMat.Comp[1][0] = 4.0e0;
   AMat.Comp[1][1] = 5.0e0;
   AMat.Comp[1][2] = 6.0e0;
   AMat.Comp[2][0] = 7.0e0;
   AMat.Comp[2][1] = 8.0e0;
   AMat.Comp[2][2] = 9.0e0;

   BVec.Comp[0] = 1.0e0;
   BVec.Comp[1] = 2.0e0;
   BVec.Comp[2] = 3.0e0;

   Expected.Comp[0] = 1.4e1;
   Expected.Comp[1] = 3.20e1;
   Expected.Comp[2] = 5.00e1;

   memset(&CVec,0,sizeof(Vector3d));

   Matrix3x3d_MultVec(&CVec, &AMat, &BVec);

   UTASSERT_BOOL(UtVector3d_EqualComp(&CVec, &Expected, UT_EQ_TOL),"Matrix3x3d_MultVec","MultVec");

} /* End TesttMultVec() */


/******************************************************************************
** Function: TestRowVecToMatrix
*/
static void TestRowVecToMatrix(void)
{

   Vector3d   V1, V2, V3;
   Matrix3x3d Result, Expected;

   V1.Comp[0] = 1.1;
   V1.Comp[1] = 1.2;
   V1.Comp[2] = 1.3;
   V2.Comp[0] = 2.1;
   V2.Comp[1] = 2.2;
   V2.Comp[2] = 2.3;
   V3.Comp[0] = 3.1;
   V3.Comp[1] = 3.2;
   V3.Comp[2] = 3.3;

   Expected.Comp[0][0] = 1.1;
   Expected.Comp[0][1] = 1.2;
   Expected.Comp[0][2] = 1.3;
   Expected.Comp[1][0] = 2.1;
   Expected.Comp[1][1] = 2.2;
   Expected.Comp[1][2] = 2.3;
   Expected.Comp[2][0] = 3.1;
   Expected.Comp[2][1] = 3.2;
   Expected.Comp[2][2] = 3.3;
	
	memset(&Result,0,sizeof(Matrix3x3d));

   Matrix3x3d_RowVecToMatrix(&Result, &V1, &V2, &V3);

   UTASSERT_BOOL(UtMatrix3x3d_EqualComp(&Result, &Expected, UT_EQ_TOL),"Matrix3x3d_RowVecToMatrix","RowVecToMatrix");

} /* End TestRowVecToMatrix() */


/******************************************************************************
** Function: TestSub
*/
static void TestSub(void)
{

   Matrix3x3d AMat, BMat, CMat, Expected;

   AMat.Comp[0][0] = 2.00000e0;
   AMat.Comp[0][1] = 1.20000e1;
   AMat.Comp[0][2] = 1.03000e2;
   AMat.Comp[1][0] = 1.20000e1;
   AMat.Comp[1][1] = 4.00000e1;
   AMat.Comp[1][2] = 2.30000e2;
   AMat.Comp[2][0] = 1.03000e2;
   AMat.Comp[2][1] = 2.30000e2;
   AMat.Comp[2][2] = 6.00000e2;
   BMat.Comp[0][0] = 1.0e0;
   BMat.Comp[0][1] = 1.0e1;
   BMat.Comp[0][2] = 1.0e2;
   BMat.Comp[1][0] = 2.0e0;
   BMat.Comp[1][1] = 2.0e1;
   BMat.Comp[1][2] = 2.0e2;
   BMat.Comp[2][0] = 3.0e0;
   BMat.Comp[2][1] = 3.0e1;
   BMat.Comp[2][2] = 3.0e2;
   Expected.Comp[0][0] = 1.0e0;
   Expected.Comp[0][1] = 2.0e0;
   Expected.Comp[0][2] = 3.0e0;
   Expected.Comp[1][0] = 1.0e1;
   Expected.Comp[1][1] = 2.0e1;
   Expected.Comp[1][2] = 3.0e1;
   Expected.Comp[2][0] = 1.0e2;
   Expected.Comp[2][1] = 2.0e2;
   Expected.Comp[2][2] = 3.0e2;

   memset(&CMat, 0, sizeof(Matrix3x3d));
   
   Matrix3x3d_Sub(&CMat, &AMat, &BMat);
   
   UTASSERT_BOOL(UtMatrix3x3d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix3x3d_Sub","Sub");

} /* End TestSub() */

/******************************************************************************
** Function: TestTrace
*/
static void TestTrace(void)
{
   Matrix3x3d CMat;
   double Expected;

   CMat.Comp[0][0] = 2.00000e0;
   CMat.Comp[0][1] = 1.20000e1;
   CMat.Comp[0][2] = 1.03000e2;
   CMat.Comp[1][0] = 1.20000e5;
   CMat.Comp[1][1] = 4.00000e1;
   CMat.Comp[1][2] = 2.30000e2;
   CMat.Comp[2][0] = 1.03000e4;
   CMat.Comp[2][1] = 2.30000e3;
   CMat.Comp[2][2] = 6.00000e2;
   
   Expected = Matrix3x3d_Trace(&CMat);
   
   UTASSERT_EQUAL(642, Expected, "Matrix3x3d_Trace","Trace");

} /* End TestTrace() */

/******************************************************************************
** Function: TestTranspose
*/
static void TestTranspose(void)
{

   Matrix3x3d AMat, BMat, Expected;

   AMat.Comp[0][0] = 1.0e0;
   AMat.Comp[0][1] = 2.0e0;
   AMat.Comp[0][2] = 3.0e0;
   AMat.Comp[1][0] = 4.0e0;
   AMat.Comp[1][1] = 5.0e0;
   AMat.Comp[1][2] = 6.0e0;
   AMat.Comp[2][0] = 7.0e0;
   AMat.Comp[2][1] = 8.0e0;
   AMat.Comp[2][2] = 9.0e0;

   Expected.Comp[0][0] = 1.0e0;
   Expected.Comp[0][1] = 4.0e0;
   Expected.Comp[0][2] = 7.0e0;
   Expected.Comp[1][0] = 2.0e0;
   Expected.Comp[1][1] = 5.0e0;
   Expected.Comp[1][2] = 8.0e0;
   Expected.Comp[2][0] = 3.0e0;
   Expected.Comp[2][1] = 6.0e0;
   Expected.Comp[2][2] = 9.0e0;
	
   memset(&BMat,0,sizeof(Matrix3x3d));

   Matrix3x3d_Transpose(&BMat, &AMat);

   UTASSERT_BOOL(UtMatrix3x3d_EqualComp(&BMat, &Expected, UT_EQ_TOL),"Matrix3x3d_Transpose","Transpose");

} /* End TestTranspose() */

/******************************************************************************
** Function: TestSkew
*/
static void TestSkew(void)
{
   Vector3d InputVector;
   
   Matrix3x3d SkewMat, ExpectedMat;

   InputVector.Comp[0] = 1.0e0;
   InputVector.Comp[1] = 2.0e0;
   InputVector.Comp[2] = 3.0e0;

   ExpectedMat.Comp[0][0] = 0.0e0;
   ExpectedMat.Comp[1][1] = 0.0e0;
   ExpectedMat.Comp[2][2] = 0.0e0;

   ExpectedMat.Comp[0][1] =  3.0e0;
   ExpectedMat.Comp[1][0] = -3.0e0;

   ExpectedMat.Comp[0][2] = -2.0e0;
   ExpectedMat.Comp[2][0] =  2.0e0;

   ExpectedMat.Comp[1][2] =  1.0e0;
   ExpectedMat.Comp[2][1] = -1.0e0;

   Matrix3x3d_Skew(&SkewMat, &InputVector);


   UTASSERT_BOOL(UtMatrix3x3d_EqualComp(&SkewMat, &ExpectedMat, UT_EQ_TOL),"Matrix3x3d_Skew","Skew");

} /* End TestSkew() */

/******************************************************************************
** Function: TestInvert1
*/
static void TestInvert1(void)
{
   int i, k, isInvertable;
	
   Matrix3x3d InputMat;
   Matrix3x3d OutputMat;
   Matrix3x3d ExpectedMat;

   /* a singular matrix */
   InputMat.Comp[0][0] = 1.0e0;
   InputMat.Comp[0][1] = 0.0e0;
   InputMat.Comp[0][2] = 0.0e0;
   InputMat.Comp[1][0] = 0.0e0;
   InputMat.Comp[1][1] = 1.0e0;
   InputMat.Comp[1][2] = 0.0e0;
   InputMat.Comp[2][0] = 0.0e0;
   InputMat.Comp[2][1] = 0.0e0;
   InputMat.Comp[2][2] = 0.0e0;

   for ( i=0; i<3; i++ ) {
	   for ( k=0; k<3; k++ ) ExpectedMat.Comp[i][k] = 0.0e0;
   }

   isInvertable = Matrix3x3d_Invert(&OutputMat, &InputMat);

   if ( !isInvertable ) {

     UTASSERT_BOOL( UtMatrix3x3d_EqualComp(&OutputMat, &ExpectedMat, UT_EQ_TOL),
		            "Matrix3x3d_Invert", "Invert1" );
   } else {

     UTASSERT_BOOL( FALSE, "Matrix3x3d_Invert return error", "Invert1" );

   }

} /* End TestInvert1() */

/******************************************************************************
** Function: TestInvert2
*/
static void TestInvert2(void)
{
   int isInvertable;
	
   Matrix3x3d InputMat;
   Matrix3x3d OutputMat;
   Matrix3x3d ExpectedMat;

   /* a diagonal matrix */
   InputMat.Comp[0][0] = 1.0e0;
   InputMat.Comp[0][1] = 0.0e0;
   InputMat.Comp[0][2] = 0.0e0;
   InputMat.Comp[1][0] = 0.0e0;
   InputMat.Comp[1][1] = 2.0e0;
   InputMat.Comp[1][2] = 0.0e0;
   InputMat.Comp[2][0] = 0.0e0;
   InputMat.Comp[2][1] = 0.0e0;
   InputMat.Comp[2][2] = 5.0e0;

   ExpectedMat.Comp[0][0] = 1.0e0;
   ExpectedMat.Comp[0][1] = 0.0e0;
   ExpectedMat.Comp[0][2] = 0.0e0;
   ExpectedMat.Comp[1][0] = 0.0e0;
   ExpectedMat.Comp[1][1] = 0.5e0;
   ExpectedMat.Comp[1][2] = 0.0e0;
   ExpectedMat.Comp[2][0] = 0.0e0;
   ExpectedMat.Comp[2][1] = 0.0e0;
   ExpectedMat.Comp[2][2] = 0.2e0;

   isInvertable = Matrix3x3d_Invert(&OutputMat, &InputMat);

   if ( isInvertable == 1 ) {

     UTASSERT_BOOL( UtMatrix3x3d_EqualComp(&OutputMat, &ExpectedMat, UT_EQ_TOL),
		            "Matrix3x3d_Invert", "Invert2" );
   } else {

     UTASSERT_BOOL( FALSE, "Matrix3x3d_Invert return error", "Invert2" );

   }

} /* End TestInvert2() */

/******************************************************************************
** Function: TestInvert3
*/
static void TestInvert3(void)
{
   int isInvertable;
	
   Matrix3x3d InputMat;
   Matrix3x3d OutputMat;
   Matrix3x3d Result;
   Matrix3x3d Identity;

   /* a diagonal matrix */
   InputMat.Comp[0][0] = 1.0;
   InputMat.Comp[0][1] = 2.0;
   InputMat.Comp[0][2] = 3.0;
   InputMat.Comp[1][0] = 4.0;
   InputMat.Comp[1][1] = 5.0;
   InputMat.Comp[1][2] = 6.0;
   InputMat.Comp[2][0] = 7.0;
   InputMat.Comp[2][1] = 8.0;
   InputMat.Comp[2][2] = 0.0;

   Identity.Comp[0][0] = 1.0;
   Identity.Comp[0][1] = 0.0;
   Identity.Comp[0][2] = 0.0;
   Identity.Comp[1][0] = 0.0;
   Identity.Comp[1][1] = 1.0;
   Identity.Comp[1][2] = 0.0;
   Identity.Comp[2][0] = 0.0;
   Identity.Comp[2][1] = 0.0;
   Identity.Comp[2][2] = 1.0;

   isInvertable = Matrix3x3d_Invert(&OutputMat, &InputMat);

   if ( isInvertable == 1 ) {

     Matrix3x3d_Mult( &Result, &OutputMat, &InputMat );

     UTASSERT_BOOL( UtMatrix3x3d_EqualComp(&Result, &Identity, UT_EQ_TOL),
		            "Matrix3x3d_Invert", "Invert3" );
   } else {

     UTASSERT_BOOL( FALSE, "Matrix3x3d_Invert return error", "Invert3" );

   }

} /* End TestInvert3() */

/******************************************************************************
** Function: TestTriad1
*/
static void TestTriad1(void)
{
   int isValidAttitude;
	
   Matrix3x3d Dcm;
   Vector3d   Ur, Vr, Ub, Vb;

   /* co-aligned vectors, invalid result */
   Ur.Comp[0] = 1.0;
   Ur.Comp[1] = 0.0;
   Ur.Comp[2] = 0.0;
   Vr.Comp[0] = 1.0;
   Vr.Comp[1] = 0.0;
   Vr.Comp[2] = 0.0;

   Ub.Comp[0] = 0.0;
   Ub.Comp[1] = 1.0;
   Ub.Comp[2] = 0.0;
   Vb.Comp[0] = 0.0;
   Vb.Comp[1] = 1.0;
   Vb.Comp[2] = 0.0;

   isValidAttitude = Matrix3x3d_Triad(&Dcm, &Ur, &Vr, &Ub, &Vb);

   if ( isValidAttitude == 0 ) {

     UTASSERT_BOOL( TRUE, "Matrix3x3d_Triad", "Triad1" );

   } else {

     UTASSERT_BOOL( FALSE, "Matrix3x3d_Triad return error", "Triad1" );

   }

} /* End TestTriad1() */

/******************************************************************************
** Function: TestTriad2
*/
static void TestTriad2(void)
{
   int isValidAttitude;
	
   Matrix3x3d Dcm, ExpectedDcm;
   Vector3d   Ur, Vr, Ub, Vb;

   /* 90 degree rotation about X */
   Ur.Comp[0] = 1.0;
   Ur.Comp[1] = 0.0;
   Ur.Comp[2] = 0.0;
   Vr.Comp[0] = 0.0;
   Vr.Comp[1] = 1.0;
   Vr.Comp[2] = 0.0;

   Ub.Comp[0] = 1.0;
   Ub.Comp[1] = 0.0;
   Ub.Comp[2] = 0.0;
   Vb.Comp[0] = 0.0;
   Vb.Comp[1] = 0.0;
   Vb.Comp[2] = 1.0;

   ExpectedDcm.Comp[0][0] = 1.0;
   ExpectedDcm.Comp[0][1] = 0.0;
   ExpectedDcm.Comp[0][2] = 0.0;

   ExpectedDcm.Comp[1][0] = 0.0;
   ExpectedDcm.Comp[1][1] = 0.0;
   ExpectedDcm.Comp[1][2] =-1.0;

   ExpectedDcm.Comp[2][0] = 0.0;
   ExpectedDcm.Comp[2][1] = 1.0;
   ExpectedDcm.Comp[2][2] = 0.0;

   isValidAttitude = Matrix3x3d_Triad(&Dcm, &Ur, &Vr, &Ub, &Vb);

   if ( isValidAttitude == 1 ) {

     UTASSERT_BOOL( UtMatrix3x3d_EqualComp(&ExpectedDcm, &Dcm, UT_EQ_TOL),
		            "Matrix3x3d_Triad", "Triad2" );

   } else {

     UTASSERT_BOOL( FALSE, "Matrix3x3d_Triad return error", "Triad2" );

   }

} /* End TestTriad2() */

/******************************************************************************
** Function: TestTriad3
*/
static void TestTriad3(void)
{
   int isValidAttitude;
	
   Matrix3x3d Dcm, ExpectedDcm;
   Vector3d   Ur, Vr, Ub, Vb;

   Quaternion R;

   /* 180 degree rotation about (X+Y)/sqrt(2) */
   Ur.Comp[0] = 1.0;
   Ur.Comp[1] = 0.0;
   Ur.Comp[2] = 0.0;
   Vr.Comp[0] = 0.0;
   Vr.Comp[1] = 1.0;
   Vr.Comp[2] = 0.0;

   Ub.Comp[0] = 0.0;
   Ub.Comp[1] = 1.0;
   Ub.Comp[2] = 0.0;
   Vb.Comp[0] = 1.0;
   Vb.Comp[1] = 0.0;
   Vb.Comp[2] = 0.0;

   R.Comp[0] = 1.0 / sqrt(2.0);
   R.Comp[1] = 1.0 / sqrt(2.0);
   R.Comp[2] = 0.0;
   R.Comp[3] = 0.0;

   Quaternion_ToMatrix( &ExpectedDcm, &R );

   isValidAttitude = Matrix3x3d_Triad(&Dcm, &Ur, &Vr, &Ub, &Vb);

   if ( isValidAttitude == 1 ) {

     UTASSERT_BOOL( UtMatrix3x3d_EqualComp(&ExpectedDcm, &Dcm, UT_EQ_TOL),
		            "Matrix3x3d_Triad", "Triad3" );

   } else {

     UTASSERT_BOOL( FALSE, "Matrix3x3d_Triad return error", "Triad3" );

   }

} /* End TestTriad3() */


/***************************************************************************
** Unit Test Function Database
*/

static UtDriver_Test TestArray[] = 
{
   { "Matrix3x3d_Add",            TestAdd            },
   { "Matrix3x3d_ColVecToMatrix", TestColVecToMatrix },
   { "Matrix3x3d_Copy",           TestCopy           },
   { "Matrix3x3d_DivScalar",      TestDivScalar      },
   { "Matrix3x3d_InitIdentity",   TestInitIdentity   },
   { "Matrix3x3d_InitZero",       TestInitZero       },
   { "Matrix3x3d_Mult",           TestMult           },
   { "Matrix3x3d_MultScalar",     TestMultScalar     },
   { "Matrix3x3d_MultVec",        TestMultVec        },
   { "Matrix3x3d_RowVecToMatrix", TestRowVecToMatrix },
   { "Matrix3x3d_Sub",            TestSub            },
   { "Matrix3x3d_Trace",          TestTrace          },
   { "Matrix3x3d_Transpose",      TestTranspose      },
   { "Matrix3x3d_Skew",           TestSkew           },
   { "Matrix3x3d_Invert",         TestInvert1        },
   { "Matrix3x3d_Invert",         TestInvert2        },
   { "Matrix3x3d_Invert",         TestInvert3        },
   { "Matrix3x3d_Triad",          TestTriad1         },
   { "Matrix3x3d_Triad",          TestTriad2         },
   { "Matrix3x3d_Triad",          TestTriad3         }

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
