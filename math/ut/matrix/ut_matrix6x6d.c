/*
** File:
** $Id: ut_matrix6x6d.c 1.1 2008/05/21 15:00:36EDT dcmccomas Exp  $
**
** Purpose: Unit test the Matrix6x6d component of the math library.
**
** References:
**   1. GN&C FSW Math Library Specification
**   2. GN&C FSW Developer Guide
**
** $Date: 2008/05/21 15:00:36EDT $
** $Revision: 1.1 $
** $Log: ut_matrix6x6d.c  $
** Revision 1.1 2008/05/21 15:00:36EDT dcmccomas 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/ut/matrix/project.pj
*/
/*
** Include Files
*/

#include <math.h>
#include <string.h>
#include "utdriver.h"
#include "utmatrix6x6d.h"
#include "utvector3d.h"

/*
** Macro Definitions
*/

#define UT_FILE_NAME   "ut_matrix6x6d"
#define UT_EQ_TOL      1.0e-10


/******************************************************************************
** Function: TestAdd
*/
static void TestAdd(void)
{

   Matrix6x6d AMat, BMat, CMat, Expected;

   AMat.Comp[0][0] = 1.0e0;
   AMat.Comp[0][1] = 2.0e0;
   AMat.Comp[0][2] = 3.0e0;
   AMat.Comp[0][3] = 4.0e0;
   AMat.Comp[0][4] = 5.0e0;
   AMat.Comp[0][5] = 6.0e0;
   AMat.Comp[1][0] = 1.0e1;
   AMat.Comp[1][1] = 2.0e1;
   AMat.Comp[1][2] = 3.0e1;
   AMat.Comp[1][3] = 4.0e1;
   AMat.Comp[1][4] = 5.0e1;
   AMat.Comp[1][5] = 6.0e1;
   AMat.Comp[2][0] = 1.0e2;
   AMat.Comp[2][1] = 2.0e2;
   AMat.Comp[2][2] = 3.0e2;
   AMat.Comp[2][3] = 4.0e2;
   AMat.Comp[2][4] = 5.0e2;
   AMat.Comp[2][5] = 6.0e2;
   AMat.Comp[3][0] = 1.0e3;
   AMat.Comp[3][1] = 2.0e3;
   AMat.Comp[3][2] = 3.0e3;
   AMat.Comp[3][3] = 4.0e3;
   AMat.Comp[3][4] = 5.0e3;
   AMat.Comp[3][5] = 6.0e3;
   AMat.Comp[4][0] = 1.0e4;
   AMat.Comp[4][1] = 2.0e4;
   AMat.Comp[4][2] = 3.0e4;
   AMat.Comp[4][3] = 4.0e4;
   AMat.Comp[4][4] = 5.0e4;
   AMat.Comp[4][5] = 6.0e4;
   AMat.Comp[5][0] = 1.0e5;
   AMat.Comp[5][1] = 2.0e5;
   AMat.Comp[5][2] = 3.0e5;
   AMat.Comp[5][3] = 4.0e5;
   AMat.Comp[5][4] = 5.0e5;
   AMat.Comp[5][5] = 6.0e5;
   BMat.Comp[0][0] = 1.0e0;
   BMat.Comp[0][1] = 1.0e1;
   BMat.Comp[0][2] = 1.0e2;
   BMat.Comp[0][3] = 1.0e3;
   BMat.Comp[0][4] = 1.0e4;
   BMat.Comp[0][5] = 1.0e5;
   BMat.Comp[1][0] = 2.0e0;
   BMat.Comp[1][1] = 2.0e1;
   BMat.Comp[1][2] = 2.0e2;
   BMat.Comp[1][3] = 2.0e3;
   BMat.Comp[1][4] = 2.0e4;
   BMat.Comp[1][5] = 2.0e5;
   BMat.Comp[2][0] = 3.0e0;
   BMat.Comp[2][1] = 3.0e1;
   BMat.Comp[2][2] = 3.0e2;
   BMat.Comp[2][3] = 3.0e3;
   BMat.Comp[2][4] = 3.0e4;
   BMat.Comp[2][5] = 3.0e5;
   BMat.Comp[3][0] = 4.0e0;
   BMat.Comp[3][1] = 4.0e1;
   BMat.Comp[3][2] = 4.0e2;
   BMat.Comp[3][3] = 4.0e3;
   BMat.Comp[3][4] = 4.0e4;
   BMat.Comp[3][5] = 4.0e5;
   BMat.Comp[4][0] = 5.0e0;
   BMat.Comp[4][1] = 5.0e1;
   BMat.Comp[4][2] = 5.0e2;
   BMat.Comp[4][3] = 5.0e3;
   BMat.Comp[4][4] = 5.0e4;
   BMat.Comp[4][5] = 5.0e5;
   BMat.Comp[5][0] = 6.0e0;
   BMat.Comp[5][1] = 6.0e1;
   BMat.Comp[5][2] = 6.0e2;
   BMat.Comp[5][3] = 6.0e3;
   BMat.Comp[5][4] = 6.0e4;
   BMat.Comp[5][5] = 6.0e5;
   Expected.Comp[0][0] = 2.00000e0;
   Expected.Comp[0][1] = 1.20000e1;
   Expected.Comp[0][2] = 1.03000e2;
   Expected.Comp[0][3] = 1.00400e3;
   Expected.Comp[0][4] = 1.00050e4;
   Expected.Comp[0][5] = 1.00006e5;
   Expected.Comp[1][0] = 1.20000e1;
   Expected.Comp[1][1] = 4.00000e1;
   Expected.Comp[1][2] = 2.30000e2;
   Expected.Comp[1][3] = 2.04000e3;
   Expected.Comp[1][4] = 2.00500e4;
   Expected.Comp[1][5] = 2.00060e5;
   Expected.Comp[2][0] = 1.03000e2;
   Expected.Comp[2][1] = 2.30000e2;
   Expected.Comp[2][2] = 6.00000e2;
   Expected.Comp[2][3] = 3.40000e3;
   Expected.Comp[2][4] = 3.05000e4;
   Expected.Comp[2][5] = 3.00600e5;
   Expected.Comp[3][0] = 1.00400e3;
   Expected.Comp[3][1] = 2.04000e3;
   Expected.Comp[3][2] = 3.40000e3;
   Expected.Comp[3][3] = 8.00000e3;
   Expected.Comp[3][4] = 4.50000e4;
   Expected.Comp[3][5] = 4.06000e5;
   Expected.Comp[4][0] = 1.00050e4;
   Expected.Comp[4][1] = 2.00500e4;
   Expected.Comp[4][2] = 3.05000e4;
   Expected.Comp[4][3] = 4.50000e4;
   Expected.Comp[4][4] = 1.00000e5;
   Expected.Comp[4][5] = 5.60000e5;
   Expected.Comp[5][0] = 1.00006e5;
   Expected.Comp[5][1] = 2.00060e5;
   Expected.Comp[5][2] = 3.00600e5;
   Expected.Comp[5][3] = 4.06000e5;
   Expected.Comp[5][4] = 5.60000e5;
   Expected.Comp[5][5] = 1.20000e6;
   
   memset(&CMat, 0, sizeof(Matrix6x6d));

   Matrix6x6d_Add(&CMat, &AMat, &BMat);

   UTASSERT_BOOL(UtMatrix6x6d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix6x6d_Add","Add");

} /* End TestAdd() */


/******************************************************************************
** Function: TestCopy
*/
static void TestCopy(void)
{

   Matrix6x6d CMat, Expected;
   Expected.Comp[0][0] = 2.00000e0;
   Expected.Comp[0][1] = 1.20000e1;
   Expected.Comp[0][2] = 1.03000e2;
   Expected.Comp[0][3] = 1.00400e3;
   Expected.Comp[0][4] = 1.00050e4;
   Expected.Comp[0][5] = 1.00006e5;
	Expected.Comp[1][0] = 1.20000e1;
   Expected.Comp[1][1] = 4.00000e1;
   Expected.Comp[1][2] = 2.30000e2;
   Expected.Comp[1][3] = 2.04000e3;
   Expected.Comp[1][4] = 2.00500e4;
   Expected.Comp[1][5] = 2.00060e5;
   Expected.Comp[2][0] = 1.03000e2;
   Expected.Comp[2][1] = 2.30000e2;
   Expected.Comp[2][2] = 6.00000e2;
   Expected.Comp[2][3] = 3.40000e3;
   Expected.Comp[2][4] = 3.05000e4;
   Expected.Comp[2][5] = 3.00600e5;
   Expected.Comp[3][0] = 1.00400e3;
   Expected.Comp[3][1] = 2.04000e3;
   Expected.Comp[3][2] = 3.40000e3;
   Expected.Comp[3][3] = 8.00000e3;
   Expected.Comp[3][4] = 4.50000e4;
   Expected.Comp[3][5] = 4.06000e5;
   Expected.Comp[4][0] = 1.00050e4;
   Expected.Comp[4][1] = 2.00500e4;	
   Expected.Comp[4][2] = 3.05000e4;	
   Expected.Comp[4][3] = 4.50000e4;	
   Expected.Comp[4][4] = 1.00000e5;	
   Expected.Comp[4][5] = 5.60000e5;	
   Expected.Comp[5][0] = 1.00006e5;	
   Expected.Comp[5][1] = 2.00060e5;	
   Expected.Comp[5][2] = 3.00600e5;
   Expected.Comp[5][3] = 4.06000e5;
   Expected.Comp[5][4] = 5.60000e5;
   Expected.Comp[5][5] = 1.20000e6;

   memset(&CMat, 0, sizeof(Matrix6x6d));
   
   Matrix6x6d_Copy(&CMat, &Expected);

   UTASSERT_BOOL(UtMatrix6x6d_IdenticalComp(&CMat, &Expected),"Matrix6x6d_Copy","Copy");

} /* End TestCopy() */


/******************************************************************************
** Function: TestDivScalar
*/
static void TestDivScalar(void)
{
   Matrix6x6d AMat, CMat, Expected;	
   double scalar_factor = -2.0e2;	
   AMat.Comp[0][0] = 1.11111e0;	
   AMat.Comp[0][1] = 2.22222e0;	
   AMat.Comp[0][2] = 3.33333e0;	
   AMat.Comp[0][3] = 4.44444e0;	
   AMat.Comp[0][4] = 5.55555e0;	
   AMat.Comp[0][5] = 6.66666e0;
   AMat.Comp[1][0] = 7.77777e0;	
   AMat.Comp[1][1] = 8.88888e0;	
   AMat.Comp[1][2] = 9.99999e0;	
   AMat.Comp[1][3] = 1.11111e1;	
   AMat.Comp[1][4] = 2.22222e1;	
   AMat.Comp[1][5] = 3.33333e1;	
   AMat.Comp[2][0] = 4.44444e1;	
   AMat.Comp[2][1] = 5.55555e1;	
   AMat.Comp[2][2] = 6.66666e1;	
   AMat.Comp[2][3] = 7.77777e1;	
   AMat.Comp[2][4] = 8.88888e1;	
   AMat.Comp[2][5] = 9.99999e1;	
   AMat.Comp[3][0] = 1.11111e2;	
   AMat.Comp[3][1] = 2.22222e2;	
   AMat.Comp[3][2] = 3.33333e2;	
   AMat.Comp[3][3] = 4.44444e2;	
   AMat.Comp[3][4] = 5.55555e2;	
   AMat.Comp[3][5] = 6.66666e2;	
   AMat.Comp[4][0] = 7.77777e2;	
   AMat.Comp[4][1] = 8.88888e2;	
   AMat.Comp[4][2] = 9.99999e2;	
   AMat.Comp[4][3] = 1.11111e3;	
   AMat.Comp[4][4] = 2.22222e3;	
   AMat.Comp[4][5] = 3.33333e3;	
   AMat.Comp[5][0] = 4.44444e3;	
   AMat.Comp[5][1] = 5.55555e3;	
   AMat.Comp[5][2] = 6.66666e3;	
   AMat.Comp[5][3] = 7.77777e3;	
   AMat.Comp[5][4] = 8.88888e3;	
   AMat.Comp[5][5] = 9.99999e3;	
   Expected.Comp[0][0] = -5.55555e-3;
   Expected.Comp[0][1] = -1.11111e-2;	
   Expected.Comp[0][2] = -1.666665e-2;
   Expected.Comp[0][3] = -2.22222e-2;	
   Expected.Comp[0][4] = -2.777775e-2;	
   Expected.Comp[0][5] = -3.33333e-2; 
   Expected.Comp[1][0] = -3.888885e-2;	
   Expected.Comp[1][1] = -4.44444e-2;	
   Expected.Comp[1][2] = -4.999995e-2;	
   Expected.Comp[1][3] = -5.55555e-2;	
   Expected.Comp[1][4] = -1.11111e-1;	
   Expected.Comp[1][5] = -1.666665e-1; 
   Expected.Comp[2][0] = -2.22222e-1;	
   Expected.Comp[2][1] = -2.777775e-1;	
   Expected.Comp[2][2] = -3.33333e-1;	
   Expected.Comp[2][3] = -3.888885e-1;	
   Expected.Comp[2][4] = -4.44444e-1;	
   Expected.Comp[2][5] = -4.999995e-1; 
   Expected.Comp[3][0] = -5.55555e-1;	
   Expected.Comp[3][1] = -1.11111e0;	
   Expected.Comp[3][2] = -1.666665e0;	
   Expected.Comp[3][3] = -2.22222e0;	
   Expected.Comp[3][4] = -2.777775e0;	
   Expected.Comp[3][5] = -3.33333e0; 
   Expected.Comp[4][0] = -3.888885e0;	
   Expected.Comp[4][1] = -4.44444e0;	
   Expected.Comp[4][2] = -4.999995e0;
   Expected.Comp[4][3] = -5.55555e0;	
   Expected.Comp[4][4] = -1.11111e1;	
   Expected.Comp[4][5] = -1.666665e1; 
   Expected.Comp[5][0] = -2.22222e1;	
   Expected.Comp[5][1] = -2.777775e1;	
   Expected.Comp[5][2] = -3.33333e1;	
   Expected.Comp[5][3] = -3.888885e1;	
   Expected.Comp[5][4] = -4.44444e1;	
   Expected.Comp[5][5] = -4.999995e1;
   
   memset(&CMat, 0, sizeof(Matrix6x6d));
   
   Matrix6x6d_DivScalar(&CMat, &AMat, scalar_factor);	
   
   UTASSERT_BOOL(UtMatrix6x6d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix6x6d_DivScalar","DivScalar");

} /* End TestDivScalar() */


/******************************************************************************
** Function: TestInitIdentity
*/
static void TestInitIdentity(void)
{
   
   Matrix6x6d CMat, Expected;

   Expected.Comp[0][0] = 1.0e0;	
   Expected.Comp[0][1] = 0.0e0;	
   Expected.Comp[0][2] = 0.0e0;	
   Expected.Comp[0][3] = 0.0e0;	
   Expected.Comp[0][4] = 0.0e0;	
   Expected.Comp[0][5] = 0.0e0; 
   Expected.Comp[1][0] = 0.0e0;	
   Expected.Comp[1][1] = 1.0e0;	
   Expected.Comp[1][2] = 0.0e0;	
   Expected.Comp[1][3] = 0.0e0;	
   Expected.Comp[1][4] = 0.0e0;	
   Expected.Comp[1][5] = 0.0e0; 
   Expected.Comp[2][0] = 0.0e0;	
   Expected.Comp[2][1] = 0.0e0;	
   Expected.Comp[2][2] = 1.0e0;	
   Expected.Comp[2][3] = 0.0e0;	
   Expected.Comp[2][4] = 0.0e0;	
   Expected.Comp[2][5] = 0.0e0; 
   Expected.Comp[3][0] = 0.0e0;	
   Expected.Comp[3][1] = 0.0e0;	
   Expected.Comp[3][2] = 0.0e0;	
   Expected.Comp[3][3] = 1.0e0;	
   Expected.Comp[3][4] = 0.0e0;	
   Expected.Comp[3][5] = 0.0e0; 
   Expected.Comp[4][0] = 0.0e0;	
   Expected.Comp[4][1] = 0.0e0;	
   Expected.Comp[4][2] = 0.0e0;	
   Expected.Comp[4][3] = 0.0e0;	
   Expected.Comp[4][4] = 1.0e0;	
   Expected.Comp[4][5] = 0.0e0; 
   Expected.Comp[5][0] = 0.0e0;	
   Expected.Comp[5][1] = 0.0e0;	
   Expected.Comp[5][2] = 0.0e0;	
   Expected.Comp[5][3] = 0.0e0;	
   Expected.Comp[5][4] = 0.0e0;	
   Expected.Comp[5][5] = 1.0e0; 
   
   memset(&CMat, 0xFF, sizeof(Matrix6x6d));
   
   Matrix6x6d_InitIdentity(&CMat);
   
   UTASSERT_BOOL(UtMatrix6x6d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix6x6d_InitIdentity","InitIdentity");

} /* End TestInitIdentity() */


/******************************************************************************
** Function: TestInitZero
*/
static void TestInitZero(void)
{
   Matrix6x6d CMat, Expected;
   
   Expected.Comp[0][0] = 0.0e0;
   Expected.Comp[0][1] = 0.0e0;
   Expected.Comp[0][2] = 0.0e0;
   Expected.Comp[0][3] = 0.0e0;
   Expected.Comp[0][4] = 0.0e0;
   Expected.Comp[0][5] = 0.0e0;
   Expected.Comp[1][0] = 0.0e0;	
   Expected.Comp[1][1] = 0.0e0;	
   Expected.Comp[1][2] = 0.0e0;	
   Expected.Comp[1][3] = 0.0e0;	
   Expected.Comp[1][4] = 0.0e0;	
   Expected.Comp[1][5] = 0.0e0;
   Expected.Comp[2][0] = 0.0e0;	
   Expected.Comp[2][1] = 0.0e0;	
   Expected.Comp[2][2] = 0.0e0;	
   Expected.Comp[2][3] = 0.0e0;	
   Expected.Comp[2][4] = 0.0e0;	
   Expected.Comp[2][5] = 0.0e0;
   Expected.Comp[3][0] = 0.0e0;	
   Expected.Comp[3][1] = 0.0e0;	
   Expected.Comp[3][2] = 0.0e0;	
   Expected.Comp[3][3] = 0.0e0;	
   Expected.Comp[3][4] = 0.0e0;	
   Expected.Comp[3][5] = 0.0e0;
   Expected.Comp[4][0] = 0.0e0;	
   Expected.Comp[4][1] = 0.0e0;	
   Expected.Comp[4][2] = 0.0e0;	
   Expected.Comp[4][3] = 0.0e0;	
   Expected.Comp[4][4] = 0.0e0;	
   Expected.Comp[4][5] = 0.0e0;
   Expected.Comp[5][0] = 0.0e0;	
   Expected.Comp[5][1] = 0.0e0;	
   Expected.Comp[5][2] = 0.0e0;	
   Expected.Comp[5][3] = 0.0e0;	
   Expected.Comp[5][4] = 0.0e0;	
   Expected.Comp[5][5] = 0.0e0;
   
   memset(&CMat, 0xFF, sizeof(Matrix6x6d));
   
   Matrix6x6d_InitZero(&CMat);
   
   UTASSERT_BOOL(UtMatrix6x6d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix6x6d_InitZero","InitZero");

} /* End TestInitZero() */


/******************************************************************************
** Function: TestMult
*/
static void TestMult(void)
{
  
   Matrix6x6d AMat, BMat, CMat, Expected;
   AMat.Comp[0][0] = 1.0e0;
   AMat.Comp[0][1] = 2.0e0;
   AMat.Comp[0][2] = 3.0e0;
   AMat.Comp[0][3] = 4.0e0;
   AMat.Comp[0][4] = 5.0e0;
   AMat.Comp[0][5] = 6.0e0;
   AMat.Comp[1][0] = 1.0e1;
   AMat.Comp[1][1] = 2.0e1;
   AMat.Comp[1][2] = 3.0e1;
   AMat.Comp[1][3] = 4.0e1;
   AMat.Comp[1][4] = 5.0e1;
   AMat.Comp[1][5] = 6.0e1;
   AMat.Comp[2][0] = 1.0e2;
   AMat.Comp[2][1] = 2.0e2;
   AMat.Comp[2][2] = 3.0e2;
   AMat.Comp[2][3] = 4.0e2;
   AMat.Comp[2][4] = 5.0e2;
   AMat.Comp[2][5] = 6.0e2;
   AMat.Comp[3][0] = 1.0e3;
   AMat.Comp[3][1] = 2.0e3;
   AMat.Comp[3][2] = 3.0e3;
   AMat.Comp[3][3] = 4.0e3;
   AMat.Comp[3][4] = 5.0e3;
   AMat.Comp[3][5] = 6.0e3;
   AMat.Comp[4][0] = 1.0e4;
   AMat.Comp[4][1] = 2.0e4;
   AMat.Comp[4][2] = 3.0e4;
   AMat.Comp[4][3] = 4.0e4;
   AMat.Comp[4][4] = 5.0e4;
   AMat.Comp[4][5] = 6.0e4;
   AMat.Comp[5][0] = 1.0e5;
   AMat.Comp[5][1] = 2.0e5;
   AMat.Comp[5][2] = 3.0e5;
   AMat.Comp[5][3] = 4.0e5;
   AMat.Comp[5][4] = 5.0e5;
   AMat.Comp[5][5] = 6.0e5;
   BMat.Comp[0][0] = 1.0e0;
   BMat.Comp[0][1] = 1.0e1;
   BMat.Comp[0][2] = 1.0e2;
   BMat.Comp[0][3] = 1.0e3;
   BMat.Comp[0][4] = 1.0e4;
   BMat.Comp[0][5] = 1.0e5;
   BMat.Comp[1][0] = 2.0e0;
   BMat.Comp[1][1] = 2.0e1;
   BMat.Comp[1][2] = 2.0e2;
   BMat.Comp[1][3] = 2.0e3;
   BMat.Comp[1][4] = 2.0e4;
   BMat.Comp[1][5] = 2.0e5;
   BMat.Comp[2][0] = 3.0e0;
   BMat.Comp[2][1] = 3.0e1;
   BMat.Comp[2][2] = 3.0e2;
   BMat.Comp[2][3] = 3.0e3;
   BMat.Comp[2][4] = 3.0e4;
   BMat.Comp[2][5] = 3.0e5;
   BMat.Comp[3][0] = 4.0e0;
   BMat.Comp[3][1] = 4.0e1;
   BMat.Comp[3][2] = 4.0e2;
   BMat.Comp[3][3] = 4.0e3;
   BMat.Comp[3][4] = 4.0e4;
   BMat.Comp[3][5] = 4.0e5;
   BMat.Comp[4][0] = 5.0e0;
   BMat.Comp[4][1] = 5.0e1;
   BMat.Comp[4][2] = 5.0e2;
   BMat.Comp[4][3] = 5.0e3;
   BMat.Comp[4][4] = 5.0e4;
   BMat.Comp[4][5] = 5.0e5;
   BMat.Comp[5][0] = 6.0e0;
   BMat.Comp[5][1] = 6.0e1;
   BMat.Comp[5][2] = 6.0e2;
   BMat.Comp[5][3] = 6.0e3;
   BMat.Comp[5][4] = 6.0e4;
   BMat.Comp[5][5] = 6.0e5;
   Expected.Comp[0][0] = 9.1e1;
   Expected.Comp[0][1] = 9.1e2;
   Expected.Comp[0][2] = 9.1e3;
   Expected.Comp[0][3] = 9.1e4;
   Expected.Comp[0][4] = 9.1e5;
   Expected.Comp[0][5] = 9.1e6;
   Expected.Comp[1][0] = 9.1e2;
   Expected.Comp[1][1] = 9.1e3;
   Expected.Comp[1][2] = 9.1e4;
   Expected.Comp[1][3] = 9.1e5;
   Expected.Comp[1][4] = 9.1e6;
   Expected.Comp[1][5] = 9.1e7;
   Expected.Comp[2][0] = 9.1e3;
   Expected.Comp[2][1] = 9.1e4;
   Expected.Comp[2][2] = 9.1e5;
   Expected.Comp[2][3] = 9.1e6;
   Expected.Comp[2][4] = 9.1e7;
   Expected.Comp[2][5] = 9.1e8;
   Expected.Comp[3][0] = 9.1e4;
   Expected.Comp[3][1] = 9.1e5;
   Expected.Comp[3][2] = 9.1e6;
   Expected.Comp[3][3] = 9.1e7;
   Expected.Comp[3][4] = 9.1e8;
   Expected.Comp[3][5] = 9.1e9;
   Expected.Comp[4][0] = 9.1e5;
   Expected.Comp[4][1] = 9.1e6;
   Expected.Comp[4][2] = 9.1e7;
   Expected.Comp[4][3] = 9.1e8;
   Expected.Comp[4][4] = 9.1e9;
   Expected.Comp[4][5] = 9.1e10;
   Expected.Comp[5][0] = 9.1e6;
   Expected.Comp[5][1] = 9.1e7;
   Expected.Comp[5][2] = 9.1e8;
   Expected.Comp[5][3] = 9.1e9;
   Expected.Comp[5][4] = 9.1e10;
   Expected.Comp[5][5] = 9.1e11;

   memset(&CMat, 0, sizeof(Matrix6x6d));

   Matrix6x6d_Mult(&CMat, &AMat, &BMat);

   UTASSERT_BOOL(UtMatrix6x6d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix6x6d_Mult","Mult");

} /* End TestMult() */



/******************************************************************************
** Function: TestMultScalar
*/
static void TestMultScalar(void)
{
   Matrix6x6d AMat, CMat, Expected;	
   double scalar_factor = -2.0e2;	
   AMat.Comp[0][0] = 1.11111e0;
   AMat.Comp[0][1] = 2.22222e0;
   AMat.Comp[0][2] = 3.33333e0;
   AMat.Comp[0][3] = 4.44444e0;
   AMat.Comp[0][4] = 5.55555e0;
   AMat.Comp[0][5] = 6.66666e0;
   AMat.Comp[1][0] = 7.77777e0;	
   AMat.Comp[1][1] = 8.88888e0;
   AMat.Comp[1][2] = 9.99999e0;
   AMat.Comp[1][3] = 1.11111e1;
   AMat.Comp[1][4] = 2.22222e1;
   AMat.Comp[1][5] = 3.33333e1;
   AMat.Comp[2][0] = 4.44444e1;
   AMat.Comp[2][1] = 5.55555e1;
   AMat.Comp[2][2] = 6.66666e1;
   AMat.Comp[2][3] = 7.77777e1;
   AMat.Comp[2][4] = 8.88888e1;
   AMat.Comp[2][5] = 9.99999e1;
   AMat.Comp[3][0] = 1.11111e2;
   AMat.Comp[3][1] = 2.22222e2;
   AMat.Comp[3][2] = 3.33333e2;
   AMat.Comp[3][3] = 4.44444e2;
   AMat.Comp[3][4] = 5.55555e2;
   AMat.Comp[3][5] = 6.66666e2;
   AMat.Comp[4][0] = 7.77777e2;
   AMat.Comp[4][1] = 8.88888e2;
   AMat.Comp[4][2] = 9.99999e2;
   AMat.Comp[4][3] = 1.11111e3;
   AMat.Comp[4][4] = 2.22222e3;
   AMat.Comp[4][5] = 3.33333e3;
   AMat.Comp[5][0] = 4.44444e3;
   AMat.Comp[5][1] = 5.55555e3;
   AMat.Comp[5][2] = 6.66666e3;
   AMat.Comp[5][3] = 7.77777e3;
   AMat.Comp[5][4] = 8.88888e3;
   AMat.Comp[5][5] = 9.99999e3;
   Expected.Comp[0][0] = -2.22222e2;
   Expected.Comp[0][1] = -4.44444e2;
   Expected.Comp[0][2] = -6.66666e2;
   Expected.Comp[0][3] = -8.88888e2;
   Expected.Comp[0][4] = -1.11111e3;
   Expected.Comp[0][5] = -1.333332e3; 
   Expected.Comp[1][0] = -1.555554e3;
   Expected.Comp[1][1] = -1.777776e3;
   Expected.Comp[1][2] = -1.999998e3;
   Expected.Comp[1][3] = -2.22222e3;
   Expected.Comp[1][4] = -4.44444e3;
   Expected.Comp[1][5] = -6.66666e3; 
   Expected.Comp[2][0] = -8.88888e3;
   Expected.Comp[2][1] = -1.11111e4;
   Expected.Comp[2][2] = -1.333332e4;
   Expected.Comp[2][3] = -1.555554e4;
   Expected.Comp[2][4] = -1.777776e4;
   Expected.Comp[2][5] = -1.999998e4; 
   Expected.Comp[3][0] = -2.22222e4;
   Expected.Comp[3][1] = -4.44444e4;
   Expected.Comp[3][2] = -6.66666e4;
   Expected.Comp[3][3] = -8.88888e4;
   Expected.Comp[3][4] = -1.11111e5;
   Expected.Comp[3][5] = -1.333332e5; 
   Expected.Comp[4][0] = -1.555554e5;
   Expected.Comp[4][1] = -1.777776e5;
   Expected.Comp[4][2] = -1.999998e5;
   Expected.Comp[4][3] = -2.22222e5;
   Expected.Comp[4][4] = -4.44444e5;
   Expected.Comp[4][5] = -6.66666e5; 
   Expected.Comp[5][0] = -8.88888e5;
   Expected.Comp[5][1] = -1.11111e6;
   Expected.Comp[5][2] = -1.333332e6;
   Expected.Comp[5][3] = -1.555554e6;
   Expected.Comp[5][4] = -1.777776e6;
   Expected.Comp[5][5] = -1.999998e6; 
   
   memset(&CMat, 0, sizeof(Matrix6x6d));
   
   Matrix6x6d_MultScalar(&CMat, &AMat, scalar_factor);
   
   UTASSERT_BOOL(UtMatrix6x6d_EqualComp(&CMat, &Expected, 1.0e-9),"Matrix6x6d_MultScalar","MultScalar");

} /* End TestMultScalar() */



/******************************************************************************
** Function: TestSub
*/
static void TestSub(void)
{
   Matrix6x6d AMat, BMat, CMat, Expected;
   AMat.Comp[0][0] = 2.00000e0;
   AMat.Comp[0][1] = 1.20000e1;
   AMat.Comp[0][2] = 1.03000e2;
   AMat.Comp[0][3] = 1.00400e3;
   AMat.Comp[0][4] = 1.00050e4;
   AMat.Comp[0][5] = 1.00006e5;
	
   AMat.Comp[1][0] = 1.20000e1;
   AMat.Comp[1][1] = 4.00000e1;
   AMat.Comp[1][2] = 2.30000e2;
   AMat.Comp[1][3] = 2.04000e3;
   AMat.Comp[1][4] = 2.00500e4;
   AMat.Comp[1][5] = 2.00060e5;
   AMat.Comp[2][0] = 1.03000e2;
   AMat.Comp[2][1] = 2.30000e2;
   AMat.Comp[2][2] = 6.00000e2;
   AMat.Comp[2][3] = 3.40000e3;
   AMat.Comp[2][4] = 3.05000e4;	
   AMat.Comp[2][5] = 3.00600e5;
   AMat.Comp[3][0] = 1.00400e3;
   AMat.Comp[3][1] = 2.04000e3;
   AMat.Comp[3][2] = 3.40000e3;
   AMat.Comp[3][3] = 8.00000e3;
   AMat.Comp[3][4] = 4.50000e4;
   AMat.Comp[3][5] = 4.06000e5;
   AMat.Comp[4][0] = 1.00050e4;
   AMat.Comp[4][1] = 2.00500e4;
   AMat.Comp[4][2] = 3.05000e4;
   AMat.Comp[4][3] = 4.50000e4;
   AMat.Comp[4][4] = 1.00000e5;
   AMat.Comp[4][5] = 5.60000e5;
   AMat.Comp[5][0] = 1.00006e5;
   AMat.Comp[5][1] = 2.00060e5;
   AMat.Comp[5][2] = 3.00600e5;
   AMat.Comp[5][3] = 4.06000e5;
   AMat.Comp[5][4] = 5.60000e5;
   AMat.Comp[5][5] = 1.20000e6;
   BMat.Comp[0][0] = 1.0e0;
   BMat.Comp[0][1] = 1.0e1;
   BMat.Comp[0][2] = 1.0e2;
   BMat.Comp[0][3] = 1.0e3;
   BMat.Comp[0][4] = 1.0e4;
   BMat.Comp[0][5] = 1.0e5;
   BMat.Comp[1][0] = 2.0e0;
   BMat.Comp[1][1] = 2.0e1;
   BMat.Comp[1][2] = 2.0e2;
   BMat.Comp[1][3] = 2.0e3;
   BMat.Comp[1][4] = 2.0e4;
   BMat.Comp[1][5] = 2.0e5;
   BMat.Comp[2][0] = 3.0e0;
   BMat.Comp[2][1] = 3.0e1;
   BMat.Comp[2][2] = 3.0e2;
   BMat.Comp[2][3] = 3.0e3;
   BMat.Comp[2][4] = 3.0e4;
   BMat.Comp[2][5] = 3.0e5;
   BMat.Comp[3][0] = 4.0e0;
   BMat.Comp[3][1] = 4.0e1;
   BMat.Comp[3][2] = 4.0e2;
   BMat.Comp[3][3] = 4.0e3;
   BMat.Comp[3][4] = 4.0e4;
   BMat.Comp[3][5] = 4.0e5;
   BMat.Comp[4][0] = 5.0e0;
   BMat.Comp[4][1] = 5.0e1;
   BMat.Comp[4][2] = 5.0e2;
   BMat.Comp[4][3] = 5.0e3;
   BMat.Comp[4][4] = 5.0e4;
   BMat.Comp[4][5] = 5.0e5;
   BMat.Comp[5][0] = 6.0e0;
   BMat.Comp[5][1] = 6.0e1;
   BMat.Comp[5][2] = 6.0e2;
   BMat.Comp[5][3] = 6.0e3;
   BMat.Comp[5][4] = 6.0e4;
   BMat.Comp[5][5] = 6.0e5;
   Expected.Comp[0][0] = 1.0e0;
   Expected.Comp[0][1] = 2.0e0;
   Expected.Comp[0][2] = 3.0e0;
   Expected.Comp[0][3] = 4.0e0;
   Expected.Comp[0][4] = 5.0e0;
   Expected.Comp[0][5] = 6.0e0;
   Expected.Comp[1][0] = 1.0e1;
   Expected.Comp[1][1] = 2.0e1;
   Expected.Comp[1][2] = 3.0e1;
   Expected.Comp[1][3] = 4.0e1;
   Expected.Comp[1][4] = 5.0e1;
   Expected.Comp[1][5] = 6.0e1;
   Expected.Comp[2][0] = 1.0e2;
   Expected.Comp[2][1] = 2.0e2;
   Expected.Comp[2][2] = 3.0e2;
   Expected.Comp[2][3] = 4.0e2;
   Expected.Comp[2][4] = 5.0e2;
   Expected.Comp[2][5] = 6.0e2;
   Expected.Comp[3][0] = 1.0e3;
   Expected.Comp[3][1] = 2.0e3;
   Expected.Comp[3][2] = 3.0e3;
   Expected.Comp[3][3] = 4.0e3;
   Expected.Comp[3][4] = 5.0e3;
   Expected.Comp[3][5] = 6.0e3;
   Expected.Comp[4][0] = 1.0e4;
   Expected.Comp[4][1] = 2.0e4;
   Expected.Comp[4][2] = 3.0e4;
   Expected.Comp[4][3] = 4.0e4;
   Expected.Comp[4][4] = 5.0e4;
   Expected.Comp[4][5] = 6.0e4;
   Expected.Comp[5][0] = 1.0e5;
   Expected.Comp[5][1] = 2.0e5;
   Expected.Comp[5][2] = 3.0e5;
   Expected.Comp[5][3] = 4.0e5;
   Expected.Comp[5][4] = 5.0e5;
   Expected.Comp[5][5] = 6.0e5;

   memset(&CMat, 0, sizeof(Matrix6x6d));
   
   Matrix6x6d_Sub(&CMat, &AMat, &BMat);
   
   UTASSERT_BOOL(UtMatrix6x6d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix6x6d_Sub","Sub");

} /* End TestSub() */

/******************************************************************************
** Function: TestTrace
*/
static void TestTrace(void)
{
   Matrix6x6d CMat;
   double Expected;
   CMat.Comp[0][0] = 2.00000e0;
   CMat.Comp[0][1] = 1.20000e1;
   CMat.Comp[0][2] = 1.03000e2;
   CMat.Comp[0][3] = 1.00400e3;
   CMat.Comp[0][4] = 1.00050e4;
   CMat.Comp[0][5] = 1.00006e5;
	CMat.Comp[1][0] = 1.20000e5;
   CMat.Comp[1][1] = 4.00000e1;
   CMat.Comp[1][2] = 2.30000e2;
   CMat.Comp[1][3] = 2.04000e3;
   CMat.Comp[1][4] = 2.00500e4;
   CMat.Comp[1][5] = 2.00060e5;
   CMat.Comp[2][0] = 1.03000e4;
   CMat.Comp[2][1] = 2.30000e3;
   CMat.Comp[2][2] = 6.00000e2;
   CMat.Comp[2][3] = 3.40000e3;
   CMat.Comp[2][4] = 3.05000e4;
   CMat.Comp[2][5] = 3.00600e5;
   CMat.Comp[3][0] = 1.00400e2;
   CMat.Comp[3][1] = 2.04000e4;
   CMat.Comp[3][2] = 3.40000e4;
   CMat.Comp[3][3] = 8.00000e3;
   CMat.Comp[3][4] = 4.50000e4;
   CMat.Comp[3][5] = 4.06000e5;
   CMat.Comp[4][0] = 1.00050e3;
   CMat.Comp[4][1] = 2.00500e5;
   CMat.Comp[4][2] = 3.05000e5;
   CMat.Comp[4][3] = 4.50000e5;
   CMat.Comp[4][4] = 1.00000e5;
   CMat.Comp[4][5] = 5.60000e5;
   CMat.Comp[5][0] = 1.00006e1;
   CMat.Comp[5][1] = 2.00060e6;
   CMat.Comp[5][2] = 3.00600e6;
   CMat.Comp[5][3] = 4.06000e6;
   CMat.Comp[5][4] = 5.60000e6;
   CMat.Comp[5][5] = -10.0e-1;
   
   Expected = Matrix6x6d_Trace(&CMat);
   
   UTASSERT_EQUAL(108641, Expected, "Matrix6x6d_Trace","Trace");

} /* End TestTrace() */


/******************************************************************************
** Function: TestTranspose
*/
static void TestTranspose(void)
{
   Matrix6x6d AMat, CMat, Expected;
   AMat.Comp[0][0] = 2.00000e0;
   AMat.Comp[0][1] = 1.20000e5;
   AMat.Comp[0][2] = 1.03000e4;
   AMat.Comp[0][3] = 1.00400e2;
   AMat.Comp[0][4] = 1.00050e3;
   AMat.Comp[0][5] = 1.00006e1;
   AMat.Comp[1][0] = 1.20000e1;
   AMat.Comp[1][1] = 4.00000e1;
   AMat.Comp[1][2] = 2.30000e3;
   AMat.Comp[1][3] = 2.04000e4;
   AMat.Comp[1][4] = 2.00500e5;
   AMat.Comp[1][5] = 2.00060e6;
   AMat.Comp[2][0] = 1.03000e2;
   AMat.Comp[2][1] = 2.30000e2;
   AMat.Comp[2][2] = 6.00000e2;
   AMat.Comp[2][3] = 3.40000e4;
   AMat.Comp[2][4] = 3.05000e5;
   AMat.Comp[2][5] = 3.00600e6;
   AMat.Comp[3][0] = 1.00400e3;
   AMat.Comp[3][1] = 2.04000e3;	
   AMat.Comp[3][2] = 3.40000e3;
   AMat.Comp[3][3] = 8.00000e3;
   AMat.Comp[3][4] = 4.50000e5;
   AMat.Comp[3][5] = 4.06000e6;
   AMat.Comp[4][0] = 1.00050e4;
   AMat.Comp[4][1] = 2.00500e4;
   AMat.Comp[4][2] = 3.05000e4;
   AMat.Comp[4][3] = 4.50000e4;
   AMat.Comp[4][4] = 1.00000e5;
   AMat.Comp[4][5] = 5.60000e6;
   AMat.Comp[5][0] = 1.00006e5;
   AMat.Comp[5][1] = 2.00060e5;
   AMat.Comp[5][2] = 3.00600e5;
   AMat.Comp[5][3] = 4.06000e5;
   AMat.Comp[5][4] = 5.60000e5;
   AMat.Comp[5][5] = 0.0e0;
   Expected.Comp[0][0] = 2.00000e0;
   Expected.Comp[0][1] = 1.20000e1;	
   Expected.Comp[0][2] = 1.03000e2;	
   Expected.Comp[0][3] = 1.00400e3;	
   Expected.Comp[0][4] = 1.00050e4;	
   Expected.Comp[0][5] = 1.00006e5;
   Expected.Comp[1][0] = 1.20000e5;
   Expected.Comp[1][1] = 4.00000e1;
   Expected.Comp[1][2] = 2.30000e2;
   Expected.Comp[1][3] = 2.04000e3;
   Expected.Comp[1][4] = 2.00500e4;	
   Expected.Comp[1][5] = 2.00060e5;	
   Expected.Comp[2][0] = 1.03000e4;	
   Expected.Comp[2][1] = 2.30000e3;	
   Expected.Comp[2][2] = 6.00000e2;
   Expected.Comp[2][3] = 3.40000e3;
   Expected.Comp[2][4] = 3.05000e4;
   Expected.Comp[2][5] = 3.00600e5;
   Expected.Comp[3][0] = 1.00400e2;
   Expected.Comp[3][1] = 2.04000e4;
   Expected.Comp[3][2] = 3.40000e4;
   Expected.Comp[3][3] = 8.00000e3;
   Expected.Comp[3][4] = 4.50000e4;
   Expected.Comp[3][5] = 4.06000e5;
   Expected.Comp[4][0] = 1.00050e3;
   Expected.Comp[4][1] = 2.00500e5;	
   Expected.Comp[4][2] = 3.05000e5;	
   Expected.Comp[4][3] = 4.50000e5;	
   Expected.Comp[4][4] = 1.00000e5;	
   Expected.Comp[4][5] = 5.60000e5;	
   Expected.Comp[5][0] = 1.00006e1;	
   Expected.Comp[5][1] = 2.00060e6;	
   Expected.Comp[5][2] = 3.00600e6;	
   Expected.Comp[5][3] = 4.06000e6;	
   Expected.Comp[5][4] = 5.60000e6;	
   Expected.Comp[5][5] = 0.0e0;
   
   memset(&CMat, 0, sizeof(Matrix6x6d));
   
   Matrix6x6d_Transpose(&CMat, &AMat);
   
   UTASSERT_BOOL(UtMatrix6x6d_EqualComp(&CMat, &Expected, UT_EQ_TOL),"Matrix6x6d_Transpose","Transpose");

} /* End TestTranspose() */


/***************************************************************************
** Unit Test Function Database
*/
static UtDriver_Test TestArray[] = 
{
   { "Matrix6x6d_Add",          TestAdd          },
   { "Matrix6x6d_Copy",         TestCopy         },
   { "Matrix6x6d_DivScalar",    TestDivScalar    },
   { "Matrix6x6d_InitIdentity", TestInitIdentity },
   { "Matrix6x6d_InitZero",     TestInitZero     },
   { "Matrix6x6d_Mult",         TestMult         },
   { "Matrix6x6d_MultScalar",   TestMultScalar   },
   { "Matrix6x6d_Sub",          TestSub          },
   { "Matrix6x6d_Trace",        TestMultScalar   },
   { "Matrix6x6d_Transpose",    TestTranspose    }
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
