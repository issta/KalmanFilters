/*
** File:
** $Id: ut_vector3d.c 1.1 2008/05/21 15:00:41EDT dcmccomas Exp  $
**
** Purpose: Unit test the vec3d component of the math library.
**
** References:
**   1. GN&C FSW Math Library Specification
**   2. GN&C FSW Developer Guide
**
** $Date: 2008/05/21 15:00:41EDT $
** $Revision: 1.1 $
** $Log: ut_vector3d.c  $
** Revision 1.1 2008/05/21 15:00:41EDT dcmccomas 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/ut/vector/project.pj
*/
/*
** Include Files
*/

#include <math.h>
#include <string.h>

#include "utvector3d.h"

#include "utdriver.h"

/*
** Macro Definitions
*/

#define UT_FILE_NAME "ut_vector3d"
#define UT_EQ_TOL    1.0e-6


/******************************************************************************
** Function: TestAdd
*/
static void TestAdd (void)
{	
   
   Vector3d Vec1, Vec2, VecR, VecE;	

   Vec1.Comp[0] = 5.23425e0;
   Vec1.Comp[1] = 6.13512e0;
   Vec1.Comp[2] = 7.54835e0;
   Vec2.Comp[0] = 2.45721e0;
   Vec2.Comp[1] = 3.24876e0;
   Vec2.Comp[2] = 5.45763e0;
   VecE.Comp[0] = 7.69146e0;
   VecE.Comp[1] = 9.38388e0;
   VecE.Comp[2] = 1.300598e1;
   
   memset(&VecR, 0, sizeof(Vector3d));
   
   Vector3d_Add(&VecR, &Vec1, &Vec2);
   UTASSERT_BOOL(UtVector3d_EqualComp(&VecR, &VecE,UT_EQ_TOL),"Vector3d_Add","Add");

} /* End TestAdd() */


/******************************************************************************
** Function: TestCopy
*/
static void TestCopy(void)
{	

   Vector3d Vec1, VecR;
   Vec1.Comp[0] = 2.45721e0;
   Vec1.Comp[1] = 3.24876e0;
   Vec1.Comp[2] = 5.45763e0;

   memset(&VecR, 0, sizeof(Vector3d));

   Vector3d_Copy(&VecR, &Vec1);
   UTASSERT_BOOL(UtVector3d_EqualComp(&VecR, &Vec1, UT_EQ_TOL),"Vector3d_Copy","Copy");

} /* End TestCopy() */


/******************************************************************************
** Function: TestCross
*/
static void TestCross(void)
{	

   Vector3d Vec1, Vec2, VecR, VecE;
   
   Vec1.Comp[0] = 5.23425e0;
   Vec1.Comp[1] = 6.13512e0;
   Vec1.Comp[2] = 7.54835e0;
   Vec2.Comp[0] = 2.45721e0;
   Vec2.Comp[1] = 3.24876e0;
   Vec2.Comp[2] = 5.45763e0;
   VecE.Comp[0] = 8.9604374196e0;
   VecE.Comp[1] = -10.018718724e0;
   VecE.Comp[2] = 1.9295438148e0;
   
   memset(&VecR, 0, sizeof(Vector3d));

   Vector3d_Cross(&VecR, &Vec1, &Vec2);
   UTASSERT_BOOL(UtVector3d_EqualComp(&VecR, &VecE, UT_EQ_TOL),"Vector3d_Cross","Cross");
   
} /* End TestCross() */


/******************************************************************************
** Function: TestDivScalar
*/
static void TestDivScalar(void)
{

   Vector3d Vec1, VecR, VecE;
   double Scalar = 3.12000e0;
   
   Vec1.Comp[0] = 5.23425e0;
   Vec1.Comp[1] = 6.13512e0;
   Vec1.Comp[2] = 7.54835e0;
   VecE.Comp[0] = 1.67764423077e0;
   VecE.Comp[1] = 1.96638461538e0;
   VecE.Comp[2] = 2.41934294872e0;
   
   memset(&VecR, 0, sizeof(Vector3d));
   
   Vector3d_DivScalar(&VecR, &Vec1, Scalar);
   UTASSERT_BOOL(UtVector3d_EqualComp(&VecR, &VecE, UT_EQ_TOL),"Vector3d_DivScalar","DivScalar");

} /* End TestDivScalar() */


/******************************************************************************
** Function: TestDot
*/
static void TestDot(void)
{

   Vector3d Vec1, Vec2;
   double ResultScalar, ExpectScalar;
   
   Vec1.Comp[0] = 5.23425e0;
   Vec1.Comp[1] = 6.13512e0;
   Vec1.Comp[2] = 7.54835e0;
   Vec2.Comp[0] = 2.45721e0;
   Vec2.Comp[1] = 3.24876e0;
   Vec2.Comp[2] = 5.45763e0;
   ExpectScalar = 7.39892853042e001;
   
   ResultScalar = Vector3d_Dot(&Vec1, &Vec2);
   
   UTASSERT_EQUAL_TOL(ResultScalar, ExpectScalar, UT_EQ_TOL, "Vector3d_Dot","Dot");

} /* End TestDot() */


/******************************************************************************
** Function: TestFromRaDec
*/
static void TestFromRaDec(void)
{

   Vector3d  VecR, VecE;
   double    RA, Dec;
   
   RA = 27;	
   Dec = 156;
   
   VecE.Comp[0] = -1.37787939178e-001;
   VecE.Comp[1] = 4.51076900139e-001;
   VecE.Comp[2] = -8.81784618815e-001;
   
   memset(&VecR, 0, sizeof(Vector3d));
   
   Vector3d_FromRaDec(&VecR, RA, Dec);
   
   UTASSERT_BOOL(UtVector3d_EqualComp(&VecR, &VecE, UT_EQ_TOL),"Vector3d_FromRaDec","FromRaDec");

} /* End TestFromRaDec() */


/******************************************************************************
** Function: TestInitZero
*/
static void TestInitZero(void)
{
   
   Vector3d Vec1, Zero;
   
   Vec1.Comp[0] = 2.45721e0;
   Vec1.Comp[1] = 3.24876e0;
   Vec1.Comp[2] = 5.45763e0;

   memset(&Zero, 0, sizeof(Vector3d));
   Vector3d_InitZero(&Vec1);	

   UTASSERT_BOOL(UtVector3d_EqualComp(&Vec1, &Zero, UT_EQ_TOL),"Vector3d_InitZero","InitZero");
   
} /* End TestInitZero() */


/******************************************************************************
** Function: TestMagnitude
*/
static void TestMagnitude(void)
{

   Vector3d Vec1;
   double ResultMagnitude, ExpectMagnitude;
   
   Vec1.Comp[0] = 2.45721e0;
   Vec1.Comp[1] = 3.24876e0;
   Vec1.Comp[2] = 5.45763e0;
   ExpectMagnitude = 6.81014300427e000;
   
   ResultMagnitude = Vector3d_Magnitude(&Vec1);
   UTASSERT_EQUAL_TOL(ResultMagnitude, ExpectMagnitude, UT_EQ_TOL, "Vector3d_Magnitude","Magnitude");

} /* End TestMagnitude() */


/******************************************************************************
** Function: TestMultScalar
*/
static void TestMultScalar(void)
{

   Vector3d Vec1, VecR, VecE;
   double   Scalar = 3.12;
   
   Vec1.Comp[0] = 5.23425e0;
   Vec1.Comp[1] = 6.13512e0;
   Vec1.Comp[2] = 7.54835e0;
   VecE.Comp[0] = 1.633086e1;
   VecE.Comp[1] = 1.91415744e1;
   VecE.Comp[2] = 2.3550852e1;
   
   memset(&VecR, 0, sizeof(Vector3d));
   
   Vector3d_MultScalar(&VecR, &Vec1, Scalar);
   UTASSERT_BOOL(UtVector3d_EqualComp(&VecR, &VecE, UT_EQ_TOL),"Vector3d_MultScalar","MultScalar");

} /* End TestMultScalar() */


/******************************************************************************
** Function: TestNormalize
*/
static void TestNormalize (void)
{
   
   Vector3d Vec1, VecR, VecE;
   
   Vec1.Comp[0] = 2.45721e0;
   Vec1.Comp[1] = 3.24876e0;
   Vec1.Comp[2] = 5.45763e0;
   VecE.Comp[0] = 0.36081621171;
   VecE.Comp[1] = 0.47704725113 ;
   VecE.Comp[2] = 0.80139726825;
   
   memset(&VecR, 0, sizeof(Vector3d));
   
   Vector3d_Normalize(&VecR, &Vec1);
   UTASSERT_BOOL(UtVector3d_EqualComp(&VecR, &VecE, UT_EQ_TOL),"Vector3d_Normalize","Normalize");

} /* End TestNormalize() */


/******************************************************************************
** Function: TestSub
*/
static void TestSub(void)
{
   
   Vector3d Vec1, Vec2, VecR, VecE;
   
   Vec1.Comp[0] = 5.23425e0;
   Vec1.Comp[1] = 6.13512e0;
   Vec1.Comp[2] = 7.54835e0;
   Vec2.Comp[0] = 2.45721e0;
   Vec2.Comp[1] = 3.24876e0;
   Vec2.Comp[2] = 5.45763e0;
   VecE.Comp[0] = 2.77704e0;
   VecE.Comp[1] = 2.88636e0;
   VecE.Comp[2] = 2.09072e0;
   
   memset(&VecR, 0, sizeof(Vector3d));
   
   Vector3d_Sub(&VecR, &Vec1, &Vec2);
   UTASSERT_BOOL(UtVector3d_EqualComp(&VecR, &VecE, UT_EQ_TOL),"Vector3d_Sub","Sub");

} /* End TestSub() */
   
/***************************************************************************
** Unit Test Function Database
*/

static UtDriver_Test TestArray[] = 
{

   { "Vector3d_Add",        TestAdd        },
   { "Vector3d_Copy",       TestCopy       },
   { "Vector3d_Cross",      TestCross      },
   { "Vector3d_DivScalar",  TestDivScalar  },
   { "Vector3d_Dot",        TestDot        },
   { "Vector3d_FromRaDec",  TestFromRaDec  },
   { "Vector3d_InitZero",   TestInitZero   },
   { "Vector3d_Magnitude",  TestMagnitude  },
   { "Vector3d_MultScalar", TestMultScalar },
   { "Vector3d_Normalize",  TestNormalize  },
   { "Vector3d_Sub",        TestSub        }

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

