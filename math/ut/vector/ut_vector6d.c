/*
** File:
** $Id: ut_vector6d.c 1.1 2008/05/21 15:00:43EDT dcmccomas Exp  $
** 
** Purpose: Implementation of the Vector6d Unit Test 
**
** References:
**   1. GN&C FSW Math Library Specification
**   2. GN&C FSW Developer Guide
**
**  $Id: ut_vector6d.c 1.1 2008/05/21 15:00:43EDT dcmccomas Exp  $
**  $Date: 2008/05/21 15:00:43EDT $
**  $Revision: 1.1 $
**  $Log: ut_vector6d.c  $
**  Revision 1.1 2008/05/21 15:00:43EDT dcmccomas 
**  Initial revision
**  Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/ut/vector/project.pj
**  Revision 1.1 2006/12/14 15:58:01EST myyang 
**  Initial revision
**  Member added to project d:/mksdata/gnc-fsw/math/project.pj
**
*/

/*
** Include Files
*/

#include <math.h>
#include <string.h>

#include "utdriver.h"
#include "utvector6d.h"

/*
** Macro Definitions
*/

#define UT_FILE_NAME  "ut_vector6d"
#define UT_EQ_TOL     1.0e-6


/******************************************************************************
** Function:  TestAdd
**
*/
static void TestAdd(void)
{

	Vector6d Vec1, Vec2, VecR, VecE;

	Vec1.Comp[0] = 5.23425e0;
	Vec1.Comp[1] = 6.13512e0;
	Vec1.Comp[2] = 7.54835e0;
	Vec1.Comp[3] = 8.24578e0;
	Vec1.Comp[4] = 9.15425e0;
	Vec1.Comp[5] = 3.21549e0;

	Vec2.Comp[0] = 2.45721e0;
	Vec2.Comp[1] = 3.24876e0;
	Vec2.Comp[2] = 5.45763e0;
	Vec2.Comp[3] = 3.21458e0;
	Vec2.Comp[4] = 4.85247e0;
	Vec2.Comp[5] = 6.81574e0;

	VecE.Comp[0] = 7.69146e0;
	VecE.Comp[1] = 9.38388e0;
	VecE.Comp[2] = 1.300598e1;
	VecE.Comp[3] = 1.146036e1;
	VecE.Comp[4] = 1.400672e1;
	VecE.Comp[5] = 1.003123e1;

	memset(&VecR, 0, sizeof(Vector6d));

	Vector6d_Add(&VecR, &Vec1, &Vec2);

   UTASSERT_BOOL(UtVector6d_EqualComp(&VecR, &VecE, UT_EQ_TOL),"Vector6d_Add","Add");

} /* End TestAdd() */


/******************************************************************************
** Function:  TestCopy
**
*/
static void TestCopy(void)
{

	Vector6d Vec1, VecR;

	Vec1.Comp[0] = 2.45721e0;
	Vec1.Comp[1] = 3.24876e0;
	Vec1.Comp[2] = 5.45763e0;
	Vec1.Comp[3] = 3.21458e0;
	Vec1.Comp[4] = 4.85247e0;
	Vec1.Comp[5] = 6.81574e0;

	memset(&VecR, 0, sizeof(Vector6d));

	Vector6d_Copy(&VecR, &Vec1);

   UTASSERT_BOOL(UtVector6d_EqualComp(&VecR, &Vec1, UT_EQ_TOL),"Vector6d_Copy","Copy");

} /* End TestCopy() */


/******************************************************************************
** Function:  TestDivScalar
**
*/
static void TestDivScalar(void)
{

	Vector6d Vec1, VecR, VecE;
	double Scalar = 3.12000e0;

	Vec1.Comp[0] = 5.23425e0;
	Vec1.Comp[1] = 6.13512e0;
	Vec1.Comp[2] = 7.54835e0;
	Vec1.Comp[3] = 8.24578e0;
	Vec1.Comp[4] = 9.15425e0;
	Vec1.Comp[5] = 3.21549e0;

	VecE.Comp[0] = 1.67764423077e0;
	VecE.Comp[1] = 1.96638461538e0;
	VecE.Comp[2] = 2.41934294872e0;
	VecE.Comp[3] = 2.64287820513e0;
	VecE.Comp[4] = 2.93405448718e0;
	VecE.Comp[5] = 1.03060576923e0;

	memset(&VecR, 0, sizeof(Vector6d));

	Vector6d_DivScalar(&VecR, &Vec1, Scalar);

   UTASSERT_BOOL(UtVector6d_EqualComp(&VecR, &VecE, UT_EQ_TOL),"Vector6d_DivScalar","DivScalar");

} /* End TestDivScalar() */


/******************************************************************************
** Function:  TestDot
**
*/
static void TestDot(void)
{

	Vector6d Vec1, Vec2;
	double ResultScalar, ExpectScalar;

	Vec1.Comp[0] = 5.23425e0;
	Vec1.Comp[1] = 6.13512e0;
	Vec1.Comp[2] = 7.54835e0;
	Vec1.Comp[3] = 8.24578e0;
	Vec1.Comp[4] = 9.15425e0;
	Vec1.Comp[5] = 3.21549e0;

	Vec2.Comp[0] = 2.45721e0;
	Vec2.Comp[1] = 3.24876e0;
	Vec2.Comp[2] = 5.45763e0;
	Vec2.Comp[3] = 3.21458e0;
	Vec2.Comp[4] = 4.85247e0;
	Vec2.Comp[5] = 6.81574e0;

	ExpectScalar = 1.668326721e002;

	ResultScalar = Vector6d_Dot(&Vec1, &Vec2);

   UTASSERT_EQUAL_TOL(ResultScalar, ExpectScalar, UT_EQ_TOL, "Vector6d_Dot","Dot");

} /* End TestDot() */


/******************************************************************************
** Function:  TestInitZero
**
*/
static void TestInitZero(void)
{

	Vector6d Vec1, Zero;

	memset(&Zero, 0, sizeof(Vector6d));

	Vector6d_InitZero(&Vec1);

   UTASSERT_BOOL(UtVector6d_EqualComp(&Vec1, &Zero, UT_EQ_TOL),"Vector6d_InitZero","InitZero");


} /* End TestInitZero() */


/******************************************************************************
** Function:  TestMagnitude
**
*/
static void TestMagnitude(void)
{

	Vector6d Vec1;

	double ResultMagnitude, ExpectMagnitude;

	Vec1.Comp[0] = 2.45721e0;
	Vec1.Comp[1] = 3.24876e0;
	Vec1.Comp[2] = 5.45763e0;
	Vec1.Comp[3] = 3.21458e0;
	Vec1.Comp[4] = 4.85247e0;
	Vec1.Comp[5] = 6.81574e0;

	ExpectMagnitude = 1.1256657992650394102138631783346e001;

	ResultMagnitude = Vector6d_Magnitude(&Vec1);

   UTASSERT_EQUAL_TOL(ResultMagnitude, ExpectMagnitude, UT_EQ_TOL, "Vector6d_Magnitude","Magnitude");

} /* End TestMagnitude() */


/******************************************************************************
** Function:  TestMultScalar
**
*/
static void TestMultScalar(void)
{

	Vector6d Vec1, VecR, VecE;

	double Scalar = 3.12;

	Vec1.Comp[0] = 5.23425e0;
	Vec1.Comp[1] = 6.13512e0;
	Vec1.Comp[2] = 7.54835e0;
	Vec1.Comp[3] = 8.24578e0;
	Vec1.Comp[4] = 9.15425e0;
	Vec1.Comp[5] = 3.21549e0;

	VecE.Comp[0] = 1.633086e1;
	VecE.Comp[1] = 1.91415744e1;
	VecE.Comp[2] = 2.3550852e1;
	VecE.Comp[3] = 2.57268336e1;
	VecE.Comp[4] = 2.856126e1;
	VecE.Comp[5] = 1.00323288e1;

	memset(&VecR, 0, sizeof(Vector6d));

	Vector6d_MultScalar(&VecR, &Vec1, Scalar);

   UTASSERT_BOOL(UtVector6d_EqualComp(&VecR, &VecE, UT_EQ_TOL),"Vector6d_MultScalar","MultScalar");

} /* End TestMultScalar() */

/******************************************************************************
** Function:  TestNormalize
**
*/
static void TestNormalize (void)
{
	Vector6d Vec1, VecR, VecE;

	Vec1.Comp[0] = 2.45721e0;
	Vec1.Comp[1] = 3.24876e0;
	Vec1.Comp[2] = 5.45763e0;
	Vec1.Comp[3] = 3.21458e0;
	Vec1.Comp[4] = 4.85247e0;
	Vec1.Comp[5] = 6.81574e0;

	VecE.Comp[0] = 0.21828947824517202501415581297875e0;
	VecE.Comp[1] = 0.28860786230879129906885811101731e0;
	VecE.Comp[2] = 0.48483573042401675024845543913105e0;
	VecE.Comp[3] = 0.2855714371084950363094749709163e0;
	VecE.Comp[4] = 0.43107554686019912674148349461584e0;
	VecE.Comp[5] = 0.60548521634485810238847405828228e0;

	memset(&VecR, 0, sizeof(Vector6d));

	Vector6d_Normalize(&VecR, &Vec1);

   UTASSERT_BOOL(UtVector6d_EqualComp(&VecR, &VecE, UT_EQ_TOL),"Vector6d_Normalize","Normalize");

} /* End TestNormalize() */

/******************************************************************************
** Function:  TestSub
**
*/
static void TestSub(void)
{

	Vector6d Vec1, Vec2, VecR, VecE;

	Vec1.Comp[0] = 5.23425e0;
	Vec1.Comp[1] = 6.13512e0;
	Vec1.Comp[2] = 7.54835e0;
	Vec1.Comp[3] = 8.24578e0;
	Vec1.Comp[4] = 9.15425e0;
	Vec1.Comp[5] = 3.21549e0;

	Vec2.Comp[0] = 2.45721e0;
	Vec2.Comp[1] = 3.24876e0;
	Vec2.Comp[2] = 5.45763e0;
	Vec2.Comp[3] = 3.21458e0;
	Vec2.Comp[4] = 4.85247e0;
	Vec2.Comp[5] = 6.81574e0;

	VecE.Comp[0] = 2.77704e0;
	VecE.Comp[1] = 2.88636e0;
	VecE.Comp[2] = 2.09072e0;
	VecE.Comp[3] = 5.03120e0;
	VecE.Comp[4] = 4.30178e0;
	VecE.Comp[5] = -3.60025e0;

	memset(&VecR, 0, sizeof(Vector6d));

	Vector6d_Sub(&VecR, &Vec1, &Vec2);

   UTASSERT_BOOL(UtVector6d_EqualComp(&VecR, &VecE, UT_EQ_TOL),"Vector6d_Sub","Sub");


} /* end TestSub() */


/***************************************************************************
** Unit Test Function Database
*/

static UtDriver_Test TestArray[] = 
{

   { "Vector6d_Add",        TestAdd        },
   { "Vector6d_Copy",       TestCopy       },
   { "Vector6d_DivScalar",  TestDivScalar  },
   { "Vector6d_Dot",        TestDot        },
   { "Vector6d_InitZero",   TestInitZero   },
   { "Vector6d_Magnitude",  TestMagnitude  },
   { "Vector6d_MultScalar", TestMultScalar },
   { "Vector6d_Normalize",  TestNormalize  },
   { "Vector6d_Sub",        TestSub        }

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
