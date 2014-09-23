/* 
** File:
**   $Id: matrix3x4d.c 1.1 2008/05/21 15:00:27EDT dcmccomas Exp  $
**
**  Purpose: Provide the implementation of the Matrix3x4d package. 
**
**  $Date: 2008/05/21 15:00:27EDT $
**
**  $Revision: 1.1 $
**
**  $Log: matrix3x4d.c  $
**  Revision 1.1 2008/05/21 15:00:27EDT dcmccomas 
**  Initial revision
**  Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/src/project.pj
**  Revision 1.2 2005/11/29 07:49:13EST dcmccomas 
**  New SDO delivery and added doxygen markup
**  Revision 1.3  2004/03/23 15:20:13  ddawson
**  removed DOS line endings
**
**  Revision 1.2  2004/02/04 20:32:44  rbjork
**  Updates resulting from vector code walkthrough
**
**  Revision 1.1  2004/01/23 13:22:59  daviddawsonuser
**  directory reorg
**
*/

/*
** Include Files
*/

#include "matrix3x4d.h"
#include "matrixmxnd.h"

/*
** Exported Functions
*/

/****************************************************************************
**
** Function: Matrix3x4d_Add
**
** Notes:   None
*/

void Matrix3x4d_Add (Matrix3x4d *Result, const Matrix3x4d *Left, const Matrix3x4d *Right) 
{

   MatrixMxNd_Add(&Result->Comp[0][0], &Left->Comp[0][0],&Right->Comp[0][0],3,4);

} /* End Matrix3x4d_Add() */


/*******************************************************************************
**
** Function: Matrix3x4d_Copy
**
** Notes:
**   None
*/

void Matrix3x4d_Copy (Matrix3x4d *Result, const Matrix3x4d *Operand) 
{

   MatrixMxNd_Copy(&Result->Comp[0][0], &Operand->Comp[0][0],3,4);

} /* End Matrix3x4d_Copy() */

/********************************************************************************
**
** Function: Matrix3x4d_DivScalar
**
** Notes: None
*/

void Matrix3x4d_DivScalar (Matrix3x4d *Result, const Matrix3x4d *Left, double Scalar) 
{
   double ScalarInv;

   ScalarInv = 1.0 / Scalar;
   MatrixMxNd_MultScalar(&Result->Comp[0][0], &Left->Comp[0][0],3,4,ScalarInv);

} /* End Matrix3x4d_DivScalar() */

/*******************************************************************************
**
** Function Matrix3x4d_InitZero
**
** Notes:   None
*/
void Matrix3x4d_InitZero( Matrix3x4d *Result)
{
   MatrixMxNd_InitZero(&Result->Comp[0][0],3,4);
} /* End Matrix3x4d_InitZero() */

/*******************************************************************************
**
** Function: Matrix3x4d_MultScalar
**
** Notes: None
*/

void Matrix3x4d_MultScalar (Matrix3x4d *Result, const Matrix3x4d *Left, double Scalar) 
{

   MatrixMxNd_MultScalar(&Result->Comp[0][0], &Left->Comp[0][0],3,4,Scalar);

} /* End Matrix3x4d_MultScalar() */


/*********************************************************************************
**
** Function: Matrix3x4d_MultVec
**
** Notes:   None
*/

void Matrix3x4d_MultVec (Vector3d *Result, const Matrix3x4d *Left, const Vector4d *Right) 
{
   
   MatrixMxNd_Mult(&Result->Comp[0], &Left->Comp[0][0],3,4,&Right->Comp[0],4,1);

} /* End Matrix3x4d_MultVec */


/*********************************************************************************
**
** Function: Matrix3x4d_Sub
**
** Notes:   None
*/

void Matrix3x4d_Sub (Matrix3x4d *Result, const Matrix3x4d *Left, const Matrix3x4d *Right) 
{

   MatrixMxNd_Sub(&Result->Comp[0][0], &Left->Comp[0][0],&Right->Comp[0][0],3,4);

} /* End Matrix3x4d_Sub() */

/* end of file */
