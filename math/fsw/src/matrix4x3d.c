/* 
** File:
**   $Id: matrix4x3d.c 1.1 2008/05/21 15:00:27EDT dcmccomas Exp  $
**
**  Purpose: Provide the implementation of the Matrix4x3d package. 
**
**  $Date: 2008/05/21 15:00:27EDT $
**
**  $Revision: 1.1 $
**
**  $Log: matrix4x3d.c  $
**  Revision 1.1 2008/05/21 15:00:27EDT dcmccomas 
**  Initial revision
**  Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/src/project.pj
**  Revision 1.2 2005/11/29 07:49:14EST dcmccomas 
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

#include "matrix4x3d.h"
#include "matrixmxnd.h"


/*
** Exported Functions
*/

/******************************************************************************
**
** Function: Matrix4x3d_Add
**
** Notes:   None
*/

void Matrix4x3d_Add (Matrix4x3d *Result, const Matrix4x3d *Left, const Matrix4x3d *Right) 
{

   MatrixMxNd_Add(&Result->Comp[0][0], &Left->Comp[0][0],&Right->Comp[0][0],4,3);

} /* End Matrix4x3d_Add() */


/******************************************************************************
**
** Function: Matrix4x3d_Copy
**
** Notes:
**   None
*/

void Matrix4x3d_Copy (Matrix4x3d *Result, const Matrix4x3d *Operand) 
{

   MatrixMxNd_Copy(&Result->Comp[0][0], &Operand->Comp[0][0],4,3);

} /* End Matrix4x3d_Copy() */


/*******************************************************************************
**
** Function: Matrix4x3d_DivScalar
**
** Notes: None
*/

void Matrix4x3d_DivScalar (Matrix4x3d *Result, const Matrix4x3d *Left, double Scalar) 
{
   double ScalarInv;

   ScalarInv = 1.0 / Scalar;
   MatrixMxNd_MultScalar(&Result->Comp[0][0], &Left->Comp[0][0],4,3,ScalarInv);

} /* End Matrix4x3d_DivScalar() */

/*******************************************************************************
**
** Function Matrix4x3d_InitZero
**
** Notes:   None
*/
void Matrix4x3d_InitZero( Matrix4x3d *Result)
{

   MatrixMxNd_InitZero(&Result->Comp[0][0],4,3);

} /* end Matrix4x3d_InitZero() */

/******************************************************************************
**
** Function: Matrix4x3d_MultScalar
**
** Notes: None
*/

void Matrix4x3d_MultScalar (Matrix4x3d *Result, const Matrix4x3d *Left, double Scalar) 
{

   MatrixMxNd_MultScalar(&Result->Comp[0][0], &Left->Comp[0][0],4,3,Scalar);

} /* End Matrix4x3d_MultScalar() */

/****************************************************************************
**
** Function: Matrix4x3d_MultVec
**
** Notes:   None
*/

void Matrix4x3d_MultVec (Vector4d *Result, const Matrix4x3d *Left, const Vector3d *Right) 
{
   
   MatrixMxNd_Mult(&Result->Comp[0], &Left->Comp[0][0],4,3,&Right->Comp[0],3,1);

} /* End Matrix4x3d_MultVec */


/****************************************************************************
**
** Function: Matrix4x3d_Sub
**
** Notes:   None
*/

void Matrix4x3d_Sub (Matrix4x3d *Result, const Matrix4x3d *Left, const Matrix4x3d *Right) 
{

   MatrixMxNd_Sub(&Result->Comp[0][0], &Left->Comp[0][0],&Right->Comp[0][0],4,3);

} /* End Matrix4x3d_Sub() */


/* end of file */
