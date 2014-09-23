/* 
** File:
**   $Id: vector4d.c 1.1 2008/05/21 15:00:30EDT dcmccomas Exp  $
**
**  Purpose: Provide the implementation of the Vector4d package. 
**
**  $Date: 2008/05/21 15:00:30EDT $
**
**  $Revision: 1.1 $
**
**  $Log: vector4d.c  $
**  Revision 1.1 2008/05/21 15:00:30EDT dcmccomas 
**  Initial revision
**  Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/src/project.pj
**  Revision 1.2 2005/11/29 07:49:53EST dcmccomas 
**  New SDO delivery and added doxygen markup
**  Revision 1.3  2004/02/04 20:32:44  rbjork
**  Updates resulting from vector code walkthrough
**
**  Revision 1.2  2004/02/04 16:57:56  daviddawsonuser
**  replaced word files with actual files
**
*/

/*
** Include Files
*/

#include "vector4d.h"
#include "matrixmxnd.h"
#include <math.h>

/*
** Exported Functions
*/

/***********************************************************************************
**
** Function: Vector4d_Add
**
** Notes:   None
*/

void Vector4d_Add (Vector4d *Result, const Vector4d *Left, const Vector4d *Right) 
{

   MatrixMxNd_Add(&Result->Comp[0], &Left->Comp[0],&Right->Comp[0],4,1);

} /* End Vector4d_Add() */


/*********************************************************************************
**
** Function: Vector4d_Copy
**
** Notes:   None
*/

void Vector4d_Copy (Vector4d *Result, const Vector4d *Operand) 
{

   MatrixMxNd_Copy(&Result->Comp[0], &Operand->Comp[0],4,1);

} /* End Vector4d_Copy() */


/***********************************************************************************
**
** Function: Vector4d_DivScalar
**
** Notes: None
*/

void Vector4d_DivScalar (Vector4d *Result, const Vector4d *Left, double Scalar) 
{
   double ScalarInv;

   ScalarInv = 1.0 / Scalar;
   MatrixMxNd_MultScalar(&Result->Comp[0], &Left->Comp[0],4,1,ScalarInv);

} /* End Vector4d_DivScalar() */

/***********************************************************************************
**
** Function: Vector4d_Dot
**
** Notes:   None
*/
double Vector4d_Dot ( const Vector4d *Left, const Vector4d *Right)
{
   double Dot;

   MatrixMxNd_Mult( &Dot, &Left->Comp[0],1,4, &Right->Comp[0],4,1);
   return (Dot);
   
} /* End of Vector4d_Dot */


/*********************************************************************************
**
** Function: Vector4d_InitZero
**
** Notes:   None
*/

void Vector4d_InitZero (Vector4d *Result) 
{

   MatrixMxNd_InitZero(&Result->Comp[0],4,1);

} /* End Vector4d_InitZero() */

/********************************************************************************
**
** Function: Vector4d_Magnitude
**
** Notes:  None  
**  
*/

double Vector4d_Magnitude (const Vector4d *Operand) 
{
   double Mag;
   
   MatrixMxNd_Mult( &Mag, &Operand->Comp[0],1,4, &Operand->Comp[0],4,1);
   Mag = sqrt(Mag);
   return(Mag);

} /* End Vector4d_Magnitude() */

/************************************************************************************
**
** Function: Vector4d_MultScalar
**
** Notes: None
*/

void Vector4d_MultScalar (Vector4d *Result, const Vector4d *Left, double Scalar) 
{

   MatrixMxNd_MultScalar(&Result->Comp[0], &Left->Comp[0],4,1,Scalar);

} /* End Vector4d_MultScalar() */


/***********************************************************************************
**
** Function: Vector4d_Normalize
**
** Notes:   None
*/

void Vector4d_Normalize (Vector4d *Result, const Vector4d *Operand) 
{
   double Magnitude;

   Magnitude = Vector4d_Magnitude(Operand);
   Vector4d_DivScalar(Result,Operand,Magnitude);

} /* End Vector4d_Normalize() */


/***********************************************************************************
**
** Function: Vector4d_Sub
**
** Notes:   None
*/

void Vector4d_Sub (Vector4d *Result, const Vector4d *Left, const Vector4d *Right) 
{

   MatrixMxNd_Sub(&Result->Comp[0], &Left->Comp[0],&Right->Comp[0],4,1);

} /* End Vector4d_Sub() */

/* end of file */
