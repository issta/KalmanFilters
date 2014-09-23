/* 
** File:
**   $Id: vector6d.c 1.1 2008/05/21 15:00:30EDT dcmccomas Exp  $
**
** Purpose: Provide the implementation of the Vector6d package. 
**
** $Date: 2008/05/21 15:00:30EDT $
** $Revision: 1.1 $
** $Log: vector6d.c  $
** Revision 1.1 2008/05/21 15:00:30EDT dcmccomas 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/src/project.pj
** Revision 1.1 2006/12/14 13:41:44EST myyang 
** Initial revision
** Member added to project d:/mksdata/gnc-fsw/math/project.pj
**
*/

/*
** Include Files
*/

#include "vector6d.h"
#include "matrixmxnd.h"
#include <math.h>

/*
** Exported Functions
*/

/***********************************************************************************
**
** Function: Vector6d_Add
**
** Notes:   None
*/
void Vector6d_Add (Vector6d *Result, const Vector6d *Left, const Vector6d *Right) 
{

   MatrixMxNd_Add(&Result->Comp[0], &Left->Comp[0],&Right->Comp[0],6,1);

} /* End Vector6d_Add() */


/*********************************************************************************
**
** Function: Vector6d_Copy
**
** Notes:   None
*/
void Vector6d_Copy (Vector6d *Result, const Vector6d *Operand) 
{

   MatrixMxNd_Copy(&Result->Comp[0], &Operand->Comp[0],6,1);

} /* End Vector6d_Copy() */


/***********************************************************************************
**
** Function: Vector6d_DivScalar
**
** Notes: None
*/
void Vector6d_DivScalar (Vector6d *Result, const Vector6d *Left, double Scalar) 
{
   double ScalarInv;

   ScalarInv = 1.0 / Scalar;
   MatrixMxNd_MultScalar(&Result->Comp[0], &Left->Comp[0],6,1,ScalarInv);

} /* End Vector6d_DivScalar() */


/***********************************************************************************
**
** Function: Vector6d_Dot
**
** Notes:   None
*/
double Vector6d_Dot ( const Vector6d *Left, const Vector6d *Right)
{
   double Dot;

   MatrixMxNd_Mult( &Dot, &Left->Comp[0],1,6, &Right->Comp[0],6,1);
   return (Dot);
   
} /* End of Vector6d_Dot */


/*********************************************************************************
**
** Function: Vector6d_InitZero
**
** Notes:   None
*/
void Vector6d_InitZero (Vector6d *Result) 
{

   MatrixMxNd_InitZero(&Result->Comp[0],6,1);

} /* End Vector6d_InitZero() */


/********************************************************************************
**
** Function: Vector6d_Magnitude
**
** Notes:  None  
**  
*/
double Vector6d_Magnitude (const Vector6d *Operand) 
{
   double Mag;
   
   MatrixMxNd_Mult( &Mag, &Operand->Comp[0],1,6, &Operand->Comp[0],6,1);
   Mag = sqrt(Mag);
   return(Mag);

} /* End Vector6d_Magnitude() */


/************************************************************************************
**
** Function: Vector6d_MultScalar
**
** Notes: None
*/
void Vector6d_MultScalar (Vector6d *Result, const Vector6d *Left, double Scalar) 
{

   MatrixMxNd_MultScalar(&Result->Comp[0], &Left->Comp[0],6,1,Scalar);

} /* End Vector6d_MultScalar() */


/***********************************************************************************
**
** Function: Vector6d_Normalize
**
** Notes:   None
*/
void Vector6d_Normalize (Vector6d *Result, const Vector6d *Operand) 
{
   double Magnitude;

   Magnitude = Vector6d_Magnitude(Operand);
   Vector6d_DivScalar(Result,Operand,Magnitude);

} /* End Vector6d_Normalize() */


/***********************************************************************************
**
** Function: Vector6d_Sub
**
** Notes:   None
*/
void Vector6d_Sub (Vector6d *Result, const Vector6d *Left, const Vector6d *Right) 
{

   MatrixMxNd_Sub(&Result->Comp[0], &Left->Comp[0],&Right->Comp[0],6,1);

} /* End Vector6d_Sub() */

/* end of file */
