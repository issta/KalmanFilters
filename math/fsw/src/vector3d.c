/* 
** File:
**   $Id: vector3d.c 1.1 2008/05/21 15:00:29EDT dcmccomas Exp  $
**
** Purpose: Provide the implementation of the Vector3d package. 
**
**  $Id: vector3d.c 1.1 2008/05/21 15:00:29EDT dcmccomas Exp  $
**  $Date: 2008/05/21 15:00:29EDT $
**  $Revision: 1.1 $
**  $Log: vector3d.c  $
**  Revision 1.1 2008/05/21 15:00:29EDT dcmccomas 
**  Initial revision
**  Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/src/project.pj
**  Revision 1.2 2005/11/29 07:49:53EST dcmccomas 
**  New SDO delivery and added doxygen markup
**  Revision 1.3.8.1  2005/11/17 04:03:56  ddawson
**  Modified comments for doxygen
**
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

#include "vector3d.h"
#include "matrixmxnd.h"
#include <math.h>

/*
** Exported Functions
*/

/***********************************************************************************
**
** Function: Vector3d_Add
**
** Notes:   None
*/

void Vector3d_Add (Vector3d *Result, const Vector3d *Left, const Vector3d *Right) 
{

   MatrixMxNd_Add(&Result->Comp[0], &Left->Comp[0],&Right->Comp[0],3,1);

} /* End Vector3d_Add() */


/*********************************************************************************
**
** Function: Vector3d_Copy
**
** Notes:   None
*/

void Vector3d_Copy (Vector3d *Result, const Vector3d *Operand) 
{

   MatrixMxNd_Copy(&Result->Comp[0], &Operand->Comp[0],3,1);

} /* End Vector3d_Copy() */

/***********************************************************************************
**
** Function: Vector3d_Cross
**
** Notes:   
**  1. The local variable Cross allows the caller to set the  result to be one of the
**     the input vectors. 
*/
void Vector3d_Cross (Vector3d *Result, const Vector3d *Left, const Vector3d *Right)
{

  Vector3d Cross;
  
  Cross.Comp[0] = Left->Comp[1] * Right->Comp[2] -
                  Left->Comp[2] * Right->Comp[1];
  Cross.Comp[1] = Left->Comp[2] * Right->Comp[0] -
                  Left->Comp[0] * Right->Comp[2];  
  Cross.Comp[2] = Left->Comp[0] * Right->Comp[1] -
                  Left->Comp[1] * Right->Comp[0];
  MatrixMxNd_Copy(&Result->Comp[0], &Cross.Comp[0],3,1);

} /* End Vector3d_Cross */

/***********************************************************************************
**
** Function: Vector3d_DivScalar
**
** Notes: None
*/

void Vector3d_DivScalar (Vector3d *Result, const Vector3d *Left, double Scalar) 
{
   double ScalarInv;

   ScalarInv = 1.0 / Scalar;
   MatrixMxNd_MultScalar(&Result->Comp[0], &Left->Comp[0],3,1,ScalarInv);

} /* End Vector3d_DivScalar() */

/***********************************************************************************
**
** Function: Vector3d_Dot
**
** Notes:   None
*/
double Vector3d_Dot ( const Vector3d *Left, const Vector3d *Right)
{
   double Dot;

   MatrixMxNd_Mult( &Dot, &Left->Comp[0],1,3, &Right->Comp[0],3,1);
   return (Dot);
   
} /* End of Vector3d_Dot */

/***********************************************************************************
**
** Function: Vector3d_FromRaDec
**
** Notes:   None
*/
void Vector3d_FromRaDec ( Vector3d *Result, double Ra, double Dec)
{
   double CosDec;
   double CosRa;
   double SinRa;
   double SinDec;

   CosDec = cos(Dec);
   CosRa = cos(Ra);
   SinRa = sin(Ra); 
   SinDec = sin(Dec);
   
   Result->Comp[0] = CosDec * CosRa;
   Result->Comp[1] = CosDec * SinRa;
   Result->Comp[2] = SinDec;

} /* end Vector3d_FromRaDec */

/*********************************************************************************
**
** Function: Vector3d_InitZero
**
** Notes:   None
*/

void Vector3d_InitZero (Vector3d *Result) 
{

   MatrixMxNd_InitZero(&Result->Comp[0],3,1);

} /* End Vector3d_InitZero() */

/********************************************************************************
**
** Function: Vector3d_Magnitude
**
** Notes:  None  
**  
*/

double Vector3d_Magnitude (const Vector3d *Operand) 
{
  double Mag;
  
  MatrixMxNd_Mult( &Mag, &Operand->Comp[0],1,3, &Operand->Comp[0],3,1);
  Mag = sqrt(Mag);
  return(Mag);

} /* End Vector3d_Magnitude() */

/************************************************************************************
**
** Function: Vector3d_MultScalar
**
** Notes: None
*/

void Vector3d_MultScalar (Vector3d *Result, const Vector3d *Left, double Scalar) 
{

   MatrixMxNd_MultScalar(&Result->Comp[0], &Left->Comp[0],3,1,Scalar);

} /* End Vector3d_MultScalar() */


/***********************************************************************************
**
** Function: Vector3d_Normalize
**
** Notes:   None
*/

void Vector3d_Normalize (Vector3d *Result, const Vector3d *Operand) 
{
   double Magnitude;

   Magnitude = Vector3d_Magnitude(Operand);
   Vector3d_DivScalar(Result,Operand,Magnitude);

} /* End Vector3d_Normalize() */


/***********************************************************************************
**
** Function: Vector3d_Sub
**
** Notes:   None
*/

void Vector3d_Sub (Vector3d *Result, const Vector3d *Left, const Vector3d *Right) 
{

   MatrixMxNd_Sub(&Result->Comp[0], &Left->Comp[0],&Right->Comp[0],3,1);

} /* End Vector3d_Sub() */

/* end of file */
