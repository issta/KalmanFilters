/* 
** File:
**   $Id: scalar.c 1.1 2008/05/21 15:00:29EDT dcmccomas Exp  $
**
** Purpose: Scalar math library function implementation.
**
** $Id: scalar.c 1.1 2008/05/21 15:00:29EDT dcmccomas Exp  $
** $Date: 2008/05/21 15:00:29EDT $
** $Revision: 1.1 $
** $Log: scalar.c  $
** Revision 1.1 2008/05/21 15:00:29EDT dcmccomas 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/src/project.pj
** Revision 1.2 2005/11/29 07:49:43EST dcmccomas 
** New SDO delivery and added doxygen markup
** Revision 1.1.4.1  2004/04/15 18:03:50  ddawson
** formatted comments for doxygen
**
** Revision 1.1  2004/01/23 13:23:00  daviddawsonuser
** directory reorg
**
** Revision 1.2  2004/01/20 19:13:29  daviddawsonuser
** Made changes based on code review
**
** Revision 1.1.1.1  2003/12/08 20:00:06  daviddawsonuser
** Imported Sources
**
**
*/

/*
** Include Files
*/

#include "scalar.h"

/*
** Exported Functions
*/

/**********************************************************************
** 
** Function: Scalar_Limit
**
*/
int Scalar_Limit(double       *LimitedValue,
		 const double InputValue,
		 const double LowerLimit,
		 const double UpperLimit)
{
  int Status;

  if (InputValue > UpperLimit)
  {
    *LimitedValue = UpperLimit;
    Status = 1;
  } 
  else if (InputValue < LowerLimit)
  {
    *LimitedValue = LowerLimit;
    Status = -1;
  }
  else
  {
    *LimitedValue = InputValue;
    Status = 0;
  }

  return (Status);
}  /* End Scalar_Limit */

/* end of file */
