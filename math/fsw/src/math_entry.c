/* 
** File:
**   $Id: math_entry.c 1.1 2009/02/18 13:15:45EST myang Exp  $
**
** Purpose: Generic entry point function for math library
**
** References:
**   1. Core Flight Executive Application Developers Guide.
**   2. The GN&C FSW Framework Programmer's Guide
**
**
** $Date: 2009/02/18 13:15:45EST $
** $Revision: 1.1 $
** $Log: math_entry.c  $
** Revision 1.1 2009/02/18 13:15:45EST myang 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/src/project.pj
** Revision 1.1 2008/05/21 15:00:26EDT dcmccomas 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/src/project.pj
** Revision 1.1 2006/07/27 13:28:42EDT jjhageman 
** Initial revision
** Member added to project d:/mksdata/gnc-fsw/math/project.pj
**
*/

/*
** Includes
*/

#include <stdio.h> 

/*
** Exported Functions
*/

/******************************************************************************
** Entry function
**
*/
unsigned int Math_Init(void)
{
  printf("GNC-FSW Math Library 1.0.0 Loaded\n");
  return 0;

} /* End Math_Init() */

