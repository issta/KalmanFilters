/** \file
** 
** \brief  Define scalar macros and functions to operate on scalars
**
** $Id: scalar.h 1.1 2008/05/21 15:00:23EDT dcmccomas Exp  $
** $Date: 2008/05/21 15:00:23EDT $
** $Revision: 1.1 $
** $Log: scalar.h  $
** Revision 1.1 2008/05/21 15:00:23EDT dcmccomas 
** Initial revision
** Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/inc/project.pj
** Revision 1.2 2005/11/29 07:49:44EST dcmccomas 
** New SDO delivery and added doxygen markup
** Revision 1.1.4.2  2004/05/06 20:51:12  ddawson
** modified quaternion prologs slightly
**
** Revision 1.1.4.1  2004/04/15 18:03:50  ddawson
** formatted comments for doxygen
**
** Revision 1.1  2004/01/23 13:23:00  daviddawsonuser
** directory reorg
**
** Revision 1.4  2004/01/20 19:13:29  daviddawsonuser
** Made changes based on code review
**
** Revision 1.3  2003/12/12 21:06:47  daviddawsonuser
** *** empty log message ***
**
** Revision 1.2  2003/12/10 20:40:07  daviddawsonuser
** clean up based on Bob's review
**
**
*/

/** 
** @addtogroup math_scalar
** @{
*/

#ifndef _scalar_h_
#define _scalar_h_

/******************** Macro Definitions ******************************/

/**
** Returns maximum of two numbers. 
*/
#define    SCALAR_MAX(x,y)          (((x) > (y)) ? (x) : (y))

/**
** Returns minimum of two numbers.
*/
#define    SCALAR_MIN(x,y)          (((x) < (y)) ? (x) : (y))


/************** Exported Function Prototypes *************************/




/**
** \brief Bound a value between upper and lower limits
**
** \par Description
**        Bound the \c InputValue between the \c LowerLimit and \c UpperLimit. 
**        If the value is between the limits the value is passed back, and zero 
**        is returned.  If it is below the lower limit the lower limit 
**        is passed back, and -1 is returned. If it is above the upper 
**        limit the upper limit is passed back and 1 is returned.
**
** \warning
**    Function does not check that \c LowerLimit is less than \c UpperLimit.  
**
** \param[out]  LimitedValue  Pointer to the bounded output value
** \param[in]   InputValue    Input value to be bounded
** \param[in]   LowerLimit    Lower bound for return value
** \param[in]   UpperLimit    Upper bound for return value
**
** \returns
** \retcode -1  \retdesc if \c InputValue was less than \c LowerLimit                 \endcode
** \retcode  0  \retdesc if \c InputValue was between \c LowerLimit and \c UpperLimit \endcode
** \retcode +1  \retdesc if \c InputValue was greater than \c UpperLimit              \endcode
** \endreturns
*/
int Scalar_Limit(double       *LimitedValue,
                 const double InputValue,
                 const double LowerLimit,
                 const double UpperLimit
                 );


#endif  /* #ifndef _scalar_h_ */
/** @} */
