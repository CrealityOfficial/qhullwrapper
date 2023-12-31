
/****************************************************************************
**
** Copyright (c) 2008-2020 C.B. Barber. All rights reserved.
** $Id: //main/2019/qhull/src/libqhullcpp/QhullQh.cpp#6 $$Change: 2953 $
** $DateTime: 2020/05/21 22:05:32 $$Author: bbarber $
**
****************************************************************************/

#//! QhullQh -- Qhull's global data structure, qhT, as a C++ class


#include "libqhullcpp/QhullQh.h"

#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullStat.h"

#include <sstream>
#include <iostream>

#include <stdarg.h>

using std::cerr;
using std::string;
using std::vector;
using std::ostream;

#ifdef _MSC_VER  // Microsoft Visual C++ -- warning level 4
#pragma warning( disable : 4611)  // interaction between '_setjmp' and C++ object destruction is non-portable
#pragma warning( disable : 4996)  // function was declared deprecated(strcpy, localtime, etc.)
#endif

namespace orgQhull {

#//!\name Global variables
const double QhullQh::
default_factor_epsilon= 1.0;

#//!\name Constructor, destructor, etc.

//! Derived from qh_new_qhull[user.c]
QhullQh::
QhullQh()
: qhull_status(qh_ERRnone)
, qhull_message()
, error_stream(0)
, output_stream(0)
, factor_epsilon(QhullQh::default_factor_epsilon)
, use_output_stream(false)
{
    // NOerrors: TRY_QHULL_ not needed since these routines do not call qh_errexit()
    qh_meminit(this, NULL);
    qh_initstatistics(this);
    qh_initqhull_start2(this, NULL, NULL, qh_FILEstderr);  // Initialize qhT
    this->ISqhullQh= True;
}//QhullQh

QhullQh::
~QhullQh()
{
    checkAndFreeQhullMemory();
}//~QhullQh

#//!\name Methods

//! Check memory for internal consistency
//! Free global memory used by qh_initbuild and qh_buildhull
//! Zero the qhT data structure, except for memory (qhmemT) and statistics (qhstatT)
//! Check and free short memory (e.g., facetT)
//! Zero the qhmemT data structure
void QhullQh::
checkAndFreeQhullMemory()
{
#ifdef qh_NOmem
    qh_freeqhull(this, qh_ALL);
#else
    qh_memcheck(this);
    qh_freeqhull(this, !qh_ALL);
    countT curlong;
    countT totlong;
    qh_memfreeshort(this, &curlong, &totlong);
    if (curlong || totlong)
        throw QhullError(10026, "Qhull error: qhull did not free %d bytes of long memory (%d pieces).", totlong, curlong);
#endif
}//checkAndFreeQhullMemory

#//!\name Messaging

void QhullQh::
appendQhullMessage(const string &s)
{
    if(output_stream && use_output_stream && this->USEstdout){
        *output_stream << s;
    }else if(error_stream){
        *error_stream << s;
    }else{
        qhull_message += s;
    }
}//appendQhullMessage

//! clearQhullMessage does not throw errors (~Qhull)
void QhullQh::
clearQhullMessage()
{
    qhull_status= qh_ERRnone;
    qhull_message.clear();
    RoadError::clearGlobalLog();
}//clearQhullMessage

//! hasQhullMessage does not throw errors (~Qhull)
bool QhullQh::
hasQhullMessage() const
{
    return (!qhull_message.empty() || qhull_status!=qh_ERRnone);
    // QH11006 FIX: inconsistent usage with Rbox.  hasRboxMessage just tests rbox_status.  No appendRboxMessage()
}

void QhullQh::
maybeThrowQhullMessage(int exitCode)
{
    if(!NOerrexit){
        if(qhull_message.size()>0){
            qhull_message.append("\n");
        }
        if(exitCode || qhull_status==qh_ERRnone){
            qhull_status= 10073;
        }else{
            qhull_message.append("QH10073: ");
        }
        qhull_message.append("Cannot call maybeThrowQhullMessage() from QH_TRY_().  Or missing 'qh->NOerrexit=true;' after QH_TRY_(){...}.");
    }
    if(qhull_status==qh_ERRnone){
        qhull_status= exitCode;
    }
    if(qhull_status!=qh_ERRnone){
        QhullError e(qhull_status, qhull_message);
        clearQhullMessage();
        //throw e; // QH11007 FIX: copy constructor is expensive if logging
    }
}//maybeThrowQhullMessage

void QhullQh::
maybeThrowQhullMessage(int exitCode, int noThrow)  throw()
{
    QHULL_UNUSED(noThrow);

    if(qhull_status==qh_ERRnone){
        qhull_status= exitCode;
    }
    if(qhull_status!=qh_ERRnone){
        QhullError e(qhull_status, qhull_message);
        e.logErrorLastResort();
    }
}//maybeThrowQhullMessage

//! qhullMessage does not throw errors (~Qhull)
std::string QhullQh::
qhullMessage() const
{
    if(qhull_message.empty() && qhull_status!=qh_ERRnone){
        return "qhull: no message for error.  Check cerr or error stream\n";
    }else{
        return qhull_message;
    }
}//qhullMessage

int QhullQh::
qhullStatus() const
{
    return qhull_status;
}//qhullStatus

void QhullQh::
setErrorStream(ostream *os)
{
    error_stream= os;
}//setErrorStream

//! Updates use_output_stream
void QhullQh::
setOutputStream(ostream *os)
{
    output_stream= os;
    use_output_stream= (os!=0);
}//setOutputStream

}//namespace orgQhull

