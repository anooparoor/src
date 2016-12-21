/*!
 * FORRComment.h
 *
 * \brief 
 *
 * \author Eric Schneider <esch@hunter.cuny.edu>
 */
#ifndef FORRCOMMENT_H
#define FORRCOMMENT_H

#include <string>

#include "Position.h"

#include "FORRAction.h"

struct FORRComment
{
    float strength;
    FORRAction action;
    string advisorName;
    bool vetoed;
};

#endif
