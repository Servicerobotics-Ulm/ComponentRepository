// --------------------------------------------------------------------------
//
//  Copyright (C) 2009 Christian Schlegel
//
//        schlegel@hs-ulm.de
//
//        ZAFH Servicerobotik Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
//
//  This file is part of the "SmartSoft smartPlannerBreadthFirstSearch component".
//  It provides planning services based on grid maps.
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// --------------------------------------------------------------------------


#include "smartPlannerGoalFifo.hh"

#include "stdlib.h"
#include "stdio.h"

#ifdef __cplusplus
extern "C" {
#endif

// -------------------------------------------------------------------------
// read next entry without removing it from the fifo. Returns 0 if 
// everything is ok, 1 if the read operation has reached the end of
// the fifo
// -------------------------------------------------------------------------
int goalFifoNext(GoalFifoStructPtr head_dest,
                 GoalFifoStructPtr tail_dest,
                 GoalFifoStructPtrPtr ptr,
                 int *entry,
                 double *x1,double *y1,double *x2,double *y2)
{
  if ((*ptr)->next == tail_dest) return 1;
  *ptr   = (*ptr)->next;
  *x1    = (*ptr)->x1;
  *y1    = (*ptr)->y1;
  *x2    = (*ptr)->x2;
  *y2    = (*ptr)->y2;
  *entry = (*ptr)->type;

  return 0;
}

// ------------------------------------------------------------------------
// read next entry from fifo and remove it. Returns 0 if everything is
// ok, 1 if there are no more entries available
// ------------------------------------------------------------------------
int goalFifoRead(GoalFifoStructPtr head_dest,
                 GoalFifoStructPtr tail_dest,
                 int *entry,
                 double *x1,double *y1,double *x2,double *y2)
{
  GoalFifoStructPtr old;

  if ((head_dest->next) == tail_dest) return 1;
  old=tail_dest->previous;
  *x1    = old->x1;
  *y1    = old->y1;
  *x2    = old->x2;
  *y2    = old->y2;
  *entry = old->type;

  tail_dest->previous=old->previous;
  (old->previous)->next=tail_dest;
  free(old);

  return 0;
}

// ------------------------------------------------------------------------
// initialize fifo for use. Returns 0 if everything is ok, 1 otherwise
// ------------------------------------------------------------------------
int goalFifoInit(GoalFifoStructPtrPtr head_dest,GoalFifoStructPtrPtr tail_dest)
{
  if ((*head_dest=(GoalFifoStructPtr)calloc(1,sizeof(GoalFifoType)))==NULL)
    return 1;
  if ((*tail_dest=(GoalFifoStructPtr)calloc(1,sizeof(GoalFifoType)))==NULL) {
    free(*head_dest);
    return 1;
  }

  (*head_dest)->next     = *tail_dest;
  (*head_dest)->previous = NULL;
  (*tail_dest)->next     = NULL;
  (*tail_dest)->previous = *head_dest;

  return 0;
}

// -----------------------------------------------------------------------
// put data into fifo. Returns 0 if everything is ok, 1 otherwise
// -----------------------------------------------------------------------
int goalFifoWrite(GoalFifoStructPtr head_dest,GoalFifoStructPtr tail_dest,
                  int entry,double x1,double y1,double x2,double y2)
{
  GoalFifoStructPtr new_;

  if ((new_=(GoalFifoStructPtr)calloc(1,sizeof(GoalFifoType)))==NULL) return 1;
  new_->x1   = x1;
  new_->y1   = y1;
  new_->x2   = x2;
  new_->y2   = y2;
  new_->type = entry;
  new_->next = head_dest->next;
  new_->next->previous=new_;
  new_->previous=head_dest;
  head_dest->next=new_;

  return 0;
}

// ----------------------------------------------------------------------
// empty the fifo. Always returns 0
// ----------------------------------------------------------------------
int goalFifoFree(GoalFifoStructPtr head_dest,GoalFifoStructPtr tail_dest)
{
  double x1,y1,x2,y2;
  int    type;
  int    status;

  status=goalFifoRead(head_dest,tail_dest,&type,&x1,&y1,&x2,&y2);
  while (status==0) status=goalFifoRead(head_dest,tail_dest,&type,&x1,&y1,&x2,&y2);

  return 0;
}

#ifdef __cplusplus
}
#endif

