//--------------------------------------------------------------------------
//  Copyright (C) 2012 Timo Hegele
//
//        hegele@mail.hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartOpenRave component".
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
//---------------------------------------------------------------------
#ifndef _PYHELPER_HH_
#define _PYHELPER_HH_

#include "python2.7/Python.h"
#include <vector>
#include <string>


using namespace std;
PyObject* createPyList(const std::vector<double>& dVec);
PyObject* createPyList(const std::vector<int>& dVec);
int CPPDoubleVecFromPyFloatSequence(PyObject* pSeq, std::vector<double>& dVec);
int CPPDoubleVec2DFromPyFloatList2D(PyObject* pList, vector<vector<double> >& dVec);
PyObject* PyFloatList2DFromCPPDoubleVec2D(vector<vector<double> >& dVec);
PyObject* PyIntList2DFromCPPIntVec2D(vector<vector<uint32_t> >& iVec);
template <typename T> string printVectorToString(const vector<T>& vec);
template <typename T> string print2DVectorToString(const vector<vector<T> >& vec);


#endif //_PYHELPER_HH_
