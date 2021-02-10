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
#include "pyhelper.hh"
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
using namespace std;

/**
 * returns a new reference
 */
PyObject* createPyList(const std::vector<double>& dVec)
{
	PyObject* result = PyList_New(dVec.size());
	for(unsigned int i = 0; i < dVec.size(); ++i)
	{
		PyList_SetItem(result,i, PyFloat_FromDouble(dVec[i]) );//SetItem steals the reference, so no need to decref PyFLoatFrom
	}
	return result;
}

PyObject* createPyList(const std::vector<int>& iVec)
{
	PyObject* result = PyList_New(iVec.size());
	for(unsigned int i = 0; i < iVec.size(); ++i)
	{
		PyList_SetItem(result,i, PyInt_FromLong(iVec[i]) );//SetItem steals the reference, so no need to decref PyFLoatFrom
	}
	return result;
}

int CPPDoubleVecFromPyFloatSequence(PyObject* pSeq, std::vector<double>& dVec)
{
	if(!PySequence_Check(pSeq))
	{
		cerr << "-----------------------------------------------------" << endl;
		cerr << "TYPE ERROR: given PythonObject is not of Type PySequence" << endl;
		cerr << "-----------------------------------------------------" << endl;
		return -1;
	}else
	{
		size_t size = PySequence_Size(pSeq);
		for(size_t i = 0; i < size; ++i)
		{
			PyObject* tmp = PySequence_GetItem(pSeq,i);				//PySequenceGetItem returns a New reference, so it is necessary to decref
			dVec.push_back(PyFloat_AsDouble(tmp));
			Py_DECREF(tmp);
		}
		return 0;
	}
}


PyObject* PyIntList2DFromCPPIntVec2D(vector<vector<uint32_t> >& iVec)
{
	PyObject* result = PyList_New(iVec.size());
	for(size_t i = 0; i < iVec.size(); ++i)
	{
		PyObject* innerList = PyList_New(iVec.at(i).size());
		for(size_t j = 0; j < iVec.at(i).size(); ++j)
		{
			PyList_SetItem(innerList, j, PyInt_FromSize_t(iVec.at(i).at(j)));
		}
		PyList_SetItem(result, i, innerList);
	}
	return result;
}

//returns new reference
PyObject* PyFloatList2DFromCPPDoubleVec2D(vector<vector<double> >& dVec)
{
	PyObject* result = PyList_New(dVec.size());
	for(size_t i = 0; i < dVec.size(); ++i)
	{
		PyObject* innerList = PyList_New(dVec.at(i).size());
		for(size_t j = 0; j < dVec.at(i).size(); ++j)
		{
			PyList_SetItem(innerList, j, PyFloat_FromDouble(dVec.at(i).at(j)));
		}
		PyList_SetItem(result, i, innerList);
	}
	return result;
}

int CPPDoubleVec2DFromPyFloatList2D(PyObject* pList, vector<vector<double> >& dVec)
{
	if(!PyList_Check(pList))														//check type
	{
		cerr << "-----------------------------------------------------" << endl;
		cerr << "TYPE ERROR: given PythonObject is not of Type PyList" << endl;
		cerr << "-----------------------------------------------------" << endl;
		return -1;
	}else
	{
		for(int i = 0; i < PyList_Size(pList); ++i)									//for each list within the list
		{
			PyObject* tmp = PyList_GetItem(pList,i);								//get the current inner list
			if(!PyList_Check(tmp))													//check for correct type
			{
				cerr << "-----------------------------------------------------" << endl;
				cerr << "TYPE ERROR: given PythonObject is not of Type PyList" << endl;
				cerr << "-----------------------------------------------------" << endl;
				return -1;
			}else{
				vector<double> tmpVec;												//create a vec that will hold the values of the inner list
				for(int j = 0; j < PyList_Size(tmp); ++j)							//for each value (py-float) of an inner list
				{
					tmpVec.push_back(PyFloat_AsDouble(PyList_GetItem(tmp,j)));		//push the value to the vec
				}
				dVec.push_back(tmpVec);												//push the value to the result
			}
		}
		//print2DVectorToString<double> (dVec);
		return 0;
	}
}

template <typename T> string printVectorToString(const vector<T>& vec)
{
	std::stringstream result;
	result << "[";
	for(size_t i = 0; i < vec.size(); ++i)
	{
		result << vec.at(i);
		result << ", ";
	}
	result << "]";
	return result.str();
}

//Type T must be printable
template <typename T> string print2DVectorToString(const vector<vector<T> >& vec)
{
	stringstream result;
	for(size_t i = 0; i < vec.size(); ++i)
	{
		result << "start element number: " << (i + 1) << " -------------------------------------" << endl;
		for(size_t j = 0; j < vec.at(i).size(); ++j)
		{
			result << vec.at(i).at(j) << endl;
		}
	}
	return result.str();
}












