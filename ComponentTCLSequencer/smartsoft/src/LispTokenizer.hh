// --------------------------------------------------------------------------
//
//  Copyright (C) 2014 Alex Lotz
//
//        lotz@hs-ulm.de
//        schlegel@hs-ulm.de
//
//        Prof. Dr. Christian Schlegel
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
//
//
//  This file is part of ACE/SmartSoft.
//
//  ACE/SmartSoft is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  ACE/SmartSoft is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with ACE/SmartSoft.  If not, see <http://www.gnu.org/licenses/>.
//
// --------------------------------------------------------------------------

#ifndef SMARTLISPTOKENIZER_HH_
#define SMARTLISPTOKENIZER_HH_

#include <string>
#include <vector>

#include "smartCommParameterRequest.hh"

namespace Smart {

/**
 * This class can get a Lisp string using the tokenize method and parse it to a tree structure.
 * Thereby opening brackets are parsed as sublists. In addition string types are preprocessed
 * before parsing the brackets. Example Lisp string is:
 * (ABC (x y z) 1 2 3 (Hello (World)(foo bar)) "This is a text (((9()" 123)
 * Other Lisp keywords (like '´`, etc.) are not supported (resp. are ignored).
 */

class LispTokenizer {
protected:
	// the tree structure (maybe there is a more elegant structure possible but that was the first thing I came up with)
	struct Node {
		std::vector<Node> edges;
		bool isLeafNode;
		std::string leafValue;
	};

	// the root node of the tree
	Node tree;
	int indexer;
	bool tag_found;

	// the recursive, actual parser methods
	void recursiveTokenize(const std::string &s, Node &parentNode);

	void recursivePrint(Node &node, const std::string &indent=" ");
	void recursiveFillCommParamRequest(Node &node, const int &level, SmartACE::CommParameterRequest &req);

public:
	LispTokenizer();
	virtual ~LispTokenizer();

	void parse(const std::string &s);
	void printTree();
	SmartACE::CommParameterRequest getCommParamRequest();
};

} /* namespace Smart */

#endif /* SMARTLISPTOKENIZER_HH_ */
