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

#include "LispTokenizer.hh"

#include <sstream>
#include <algorithm>
#include <iostream>

namespace Smart {

// #define DEBUG

// trim from start
static inline std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}

LispTokenizer::LispTokenizer()
: indexer(0)
, tag_found(false)
{  }

LispTokenizer::~LispTokenizer() {  }

void LispTokenizer::recursiveTokenize(const std::string &s, Node &parentNode) {
	std::string copy = s;
	copy = trim(copy);

	size_t first_starting_string_pos = copy.find_first_of('"');
	size_t first_opening_bracket_pos = copy.find_first_of('(');

	if(first_starting_string_pos == 0) {
		size_t string_end_pos = copy.find_first_of('"', 1);

		Node curr_node;
		curr_node.isLeafNode = true;
		// get the string without ""
		curr_node.leafValue = copy.substr(1, string_end_pos-1);
#ifdef DEBUG
		std::cout << "Found String: '" << curr_node.leafValue << "'" << std::endl;
#endif
		parentNode.edges.push_back(curr_node);

		// get the rest after the string
		copy = copy.substr(string_end_pos+1);
#ifdef DEBUG
		std::cout << "Rest after string: '" << copy << "'" << std::endl;
#endif
		this->recursiveTokenize(copy, parentNode);

	} else if(first_opening_bracket_pos == 0) {
		// this is a starting list
		int bracket_counter = 0;
		size_t closing_bracket_pos = 0;

		for(size_t i=first_opening_bracket_pos; i<copy.size(); ++i) {
			// ignore strings in between
			if(copy[i] == '"') {
				i = copy.find_first_of('"', i+1);
				continue;
			}

			// if not a string increment counter for each opening bracket or decrement for each closing bracket
			if(copy[i] == '(') {
				bracket_counter++;
			} else if(copy[i] == ')') {
				bracket_counter--;
			}

			// if the counter reaches 0 value again, this means we have found the closing bracket
			if(bracket_counter == 0) {
				// we have found the closing bracket from the top (current) level
				closing_bracket_pos = i;
				break;
			}
		}
		if(closing_bracket_pos > 1) {
			// now get the sublist which is anything (i.e. further sublists, strings, atomic elements)
			// between the opening bracket "(" and the closing bracket ")",
			// thereby the outer brackets need to be removed before passing to the recursive call!
			std::string sublist = copy.substr(1, closing_bracket_pos-1);
#ifdef DEBUG
			std::cout << "Sublist: " << sublist << std::endl;
#endif

			// the sublist gets a new node passed together with the list itself to the recursive call
			Node new_sub_tree;
			new_sub_tree.isLeafNode = false;
			new_sub_tree.leafValue = "";
			this->recursiveTokenize(sublist, new_sub_tree);
			parentNode.edges.push_back(new_sub_tree);

			// tokenize the rest (skipping the previous close bracket)
			std::string rest = copy.substr(closing_bracket_pos+1);
			this->recursiveTokenize(rest, parentNode);
		}
	} else {
		// first element in "copy" is an atomic element

		// get the position of the last atomic element (which is neither a string nor a next bracket)
		size_t end_pos = (first_starting_string_pos<first_opening_bracket_pos)? first_starting_string_pos:first_opening_bracket_pos;

		std::string atomic_elements = s.substr(0, end_pos);
		std::stringstream ss(atomic_elements);
		// std::cout << "atomic elements: " << atomic_elements << std::endl;

		while(ss.good()) {
			std::string element = "";
			ss >> element;
			element = trim(element);
			if(element.size() > 0) {
				Node curr_node;
				curr_node.isLeafNode = true;
				curr_node.leafValue = element;
#ifdef DEBUG
				std::cout << "Element: " << element << std::endl;
#endif
				parentNode.edges.push_back(curr_node);
			}
		}
		if(end_pos != std::string::npos) {
			// tokenize the rest including sublists
			this->recursiveTokenize(copy.substr(end_pos), parentNode);
		}
	}
}



void LispTokenizer::parse(const std::string &s) {
	this->tree.isLeafNode = false;
	this->tree.leafValue = "";
	this->tree.edges.clear();
	this->recursiveTokenize(s, this->tree);
}

void LispTokenizer::printTree() {
	recursivePrint(tree);
}

void LispTokenizer::recursivePrint(Node &node, const std::string &indent) {
	std::string local_indent = indent + " ";
	if(node.isLeafNode) {
		std::cout << local_indent << node.leafValue << std::endl;
	} else {
		for(size_t i=0; i<node.edges.size(); ++i) {
			std::cout << local_indent << " --" << std::endl;
			this->recursivePrint(node.edges[i], local_indent);
		}
	}
}

void LispTokenizer::recursiveFillCommParamRequest(Node &node, const int &level, SmartACE::CommParameterRequest &req)
{
	if(node.isLeafNode) {
		if(level == 2) {
			// this is the Tag:
			// std::cout << "SetTag: " << node.leafValue << std::endl;
			if(!tag_found) {
				tag_found = true;
				req.setTag(node.leafValue);
			} else {
				indexer++;
				std::stringstream index_ss;
				index_ss << indexer;
				req.setString(index_ss.str(), node.leafValue);
			}
		} else if(level > 2) {
			if(level == 3) indexer++;

			// these are the actual values
			//std::cout << "setString(" << indexer << ", " << node.leafValue << ");" << std::endl;
			std::stringstream index_ss;
			index_ss << indexer;
			req.setString(index_ss.str(), node.leafValue);
		}
	} else {
		if(level == 3) indexer++;
		for(size_t i=0; i<node.edges.size(); ++i) {
			this->recursiveFillCommParamRequest(node.edges[i], level+1, req);
		}
	}
}

SmartACE::CommParameterRequest LispTokenizer::getCommParamRequest() {
	indexer = 0;
	tag_found = false;
	SmartACE::CommParameterRequest req;
	recursiveFillCommParamRequest(tree, 0, req);
	return req;
}


} /* namespace Smart */
