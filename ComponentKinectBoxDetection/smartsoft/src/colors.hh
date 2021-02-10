// color codes to use with cout for colored text printing on console.
// see http://en.wikipedia.org/wiki/ANSI_escape_code#graphics
// usage: std::cout << COLOR_RED << "hello, this is red, " << COLOR_DEFAULT << "this is default" << std::endl;


#ifndef __COLORS_HH
#define __COLORS_HH __COLORS_HH

#define COLOR_RED	"\033[0;31m"
#define COLOR_GREEN	"\033[0;32m"
#define COLOR_YELLOW	"\033[0;33m"
#define COLOR_BLUE	"\033[0;34m"
#define COLOR_MAGENTA	"\033[0;35m"
#define COLOR_CYAN	"\033[0;36m"

#define COLOR_DEFAULT 	"\033[0m"

#endif
