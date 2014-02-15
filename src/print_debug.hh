/*
 * FILE: print_debug.hh
 * DESCRIPTION:
 * Several macros for debugging information.
 */

#ifndef PRINT_DEBUG_HH_
#define PRINT_DEBUG_HH_

#include <iostream>
#include <cstdlib>
#include <cassert>

/* Return codes for program.*/
#define RC_OK                 (0) // Everything went ok
#define RC_INVALID_ARGUMENTS  (1) // Invalid arguments given to the program
#define RC_INPUT_ERROR        (2) // Invalid input to the program
#define RC_UNKNOWN_ERROR      (3) // Other (unknown) errors

using std::cout;
using std::cerr;
using std::endl;

#ifndef NDEBUG //debugger code

#define ERR_POS \
std::cerr << __FILE__ << ':' << __LINE__ << " [" << __PRETTY_FUNCTION__ << "]\n"

#define DEBUG_EXEC(code) code

#else //release code

#define ERR_POS void(0);
#define DEBUG_EXEC(code) void(0);

#endif //NDEBUG

#endif // PRINT_DEBUG_HH_
