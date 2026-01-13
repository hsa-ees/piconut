/************************************************************************
*
  This file is part of the PicoNut project.

  Copyright (C) 2010-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
                          Marco Milenkovic <marco.milenkovic@hs-augsburg.de>
                     2024 Lukas Bauer <lukas.bauer1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This module contains various types, constants and helper functions
    for the SystemC model of PicoNut.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/


/// @file
/// @brief Helpers, Makros and performance measuring Classes used in most *PicoNut* files.
/// @defgroup helpers Helpers
/// @brief Helpers, Makros and performance measuring Classes used in most *PicoNut* files.
/// @{


// clang-format off
// There are too many macros in this file so we disable clang-format for this file.


#ifndef __PN_BASE_H__
#define __PN_BASE_H__

#include <systemc.h>





// *************************** Basic Types and Constants ***********************


/// @name Basic types and constants...
/// @{

/// @brief Number of Bytes in a Kilobyte.
#define KB 1024U
/// @brief Number of Bytes in a Megabyte.
#define MB (1024 * 1024)U
/// @brief Number of Bytes in a Gigabyte.
#define GB (1024 * 1024 * 1024)U

/// @brief Minimum of A and B.
#define MIN(A, B) ((A) < (B) ? (A) : (B))
/// @brief Maximum of A and B.
#define MAX(A, B) ((A) > (B) ? (A) : (B))

///@} // @name Basic types and constants...





// *************************** Command Line Options ****************************


/// @name Command line options ...
/// @{

/// @brief VCD trace level.
///
/// Holds the VCD trace level (0 = no VCD file) set by the "-t" command line option supplied to the
/// MPicoNutSystem constructor.
// TBD: replace by a function `pn_arg_vcd_level()`
//~ #ifndef __SYNTHESIS__
extern int pn_cfg_vcd_level;
//~ #else
//~ #define pn_cfg_vcd_level 0
//~ #endif

/// @brief Internal simulation instruction trace level.
///
/// Holds the internal simulation instruction trace level set by the "-i" command line option supplied to the
/// MPicoNutSystem constructor.
/// \arg 0 - no trace
/// \arg 1 - instruction Trace (shows register changes)
/// \arg >= 2 - same as 1 + prints complete register file after each instruction
// TBD: replace by a function `pn_arg_itrace_level()`
extern int pn_cfg_itrace_level;

/// @brief Cache enable override.
///
/// Holds the disable cache bit set by the "-c" command line option supplied to the MPicoNutSystem constructor.
/// If set, disables instruction and data caching irrespective of the value in the pncache CSR.
// TBD: replace by a function `pn_arg_disable_cache()`
extern bool pn_cfg_disable_cache;

/// @brief Interactive debug mode enable.
///
/// Holds the debug mode enable bit set by the "-d" command line option supplied to the MPicoNutSystem
/// constructor. If set, the MPicoNutSystem will set up a remote bitbang interface on port 9824 and waits
/// for you to connect to it. Use OpenOCD and GDB to debug software running in the SystemC model.
///
/// See doc/paranut_manual.pdf for more information.
// TBD: replace by a function `pn_arg_debug_mode()`
extern bool pn_cfg_debug_mode;

/// @brief Disable assert abort
///
/// Holds the disable assert abort bit set by the "-a" command line option supplied to the the
/// PN_PARSE_CMD_ARGS() function. If set, a PN_ASSERT(), PN_ASSERTF() or PN_ASSERTM() will not abort
/// the program execution but only print the error message.
// TBD: Eliminate (what is this needed for? -> indroduce PN_ASSERT_WARN()?)
//      Note: a *compile-time* option to disable all PN_ASSERT*() and PN_INFO*() invocations may speed up simulation.
extern bool pn_disable_assert_abort;

/// @brief Enable to trace core requests.
///
/// If set, the core requests are traced and written to a file
// TBD: What are "core requests"?
extern bool pn_cfg_trace_core;

/// @brief Enables the parser to make use of the trace_core flag.
///
/// Enables the -c command to set pn_cfg_trace_core to true.
/// If set, the core requests are traced and written to a file
// TBD: Why is this needed? Make the default "true"
extern bool pn_parse_enable_trace_core; // enable to trace core requests

/// @brief Enable application path
///
/// If set, the testbench accepts a path to an application which can be used
/// inside the testbench.
// TBD: Why is this needed?
extern bool pn_cfg_enable_application_path;

/// @brief Application path
///
/// The path to the application which is executed inside the tesbench.
// TBD: Why is this needed?
extern std::string pn_cfg_application_path;

/// @brief Enable memory dump at begin and end of simulation
extern bool pn_cfg_dump_memory;

/// @name Riscof
/// @{
/// The path to where to store the signature file for a test run.
// TBD: Replace by a function `const char *pn_arg_riscof_signature_file()`
extern std::string pn_cfg_signature_path;
/// The start address of the signature.
// TBD: Replace by a function `const char *pn_arg_riscof_signature_begin_addr()`
extern uint64_t pn_cfg_signature_begin_addr; // Changed to uint64_t
/// The end address of the signature.
// TBD: Replace by a function `const char *pn_arg_riscof_signature_end_addr()`
extern uint64_t pn_cfg_signature_end_addr; // Changed to uint64_t
///@} // @name Riscof

///@} // @name Command line options ...





// *************************** Heap Helpers ************************************


/// @name Heap helpers (C-style) ...
///
/// Type-safe and null-pointer-aware heap operations.
///
/// @{
#define PN_MALLOC(T, N) (T*) malloc (sizeof (T) * (N))
  ///< @brief Allocate an array of `N` objects of type `T` (type-safe)
#define PN_REALLOC(T, P, N) (T*) realloc (P, sizeof (T) * (N))
  ///< @brief Reallocate an array of `N` objects of type `T` (type-safe)
#define PN_SETP(P, X) { if (P) free (P); P = (X); }
  ///< @brief Replace `P` with a new object/array, eventually free'ing the old one.
#define PN_FREEP(P) PN_SETP(P, NULL)
  ///< @brief Free object/array `P` and set `P = NULL`, but only if `P` was `!= NULL`.
/// @}


/// @name Heap helpers (C++-style) ...
///
/// Type-safe and null-pointer-aware heap operations.
///
/// @{
#define PN_SETO(O, X) { if (O) delete O; O = (X); }
  ///< @brief Replace `O` with a new object, eventually deleting the old one.
#define PN_FREEO(O) PN_SETO(O, NULL)
  ///< @brief Free object `O`, but only if `O != NULL`

#define PN_SETA(A, X) { if (A) delete [] A; A = (X); }
  ///< @brief Replace `A` with a new array, eventually deleting the old one.
#define PN_FREEA(A) PN_SETA(A, NULL)
  ///< @brief Free array `A`, but only if `A != NULL`
/// @}



// Implement sc_new + sc_new_array for Accelera SystemC ...
//   These template functions are used by ISCS.

#ifndef __SC_TOOL__

namespace sc_core {

  template<class T, class... Args> T* sc_new(Args&&... args)
  {
      T* rptr = ::new T(std::forward<Args>(args)...);
      return rptr;
  }

  template<class T> T* sc_new_array(size_t n)
  {
      T* rptr = ::new T[n];
      return rptr;
  }
}

#endif // __SC_TOOL__





// *************************** SystemC Tracing *********************************


/// @name SystemC tracing...
/// @{


/// @brief Convinient macro to automatically write signal initialization with "name" parameter
/// @name converts user input into a stringified name
/// differentiate between Vivado naming and ICSC naming
#define PN_NAME(name) name{#name}
#define PN_NAME_VEC(name, n) name{#name, n}


/// @brief Output verbose tracing information.
///
/// If set, the system modules will output verbose information when they trace their internal
/// signals.
extern bool pn_trace_verbose;


/// @brief Generates and returns a trace file compatible string.
///
/// Generates and returns a trace file compatible string using the supplied information to
/// parse multidimensional inputs.
/// \param obj is the sc_object (port/signal) pointer.
/// \param name is the base name appearing in the trace file.
/// \param dim is the number of dimensions the obj has (only 0 to 2 are supported).
/// \param arg1 is the number to add to the base name to represent the first dimension
/// (should be 0 for dim < 1).
/// \param arg2 is the number to add to the base name + first dimension to represent the second
/// dimension (should be 0  for dim < 2).
/// \return A std::string formatted for the sc_trace() function (e.g. MODULE.name(arg1)(arg2)).
#ifndef __SYNTHESIS__
std::string pn_get_trace_name (sc_object *obj, const char *name, int dim, int arg1, int arg2);
#endif


#ifndef __SYNTHESIS__


/// @brief Open a trace (VCD) file if tracing is enabled.
///
/// Read the \p pn_cfg_vcd_level and if it is greater than 0, open a VCD trace file with the
/// supplied filename.
/// \param FNAME is the filename of the trace file that is opened.
// TBD: Eliminate: users can call pn_tb_start_trace() directly. On synthesis, pn_tb_start_trace() can be an empty (inline) function.
#define PN_BEGIN_TRACE(FNAME) pn_tb_start_trace(FNAME)

/// @brief Parse the command line arguments and set the global variables
///
/// Process the command line arguments given from the \p sc_main function and set the global variables.
/// Also print the usage information if the \arg "-h" command line option is supplied.
/// And print information about the parsed command line arguments.
///
/// Parse the command line arguments:
/// \arg "-t" sets the VCD trace level (0 = no VCD file) into \p pn_cfg_vcd_level.
///
/// \param ARGC is the number of command line arguments of the \p sc_main function.
/// \param ARGV is the command line arguments of the \p sc_main function.
// TBD: Eliminate: users can call pn_tb_parse_cmd_args() directly. On synthesis, pn_tb_parse_cmd_args() can be an empty (inline) function.
#define PN_PARSE_CMD_ARGS(ARGC, ARGV) pn_tb_parse_cmd_args(ARGC, ARGV)


/// @brief Close the trace file.
///
/// Close the trace file saved in the global variable \p pn_trace_file if
/// \p pn_trace_file is not NULL.
// TBD: Eliminate: users can call pn_tb_end_trace() directly. On synthesis, pn_tb_end_trace() can be an empty (inline) function.
#define PN_END_TRACE() pn_tb_end_trace()


/// @brief Add sc_object OBJ to the trace file TF.
///
/// Macro for easily adding the sc_object (port/signal) to a trace file. Uses pn_get_trace_name to
/// generate the correct object name.
/// \param TF is the trace file.
/// \param OBJ is the sc_object (port/signal).
#define PN_TRACE(TF, OBJ)                                                           \
    {                                                                               \
        if (TF) sc_trace (TF, OBJ, pn_get_trace_name (&(OBJ), #OBJ, 0, 0, 0));      \
        if (pn_trace_verbose) cout << "  " #OBJ " = '" << (OBJ).name () << "'\n";   \
    }


/// @brief Add each sc_object of OBJ array/bus to the trace file TF.
///
/// Macro for easily adding the array/bus of sc_objects (port/signal) to a trace file. Uses
/// pn_get_trace_name to generate the correct object name.
/// \param TF is the trace file.
/// \param OBJ is the array/bus of sc_objects (port/signal).
/// \param N_MAX is number of array/bus elements.
#define PN_TRACE_BUS(TF, OBJ, N_MAX)                                                        \
    {                                                                                       \
        for (uint n = 0; n < N_MAX; n++) {                                                  \
            if (TF) sc_trace (TF, (OBJ)[n], pn_get_trace_name (&(OBJ)[n], #OBJ, 1, n, 0));  \
            if (pn_trace_verbose)                                                           \
                cout << "  " #OBJ "[" << n << "] = '" << (OBJ)[n].name () << "'\n";         \
        }                                                                                   \
    }

#define PN_TRACE_BUS_FROM(TF, OBJ, N_MIN, N_MAX)                                            \
    {                                                                                       \
        for (uint n = N_MIN; n < N_MAX; n++) {                                              \
            if (TF) sc_trace (TF, (OBJ)[n], pn_get_trace_name (&(OBJ)[n], #OBJ, 1, n, 0));  \
            if (pn_trace_verbose)                                                           \
                cout << "  " #OBJ "[" << n << "] = '" << (OBJ)[n].name () << "'\n";         \
        }                                                                                   \
    }

/// @brief Add each sc_object of 2D OBJ array/bus to the trace file TF.
///
/// Macro for easily adding the two dimensional array/bus of sc_objects (port/signal) to a trace file.
/// Uses pn_get_trace_name to generate the correct object name.
/// \param TF is the trace file.
/// \param OBJ is the array/bus of sc_objects (port/signal).
/// \param N_MAX is number of array/bus elements in the first dimension.
/// \param K_MAX is number of array/bus elements in the second dimension.
#define PN_TRACE_BUS_BUS(TF, OBJ, N_MAX, K_MAX)                                                         \
    {                                                                                                   \
        for (uint n = 0; n < N_MAX; n++)                                                                \
            for (uint k = 0; k < K_MAX; k++) {                                                          \
                if (TF) sc_trace (TF, (OBJ)[n][k], pn_get_trace_name (&(OBJ)[n][k], #OBJ, 2, n, k));    \
                if (!TF || pn_trace_verbose)                                                            \
                    cout << "  " #OBJ "[" << n << "][" << k << "] = '" << (OBJ)[n][k].name () << "'\n"; \
            }                                                                                           \
    }

/// @brief Helper macro for recursively calling sc_trace in own types/structs.
///
/// Macro for easily adding a sc_object member (port/signal) of an sc_object to a trace file.
/// Use PN_TRACE_R when overloading sc_trace for own types/structs to easily trace its members.
/// \param TF is the trace file.
/// \param OBJ is the sc_objects having the member MEMBER.
/// \param MEMBER is the member sc_object (port/signal).
/// \param STR is the calling sc_trace string to which the member name will be appended.
#define PN_TRACE_R(TF, OBJ, MEMBER, STR)                        \
    {                                                           \
        if (TF) sc_trace (TF, (OBJ).MEMBER, STR + "."#MEMBER);  \
    }

/// @brief Helper macro for recursively calling sc_trace in own types/structs.
///
/// Macro for easily adding each sc_object of an array member (ports/signals) of an sc_object to a
/// trace file.
/// Use PN_TRACE_R_BUS when overloading sc_trace for own types/structs to easily trace its members.
/// \param TF is the trace file.
/// \param OBJ is the sc_objects having the member MEMBER.
/// \param MEMBER is the member sc_object (port/signal).
/// \param STR is the calling sc_trace string to which the member name + index will be appended.
/// \param N_MAX is number of MEMBER array/bus elements.
#define PN_TRACE_R_BUS(TF, OBJ, MEMBER, STR, N_MAX)                           \
    {                                                                         \
        std::stringstream ss;                                                 \
        for (uint n = 0; n < N_MAX; n++) {                                    \
            ss << n;                                                          \
            if (TF) sc_trace (TF, (OBJ).MEMBER[n], STR + "."#MEMBER"(" + ss.str().c_str() + ")"); \
        }                                                                     \
    }

/// @brief Helper macro for recursively calling sc_trace in own types/structs.
///
/// Macro for easily adding each sc_object of an two-dimensional array member (ports/signals) of an sc_object to a
/// trace file.
/// Use PN_TRACE_R_BUS_BUS when overloading sc_trace for own types/structs to easily trace its members.
/// \param TF is the trace file.
/// \param OBJ is the sc_objects having the member MEMBER.
/// \param MEMBER is the member sc_object (port/signal).
/// \param STR is the calling sc_trace string to which the member name + index will be appended.
/// \param N_MAX is number of array/bus elements in the first dimension.
/// \param K_MAX is number of array/bus elements in the second dimension.
#define PN_TRACE_R_BUS_BUS(TF, OBJ, MEMBER, STR, N_MAX, K_MAX)                \
    {                                                                         \
        std::stringstream ss_n, ss_k;                                         \
        for (uint n = 0; n < N_MAX; n++) {                                    \
            for (int k = 0; k < K_MAX; k++) {                                 \
                ss_n << n;                                                    \
                ss_k << k;                                                    \
                if (TF) sc_trace (TF, (OBJ).MEMBER[n][k], STR + "."#MEMBER"(" + ss_n.str().c_str() + ")(" + ss_k.str().c_str() + ")");             \
            }                                                                 \
        }                                                                     \
    }


#else  // __SYNTHESIS__

#define PN_BEGIN_TRACE(FNAME) NULL
#define PN_PARSE_CMD_ARGS(ARGC, ARGV)
#define PN_END_TRACE()
#define PN_TRACE(TF, OBJ)
#define PN_TRACE_BUS(TF, OBJ, N_MAX)
#define PN_TRACE_BUS_BUS(TF, OBJ, N_MAX, K_MAX)
#define PN_TRACE_R(TF, OBJ, MEMBER, STR)
#define PN_TRACE_R_BUS(TF, OBJ, MEMBER, STR, N_MAX)
#define PN_TRACE_R_BUS_BUS(TF, OBJ, MEMBER, STR, N_MAX, K_MAX)

#endif  // __SYNTHESIS__


///@} // @name SystemC tracing...





// ***************** Assertions and Info/Warning/Error Macros ******************


/// @name Testbench helpers...
/// @{

/// @brief Parses the command line arguments and sets the global variables accordingly.
///
/// Parses the command line arguments given from the \p sc_main function and sets the global variables
/// Also prints the usage information if the \arg "-h" command line option is supplied.
/// And prints information about the parsed command line arguments.
///
/// \arg "-t" sets the VCD trace level (0 = no VCD file) into \p pn_cfg_vcd_level.
///
/// @param argc the number of command line arguments of the \p sc_main function.
/// @param argv the command line arguments of the \p sc_main function.
void pn_tb_parse_cmd_args(const int argc, char** argv);

/// @brief PicoNut trace file pointer.
///
/// If set the trace file is closed using sc_close_vcd_trace_file() after errors and asserts.
// TBD: Remove from interface: For this purpose, it is only requried internally by pn_tb_start_trace() and pn_tb_end_trace().
extern sc_trace_file *pn_trace_file;

/// @brief  Creates a trace file if tracing is enabled.
///
/// Creates a trace file if the global variable \p pn_cfg_vcd_level is greater than 0.
/// It also returns a pointer to the trace file.
///
/// @param filename the name of the trace file that is created.
/// @return a pointer to the trace file saved in the global variable \p pn_trace_file.
// TBD: What happens and what is returned if pn_cfg_vcd_level == 0?
// TBD: Change to let it always create a trace file.
// TBD: Eliminate global variable pn_trace_file & adapt doc.
// TBD: Establish constistent naming ("pn_cfg_vcd_level" vs. "pn_trace_file") -> "*vcd*"
sc_trace_file* pn_tb_start_trace(const char* filename);

/// @brief Closes the trace file.
///
/// Closes the trace file saved in the global variable \p pn_trace_file if it is not NULL.
///
/// @param tf
void pn_tb_end_trace();

/// @brief Testbench printf helper.
///
/// Returns a C string formatted according to supplied format and addistional arguments.
///
/// \param format is a C string that contains the text to be formatted.
/// \param ... (additional arguments) are the parameters according to the format string.
/// \return formatted C string according to supplied format and additional arguments.
// TBD: Eliminate or document the benefit over snprintf() / asprintf()!!
char *pn_tb_printf (const char *format, ...);

/// @brief Testbench assert helper.
///
/// Asserts condition cond and if it fails prints the supplied msg, filename and line to stderr, closes
/// the pn_trace_file and aborts program execution.
///
/// Should be called through the PN_ASSERT(), PN_ASSERTF() and PN_ASSERTM() macros.
///
/// \param cond is the condition to assert.
/// \param warn_only decides whether to abort (false) or continue (true).
/// \param msg is a C string to print if the assertion fails.
/// \param filename is a C string to print if the assertion fails containig the source filename.
/// \param line is the line number in the source file to print if the assertion fails.
void pn_tb_assert (bool cond, bool warn_only, const char* msg, const char* filename, const int line);

/// @brief Testbench information helper.
///
/// Prints the supplied msg to stderr with a defined header containing the current SytemC simulation
/// time stamp, filename and line and the "(INFO):" keyword.
///
/// Should be called through the PN_INFO() and PN_INFOF() macros.
///
/// \param msg is a C string to print.
/// \param filename is a C string to print containig the source filename.
/// \param line is the line number in the source file to print.
void pn_tb_info (const char* msg, const char* filename, const int line);

/// @brief Testbench warning helper.
///
/// Prints the supplied msg to stderr with a defined header containing the current SytemC simulation
/// time stamp, filename and line and the "(WARNING):" keyword.
///
/// Should be called through the PN_WARNING() and PN_WARNINGF() macros.
///
/// \param msg is a C string to print.
/// \param filename is a C string to print containig the source filename.
/// \param line is the line number in the source file to print.
void pn_tb_warning (const char* msg, const char* filename, const int line);

/// @brief Testbench error helper.
///
/// Prints the supplied msg to stderr with a defined header containing the current SystemC simulation
/// time stamp, filename and line and the "(ERROR):" keyword. Also closes the pn_trace_file and
/// exits the program execution with an error.
///
/// Should be called through the PN_ERROR() and PN_ERRORF() macros.
///
/// \param msg is a C string to print.
/// \param filename is a C string to print containig the source filename.
/// \param line is the line number in the source file to print
void pn_tb_error (const char* msg, const char* filename, const int line);

/// @brief Print a message to a file.
///
/// Prints the supplied msg to the supplied file with a defined header containing the current
/// SystemC simulation time stamp, filename and line.
///
/// Should be called through the PN_INFO_FILE() macro.
///
/// \param msg is a C string to print.
/// \param filename is a C string to print containig the source filename.
/// \param line is the line number in the source file to print
/// \param coreDumpFile is the file to print the message to.
void pn_info_file(const char *msg, const char *filename, const int line, FILE *coreDumpFile);


#ifndef __SYNTHESIS__


/// @brief Testbench assert without message.
///
/// Calls pn_tb_assert() with the condition COND, an empty message and the filename and line macros.
///
/// \param COND is the condition to assert.
#define PN_ASSERT(COND) pn_tb_assert (COND, false, NULL, __FILE__, __LINE__)

/// @brief Testbench assert with formatted message.
///
/// Calls pn_tb_assert() with the condition COND, the formatted message according to FMT and the
/// filename and line macros.
///
/// __NOTE__: You need to put the FMT argument in brackets (e.g. PN_ASSERTF(1, ("E: %d", 5));)
///
/// \param COND is the condition to assert.
/// \param FMT is the format C string and all additional arguments needed to print if the assertion fails.
#define PN_ASSERTF(COND, FMT) pn_tb_assert (COND, false, pn_tb_printf FMT, __FILE__, __LINE__)

/// @brief Testbench assert with message.
///
/// Calls pn_tb_assert() with the condition COND, the supplied message MSG and the
/// filename and line macros.
///
///
/// \param COND is the condition to assert.
/// \param MSG is the C string to print if the assertion fails.
#define PN_ASSERTM(COND, MSG) pn_tb_assert (COND, false, MSG, __FILE__, __LINE__)


/// @brief Warning assert without message.
///
/// Calls pn_tb_assert() with the condition COND, an empty message and the filename and line macros.
///
/// \param COND is the condition to assert.
#define PN_ASSERT_WARN(COND) pn_tb_assert (COND, true, NULL, __FILE__, __LINE__)

/// @brief Warning assert with formatted message.
///
/// Calls pn_tb_assert() with the condition COND, the formatted message according to FMT and the
/// filename and line macros.
///
/// __NOTE__: You need to put the FMT argument in brackets (e.g. PN_ASSERTF(1, ("E: %d", 5));)
///
/// \param COND is the condition to assert.
/// \param FMT is the format C string and all additional arguments needed to print if the assertion fails.
#define PN_ASSERT_WARNF(COND, FMT) pn_tb_assert (COND, true, pn_tb_printf FMT, __FILE__, __LINE__)

/// @brief Warning assert with message.
///
/// Calls pn_tb_assert() with the condition COND, the supplied message MSG and the
/// filename and line macros.
///
/// \param COND is the condition to assert.
/// \param MSG is the C string to print if the assertion fails.
#define PN_ASSERT_WARNM(COND, MSG) pn_tb_assert (COND, true, MSG, __FILE__, __LINE__)


/// @brief Testbench info with message.
///
/// Calls pn_tb_info() with the supplied message MSG and the filename and line macros.
///
/// \param MSG is the C string to print.
#define PN_INFO(MSG) pn_tb_info (MSG, __FILE__, __LINE__)

/// @brief Testbench info with formatted message.
///
/// Calls pn_tb_info() with the formatted message according to FMT and the filename and line macros.
///
/// __NOTE__: You need to put the FMT argument in brackets (e.g. PN_INFO(("Val: %d", 5));)
///
/// \param FMT is the format C string and all additional arguments needed to print.
#define PN_INFOF(FMT) pn_tb_info (pn_tb_printf FMT, __FILE__, __LINE__)

/// @brief Testbench info with message to file.
///
/// Calls pn_info_file() with the supplied message MSG and the filename and line macros.
///
/// \param MSG is the C string to print.
/// \param OFILE is the file to print the message to.
#define PN_INFO_FILE(MSG, OFILE) pn_info_file (MSG, __FILE__, __LINE__, OFILE)


/// @brief Testbench warning with message.
///
/// Calls pn_tb_warning() with the supplied message MSG and the filename and line macros.
///
/// \param MSG is the C string to print.
#define PN_WARNING(MSG) pn_tb_warning (MSG, __FILE__, __LINE__)

/// @brief Testbench warning with formatted message.
///
/// Calls pn_tb_warning() with the formatted message according to FMT and the filename and line macros.
///
/// __NOTE__: You need to put the FMT argument in brackets (e.g. PN_WARNINGF(("Val: %d", 5));)
///
/// \param FMT is the format C string and all additional arguments needed to print.
#define PN_WARNINGF(FMT) pn_tb_warning (pn_tb_printf FMT, __FILE__, __LINE__)


/// @brief Testbench error with message.
///
/// Calls pn_tb_error() with the supplied message MSG and the filename and line macros.
///
/// \param MSG is the C string to print.
#define PN_ERROR(MSG) pn_tb_error (MSG, __FILE__, __LINE__)

/// @brief Testbench error with formatted message.
///
/// Calls pn_tb_error() with the formatted message according to FMT and the filename and line macros.
///
/// __NOTE__: You need to put the FMT argument in brackets (e.g. PN_ERRORF(("Val: %d", 5));)
///
/// \param FMT is the format C string and all additional arguments needed to print.
#define PN_ERRORF(FMT) pn_tb_error (pn_tb_printf FMT, __FILE__, __LINE__)


#else // #ifdef __SYNTHESIS__

#define PN_ASSERT(COND)
#define PN_ASSERTF(COND, FMT)
#define PN_ASSERTM(COND, MSG)

#define PN_ASSERT_WARN(COND)
#define PN_ASSERT_WARNF(COND, FMT)
#define PN_ASSERT_WARNM(COND, MSG)

#define PN_INFO(MSG)
#define PN_INFOF(FMT)

#define PN_WARNING(MSG)
#define PN_WARNINGF(FMT)

#define PN_ERROR(MSG)
#define PN_ERRORF(FMT)

#endif // #ifndef __SYNTHESIS__





// *************************** Special values for synthesis ********************


#ifndef __SYNTHESIS__


/// @brief vh_open struct (vhdl open equivalent)
///
/// This static constant struct can be used to connect unused ports to "nothing" to avoid elaboration
/// warnings and errors.
// TBD: Eliminate? / move this to a better place
static struct {
    template <typename T> operator sc_core::sc_signal_inout_if<T> & () const {
        return *(new sc_core::sc_signal<T> (sc_core::sc_gen_unique_name ("vh_open")));
    }

} const vh_open = {};


/// @brief vvh_const type/struct (vhdl constant value equivalent)
///
/// This type can be used to easily connect ports or signals to a "constant" value.
///
/// @tparam typename is the type the constant value will be applied to.
/// @param v is the constant value.
/// @todo The current implementation has a memory leak.
// TBD: Eliminate? / move to a better place; fix memory leak
template <typename T>
sc_core::sc_signal_in_if<T> const &vh_const (T const &v) // keep the name consistent with vh_open
{
    // Yes, this is an (elaboration-time) memory leak.  You can avoid it with some extra effort
    sc_core::sc_signal<T> *sig_p =
    new sc_core::sc_signal<T> (sc_core::sc_gen_unique_name ("vh_const"));
    sig_p->write (v);
    return *sig_p;
}


#else // #ifdef __SYNTHESIS__


///@brief Creates a constant signal. For synthesis
///
///This function creates a constant signal of type T. It can be used to easily connect ports
///or signals to a constant value in a SystemC model.
///
///@tparam T The data type of the signal.
///@param v The constant value to be assigned to the signal.
///@return A reference to the constant signal.

template <typename T>
sc_core::sc_signal_in_if<T>& vh_const (T const &v) {
    static sc_core::sc_signal<T> sig(sc_core::sc_gen_unique_name("vh_const"));
    sig.write(v);
    return sig;
}


///@brief A struct to represent an open signal. For synthesis
///
///This struct can be used to create an open signal in a SystemC model. An open signal
///is typically used to represent an unconnected port. This struct provides a conversion
///operator to create an open signal of any specified type.
// TBD: Duplicate of vh_open() above? -> Move vh_open for sim/syn to *one* place.
static struct {

  /// Conversion operator to create an open signal of type T.
  ///
  /// @tparam T The data type of the signal.
  /// @return A reference to the open signal.

  template <typename T>
  operator sc_core::sc_signal_inout_if<T>& () const
  {
    static sc_core::sc_signal<T> sig(sc_core::sc_gen_unique_name("vh_open"));
    return sig;
  }
} const vh_open = {};



#define PN_ASSERT(COND)
#define PN_ASSERTF(COND, FMT)
#define PN_ASSERTM(COND, MSG)

#define PN_INFO(MSG)
#define PN_INFOF(FMT)

#define PN_WARNING(MSG)
#define PN_WARNINGF(FMT)

#define PN_ERROR(MSG)
#define PN_ERRORF(FMT)


#endif // #ifndef __SYNTHESIS__





// *************************** Disassembly / Visualization *********************


#include <pn_riscv_disasm.h>


/// @brief Disassemble RISC-V instruction to C string (LEGACY implementation).
///
/// Take the RISC-V instruction `insn` and disassemble it into a human readable C string.
/// Disassembles all instructions the PicoNut processor can execute (RV32IMA currently).
/// Returns "? 0x<HEX_VALUE_OF_INSN>" if the instruction is unknown or invalid.
///
/// __NOTE:__ The returned string is only valid until the next call to this function.
///
/// @param insn is the instruction to disassemble.
/// @return A human readable C string representing the disassemble of insn.
const char *pn_disassemble_legacy (uint32_t insn);



///@} // @name Testbench helpers...





#endif // __PN_BASE_H__
