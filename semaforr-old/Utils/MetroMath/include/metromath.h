#ifndef METROMATH
#define METROMATH
/**
 * \mainpage Metromath Documentation
 *
 * \section  main_intro Introduction
 *
 *     This library was created for the MetroBotics project at CUNY.  Its purpose is to provide a
 *     framework of commonly used classes and functions that aid in the development of artificial
 *     intelligence and robotics applications.
 *
 * \section  main_usage Using the Library
 *     <ol>
 *       <li>
 *         Include the \link metromath.h \endlink header file into your source code.
 *             \code
 *                 #include "metromath.h"
 *             \endcode
 *       </li>
 *       <li>
 *         All classes and functions in the library exist within the \link metrobotics \endlink
 *         namespace.
 *             \code
 *                 using namespace metrobotics;
 *             \endcode
 *       </li>
 *       <li>
 *         Link the \em libMetromath.a library file into your project.
 *       </li>
 *     </ol>
 * 
 * \section main_acknowledgements Acknowledgements
 *     <ul>
 *       <li>
 *         Mark Manashirov <mark.manashirov@gmail.com>
 *       </li>
 *     </ul>
 *
 */


/**
 * \file    "metromath.h"
 *
 * \brief   The all-inclusive header file for this utility library.
 *
 * \details It is sufficient to include just this one header file into your source code
 *          to gain access to all of the library's constituent classes and functions.
 */

// [Simply include everything!]
#include "RealPredicate.h"
#include "RealEquality.h"
#include "RealLessThan.h"
#include "VectorN.h"
#include "RealVectorN.h"
#include "Lerp.h"

/**
 * \namespace  metrobotics
 *
 * \brief      Contains all of the library's classes and functions.
 *
 * \details    The namespace under which all of the library's classes and functions are defined.
 */
namespace metrobotics
{
	// [Nothing is actually defined in this file.]
}

#endif
