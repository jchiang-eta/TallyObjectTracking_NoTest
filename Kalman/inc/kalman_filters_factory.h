/************************************************************************/
/* Check for inputs                                                     */
/************************************************************************/

#ifndef KALMAN_NAME
#error KALMAN_NAME needs to be defined prior to inclusion of this file.
#endif

#ifndef KALMAN_NUM_FILTERS
#error KALMAN_NUM_FILTERS needs to be defined prior to inclusion of this file.
#elif KALMAN_NUM_FILTERS <= 0
#error KALMAN_NUM_FILTERS must be a positive integer
#endif

#ifndef KALMAN_NUM_STATES
#error KALMAN_NUM_STATES needs to be defined prior to inclusion of this file.
#elif KALMAN_NUM_STATES <= 0
#error KALMAN_NUM_STATES must be a positive integer
#endif

#ifndef KALMAN_NUM_INPUTS
#error KALMAN_NUM_INPUTS needs to be defined prior to inclusion of this file.
#elif KALMAN_NUM_INPUTS < 0
#error KALMAN_NUM_INPUTS must be a positive integer or zero if no inputs are used
#endif

#ifdef KALMAN_SHARE_MATRICES
#pragma message ("Matrices A, B, Q and aux/temp will be shared ")
#else
#pragma message ("Matrices A, B, Q and aux/temp will NOT be shared ")
#endif

/************************************************************************/
/* Prepare dimensions                                                   */
/************************************************************************/

#define __KALMAN_A_ROWS     KALMAN_NUM_STATES
#define __KALMAN_A_COLS     KALMAN_NUM_STATES

#define __KALMAN_P_ROWS     KALMAN_NUM_STATES
#define __KALMAN_P_COLS     KALMAN_NUM_STATES

#define __KALMAN_x_ROWS     KALMAN_NUM_STATES
#define __KALMAN_x_COLS     1

#define __KALMAN_B_ROWS     KALMAN_NUM_STATES
#define __KALMAN_B_COLS     KALMAN_NUM_INPUTS

#define __KALMAN_u_ROWS     KALMAN_NUM_INPUTS
#define __KALMAN_u_COLS     1

// new and, hopefully, correct Q matrix
#define __KALMAN_Q_ROWS     KALMAN_NUM_STATES
#define __KALMAN_Q_COLS     KALMAN_NUM_STATES

//#define __KALMAN_Q_ROWS     KALMAN_NUM_INPUTS
//#define __KALMAN_Q_COLS     KALMAN_NUM_INPUTS

#define __KALMAN_aux_ROWS     ((KALMAN_NUM_STATES > KALMAN_NUM_INPUTS) ? KALMAN_NUM_STATES : KALMAN_NUM_INPUTS)
#define __KALMAN_aux_COLS     1

#define __KALMAN_tempP_ROWS  __KALMAN_P_ROWS
#define __KALMAN_tempP_COLS  __KALMAN_P_COLS

#define __KALMAN_tempBQ_ROWS __KALMAN_B_ROWS
#define __KALMAN_tempBQ_COLS __KALMAN_B_COLS

/************************************************************************/
/* Name helper macro                                                    */
/************************************************************************/

#ifndef STRINGIFY
#define __STRING2(x) #x
#define STRINGIFY(x) __STRING2(x)
#endif

#pragma message("** Instantiating Kalman filter \"" STRINGIFY(KALMAN_NAME) "\" with " STRINGIFY(KALMAN_NUM_STATES) " states and " STRINGIFY(KALMAN_NUM_INPUTS) " inputs")

#define __CONCAT(x, y)                                  x ## y

#define KALMAN_FILTER_BASENAME_HELPER(name)             __CONCAT(kalman_filter_, name)
#define KALMAN_FILTER_BASENAME                          KALMAN_FILTER_BASENAME_HELPER(KALMAN_NAME)
#define KALMAN_BASENAME_HELPER(basename)                __CONCAT(basename, _)

/************************************************************************/
/* Name macro                                                           */
/************************************************************************/

#define KALMAN_BUFFER_HELPER2(basename, element)         basename ## element ## _buffer
#define KALMAN_BUFFER_HELPER(basename, element)         KALMAN_BUFFER_HELPER2(basename, element)
#define KALMAN_BUFFER_NAME(element)                     KALMAN_BUFFER_HELPER(KALMAN_BASENAME_HELPER(KALMAN_FILTER_BASENAME), element)

#define KALMAN_FUNCTION_HELPER2(basename, name)         basename ## name
#define KALMAN_FUNCTION_HELPER(basename, name)          KALMAN_FUNCTION_HELPER2(basename, name)
#define KALMAN_FUNCTION_NAME(name)                      KALMAN_FUNCTION_HELPER(KALMAN_BASENAME_HELPER(KALMAN_FILTER_BASENAME), name)

#define KALMAN_STRUCT_NAME                              KALMAN_FILTER_BASENAME

/************************************************************************/
/* Construct Kalman filter buffers: State                               */
/************************************************************************/

#include "compiler.h"
#include "matrix.h"
#include "kalman.h"

#define __KALMAN_BUFFER_A   KALMAN_BUFFER_NAME(A)
#define __KALMAN_BUFFER_P   KALMAN_BUFFER_NAME(P)
#define __KALMAN_BUFFER_x   KALMAN_BUFFER_NAME(x)
#define __KALMAN_BUFFER_Q   KALMAN_BUFFER_NAME(Q)

// we can share certain matrices which only hold constants
#pragma message("Creating Kalman filter A buffer: " STRINGIFY(__KALMAN_BUFFER_A))
#pragma message("Creating Kalman filter Q buffer: " STRINGIFY(__KALMAN_BUFFER_Q))
#ifdef KALMAN_SHARE_MATRICES
static matrix_data_t __KALMAN_BUFFER_A[__KALMAN_A_ROWS * __KALMAN_A_COLS];
static matrix_data_t __KALMAN_BUFFER_Q[__KALMAN_Q_ROWS * __KALMAN_Q_COLS];
#else
static matrix_data_t __KALMAN_BUFFER_A[__KALMAN_A_ROWS * __KALMAN_A_COLS * KALMAN_NUM_FILTERS];
static matrix_data_t __KALMAN_BUFFER_Q[__KALMAN_Q_ROWS * __KALMAN_Q_COLS * KALMAN_NUM_FILTERS];
#endif

#pragma message("Creating Kalman filter P buffer: " STRINGIFY(__KALMAN_BUFFER_P))
static matrix_data_t __KALMAN_BUFFER_P[__KALMAN_P_ROWS * __KALMAN_P_COLS * KALMAN_NUM_FILTERS];

#pragma message("Creating Kalman filter x buffer: " STRINGIFY(__KALMAN_BUFFER_x))
static matrix_data_t __KALMAN_BUFFER_x[__KALMAN_x_ROWS * __KALMAN_x_COLS * KALMAN_NUM_FILTERS];


/************************************************************************/
/* Construct Kalman filter buffers: Inputs                              */
/************************************************************************/

#if KALMAN_NUM_INPUTS > 0

#define __KALMAN_BUFFER_B   KALMAN_BUFFER_NAME(B)
#define __KALMAN_BUFFER_u   KALMAN_BUFFER_NAME(u)

#pragma message("Creating Kalman filter B buffer: " STRINGIFY(__KALMAN_BUFFER_B))
#ifdef KALMAN_SHARE_MATRICES
static matrix_data_t __KALMAN_BUFFER_B[__KALMAN_B_ROWS * __KALMAN_B_COLS];
#else
static matrix_data_t __KALMAN_BUFFER_B[__KALMAN_B_ROWS * __KALMAN_B_COLS * KALMAN_NUM_FILTERS];
#endif

#pragma message("Creating Kalman filter u buffer: " STRINGIFY(__KALMAN_BUFFER_u))
static matrix_data_t __KALMAN_BUFFER_u[__KALMAN_x_ROWS * __KALMAN_u_COLS * KALMAN_NUM_FILTERS];

#else

#pragma message("Skipping Kalman filter B buffer: (zero inputs)")
#define __KALMAN_BUFFER_B ((matrix_data_t*)0)

#pragma message("Skipping Kalman filter u buffer: (zero inputs)")
#define __KALMAN_BUFFER_u ((matrix_data_t*)0)

#endif

/************************************************************************/
/* Construct Kalman filter buffers: Temporaries                         */
/************************************************************************/

#define __KALMAN_BUFFER_aux     KALMAN_BUFFER_NAME(aux)
#define __KALMAN_BUFFER_tempPBQ KALMAN_BUFFER_NAME(tempPBQ)

#define __KALMAN_aux_size       (__KALMAN_aux_ROWS * __KALMAN_aux_COLS)
#define __KALMAN_tempP_size     (__KALMAN_tempP_ROWS * __KALMAN_tempP_COLS)
#define __KALMAN_tempBQ_size    (__KALMAN_tempBQ_ROWS * __KALMAN_tempBQ_COLS)

#define __KALMAN_tempPBQ_size   ((__KALMAN_tempP_size > __KALMAN_tempBQ_size) ? __KALMAN_tempP_size : __KALMAN_tempBQ_size)

#pragma message("Creating Kalman filter aux buffer: " STRINGIFY(__KALMAN_BUFFER_aux))
#pragma message("Creating Kalman filter temporary P/BQ buffer: " STRINGIFY(__KALMAN_BUFFER_tempPBQ))

#ifdef KALMAN_SHARE_MATRICES
static matrix_data_t __KALMAN_BUFFER_aux[__KALMAN_aux_size];
static matrix_data_t __KALMAN_BUFFER_tempPBQ[__KALMAN_tempPBQ_size];
#else
static matrix_data_t __KALMAN_BUFFER_aux[__KALMAN_aux_size * KALMAN_NUM_FILTERS];
static matrix_data_t __KALMAN_BUFFER_tempPBQ[__KALMAN_tempPBQ_size * KALMAN_NUM_FILTERS];
#endif

/************************************************************************/
/* Construct Kalman filter                                              */
/************************************************************************/

#pragma message("Creating Kalman filter structure: " STRINGIFY(KALMAN_STRUCT_NAME))

/*!
* \brief The Kalman filter structure
*/
static kalman_t KALMAN_STRUCT_NAME[KALMAN_NUM_FILTERS];

#pragma message ("Creating Kalman filter initialization function: " STRINGIFY(KALMAN_FUNCTION_NAME(init()) ))

/*!
* \brief Initializes the Kalman Filter
* \return Pointer to the filter.
*/
kalman_t* KALMAN_FUNCTION_NAME(init)()
{
    int i;

#ifdef KALMAN_SHARE_MATRICES
    for (i = 0; i < __KALMAN_A_ROWS * __KALMAN_A_COLS; ++i) { __KALMAN_BUFFER_A[i] = 0; }
    for (i = 0; i < __KALMAN_Q_ROWS * __KALMAN_Q_COLS; ++i) { __KALMAN_BUFFER_Q[i] = 0; }
    for (i = 0; i < __KALMAN_B_ROWS * __KALMAN_B_COLS; ++i) { __KALMAN_BUFFER_B[i] = 0; }
#else
    for (i = 0; i < __KALMAN_A_ROWS * __KALMAN_A_COLS * KALMAN_NUM_FILTERS; ++i) { __KALMAN_BUFFER_A[i] = 0; }
    for (i = 0; i < __KALMAN_Q_ROWS * __KALMAN_Q_COLS * KALMAN_NUM_FILTERS; ++i) { __KALMAN_BUFFER_Q[i] = 0; }
    for (i = 0; i < __KALMAN_B_ROWS * __KALMAN_B_COLS * KALMAN_NUM_FILTERS; ++i) { __KALMAN_BUFFER_B[i] = 0; }
#endif

    for (i = 0; i < __KALMAN_x_ROWS * __KALMAN_x_COLS * KALMAN_NUM_FILTERS; ++i) { __KALMAN_BUFFER_x[i] = 0; }
    for (i = 0; i < __KALMAN_P_ROWS * __KALMAN_P_COLS * KALMAN_NUM_FILTERS; ++i) { __KALMAN_BUFFER_P[i] = 0; }


#if KALMAN_NUM_INPUTS > 0
    for (i = 0; i < __KALMAN_u_ROWS * __KALMAN_x_COLS * KALMAN_NUM_FILTERS; ++i) { __KALMAN_BUFFER_u[i] = 0; } // x replaced with u
#endif

#ifdef KALMAN_SHARE_MATRICES
    int A_size = 0;
    int Q_size = 0;
    int B_size = 0;

    int tempPBQ_size = 0;
    int aux_size = 0;
#else
    int A_size = __KALMAN_A_ROWS * __KALMAN_A_COLS;
    int Q_size = __KALMAN_Q_ROWS * __KALMAN_Q_COLS;
    int B_size = __KALMAN_B_ROWS * __KALMAN_B_COLS;

    int tempPBQ_size = __KALMAN_tempPBQ_size;
    int aux_size = __KALMAN_aux_size;
#endif

    int x_size = __KALMAN_x_ROWS * __KALMAN_x_COLS;
    int u_size = __KALMAN_u_ROWS * __KALMAN_u_COLS;
    int P_size = __KALMAN_P_ROWS * __KALMAN_P_COLS;

    for( i = 0; i < KALMAN_NUM_FILTERS; i++)
        kalman_filter_initialize(&KALMAN_STRUCT_NAME[i], KALMAN_NUM_STATES, KALMAN_NUM_INPUTS, 
            __KALMAN_BUFFER_A + i*A_size,
            __KALMAN_BUFFER_x + i*x_size,
            __KALMAN_BUFFER_B + i*B_size,
            __KALMAN_BUFFER_u + i*u_size,
            __KALMAN_BUFFER_P + i*P_size,
            __KALMAN_BUFFER_Q + i*Q_size,
            __KALMAN_BUFFER_aux + i*aux_size,
            __KALMAN_BUFFER_aux + i*aux_size,
            __KALMAN_BUFFER_tempPBQ + i*tempPBQ_size,
            __KALMAN_BUFFER_tempPBQ + i*tempPBQ_size
        );
    return KALMAN_STRUCT_NAME;
}

