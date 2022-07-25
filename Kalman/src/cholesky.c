//#include <assert.h>
#include "assertion.h"
#include <stdint.h>
//#include <math.h>
#include "matrix.h"

/**
* \brief Decomposes a matrix into lower triangular form using Cholesky decomposition.
* \param[in] mat The matrix to decompose in place into a lower triangular matrix.
* \return Zero in case of success, nonzero if the matrix is not positive semi-definite.
*
* Kudos: https://code.google.com/p/efficient-java-matrix-library
*/

#define SQRT_TOLERANCE              0.0001

// https://github.com/MichaelDipperstein/sqrt/blob/master/sqrt.c
double BisectionSqrt(float value)
{
    double guess, min, max, delta;

    /* check for valid input value */
    if (value < 0.0)
        return 0.0;

    /* come up with initail guess and bounds */
    if (value < 1.0)
    {
        guess = value * 2.0;
        max = 1.0;
    }
    else
    {
        guess = value / 2.0;
        max = value;
    }
    min = 0.0;
    delta = (guess * guess) - value;

    /* narrow in on answer until close enough approximation */
    while (1)
    {
        delta = (guess * guess) - value;

        if (delta > SQRT_TOLERANCE)
        {
            /* guess is too high bisect min and guess */
            max = guess;
        }
        else
            if (delta < (-SQRT_TOLERANCE))
            {
                /* guess is too low bisect max and guess */
                min = guess;
            }
            else
            {
                /* our guess is good enough */
                break;
            }

        /* bisect new bound to get new guess */
        guess = (min + max) / 2.0;
    }

    return (guess);
}

int cholesky_decompose_lower(register const matrix_t *const mat)
{
    uint_fast8_t i, j;
    uint_fast8_t n = mat->rows;
    matrix_data_t *t = mat->data;

    matrix_data_t el_ii;
    matrix_data_t div_el_ii = 0;

    // assert(mat != (matrix_t*)0);
    // assert(mat->rows == mat->cols);
    // assert(mat->rows > 0);

    for( i = 0; i < n; ++i ) 
    {
        for( j = i; j < n; ++j ) 
        {
            matrix_data_t sum = t[i*n+j];

            uint_fast16_t iEl = i*n;
            uint_fast16_t jEl = j*n;
            uint_fast16_t end = iEl+i;
            // k = 0:i-1
            for( ; iEl<end; ++iEl,++jEl ) 
            {
                // sum -= el[i*n+k]*el[j*n+k];
                sum -= t[iEl]* t[jEl];
            }

            if( i == j ) 
            {
                // is it positive-definite?
                if( sum <= 0.0 ) return 1;

                //el_ii = (matrix_data_t)sqrt(sum);
                el_ii = (matrix_data_t)BisectionSqrt(sum);
                t[i*n+i] = el_ii;
                div_el_ii = (matrix_data_t)1.0/el_ii;
            } 
            else 
            {
                t[j*n+i] = sum*div_el_ii;
            }
        }
    }

    // zero the top right corner.
    for( i = 0; i < n; ++i ) 
    {
        for( j = i+1; j < n; ++j ) 
        {
            t[i*n+j] = 0.0;
        }
    }

    return 0;
}
