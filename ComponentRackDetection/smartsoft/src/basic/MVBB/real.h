/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
 * real.h -
 *     
\*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#ifndef  __REAL_MVBB_H
#define  __REAL_MVBB_H

typedef  double  real_mvbb;

#define  REAL_EPSILON  1e-7

inline bool  real_lesseq( const real_mvbb  & a, const real_mvbb  & b )
{
    return  ( a <= (b + REAL_EPSILON ) );
}

inline bool  real_eq( const real_mvbb  & a, const real_mvbb  & b )
{
    return  ( ( a <= (b + REAL_EPSILON ) )
              &&  ( b <= (a + REAL_EPSILON ) ) );
}

#else   /* __REAL_MVBB_H */
#error  Header file real.h included twice
#endif  /* __REAL_MVBB_H */

/* real.h - End of File ------------------------------------------*/
