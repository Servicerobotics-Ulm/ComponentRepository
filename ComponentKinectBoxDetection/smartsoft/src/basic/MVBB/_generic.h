/*--------------------------------------------------------------------
 * Copyright (C) 1998 Sariel Har-Peled
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston,
 * MA  02111-1307, USA.
\*--------------------------------------------------------------------*/

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
 * generic.h -
 *     Impelment various generics operations.
\*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#define WORK_MODE_TXT
#define WORK_MODE_DIALOG

#ifndef  ___XGENERIC__H
#define  ___XGENERIC__H

#ifndef  GLOBAL 
#define  GLOBAL    
#endif 

#ifdef  __GNUG__
#ifndef  true 
#define  true     1
#endif 

#ifndef  false
#define  false    0
#endif 
#endif  // __GNUG__

#ifndef  USE_ARGUMENT
#define  USE_ARGUMENT(X)  (void)(X)
#define  USE_VARIABLE(X)  (void)(&X)
#endif  /* USE_ARGUMENT */

#define  LINE_SIZE   1024

#include <time.h>

template <class T>
inline void     exchange( T   & a, T    & b )
{
    T   tmp = a;

    a = b;
    b = tmp;
}

#define  DEFINE_ACCESS_FUNCTIONS( TYPE, VAR, GET_FUNC, PUT_FUNC ) \
              TYPE    GET_FUNC( void ) const  \
              {  \
                  return  VAR; \
              } \
              void    PUT_FUNC( TYPE   xxxx ) \
              { \
                  VAR = xxxx; \
              }
#define  DEFINE_BIT_ACCESS_FUNCTIONS( VAR, BIT_MASK, GET_FUNC, PUT_FUNC ) \
              BOOL    GET_FUNC( void ) const  \
              {  \
                  return  ( ( VAR & BIT_MASK ) != 0 );\
              } \
              void    PUT_FUNC( BOOL    fFlag ) \
              { \
                  VAR = ( (VAR & (~BIT_MASK))  |  (fFlag? BIT_MASK : 0) ); \
              }


/*----------------------------------------------------------------------
 * ADDR -
 *     A generic functon to advance the specified number of bytes to
 *  an address which is relative to the address of 'x'.
 *
 * min, max - standart functions.
\*----------------------------------------------------------------------*/

#define Min(a, b)  (((a) < (b)) ? (a) : (b))
#define Max(a, b)  (((a) > (b)) ? (a) : (b))
/*inline int  max( int a, int b )
{
   return  (a>b)? a : b;
}*/


template <class T, class  Q>
inline T  * ADDR( const T          * x, Q  cbSize )
{
  return  (T *)(((char *)x) + (unsigned int) cbSize);
}

/*template <class T, class  Q>
inline T  * ADDR( T   * x, Q  cbSize )
{
  return  (T *)(((char *)x) + (unsigned int) cbSize);
}*/
#ifndef __MINMAX_DEFINED
#define __MINMAX_DEFINED
/*
template <class T> const inline   T& min( const T& t1, const T& t2 )
{
         return t1>t2 ? t2 : t1;
}

template <class T> inline  T& max( const T& t1, const T& t2 )
{
         return t1>t2 ? t1 : t2;
}

template <class T>
inline T  min( T t1, T t2 )
{
         return t1>t2 ? t2 : t1;
}

template <class T>
inline T  max( T t1, T t2 )
{
         return t1>t2 ? t1 : t2;
}*/
#endif /* MIN_MAX */

/*
template <class T>
inline T max( T x,  T y, T z)
{
  return (x > y) ? max( x, z ) : max( y, z );
};


template <class T>
inline T min( T  x, T   y, T  z )
{
  return (x < y) ? min( x, z ) : min( y, z );
};*/


template <class T>
inline T              chUpcase( T    ch )
{
    if ( ( ch >= ((T)'a') ) && ( ch <= ((T)'z') ) )
        ch = (T)( ch - 'a' + 'A' );
    return   ch;
}


template <class T>
inline int            Sign( const T  & val )
{
    return  ( val < 0 )? -1 : ( (val > 0)? 1 : 0 );
}


void        my_error( char        * err );


void  print_spaces( int   ind );

/*-----------------------------------------------------------------------
 * Exact arithmetic 
 *-----------------------------------------------------------------------*/

template <class NT>
NT CG_abs(const NT &x)
{
    return (x >= NT(0)) ? x: -x;
}


//#define  DASSERT( X, Y)   {if  ( !(X) ) { fprintf( stderr, Y ); assert(X); }}


template <class FT>
FT  CG_det2x2_by_formula( const FT& a, const FT& b,
                        const FT& c, const FT& d)
{
    return (   a*d - b*c );
}


template <class FT>
FT
CG_det3x3_by_formula( const FT& a, const FT& b, const FT& c,
                        const FT& d, const FT& e, const FT& f,
                        const FT& g, const FT& h, const FT& i )
{
return (   a*(e*i - f*h)
         - d*(b*i - c*h)
         + g*(b*f - c*e) );
}


template <class FT>
FT
CG_det4x4_by_formula( const FT& a, const FT& b, const FT& c, const FT& d,
                        const FT& e, const FT& f, const FT& g, const FT& h,
                        const FT& i, const FT& j, const FT& k, const FT& l,
                        const FT& m, const FT& n, const FT& o, const FT& p )
{
return (  a * CG_det3x3_by_formula( f, g, h,
                                      j, k, l,
                                      n, o, p )
        - e * CG_det3x3_by_formula( b, c, d,
                                      j, k, l,
                                      n, o, p )
        + i * CG_det3x3_by_formula( b, c, d,
                                      f, g, h,
                                      n, o, p )
        - m * CG_det3x3_by_formula( b, c, d,
                                      f, g, h,
                                      j, k, l ) );
}

/* measuring times */
#ifndef  CLOCKS_PER_SEC
#define   CLOCKS_PER_SEC  1000000.0
#endif 

//extern long clock (void);

class  Timer
{
private:
  long int    start_t, end_t;

  static inline float  SECONDS( long int   time ) 
  {
      return  (float)( (double)time  / ((double)CLOCKS_PER_SEC));
  }

public:
  void  start( void )
  {
    start_t = clock();
  }

  void  end( void )
  {
    end_t = clock();
  }

  float  seconds( void )
  {
    return  SECONDS( end_t - start_t ); 
  }
};

#ifndef  RAND_MAX
#define  RAND_MAX    0x7fffffff
#endif  // RAND_MAX

inline  double    realRand()
{
    //double  a( rand() );
    //double  b( RAND_MAX );

    //return  a / b;
    //return  drand48();
    return rand()/(double)RAND_MAX;
}


inline FILE   * safe_fopen( const char  * file_name, const char  * mode ) 
{
    FILE  * fl;

    fl = fopen( file_name, mode );
    if  ( fl == NULL ) {
        fprintf( stderr, "Unable to open file: [%s]\n", file_name );
        exit( -1 );
    }

    return  fl;
}

#else   /* ___GENERIC__H */
//#error  Header file generic.h included twice
#endif  /* ___GENERIC__H */

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
 *     
 * generic.h - End of File
\*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
