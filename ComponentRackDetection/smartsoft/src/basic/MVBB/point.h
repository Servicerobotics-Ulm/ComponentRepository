/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
 * point.h -
 *     
\*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#ifndef  __POINT__H
#define  __POINT__H


#include "_generic.h"
#include "real.h"
#include <math.h>


template <int  DIM>
class PointGeneric
{
public:
    real_mvbb  coords[ DIM ];
    int  id;

public:
    PointGeneric() {
        for  ( int  ind = 0; ind < DIM; ind++ ) 
            coords[ ind ] = 0;
        id = -1;
    }

    int   getID() const {
        return  id;
    }

    void  setID( int  _id ) { 
        id = _id;
    }
    void  setCoord( int  coord, const real_mvbb  & vl ) {
        assert( ( 0 <= coord )  &&  ( coord < DIM ) ); 
        coords[ coord ] = vl;
    }
    const real_mvbb  & operator[]( int  coord ) const {
        return  coords[ coord ];
    }
    real_mvbb  & operator[]( int  coord ) {
        return  coords[ coord ];
    }
    
    void  print() const {
        printf( "(" );
        for  ( int  ind = 0; ind < DIM; ind++ ) {
            if  ( ind > 0 ) 
                printf( ", " );
        
            printf( "%g", (double)coords[ ind ] );
        }
        printf( ") [len: %g]\n", length() );
    }

        
    void  add( const PointGeneric  & pnt ) {
        for  ( int  ind = 0; ind < DIM; ind++ ) 
            coords[ ind ] += pnt.coords[ ind ];
    }
    void  subtract( const PointGeneric  & pnt ) {
        for  ( int  ind = 0; ind < DIM; ind++ ) 
            coords[ ind ] -= pnt.coords[ ind ];
    }
    void  scale( const real_mvbb  & scale ) {
        for  ( int  ind = 0; ind < DIM; ind++ ) 
            coords[ ind ] *= scale;
    }
    void  add_scale( const PointGeneric  & pnt, const real_mvbb  & scale ) {
        for  ( int  ind = 0; ind < DIM; ind++ ) 
            coords[ ind ] += scale * pnt.coords[ ind ];
    }

    real_mvbb  dot_prod( const PointGeneric  & pnt ) const {
        double  sum;

        sum = 0;
        for  ( int  ind = 0; ind < DIM; ind++ ) 
            sum += coords[ ind ] * pnt.coords[ ind ];

        return  sum;
    }

    void  fillRandom() {
        for  ( int  ind = 0; ind < DIM; ind++ ) 
            coords[ ind ] = realRand();
    }
    real_mvbb  length() const {
        return sqrt( dot_prod( *this ) );
    }

    void  normalize() {
        real_mvbb  len = sqrt( dot_prod( *this ) );
        if  ( len == 0 )
            return;
        for  ( int  ind = 0; ind < DIM; ind++ ) 
            coords[ ind ] /= len;
    }
    void  normalize_big() {
        real_mvbb  len = 0;
        for  ( int  ind = 0; ind < DIM; ind++ ) 
            len += fabs( coords[ ind ] );
        
        if  ( len == 0 )
            return;
        for  ( int  ind = 0; ind < DIM; ind++ ) 
            coords[ ind ] /= len;

        normalize();
    }


};

class  Point3d : public PointGeneric<3> {
public:
    Point3d( const real_mvbb   & x,
             const real_mvbb   & y,
             const real_mvbb   & z ) {
        coords[ 0 ] = x;
        coords[ 1 ] = y;
        coords[ 2 ] = z;
        id = -1;
    }
        
    /*    Point3d& operator=(Point3d &ref){
      if(this == &ref){
	return *this;
      }
      coords[0] = ref[0];
      coords[1] = ref[1];
      coords[2] = ref[2];
      return *this;
      }*/

    bool  isEqual( const Point3d  & pnt ) const {
        return  ( ( coords[ 0 ] == pnt.coords[ 0 ] )
                  &&  ( coords[ 1 ] == pnt.coords[ 1 ] )
                  &&  ( coords[ 2 ] == pnt.coords[ 2 ] ) );
    }

    Point3d( const PointGeneric<3>  & pnt ) {
        coords[ 0 ] = pnt.coords[ 0 ];
        coords[ 1 ] = pnt.coords[ 1 ];
        coords[ 2 ] = pnt.coords[ 2 ];
        id = pnt.id;
    }

    Point3d() {
        coords[ 0 ] = coords[ 1 ] = coords[ 2 ] = 0;
        id = -1;
    }
    void  zero() {
        coords[ 0 ] = coords[ 1 ] = coords[ 2 ] = 0;
        id = -1;
    }

    bool operator<( const PointGeneric<3> &p ) const
    {
      return (coords[ 1 ] < p.coords[ 1 ] || (coords[ 1 ] == p.coords[ 1 ] && coords[ 0 ] < p.coords[ 0 ]));
    }
};

class  Point2d : public PointGeneric<2> {
public:
    Point2d( const PointGeneric<2>  & pnt ) {
        coords[ 0 ] = pnt.coords[ 0 ];
        coords[ 1 ] = pnt.coords[ 1 ];
        id = -1;
    }
    Point2d( const real_mvbb  & x, const real_mvbb  & y ) {
        coords[ 0 ] = x;
        coords[ 1 ] = y;
        id = -1;
    }

    Point2d() {
        coords[ 0 ] = coords[ 1 ] = 0;
        id = -1;
    }          

    real_mvbb x() {return coords[0];}
    real_mvbb y() {return coords[1];}                      
    
    bool operator<( const PointGeneric<2> &p ) const
    {
      return (coords[ 0 ] < p.coords[ 0 ] || (coords[ 0 ] == p.coords[ 0 ] && coords[ 1 ] < p.coords[ 1 ]));
    }
    
};


inline double  pnt_distance( const Point3d  & p, 
                         const Point3d  & q ) 
{
    double  valx = (p[ 0 ] - q[ 0 ]);
    double  valy = (p[ 1 ] - q[ 1 ]);
    double  valz = (p[ 2 ] - q[ 2 ]);

    return  sqrt( valx * valx + valy * valy + valz * valz );
}
inline double  pnt_distance( const Point2d  & p, 
                             const Point2d  & q ) 
{
    double  valx = (p[ 0 ] - q[ 0 ]);
    double  valy = (p[ 1 ] - q[ 1 ]);

    return sqrt ( valx * valx + valy * valy );
}


inline Point3d  changeBase( const Point3d  & vec1,
                            const Point3d  & vec2,
                            const Point3d  & vec3,
                            const Point3d  & pnt ) 
{
    Point3d  sum;
    
    sum.add_scale( vec1, pnt[ 0 ] );
    sum.add_scale( vec2, pnt[ 1 ] );
    sum.add_scale( vec3, pnt[ 2 ] );
    
    return  sum;
}

inline Point3d  pnt_subtract( const Point3d  & p, 
                                const Point3d  & q ) 
{
    Point3d  pnt;

    pnt.setCoord( 0, p[ 0 ] - q[ 0 ] );
    pnt.setCoord( 1, p[ 1 ] - q[ 1 ] );
    pnt.setCoord( 2, p[ 2 ] - q[ 2 ] );

    return  pnt;
}

inline Point3d   cross_prod( const Point3d  & a, const Point3d  & b )
{
    Point3d  c;

    c[ 0 ] = a[ 1 ] * b[ 2 ] - a[ 2 ] * b[ 1 ];
    c[ 1 ] = - ( a[ 0 ] * b[ 2 ] - a[ 2 ] * b[ 0 ] );
    c[ 2 ] = a[ 0 ] * b[ 1 ] - a[ 1 ] * b[ 0 ];

    return  c;
}


inline Point3d  pnt_get_triangle_norm( 
                       const Point3d  & a, 
                       const Point3d  & b,
                       const Point3d  & c )
{
    Point3d  e1, e2;

    e1 = pnt_subtract( b, a );
    e2 = pnt_subtract( c, a );
    
    return  cross_prod( e1, e2 );
}


class  PointPair
{
public:
    real_mvbb  distance;
    Point3d  p, q;

    PointPair() : p(), q() {
        distance = 0.0;
    }

    void  init( const Point3d  & _p, 
                const Point3d  & _q ) {
        p = _p;
        q = _q;
        distance = pnt_distance( p, q );
    }

    void  init( const Point3d  & pnt ) {
        distance = 0;
        p = q = pnt;
    }

    void  update_diam_simple( const Point3d  & _p, const Point3d  & _q ) {
        real_mvbb  new_dist;

        new_dist = pnt_distance( _p, _q );
        if  ( new_dist <= distance )
            return;
        
        distance = new_dist;
        p = _p;
        q = _q;
    }

    void  update_diam( const Point3d  & _p, const Point3d  & _q ) { 
        //update_diam_simple( p, _p );
        //update_diam_simple( p, _q );
        //update_diam_simple( q, _p );
        //update_diam_simple( q, _q );
        update_diam_simple( _p, _q );
    }


    void  update_diam( PointPair  & pp ) {
        //update_diam_simple( p, pp.p );
        //update_diam_simple( p, pp.q );
        //update_diam_simple( q, pp.p );
        //update_diam_simple( q, pp.q );
        update_diam_simple( pp.p, pp.q );
    }
};


class  PointPair2d
{
public:
    real_mvbb  distance;
    Point2d  p, q;

    PointPair2d() : p(), q() {
        distance = 0.0;
    }

    void  init( const Point2d  & _p, 
                const Point2d  & _q ) {
        p = _p;
        q = _q;
        distance = pnt_distance( p, q );
    }

    void  init( const Point2d  & pnt ) {
        distance = 0;
        p = q = pnt;
    }

    void  update_diam_simple( const Point2d  & _p, const Point2d  & _q ) {
        real_mvbb  new_dist;

        new_dist = pnt_distance( _p, _q );
        if  ( new_dist <= distance )
            return;
        
        distance = new_dist;
        p = _p;
        q = _q;
    }

    void  update_diam( const Point2d  & _p, const Point2d  & _q ) { 
        //update_diam_simple( p, _p );
        //update_diam_simple( p, _q );
        //update_diam_simple( q, _p );
        //update_diam_simple( q, _q );
        update_diam_simple( _p, _q );
    }


    void  update_diam( PointPair2d  & pp ) {
        //update_diam_simple( p, pp.p );
        //update_diam_simple( p, pp.q );
        //update_diam_simple( q, pp.p );
        //update_diam_simple( q, pp.q );
        update_diam_simple( pp.p, pp.q );
    }
};

//#else   /* __POINT__H */
//#error  Header file point.h included twice
#endif  /* __POINT__H */

/* point.h - End of File ------------------------------------------*/


