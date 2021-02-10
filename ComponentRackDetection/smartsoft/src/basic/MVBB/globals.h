#ifndef __GLOBALS_H__
#define __GLOBALS_H__

enum XmlSelection 
{
  NULL_XML = 0,
  ALL = 1,
  NOCOLOR = 2,
  NOPOINTS = 3,
  MODEL = 4
};

enum BoxSelection 
{
  NULL_BOX,
  ALL_BOXES,
  BIGGEST,
  SMALLEST,
  OUTERMOST,
  INNERMOST
};

enum FaceSelection
{
  NULL_FACE,
  ALL_FACES,
  NOSIDES
};

enum ViewSelection 
{
  NULL_VIEW = 0,
  ALL_VIEWS = 1,
  SET_VIEW = 2,
  ZERO_VIEW = 3,
  TURNTABLE_VIEW = 4
};

enum TrainSelection
{
  NULL_TRAIN,
  TRAIN,
  TEST,
  APPLY
};

enum ShapeClassMan
{
  SH_ROUND  = 1,
  SH_HALFROUND = 2,
  SH_ARC = 3,
  SH_CIRCLE = 4,
  SH_SQUARE = 5,
  SH_TRIANGLE = 6,
  SH_NOISE  = 7,
  numberOfManShapeClasses
};

enum ShapeClassAut
{
  UNDEF = 0,
  SH_SPHERE_POS = 1,
  SH_SPHERE_NEG = 2,
  SH_SPHERE_PLN = 3,
  SH_PYRAMID_POS = 4,
  SH_PYRAMID_NEG = 5,
  SH_RECTANGLE_PLN = 6,
  SH_CYLINDERV_POS = 7,
  SH_CYLINDERV_NEG = 8,
  SH_CYLINDERH_POS = 9,
  SH_CYLINDERH_NEG = 10,
  SH_RECTANGLE_NEG = 11,
  SH_ARC0_POS = 12,
  SH_ARC0_NEG = 13,
  SH_ARC90_POS = 14,
  SH_ARC90_NEG = 15,
  SH_ARC180_POS = 16,
  SH_ARC180_NEG = 17,
  SH_ARC270_POS = 18,
  SH_ARC270_NEG = 19,
  numberOfAutShapeClasses
};

#endif // __GLOBALS_H__
