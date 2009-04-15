#ifndef RECT2D_H
#define RECT2D_H

#include <milton.h>

class Rect2D {

 public:

  Rect2D();
  Rect2D(const float _x0, const float _y0,
	 const float _width, const float _height);

  virtual ~Rect2D() {}

  static Rect2D xywh(const float x0, const float y0,
		     const float width, const float height);

  static Rect2D xywh(const Vector2 x0y0, const Vector2 wh);
  
  float width;
  float height;
  
  Vector2 x0y0;

};

#endif
