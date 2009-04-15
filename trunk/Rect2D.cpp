#include "Rect2D.h"

Rect2D::Rect2D()
{
  width = 0;
  height = 0;
  x0y0 = Vector2::zero();
}


Rect2D::Rect2D(const float _x0, const float _y0,
	       const float _width, const float _height)
{
  x0y0 = Vector2(_x0, _y0);
  width = _width;
  height = _height;
}


Rect2D Rect2D::xywh(const float x0, const float y0,
		    const float width, const float height)
{
  return Rect2D(x0, y0, width, height);
}


Rect2D Rect2D::xywh(const Vector2 x0y0, const Vector2 wh)
{
  return Rect2D(x0y0[0], x0y0[1], wh[0], wh[1]);
}
