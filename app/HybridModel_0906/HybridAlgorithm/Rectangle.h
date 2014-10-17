#ifndef RECTANGLE_H
#define RECTANGLE_H

#define RECT_EMPTY(w,h)        (w <= 0 || h <= 0)

#ifndef EPSLON
#define EPSLON 0.000001
#endif

#ifndef EDGE_THRESHOLD
#define EDGE_THRESHOLD 0.01
#endif

enum POINT_TILE_STATUS
{
	INSIDE = 100,
	LEFTTOP_CORNER = 1,
	RIGHTTOP_CORNER = 2,
	RIGHTBOTTOM_CORNER = 3,
	LEFTBOTTOM_CORNER = 4,
	TOP_EDGE = 11,
	BOTTOM_EDGE = 12,
	LEFT_EDGE = 13,
	RIGHT_EDGE = 14,
	OUTSIDE = 1000
};

template <class _T>
class CRectangle {
  _T x_, y_, w_, h_;
public:

  /*! Left edge */
  _T x() const {return x_;}
  /*! Top edge */
  _T y() const {return y_;}
  /*! Distance between left and right edges */
  _T w() const {return w_;}
  /*! Distance between top and bottom edges */
  _T h() const {return h_;}
  /*! Return x()+w(), the right edge of the rectangle. */
  _T r() const {return x_+w_;}
  /*! Return y()+h(), the bottom edge of the rectangle. */
  _T b() const {return y_+h_;}
  /*! Move the rectangle so the left edge is at \a v. */
  void x(_T v) {x_ = v;}
  /*! Move the rectangle so the top edge is at \a v. */
  void y(_T v) {y_ = v;}
  /*! Change w() by moving the right edge. x() does not change. */
  void w(_T v) {w_ = v-EPSLON;}
  /*! Change h() by moving the bottom edge. y() does not change. */
  void h(_T v) {h_ = v-EPSLON;}
  /*! Change x() without changing r(), by changing the width. */
  void set_x(_T v) {w_ -= v-x_; x_ = v;}
  /*! Change y() without changing b(), by changing the height. */
  void set_y(_T v) {h_ -= v-y_; y_ = v;}
  /*! Change r() without changing x(), by changine the width. */
  void set_r(_T v) {w_ = v-x_;}
  /*! Change b() without changing y(), by changine the height. */
  void set_b(_T v) {h_ = v-y_;}
  /*! Set x(), y(), w(), and h() all at once. */
  void set(_T x, _T y, _T w, _T h) {x_=x; y_=y; w_=w-EPSLON; h_=h-EPSLON;}
  /*! Sets  x, y, w, h so that's it's centered or aligned (if flags!=0) inside the source r */
  void set (const CRectangle& r, _T w, _T h, _T flags = 0);
  /*! Add \a d to x() without changing r() (it reduces w() by \a d). */
  void move_x(_T d) {x_ += d; w_ -= d;}
  /*! Add \a d to y() without changing b() (it reduces h() by \a d). */
  void move_y(_T d) {y_ += d; h_ -= d;}
  /*! Add \a d to r() and w(). */
  void move_r(_T d) {w_ += d;}
  /*! Add \a d to b() and h(). */
  void move_b(_T d) {h_ += d;}
  /*! Move all edges in by \a d. See also Symbol::inset() */
  void inset(_T d) {x_ += d; y_ += d; w_ -= 2*d; h_ -= 2*d;}
  /*! Move entire rectangle by given distance in x and y. */
  void move(_T dx, _T dy) {x_ += dx; y_ += dy;}
  /*! True if w() or h() are less or equal to zero. */
  bool empty() const {return RECT_EMPTY(w_, h_);}
  /*! Same as !empty(), true if w() and h() are both greater than zero. */
  bool not_empty() const {return  !RECT_EMPTY(w_, h_);}
  /*! _Teger center position. Rounded to the left if w() is odd. */
  _T center_x() const {return x_+(w_/2);}
  /*! _Teger center position. Rounded to lower y if h() is odd. */
  _T center_y() const {return y_+(h_/2);}
  /*! Where to put baseline to center current font nicely */
  _T baseline_y() const;
  _T getwidth() {return this->w_;}
  _T getheight() {return this->h_;}
  CRectangle() {}

  /*! Constructor that sets x(), y(), w(), and h(). */
  CRectangle(_T x, _T y, _T w, _T h) : x_(x), y_(y), w_(w-EPSLON), h_(h-EPSLON) {}

  /*! Constructor that sets x() and y() to zero, and sets w() and h(). */
  CRectangle(_T w, _T h) : x_(0), y_(0), w_(w-EPSLON), h_(h-EPSLON) {}

  /*! Copy constructor. */
  CRectangle(const CRectangle& r) : x_(r.x_),y_(r.y_),w_(r.w_),h_(r.h_) {}

  /*! Constructor that calls set(). */
  CRectangle(const CRectangle& r, _T w, _T h, _T flags = 0) {set(r,w,h,flags);}

  /*! True if rectangle contains the pixel who's upper-left corner is at x,y */
  bool contains(_T x, _T y) const {return x>=x_ && y>=y_ && x<x_+w_ && y<y_+h_;}

  POINT_TILE_STATUS GetStatus(_T x, _T y) 
  {
	if(x > x_ && y > y_ && x < x_+w_ && y < y_+h_)
	{
		return INSIDE;
	}
	else if(x < x_-EDGE_THRESHOLD || y < y_-EDGE_THRESHOLD || x > x_+w_+EDGE_THRESHOLD || y > y_+h_+EDGE_THRESHOLD)
	{
		return OUTSIDE;
	}
	else if(x >= x_-EDGE_THRESHOLD && x <= x_+EDGE_THRESHOLD && y >= y_-EDGE_THRESHOLD && y <= y_+EDGE_THRESHOLD)
	{
		return LEFTTOP_CORNER;
	}
	else if(x >= x_-EDGE_THRESHOLD && x<= x_+EDGE_THRESHOLD && y >= y_+h_-EDGE_THRESHOLD && y <= y_+h_+EDGE_THRESHOLD)
	{
		return LEFTBOTTOM_CORNER;
	}
	else if(x >= x_+w_-EDGE_THRESHOLD && x <= x_+w_+EDGE_THRESHOLD && y >= y_-EDGE_THRESHOLD && y <= y_+EDGE_THRESHOLD)
	{
		return RIGHTTOP_CORNER;
	}
	else if(x >= x_+w_-EDGE_THRESHOLD && x <= x_+w_+EDGE_THRESHOLD && y >= y_+h_-EDGE_THRESHOLD && y <= y_+h_+EDGE_THRESHOLD)
	{
		return RIGHTBOTTOM_CORNER;
	}
	else if(x >= x_-EDGE_THRESHOLD && x<= x_+EDGE_THRESHOLD)
	{
		return LEFT_EDGE;
	}
	else if(x >= x_+w_-EDGE_THRESHOLD && x <= x_+w_+EDGE_THRESHOLD)
	{
		return RIGHT_EDGE;
	}
	else if(y >= y_-EDGE_THRESHOLD && y <= y_+EDGE_THRESHOLD)
	{
		return TOP_EDGE;
	}
	else if(y >= y_+h_-EDGE_THRESHOLD && y <= y_+h_+EDGE_THRESHOLD)
	{
		return BOTTOM_EDGE;
	}
  }

};

#endif
