#ifndef VECTOR2D_HPP_
#define VECTOR2D_HPP_


class Vec2d
{
public:
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y)
  {
  }

  constexpr Vec2d() noexcept : Vec2d(0, 0)
  {
  }

  double x() const
  {
    return x_;
  }

  double y() const
  {
    return y_;
  }

  void setX(const double x)
  {
    x_ = x;
  }

  void setY(const double y)
  {
    y_ = y;
  }

  double length() const;

  double crossProd(const Vec2d& other) const;

  Vec2d operator+(const Vec2d& other) const;

  Vec2d operator-() const;
  Vec2d operator-(const Vec2d& other) const;
  Vec2d operator*(const double ratio) const;
  Vec2d operator/(const double ratio) const;
  Vec2d& operator+=(const Vec2d& other);
  Vec2d& operator-=(const Vec2d& other);
  Vec2d& operator*=(const double ratio);
  Vec2d& operator/=(const double ratio);

  bool operator==(const Vec2d& other) const;

protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

Vec2d operator*(const double ratio, const Vec2d& vec);


#endif