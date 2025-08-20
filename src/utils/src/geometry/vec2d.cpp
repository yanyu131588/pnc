#include <cmath>


#include "geometry/vec2d.hpp"
#include "math/mathHelper.hpp"

double Vec2d::length() const
{
  return std::hypot(x_, y_);
}

double Vec2d::crossProd(const Vec2d& other) const
{
  return x_ * other.y() - y_ * other.x();
}

Vec2d Vec2d::operator+(const Vec2d& other) const
{
  return Vec2d(x_ + other.x(), y_ + other.y());
}

Vec2d Vec2d::operator-() const
{
  return Vec2d(-x_, -y_);
}

Vec2d Vec2d::operator-(const Vec2d& other) const
{
  return Vec2d(x_ - other.x(), y_ - other.y());
}

Vec2d Vec2d::operator*(const double ratio) const
{
  return Vec2d(x_ * ratio, y_ * ratio);
}

Vec2d Vec2d::operator/(const double ratio) const
{
  return Vec2d(x_ / ratio, y_ / ratio);
}

Vec2d& Vec2d::operator+=(const Vec2d& other)
{
  x_ += other.x();
  y_ += other.y();
  return *this;
}

Vec2d& Vec2d::operator-=(const Vec2d& other)
{
  x_ -= other.x();
  y_ -= other.y();
  return *this;
}

Vec2d& Vec2d::operator*=(const double ratio)
{
  x_ *= ratio;
  y_ *= ratio;
  return *this;
}

Vec2d& Vec2d::operator/=(const double ratio)
{
  x_ /= ratio;
  y_ /= ratio;
  return *this;
}

bool Vec2d::operator==(const Vec2d& other) const
{
  return (std::abs(x_ - other.x()) < kMathEpsilon &&
          std::abs(y_ - other.y()) < kMathEpsilon);
}

Vec2d operator*(const double ratio, const Vec2d& vec)
{
  return vec * ratio;
}


