#ifndef NODE_HPP_
#define NODE_HPP_

template <typename T>
class Node
{
public:
  Node(T x = 0, T y = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0)
    : x_(x), y_(y), g_(g), h_(h), id_(id), pid_(pid){};
  Node(const Node& n) : x_(n.x()), y_(n.y()), g_(n.g()), h_(n.h()), id_(n.id()), pid_(n.pid()){};

  T x() const
  {
    return x_;
  };
  T y() const
  {
    return y_;
  };
  double g() const
  {
    return g_;
  };
  double h() const
  {
    return h_;
  };
  int id() const
  {
    return id_;
  };
  int pid() const
  {
    return pid_;
  };

  void set_x(T x)
  {
    x_ = x;
  };
  void set_y(T y)
  {
    y_ = y;
  };
  void set_g(double g)
  {
    g_ = g;
  };
  void set_h(double h)
  {
    h_ = h;
  };
  void set_id(int id)
  {
    id_ = id;
  };
  void set_pid(int pid)
  {
    pid_ = pid;
  };

  Node operator+(const Node& n) const
  {
    return Node(x_ + n.x(), y_ + n.y());
  };

  Node operator-(const Node& n) const
  {
    return Node(x_ - n.x(), y_ - n.y());
  };

  bool operator==(const Node& n) const
  {
    return x_ == n.x() && y_ == n.y();
  };

  bool operator!=(const Node& n) const
  {
    return !operator==(n);
  };

  struct compare_cost
  {
    bool operator()(const Node& n1, const Node& n2) const
    {
      return (n1.g() + n1.h() > n2.g() + n2.h()) || ((n1.g() + n1.h() == n2.g() + n2.h()) && (n1.h() > n2.h()));
    };
  };

  struct compare_coordinates
  {
    bool operator()(const Node& n1, const Node& n2) const
    {
      return (n1.x() == n2.x()) && (n1.y() == n2.y());
    };
  };

protected:
  T x_, y_;       
  double g_, h_;  
  int id_, pid_;  
};

#endif