#ifndef SPLINTER_ROS__SPLINTER1D_HPP_
#define SPLINTER_ROS__SPLINTER1D_HPP_

#include <memory>
#include <vector>


namespace splinter_ros
{
struct Splinter1dImpl;
class Splinter1d
{
public:
  explicit Splinter1d(const std::vector<double> & x,
    const std::vector<double> & y);

  ~Splinter1d();

  double eval(const double & x);

private:
  std::unique_ptr<Splinter1dImpl> impl_;
};
}  // namespace splinter_ros
#endif  // SPLINTER_ROS__SPLINTER1D_HPP_
