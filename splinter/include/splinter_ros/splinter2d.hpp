#ifndef SPLINTER_ROS__SPLINTER2D_HPP_
#define SPLINTER_ROS__SPLINTER2D_HPP_

#include <memory>
#include <vector>


namespace splinter_ros
{
struct Splinter2dImpl;
class Splinter2d
{
public:
  explicit Splinter2d(const std::vector<double> & x,
    const std::vector<double> & y,
    const std::vector<std::vector<double>> & z);

  ~Splinter2d();

  void update(const std::vector<double> & x,
    const std::vector<double> & y,
    const std::vector<std::vector<double>> & z);

  double eval(const double & x,
    const double & y) const;

private:
  std::unique_ptr<Splinter2dImpl> impl_;
};
}  // namespace splinter_ros
#endif  // SPLINTER_ROS__SPLINTER2D_HPP_
