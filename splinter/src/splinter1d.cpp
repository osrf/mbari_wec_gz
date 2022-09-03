

#include <splinter_ros/splinter1d.hpp>

#include <memory>
#include <vector>

// Interpolation library
#include <splinter/datatable.h>
#include <splinter/bspline.h>
#include <splinter/bsplinebuilder.h>


namespace splinter_ros
{
struct Splinter1dImpl
{
  SPLINTER::DataTable samples;
  std::unique_ptr<SPLINTER::BSpline> splinter1d;

  explicit Splinter1dImpl(const std::vector<double> & _x,
    const std::vector<double> & _y)
  {
    for (size_t idx = 0U; idx < _x.size(); ++idx) {
      // Sample function at x
      SPLINTER::DenseVector x(1);
      x(0) = _x[idx];
      double y = _y[idx];

      // Store sample
      samples.addSample(x, y);
    }
    // Build B-splines that interpolate the samples
    splinter1d = std::make_unique<SPLINTER::BSpline>(
      SPLINTER::BSpline::Builder(samples)
      .degree(1).build());
  }

  ~Splinter1dImpl() {}

  double eval(const double & _x)
  {
    SPLINTER::DenseVector x(1);
    x(0) = _x;
    return splinter1d->eval(x);
  }
};

Splinter1d::Splinter1d(const std::vector<double> & x,
  const std::vector<double> & y)
  : impl_(std::make_unique<Splinter1dImpl>(x, y))
{
}

Splinter1d::~Splinter1d()
{
}

double Splinter1d::eval(const double & x)
{
  return impl_->eval(x);
}
}  // namespace splinter_ros
