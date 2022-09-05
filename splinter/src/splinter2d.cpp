

#include <splinter_ros/splinter2d.hpp>

#include <memory>
#include <vector>

// Interpolation library
#include <splinter/datatable.h>
#include <splinter/bspline.h>
#include <splinter/bsplinebuilder.h>


namespace splinter_ros
{
struct Splinter2dImpl
{
  std::unique_ptr<SPLINTER::BSpline> splinter2d;
  SPLINTER::DataTable samples;

  explicit Splinter2dImpl(const std::vector<double> & _x,
    const std::vector<double> & _y,
    const std::vector<std::vector<double>> & _z)
  {
    for (size_t idx = 0U; idx < _x.size(); ++idx) {
      for (size_t jdx = 0U; jdx < _y.size(); ++jdx) {
        // Sample function at x
        SPLINTER::DenseVector x(2);
        x(0) = _x[idx];
        x(1) = _y[jdx];
        double y = _z[jdx][idx];

        // Store sample
        samples.addSample(x, y);
      }
    }
    // Build B-splines that interpolate the samples
    splinter2d = std::make_unique<SPLINTER::BSpline>(
      SPLINTER::BSpline::Builder(samples)
      .degree(1).build());
  }

  ~Splinter2dImpl()
  {
  }

  double eval(const double & _x,
    const double & _y) const
  {
    SPLINTER::DenseVector x(2);
    x(0) = _x;
    x(1) = _y;
    return splinter2d->eval(x);
  }
};

Splinter2d::Splinter2d(const std::vector<double> & x,
  const std::vector<double> & y,
  const std::vector<std::vector<double>> & z)
  : impl_(std::make_unique<Splinter2dImpl>(x, y, z))
{
}

Splinter2d::~Splinter2d()
{
}

void Splinter2d::update(const std::vector<double> & x,
  const std::vector<double> & y,
  const std::vector<std::vector<double>> & z)
{
  impl_ = std::make_unique<Splinter2dImpl>(x, y, z);
}

double Splinter2d::eval(const double & x,
  const double & y) const
{
  return impl_->eval(x, y);
}
}  // namespace splinter_ros
