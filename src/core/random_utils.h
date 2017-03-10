#ifndef SLAM_CTOR_CORE_RANDOM_UTILS_H_INCLUDED
#define SLAM_CTOR_CORE_RANDOM_UTILS_H_INCLUDED

#include <random>

template<typename T>
class RandomVariable1D {
public:
  virtual double sample(T& rnd_engine) = 0;
  virtual std::unique_ptr<RandomVariable1D> clone() const = 0;
  virtual ~RandomVariable1D() {}
};

class GaussianRV1D : public RandomVariable1D<std::mt19937> {
public:
  GaussianRV1D(double mean, double sigma)
    : _mean{mean}, _sigma{sigma}, _distr{_mean, _sigma} {}

  std::unique_ptr<RandomVariable1D> clone() const override {
    return std::make_unique<GaussianRV1D>(_mean, _sigma);
  }

  double sample(std::mt19937 &rnd_engine) override {
    return _distr(rnd_engine);
  }

private:
  double _mean;
  double _sigma;
  std::normal_distribution<> _distr;
};

class UniformRV1D : public RandomVariable1D<std::mt19937> {
public:
  UniformRV1D(double from, double to)
    : _from{from}, _to{to}, _distr{_from, _to} {}

  double sample(std::mt19937 &rnd_engine) override {
    return _distr(rnd_engine);
  }

  std::unique_ptr<RandomVariable1D> clone() const override {
    return std::make_unique<UniformRV1D>(_from, _to);
  }

private:
  double _from;
  double _to;
  std::uniform_real_distribution<> _distr;
};

#endif
