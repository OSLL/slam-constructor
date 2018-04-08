#ifndef SLAM_CTOR_CORE_RANDOM_UTILS_H
#define SLAM_CTOR_CORE_RANDOM_UTILS_H

#include <random>

// Random variables

template<typename T>
class RandomVariable1D {
public:
  virtual double sample(T& rnd_engine) = 0;
  virtual std::unique_ptr<RandomVariable1D<T>> clone() const = 0;
  virtual ~RandomVariable1D() {}
};

template <typename Engine>
class GaussianRV1D : public RandomVariable1D<Engine> {
public:
  GaussianRV1D(double mean, double stddev)
    : _mean{mean}, _stddev{stddev}, _distr{_mean, _stddev} {}

  double sample(Engine &rnd_engine) override {
    return _distr(rnd_engine);
  }

  std::unique_ptr<RandomVariable1D<Engine>> clone() const override {
    return std::make_unique<GaussianRV1D<Engine>>(_mean, _stddev);
  }

private:
  double _mean;
  double _stddev;
  std::normal_distribution<> _distr;
};

template <typename Engine>
class UniformRV1D : public RandomVariable1D<Engine> {
public:
  UniformRV1D(double from, double to)
    : _from{from}, _to{to}, _distr{_from, _to} {}

  double sample(Engine &rnd_engine) override {
    return _distr(rnd_engine);
  }

  std::unique_ptr<RandomVariable1D<Engine>> clone() const override {
    return std::make_unique<UniformRV1D<Engine>>(_from, _to);
  }

private:
  double _from;
  double _to;
  std::uniform_real_distribution<> _distr;
};

#endif
