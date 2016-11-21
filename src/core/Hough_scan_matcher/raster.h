#pragma once
#include <vector>
#include <functional>
#include <iostream>


template <typename PointT>
class RasterGrid{
public: // typenames
  using world_t      = double;
  using Array_coords = std::vector<world_t>;
  using Array_points = std::vector<PointT>;

private: //fields
  world_t _delta_x;
  world_t _delta_y;

public:
  //constructor
  RasterGrid(double delta_x, double delta_y):_delta_x(delta_x),
                                             _delta_y(delta_y) {}

  std::vector<PointT> raster_build(const world_t x_st, const world_t x_end,
                                   std::function<world_t(world_t)> func) {
    Array_points res;

    Array_coords x1 = points_x(x_st,x_end);
    Array_coords y1 = points_y(func,x1);
    Array_points points = create(x1,y1);
    size_t end = points.size() - 1;

    for (size_t i = 0; i < end; i++) {
      PointT current = points[i];
      PointT next = points[i+1];
      res.push_back(current);
      if (current.y < next.y) {
        for(int y = current.y+1; y < next.y; y++) {
          res.push_back({current.x,y});
        }
      }
      else {
        for(int y = current.y-1; next.y < y; y--) {
          res.push_back({current.x,y});
        }
      }
    }
    res.push_back(points[end]);
    return res;
  }

  world_t delta_x() const { return _delta_x; }
  world_t delta_y() const { return _delta_y; }
private: // methods
  Array_points create(const Array_coords& x, const Array_coords& y) {
    Array_points out;
    for (size_t i = 0; i < x.size(); i++) {
      int x_cell = x_to_cell(x[i]+_delta_x/3);
      int y_cell = y_to_cell(y[i]+_delta_y/3);
      out.push_back({x_cell,y_cell});
    }
    return out;
  }

  int x_to_cell(const world_t x_world) { return std::floor(x_world/_delta_x); }
  int y_to_cell(const world_t y_world) { return std::floor(y_world/_delta_y); }

  Array_coords points_x(const world_t x_st, const world_t x_end){
    Array_coords out;
    if (x_st < x_end) {
      for (world_t x = x_st; x <= x_end; x+=_delta_x)
        out.push_back(x);
    }
    else {
      for (world_t x = x_st; x_end <= x; x-=_delta_x)
        out.push_back(x);
    }
    return out;
  }

  Array_coords points_y(std::function<world_t(world_t)> func,
                        const Array_coords& x_array) {
    Array_coords out(x_array.size());
    for (size_t i = 0; i < x_array.size(); i++)
      out[i] = func(x_array[i]);
    return out;
  }
};
