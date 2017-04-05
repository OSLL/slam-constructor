#ifndef SLAM_CTOR_UTILS_CONSOLE_VIEW_H_INCLUDED
#define SLAM_CTOR_UTILS_CONSOLE_VIEW_H_INCLUDED

void show_grid_map(const GridMap &map, const Point2D &center,
                   int d_x_left, int d_x_right, int d_y_up, int d_y_down) {
  auto center_cell = map.world_to_cell(center);
  auto sc = map.scale();

  auto delta_c = DiscretePoint2D{0, 0};
  for (delta_c.y = d_y_up / sc; -d_y_down / sc <= delta_c.y; --delta_c.y) {
    for (delta_c.x = -d_x_left / sc; delta_c.x <= d_x_right / sc; ++delta_c.x) {
      auto coord = center_cell + delta_c;
      auto cell_occ = map[coord];
      char map_cell_content;
      if (center_cell == coord) {
        map_cell_content = 'C';
      } else if (0.75 < cell_occ) {
        map_cell_content = '#';
      } else if (cell_occ < 0.25) {
        map_cell_content = '.';
      } else {
        map_cell_content = '~';
      }
      std::cout << map_cell_content;
    }
    std::cout << std::endl;
  }
}

void show_grid_map(const GridMap &map, const Point2D &center,
                   int d_x, int d_y) {
  show_grid_map(map, center, d_x, d_x, d_y, d_y);
}

#endif
