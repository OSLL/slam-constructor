#ifndef SLAM_CTOR_UTILS_CONSOLE_VIEW_H_INCLUDED
#define SLAM_CTOR_UTILS_CONSOLE_VIEW_H_INCLUDED

void show_grid_map(const GridMap &map, const Point2D &center,
                   double d_x_left, double d_x_right,
                   double d_y_up, double d_y_down) {
  auto center_cell = map.world_to_cell(center);

  auto delta_c = DiscretePoint2D{0, 0};
  auto top_left = map.world_to_cell(d_x_left, d_y_up);
  auto bot_right = map.world_to_cell(d_x_right, d_y_down);
  for (delta_c.y = top_left.y; -bot_right.y <= delta_c.y; --delta_c.y) {
    for (delta_c.x = -top_left.x; delta_c.x <= bot_right.x; ++delta_c.x) {
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
                   double d_x, double d_y) {
  show_grid_map(map, center, d_x, d_x, d_y, d_y);
}

#endif
