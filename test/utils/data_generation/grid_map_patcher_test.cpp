#include <gtest/gtest.h>

#include <sstream>

#include "../../../src/utils/data_generation/grid_map_patcher.h"
#include "../../../src/core/maps/plain_grid_map.h"

constexpr int Map_Width = 100;
constexpr int Map_Height = 100;
constexpr double Map_Scale = 0.1;

const std::string Cecum_Corridor_Map_Patch =
  "+-+\n"
  "| |\n"
  "| |";
constexpr int Cecum_Patch_W = 3, Cecum_Patch_H = 3;

constexpr int Cecum_Free_X_Start = 1, Cecum_Free_Y_Start = -1;
constexpr int Cecum_Free_W = 1, Cecum_Free_H = 2;

class TestGridCell : public GridCell {
public:
  TestGridCell() : GridCell{Occupancy{0, 0}} {}
  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<TestGridCell>(*this);
  }
};

class GridMapPatcherTest : public ::testing::Test {
protected: // methods
  GridMapPatcherTest()
    : map{std::make_shared<TestGridCell>(),
          GridMapParams{Map_Width, Map_Height, Map_Scale}} {}
protected: // fields

  void test_cecum_patch_application(int w_scale, int h_scale,
                                    const DiscretePoint2D& patch_offset,
                                    bool use_auto_offset = false) {
    std::stringstream raster{Cecum_Corridor_Map_Patch};
    auto offset = patch_offset;
    if (use_auto_offset) {
      gm_patcher.apply_text_raster(map, raster, w_scale, h_scale);
      offset = DiscretePoint2D{-Cecum_Patch_W * w_scale / 2,
                               Cecum_Patch_H * h_scale / 2};
    } else {
      gm_patcher.apply_text_raster(map, raster, offset, w_scale, h_scale);
    }

    // free area bounds
    double free_x_min = offset.x + Cecum_Free_X_Start * w_scale;
    double free_x_max = free_x_min + Cecum_Free_W * w_scale;
    double free_y_max = offset.y + Cecum_Free_Y_Start * h_scale;
    double free_y_min = free_y_max - Cecum_Free_H * h_scale;
    auto free_area = Rectangle{free_y_min, free_y_max + 1,
                               free_x_min, free_x_max};

    // check area bounds
    const int Check_Left = offset.x - 1;
    const int Check_Right = Check_Left + Cecum_Patch_W * w_scale + 1;
    const int Check_Top = offset.y + 1;
    const int Check_Bot = Check_Top - Cecum_Patch_H * h_scale - 1;

    DiscretePoint2D coord;
    for (coord.y = Check_Top; Check_Bot <= coord.y; --coord.y) {
      for (coord.x = Check_Left; coord.x <= Check_Right; ++coord.x) {
        // std::cout << map[coord] << " ";
        if (coord.x == Check_Left || coord.x == Check_Right ||
            coord.y == Check_Top || coord.y == Check_Bot) {
          ASSERT_EQ(0.0, map[coord]);
          continue;
        }

        bool is_free = free_area.contains(Point2D{coord.x + map.scale(),
                                                  coord.y + map.scale()});
        double expected_cell_value = is_free ? 0.0 : 1.0;
        ASSERT_EQ(expected_cell_value, map[coord]);
      }
      //std::cout << std::endl;
    }
  }

  UnboundedPlainGridMap map;
  GridMapPatcher gm_patcher;
};

//-- Scale

TEST_F(GridMapPatcherTest, patchCecumNoScaleNoOffset) {
  test_cecum_patch_application(1, 1, DiscretePoint2D{0, 0});
}

TEST_F(GridMapPatcherTest, patchCecumHorizScaleNoOffset) {
  test_cecum_patch_application(2, 1, DiscretePoint2D{0, 0});
}

TEST_F(GridMapPatcherTest, patchCecumVertScaleNoOffset) {
  test_cecum_patch_application(1, 3, DiscretePoint2D{0, 0});
}

TEST_F(GridMapPatcherTest, patchCecumEvenScaleNoOffset) {
  test_cecum_patch_application(8, 8, DiscretePoint2D{0, 0});
}

//-- Offset

TEST_F(GridMapPatcherTest, patchCecumNoScaleLeftOffset) {
  test_cecum_patch_application(1, 1, DiscretePoint2D{-1, 0});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleRightOffset) {
  test_cecum_patch_application(1, 1, DiscretePoint2D{3, 0});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleDownOffset) {
  test_cecum_patch_application(1, 1, DiscretePoint2D{-6, 0});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleUpOffset) {
  test_cecum_patch_application(1, 1, DiscretePoint2D{7, 0});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleLeftUpOffset) {
  test_cecum_patch_application(1, 1, DiscretePoint2D{-5, 6});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleLeftDownOffset) {
  test_cecum_patch_application(1, 1, DiscretePoint2D{-4, -1});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleRightUpOffset) {
  test_cecum_patch_application(1, 1, DiscretePoint2D{3, 7});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleRightDownOffset) {
  test_cecum_patch_application(1, 1, DiscretePoint2D{7, 5});
}

TEST_F(GridMapPatcherTest, patchCecumUnevenScaleOffset) {
  test_cecum_patch_application(6, 4, DiscretePoint2D{11, 15});
}

//-- Auto Offset

TEST_F(GridMapPatcherTest, patchCecumEvenScaleAutoOffset) {
  test_cecum_patch_application(7, 7, {}, true);
}

TEST_F(GridMapPatcherTest, patchCecumUnevenScaleAutoOffset) {
  test_cecum_patch_application(3, 8, {}, true);
}


int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
