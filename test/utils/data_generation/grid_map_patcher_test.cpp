#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>

#include "../../core/mock_grid_cell.h"

#include "../../../src/utils/data_generation/map_primitives.h"
#include "../../../src/utils/data_generation/grid_map_patcher.h"
#include "../../../src/core/maps/plain_grid_map.h"

class GridMapPatcherTest : public ::testing::Test {
protected: // methods
  GridMapPatcherTest()
    : map{std::make_shared<MockGridCell>(),
          GridMapParams{Map_Width, Map_Height, Map_Scale}} {}

protected: // consts
  static constexpr int Map_Width = 100;
  static constexpr int Map_Height = 100;
  static constexpr double Map_Scale = 0.1;
protected: // fields

  Rectangle scale_and_move_free_space(const Rectangle &fs,
                                      const DiscretePoint2D &offset,
                                      int w_scale, int h_scale) {
    assert(fs.top() <= 1 && fs.bot() <= 1 && 0 <= fs.left() && 0 <= fs.right());
    double left = offset.x + fs.left() * w_scale;
    double top = offset.y + (fs.top() - 1) * h_scale + 1;
    return Rectangle{top - fs.vside_len() * h_scale, top,
                     left, left + fs.hside_len() * w_scale};
  }

  void test_patching(const TextRasterMapPrimitive &mp, int w_scale, int h_scale,
                     const DiscretePoint2D &patch_offset = {0, 0},
                     bool use_auto_offset = false) {
    auto offset = patch_offset;
    if (use_auto_offset) {
      gm_patcher.apply_text_raster(map, mp.to_stream(), w_scale, h_scale);
      offset = DiscretePoint2D{-mp.width() * w_scale / 2,
                                mp.height() * h_scale / 2};
    } else {
      gm_patcher.apply_text_raster(map, mp.to_stream(),
                                   offset, w_scale, h_scale);
    }

    // free area bounds
    std::vector<Rectangle> free_space = mp.free_space();
    std::vector<Rectangle> free_areas;
    std::transform(free_space.begin(), free_space.end(),
                   std::back_inserter(free_areas),
                   [&,this](const Rectangle &free_space) {
                     return scale_and_move_free_space(free_space, offset,
                                                      w_scale, h_scale);
                   });

    // check area bounds
    const int Check_Left = offset.x - 1;
    const int Check_Right = Check_Left + mp.width() * w_scale + 1;
    const int Check_Top = offset.y + 1;
    const int Check_Bot = Check_Top - mp.height() * h_scale - 1;

    DiscretePoint2D coord;
    for (coord.y = Check_Top; Check_Bot <= coord.y; --coord.y) {
      for (coord.x = Check_Left; coord.x <= Check_Right; ++coord.x) {
        // std::cout << map[coord] << " ";
        if (coord.x == Check_Left || coord.x == Check_Right ||
            coord.y == Check_Top || coord.y == Check_Bot) {
          ASSERT_EQ(MockGridCell::Default_Occ_Prob, map[coord]);
          continue;
        }

        auto c_mid = Point2D{coord.x + map.scale()/2, coord.y + map.scale()/2};
        auto is_free = true;
        for (auto &fa : free_areas) {
          is_free &= fa.contains(c_mid);
          if (!is_free) { break; }
        }
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
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{7, 4, bnd_pos};
  test_patching(cecum_mp, 1, 1, DiscretePoint2D{0, 0});
}


TEST_F(GridMapPatcherTest, patchCecumHorizScaleNoOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Left;
  auto cecum_mp = CecumTextRasterMapPrimitive{7, 8, bnd_pos};
  test_patching(cecum_mp, 2, 1);
}

TEST_F(GridMapPatcherTest, patchCecumVertScaleNoOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Right;
  auto cecum_mp = CecumTextRasterMapPrimitive{4, 3, bnd_pos};
  test_patching(cecum_mp, 1, 3);
}

TEST_F(GridMapPatcherTest, patchCecumEvenScaleNoOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Bot;
  auto cecum_mp = CecumTextRasterMapPrimitive{4, 9, bnd_pos};

  test_patching(cecum_mp, 8, 8);
}

//-- Offset

TEST_F(GridMapPatcherTest, patchCecumNoScaleLeftOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Bot;
  auto cecum_mp = CecumTextRasterMapPrimitive{4, 5, bnd_pos};

  test_patching(cecum_mp, 1, 1, DiscretePoint2D{-1, 0});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleRightOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{4, 5, bnd_pos};

  test_patching(cecum_mp, 1, 1, DiscretePoint2D{3, 0});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleDownOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Left;
  auto cecum_mp = CecumTextRasterMapPrimitive{4, 5, bnd_pos};

  test_patching(cecum_mp, 1, 1, DiscretePoint2D{-6, 0});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleUpOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Right;
  auto cecum_mp = CecumTextRasterMapPrimitive{7, 5, bnd_pos};

  test_patching(cecum_mp, 1, 1, DiscretePoint2D{7, 0});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleLeftUpOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Bot;
  auto cecum_mp = CecumTextRasterMapPrimitive{9, 5, bnd_pos};

  test_patching(cecum_mp, 1, 1, DiscretePoint2D{-5, 6});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleLeftDownOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{16, 8, bnd_pos};

  test_patching(cecum_mp, 1, 1, DiscretePoint2D{-4, -1});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleRightUpOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Left;
  auto cecum_mp = CecumTextRasterMapPrimitive{4, 7, bnd_pos};

  test_patching(cecum_mp, 1, 1, DiscretePoint2D{3, 7});
}

TEST_F(GridMapPatcherTest, patchCecumNoScaleRightDownOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Right;
  auto cecum_mp = CecumTextRasterMapPrimitive{3, 5, bnd_pos};

  test_patching(cecum_mp, 1, 1, DiscretePoint2D{7, 5});
}

TEST_F(GridMapPatcherTest, patchCecumUnevenScaleOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Bot;
  auto cecum_mp = CecumTextRasterMapPrimitive{4, 5, bnd_pos};

  test_patching(cecum_mp, 6, 4, DiscretePoint2D{11, 15});
}

//-- Auto Offset

TEST_F(GridMapPatcherTest, patchCecumEvenScaleAutoOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Right;
  auto cecum_mp = CecumTextRasterMapPrimitive{7, 6, bnd_pos};

  test_patching(cecum_mp, 7, 7, {}, true);
}

TEST_F(GridMapPatcherTest, patchCecumUnevenScaleAutoOffset) {
  auto bnd_pos = CecumTextRasterMapPrimitive::BoundPosition::Top;
  auto cecum_mp = CecumTextRasterMapPrimitive{6, 7, bnd_pos};

  test_patching(cecum_mp, 3, 8, {}, true);
}

//------------------------------------------------------------------------------

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
