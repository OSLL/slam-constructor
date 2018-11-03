#include <gtest/gtest.h>

#include "../mock_grid_cell.h"
#include "../../../src/core/maps/plain_grid_map.h"
#include "../../../src/core/maps/lazy_tiled_grid_map.h"
#include "../../../src/core/maps/rescalable_caching_grid_map.h"

template <typename MapT>
class GridCommonTest : public ::testing::Test {
public: // type aliases
  using MapType = MapT;
};

class TaggedMockGridCell : public MockGridCell {
public:
  TaggedMockGridCell(double occ_prob = Default_Occ_Prob)
    : TaggedMockGridCell{0, occ_prob} {}
  TaggedMockGridCell(int tag, double occ_prob)
    : MockGridCell{occ_prob}, _tag{tag} {}

  std::unique_ptr<GridCell> clone() const override {
    return std::make_unique<TaggedMockGridCell>(*this);
  }

  int tag() const { return _tag; }
private:
  int _tag;
};

//------------------------------------------------------------------------------
// Tests

using MapTs = ::testing::Types<
  PlainGridMap, UnboundedPlainGridMap,
  LazyTiledGridMap, UnboundedLazyTiledGridMap,
  RescalableCachingGridMap<UnboundedPlainGridMap>,
  RescalableCachingGridMap<UnboundedLazyTiledGridMap>>;

TYPED_TEST_CASE(GridCommonTest, MapTs);

TYPED_TEST(GridCommonTest, reset) {
  using MapT = typename TestFixture::MapType;
  auto map = MapT{std::make_shared<TaggedMockGridCell>(), {10, 10, 1}};
  auto area_id = map.internal2external({2, 4});
  static constexpr int Tag_ID = 42;
  static constexpr double Tagged_Prob = 14;
  auto area = TaggedMockGridCell(Tag_ID, Tagged_Prob);
  map.reset(area_id, area);

  auto stored_area = dynamic_cast<const TaggedMockGridCell*>(&map[area_id]);
  ASSERT_NE(stored_area, nullptr);
  ASSERT_DOUBLE_EQ(double(*stored_area), Tagged_Prob);
  ASSERT_EQ(stored_area->tag(), Tag_ID);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
