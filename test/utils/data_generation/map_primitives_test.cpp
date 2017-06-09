#include <gtest/gtest.h>

#include <algorithm>

#include "../../../src/utils/data_generation/map_primitives.h"

class CecumTextRasterMapPrimitiveTest : public ::testing::Test {
protected:
  using CecumMPBndPos = CecumTextRasterMapPrimitive::BoundPosition;;
protected: // methods
  CecumTextRasterMapPrimitiveTest() {}

  std::string to_str(const CecumTextRasterMapPrimitive &cecum_mp) {
    auto str = std::string{};

    std::istream& txt_stream = cecum_mp.to_stream();
    auto buf = std::string{};
    while (txt_stream.good()) {
      std::getline(txt_stream, buf);
      str += buf;
    }
    return str;
  }

  std::string pattern_to_expected(const std::string &pattern) {
    std::string result = pattern;
    std::replace(result.begin(), result.end(),
                 '1', TextRasterMapPrimitive::Completely_Occupied_Marker);
    std::replace(result.begin(), result.end(),
                 '0', TextRasterMapPrimitive::Completely_Free_Marker);
    return result;
  }
};

//------------------------------------------------------------------------------
// Degenerate cases

TEST_F(CecumTextRasterMapPrimitiveTest, singlePixelTop) {
  auto cecum_mp = CecumTextRasterMapPrimitive{1, 1, CecumMPBndPos::Top};

  const std::string Map_Pattern = "1";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(1, cecum_mp.width());
  ASSERT_EQ(1, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(0, cecum_mp.free_space()[0].area());
}

TEST_F(CecumTextRasterMapPrimitiveTest, singlePixelWidthTop) {
  auto cecum_mp = CecumTextRasterMapPrimitive{1, 2, CecumMPBndPos::Top};

  const std::string Map_Pattern = "1"
                                  "1";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(1, cecum_mp.width());
  ASSERT_EQ(2, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(0, cecum_mp.free_space()[0].area());
}

TEST_F(CecumTextRasterMapPrimitiveTest, singlePixelHeightTop) {
  auto cecum_mp = CecumTextRasterMapPrimitive{3, 1, CecumMPBndPos::Top};

  const std::string Map_Pattern = "111";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(3, cecum_mp.width());
  ASSERT_EQ(1, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(0, cecum_mp.free_space()[0].area());
}

TEST_F(CecumTextRasterMapPrimitiveTest, singlePixelHeightBot) {
  auto cecum_mp = CecumTextRasterMapPrimitive{3, 1, CecumMPBndPos::Bot};

  const std::string Map_Pattern = "111";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(3, cecum_mp.width());
  ASSERT_EQ(1, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(0, cecum_mp.free_space()[0].area());
}

TEST_F(CecumTextRasterMapPrimitiveTest, singlePixelLeft) {
  auto cecum_mp = CecumTextRasterMapPrimitive{1, 1, CecumMPBndPos::Left};

  const std::string Map_Pattern = "1";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(1, cecum_mp.width());
  ASSERT_EQ(1, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(0, cecum_mp.free_space()[0].area());
}

TEST_F(CecumTextRasterMapPrimitiveTest, singlePixelWidthLeft) {
  auto cecum_mp = CecumTextRasterMapPrimitive{1, 3, CecumMPBndPos::Left};

  const std::string Map_Pattern = "1"
                                  "1"
                                  "1";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(1, cecum_mp.width());
  ASSERT_EQ(3, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(0, cecum_mp.free_space()[0].area());
}

TEST_F(CecumTextRasterMapPrimitiveTest, singlePixelWidthRight) {
  auto cecum_mp = CecumTextRasterMapPrimitive{1, 3, CecumMPBndPos::Right};

  const std::string Map_Pattern = "1"
                                  "1"
                                  "1";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(1, cecum_mp.width());
  ASSERT_EQ(3, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(0, cecum_mp.free_space()[0].area());
}

TEST_F(CecumTextRasterMapPrimitiveTest, singlePixelHeightRight) {
  auto cecum_mp = CecumTextRasterMapPrimitive{5, 1, CecumMPBndPos::Right};

  const std::string Map_Pattern = "11111";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(5, cecum_mp.width());
  ASSERT_EQ(1, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(0, cecum_mp.free_space()[0].area());
}

//------------------------------------------------------------------------------
// No free space cases

TEST_F(CecumTextRasterMapPrimitiveTest, noFreeSpaceTop) {
  auto cecum_mp = CecumTextRasterMapPrimitive{2, 3, CecumMPBndPos::Top};

  const std::string Map_Pattern = "11"
                                  "11"
                                  "11";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(2, cecum_mp.width());
  ASSERT_EQ(3, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(0, cecum_mp.free_space()[0].area());
}

TEST_F(CecumTextRasterMapPrimitiveTest, noFreeSpaceBot) {
  auto cecum_mp = CecumTextRasterMapPrimitive{2, 4, CecumMPBndPos::Bot};

  const std::string Map_Pattern = "11"
                                  "11"
                                  "11"
                                  "11";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(2, cecum_mp.width());
  ASSERT_EQ(4, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(0, cecum_mp.free_space()[0].area());
}

TEST_F(CecumTextRasterMapPrimitiveTest, noFreeSpaceLeft) {
  auto cecum_mp = CecumTextRasterMapPrimitive{4, 2, CecumMPBndPos::Left};

  const std::string Map_Pattern = "1111"
                                  "1111";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(4, cecum_mp.width());
  ASSERT_EQ(2, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(0, cecum_mp.free_space()[0].area());
}

TEST_F(CecumTextRasterMapPrimitiveTest, noFreeSpaceRight) {
  auto cecum_mp = CecumTextRasterMapPrimitive{5, 2, CecumMPBndPos::Right};

  const std::string Map_Pattern = "11111"
                                  "11111";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(5, cecum_mp.width());
  ASSERT_EQ(2, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(0, cecum_mp.free_space()[0].area());
}

//------------------------------------------------------------------------------
// Common cases

TEST_F(CecumTextRasterMapPrimitiveTest, plainCecumTop) {
  auto cecum_mp = CecumTextRasterMapPrimitive{5, 4, CecumMPBndPos::Top};

  const std::string Map_Pattern = "11111"
                                  "10001"
                                  "10001"
                                  "10001";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(5, cecum_mp.width());
  ASSERT_EQ(4, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(Rectangle(-3, 0, 1, 4), cecum_mp.free_space()[0]);
}

TEST_F(CecumTextRasterMapPrimitiveTest, plainCecumBot) {
  auto cecum_mp = CecumTextRasterMapPrimitive{6, 3, CecumMPBndPos::Bot};

  const std::string Map_Pattern = "100001"
                                  "100001"
                                  "111111";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(6, cecum_mp.width());
  ASSERT_EQ(3, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(Rectangle(-1, 1, 1, 5), cecum_mp.free_space()[0]);
}

TEST_F(CecumTextRasterMapPrimitiveTest, plainCecumLeft) {
  auto cecum_mp = CecumTextRasterMapPrimitive{6, 4, CecumMPBndPos::Left};

  const std::string Map_Pattern = "111111"
                                  "100000"
                                  "100000"
                                  "111111";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(6, cecum_mp.width());
  ASSERT_EQ(4, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(Rectangle(-2, 0, 1, 6), cecum_mp.free_space()[0]);
}

TEST_F(CecumTextRasterMapPrimitiveTest, plainCecumRight) {
  auto cecum_mp = CecumTextRasterMapPrimitive{7, 5, CecumMPBndPos::Right};

  const std::string Map_Pattern = "1111111"
                                  "0000001"
                                  "0000001"
                                  "0000001"
                                  "1111111";
  ASSERT_EQ(pattern_to_expected(Map_Pattern), to_str(cecum_mp));
  ASSERT_EQ(7, cecum_mp.width());
  ASSERT_EQ(5, cecum_mp.height());
  ASSERT_EQ(1, cecum_mp.free_space().size());
  ASSERT_EQ(Rectangle(-3, 0, 0, 6), cecum_mp.free_space()[0]);
}

//------------------------------------------------------------------------------

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
