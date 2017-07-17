#include <gtest/gtest.h>

#include "directions.h"
#include "../../src/core/geometry_primitives.h"

/*----------------------------------------------------------------------------*/
/* LightWeightRectangle.intersection/overlap                                  */

class LWRIntersectionOverlapTest : public ::testing::Test {
protected:
  using Rect = LightWeightRectangle;
protected:
  static constexpr auto Base_Outer_Overlap = 1.0 / 9;

  void check_intersection(const Rect &r1, const Rect &r2,
                          const Rect &exp_inters, double exp_overlap) const {

    ASSERT_EQ(exp_inters, r1.intersect(r2));
    ASSERT_EQ(r1.intersect(r2), r2.intersect(r1));
    ASSERT_EQ(exp_overlap, r1.overlap(r2));
  }

  void test_corner_inclusion(const Directions &dirs) const {
    assert(dirs.horz() * dirs.vert());
    auto r1 = Rect{-6, 6, -6, 6};
    auto out_coffset = Point2D{dirs.horz() * r1.hside_len(),
                               dirs.vert() * r1.vside_len()};
    // FIXME: move to separate test cases
    { // mutual corner inclusion
      auto r2 = r1.move_center(out_coffset * 0.5);
      auto intersection = r1.shrink(2).move_center(out_coffset * 0.25);
      check_intersection(r1, r2, intersection, 0.25);
    }
    { // common corner
      auto r2 = r1.move_center(out_coffset);
      auto intersection_sides = out_coffset * 0.5;
      auto intersection = Rect{intersection_sides.y, intersection_sides.y,
                               intersection_sides.x, intersection_sides.x};
      check_intersection(r1, r2, intersection, 0.0);
    }
    { // common horiz edge part
      auto hshift = Point2D{dirs.horz() * -1.0, 0} + out_coffset;
      auto intersection_sides = out_coffset * 0.5;
      auto intersection = Rect{intersection_sides.y, intersection_sides.y,
                               (dirs.right() ? -1 : 0) + intersection_sides.x,
                               (dirs.left() ? 1 : 0) + intersection_sides.x};
      check_intersection(r1, r1.move_center(hshift), intersection, 0.0);
    }
    { // common vert edge part
      auto vshift = Point2D{0, dirs.vert() * -1.0} + out_coffset;
      auto intersection_sides = out_coffset * 0.5;
      auto intersection = Rect{(dirs.top() ? -1 : 0) + intersection_sides.y,
                               (dirs.bot() ? 1 : 0) + intersection_sides.y,
                               intersection_sides.x, intersection_sides.x};
      check_intersection(r1, r1.move_center(vshift), intersection, 0.0);
    }
  }

  void test_edge_inclusion(const Directions &dir) {
    assert(static_cast<bool>(dir.horz()) ^
           static_cast<bool>(dir.vert()));
    auto r1 = Rect{-6, 6, -6, 6};
    auto out_coffset = Point2D{dir.horz() * r1.hside_len(),
                               dir.vert() * r1.vside_len()};
    // FIXME: split cases
    {  // edge through the middle
      auto halves = (dir.horz() ? r1.split_horz() : r1.split_vert());
      auto half_r1 = halves[(1 == dir.horz() || 1 == dir.vert()) ? 1 : 0];
      check_intersection(r1, r1.move_center(out_coffset * 0.5),
                         half_r1.move_center(out_coffset * 0.25), 0.5);
    }
    { // common edge
      auto intersection_sides = out_coffset * 0.5;
      auto intersection = Rect{dir.horz() ? r1.bot() : intersection_sides.y,
                               dir.horz() ? r1.top() : intersection_sides.y,
                               dir.vert() ? r1.left() : intersection_sides.x,
                               dir.vert() ? r1.right() : intersection_sides.x};
      check_intersection(r1, r1.move_center(out_coffset), intersection, 0);
    }
  }

  void test_proper_inclusion(const Directions &dirs,
                             double exp_outer_overlap) const {
    auto outer = Rect{0, 12, 0, 12};

    double inner_left = 4, inner_right = 8,
           inner_bot = 4, inner_top = 8;

    #define ALIGN_SIDE(side)              \
      if (dirs.side()) {                  \
        inner_##side = outer.side();   \
      }

    ALIGN_SIDE(left);
    ALIGN_SIDE(right);
    ALIGN_SIDE(top);
    ALIGN_SIDE(bot);

    #undef ALIGN_SIDE

    auto inner = Rect{inner_bot, inner_top, inner_left, inner_right};
    check_intersection(inner, outer, inner, 1);
    check_intersection(outer, inner, inner, exp_outer_overlap);
  }

};

//------------------------------------------------------------------------------
// no overlap

TEST_F(LWRIntersectionOverlapTest, noIntersectionSameSize) {
  check_intersection({1, 2, 1, 2}, {3, 4, 3, 4}, {0, 0, 0, 0}, 0);
}

TEST_F(LWRIntersectionOverlapTest, noIntersectionDifferentSize) {
  check_intersection({1, 2, 1, 2}, {3, 5, 3, 5}, {0, 0, 0, 0}, 0);
}

//------------------------------------------------------------------------------
// corner inclusion

TEST_F(LWRIntersectionOverlapTest, cornerInclusionLT) {
  test_corner_inclusion(Directions{}.set_left().set_top());
}

TEST_F(LWRIntersectionOverlapTest, cornerInclusionRT) {
  test_corner_inclusion(Directions{}.set_right().set_top());
}

TEST_F(LWRIntersectionOverlapTest, cornerInclusionLB) {
  test_corner_inclusion(Directions{}.set_left().set_bot());
}

TEST_F(LWRIntersectionOverlapTest, cornerInclusionRB) {
  test_corner_inclusion(Directions{}.set_right().set_bot());
}

// TODO: check rectangles with different sizes

//------------------------------------------------------------------------------
// edge overlap

TEST_F(LWRIntersectionOverlapTest, edgeInclusionLeft) {
  test_edge_inclusion(Directions{}.set_left());
}

TEST_F(LWRIntersectionOverlapTest, edgeInclusionRight) {
  test_edge_inclusion(Directions{}.set_right());
}

TEST_F(LWRIntersectionOverlapTest, edgeInclusionTop) {
  test_edge_inclusion(Directions{}.set_top());
}

TEST_F(LWRIntersectionOverlapTest, edgeInclusionBot) {
  test_edge_inclusion(Directions{}.set_bot());
}

// TODO: check rectangles with different sizes

//------------------------------------------------------------------------------
// proper inclusion

TEST_F(LWRIntersectionOverlapTest, properInclusionNotSideAligned) {
  test_proper_inclusion(Directions{}, Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionLSideAligned) {
  test_proper_inclusion(Directions{}.set_left(), 2*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionRSideAligned) {
  test_proper_inclusion(Directions{}.set_right(), 2*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionRLSideAligned) {
  test_proper_inclusion(Directions{}.set_left().set_right(),
                        3*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionBSideAligned) {
  test_proper_inclusion(Directions{}.set_bot(), 2*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionBLSideAligned) {
  test_proper_inclusion(Directions{}.set_bot().set_left(),
                        4*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionBRSideAligned) {
  test_proper_inclusion(Directions{}.set_bot().set_right(),
                        4*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionBRLSideAligned) {
  test_proper_inclusion(Directions{}.set_bot().set_right().set_left(),
                        6*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionTSideAligned) {
  test_proper_inclusion(Directions{}.set_top(), 2*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionTLSideAligned) {
  test_proper_inclusion(Directions{}.set_top().set_left(),
                        4*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionTRSideAligned) {
  test_proper_inclusion(Directions{}.set_top().set_right(),
                        4*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionTRLSideAligned) {
  test_proper_inclusion(Directions{}.set_left().set_right().set_top(),
                        6*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionTBSideAligned) {
  test_proper_inclusion(Directions{}.set_bot().set_top(), 3*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionTBLSideAligned) {
  test_proper_inclusion(Directions{}.set_bot().set_left().set_top(),
                        6*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionTBRSideAligned) {
  test_proper_inclusion(Directions{}.set_bot().set_right().set_top(),
                        6*Base_Outer_Overlap);
}

TEST_F(LWRIntersectionOverlapTest, properInclusionTBRLSideAligned) {
  test_proper_inclusion(Directions{}.set_bot().set_right().set_left().set_top(),
                        9*Base_Outer_Overlap);
}

//------------------------------------------------------------------------------
// Degenerate rectangles: point/line-like rectangles

// point-rects

TEST_F(LWRIntersectionOverlapTest, pointRectInsidePointRect) {
  auto pnt_rect1 = Rect{0, 0, 0.5, 0.5}, pnt_rect2 = pnt_rect1;
  check_intersection(pnt_rect1, pnt_rect2, pnt_rect1, 1.0);
}

TEST_F(LWRIntersectionOverlapTest, pointRectOutsidePointRect) {
  auto pnt_rect1 = Rect{0, 0, 0, 0}, pnt_rect2 = Rect{0, 0, 1, 1};
  check_intersection(pnt_rect1, pnt_rect2, {}, 0);
  check_intersection(pnt_rect2, pnt_rect1, {}, 0);
}

TEST_F(LWRIntersectionOverlapTest, pointRectangleInsidePlainRectangle) {
  auto pnt_rect = Rect{0.5, 0.5, 0.5, 0.5}, rect = Rect{0, 1, 0, 1};
  check_intersection(pnt_rect, rect, pnt_rect, 1);
  check_intersection(rect, pnt_rect, pnt_rect, 0);
}

TEST_F(LWRIntersectionOverlapTest, pointRectangleOutsidePlainRectangle) {
  auto pnt_rect = Rect{2, 2, 2, 2}, rect = Rect{0, 1, 0, 1};
  check_intersection(pnt_rect, rect, {}, 0);
  check_intersection(rect, pnt_rect, {}, 0);
}

// line-rects

/* FIXME: broken test
TEST_F(LWRIntersectionOverlapTest, pointRectInsideLineRect) {
  auto pnt_rect = Rect{0, 0, 0.5, 0.5}, line_rect = Rect{0, 0, 0, 1};
  check_intersection(pnt_rect, line_rect, pnt_rect, 1.0);
  check_intersection(line_rect, pnt_rect, pnt_rect, 1.0);
}
*/

/* FIXME: broken test
TEST_F(LWRIntersectionOverlapTest, pointRectOutsideLineRect) {
  auto pnt_rect = Rect{0, 0, 0, 0}, line_rect = Rect{0, 0, 1, 2};
  check_intersection(pnt_rect, line_rect, {}, 1.0);
  check_intersection(line_rect, pnt_rect, {}, 1.0);
}
*/

// TODO: add line-line-in, ll-out, l-non_emty-in, l-non_empty-out

//------------------------------------------------------------------------------
// Misc

/* FIXME: failed, detects no intersection.
TEST_F(LWRIntersectionOverlapTest, overlappedRectanglesNotIncludedCorners) {
  check_intersection({-6, 6, -3, 3}, {-3, 3, -6, 6}, {-3, 3, -3, 3}, 1.0 / 3);
}
*/

/*============================================================================*/

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
