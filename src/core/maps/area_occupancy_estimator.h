#ifndef AREA_OCCUPANCY_ESTIMATOR_H
#define AREA_OCCUPANCY_ESTIMATOR_H

#include <cassert>
#include <vector>
#include <cmath>
#include <tuple>
#include <algorithm>

#include "cell_occupancy_estimator.h"

class AreaOccupancyEstimator : public CellOccupancyEstimator {
private: // types
  enum class SegmentPositionType : char {
    Unrelated = 0, LiesInside, StopsInside, StartsInside, Pierces, Touches
  };
public: //methods

  AreaOccupancyEstimator(const Occupancy& base_occupied,
                         const Occupancy& base_empty,
                         double low_qual = 0.01, double unknown_qual = 0.5)
    : CellOccupancyEstimator{base_occupied, base_empty}
    , _low_qual{low_qual}, _unknown_qual{unknown_qual} {
    assert(_low_qual < 1);
  }

  Occupancy estimate_occupancy(const Segment2D &beam, const Rectangle &cell,
                               bool is_occ) override {
    Segment2D effective_beam = ensure_segment_not_on_edge(beam, cell);
    switch (classify_segment(effective_beam, cell)) {
    case SegmentPositionType::Unrelated:
    case SegmentPositionType::Touches:
      return Occupancy::invalid();
    case SegmentPositionType::Pierces:
    case SegmentPositionType::StartsInside:
      if (is_occ) { return Occupancy::invalid(); }
      break;
    case SegmentPositionType::LiesInside:
      return is_occ ? Occupancy::invalid() :
                      Occupancy{base_empty().prob_occ, _unknown_qual};
    case SegmentPositionType::StopsInside:
      break;
    }

    Intersections intrs = find_intersections(effective_beam, cell, is_occ);
    if (intrs.size() == 1) {
      if (!is_occ) { // StopsInside/FrontEdge/Vertex
        return Occupancy{base_empty().prob_occ, _unknown_qual};
      } else { // occupied, stops at some vertex
        auto raw_intersections = cell.find_intersections(effective_beam);
        switch (raw_intersections.size()) {
        case 0: assert(false && "BUG: no inters must be detected earlier");
        case 1: // stops at front vertex -> entire cell is occupied
          return estimate_occupancy(cell.area(), cell.area(), is_occ);
        case 2: // stops at rear vertex, treat the cell as empty
          intrs = raw_intersections;
          is_occ = false;
          break;
        }
      }
    }
    double chunk_area = compute_chunk_area(effective_beam, cell, is_occ, intrs);
    return estimate_occupancy(chunk_area, cell.area(), is_occ);
  }

private: // methods

  Segment2D ensure_segment_not_on_edge(const Segment2D& s,
                                       const Rectangle &cell) const {
    // NB: Invariant: beg/end movement should not effect intersections
    static const double Shift_Amount = _low_qual * cell.side();
    if (!cell.has_on_edge_line(s)) { return s; }

    Point2D shift;
    if (s.is_horiz()) {
      shift = {0, (are_equal(s.beg().y, cell.top()) ? -1 : 1) * Shift_Amount};
    } else if (s.is_vert()) {
      shift = {(are_equal(s.beg().x, cell.right()) ? -1 : 1) * Shift_Amount, 0};
    } else {
      assert(false && "BUG: non-axis aligned segment is on rectange edge");
    }

    Segment2D shifted_s{s.beg() + shift, s.end() + shift};
    assert(!cell.has_on_edge_line(shifted_s) && "BUG");
    return shifted_s;
  }

  SegmentPositionType classify_segment(const Segment2D &s,
                                       const Rectangle &cell) const {
    bool beg_is_inside, end_is_inside;
    std::tie(beg_is_inside, end_is_inside) = modified_is_inside(s, cell);

    if (beg_is_inside ^ end_is_inside) {
      return beg_is_inside ? SegmentPositionType::StartsInside :
                             SegmentPositionType::StopsInside;
    }

    // both are eithes inside or outside
    if (beg_is_inside) {
      return SegmentPositionType::LiesInside;
    }

    Intersections intersections = cell.find_intersections(s);
    switch (intersections.size()) {
    case 0: return SegmentPositionType::Unrelated;
    case 1: return SegmentPositionType::Touches;
    case 2: return SegmentPositionType::Pierces;
    }
    assert(false && "BUG: unable to classify segment");
    return SegmentPositionType::Unrelated;
  }


  std::tuple<bool, bool> modified_is_inside(const Segment2D &s,
                                            const Rectangle &cell) const {
    // Main idea: check if the s is inside cell
    // "extending/shrinking" s if its ends are on cell's edges
    auto beg_edge = cell.find_containing_edge(s.beg());
    auto end_edge = cell.find_containing_edge(s.end());
    if (beg_edge && end_edge) {
      return std::make_tuple(false, false); // move out of border
    }

    // NB: contains includes edge into analysis
    bool beg_contains = cell.contains(s.beg());
    bool end_contains = cell.contains(s.end());
    if (!beg_edge && !end_edge) { // contains is ok
      return std::make_tuple(beg_contains, end_contains);
    }

    // beg_edge, end_edge are opposite
    // 1. Beg is on edge, End is out -> both are out,
    //    since s doesn't adds any info about cell
    // 2. End is on edge -> extend it to the side opposide to s begin
    return beg_edge ? std::make_tuple(false, end_contains) :
                      std::make_tuple(beg_contains, !beg_contains);
  }

  Intersections find_intersections(const Segment2D &beam,
                                   const Rectangle &bnds, bool is_occ) {
    // if the cell is occupied, rotate ray around beam end by 90 degrees
    auto ray = is_occ ?
      Ray{beam.end().x, beam.beg().y - beam.end().y,
          beam.end().y, beam.end().x - beam.beg().x} :
      Ray{beam.beg().x, beam.end().x - beam.beg().x,
          beam.beg().y, beam.end().y - beam.beg().y};
    auto inters = bnds.find_intersections(ray);
    if (is_occ) {
      // occupancy is intersected via fake ray, so not need to filter points
      return inters;
    }

    Intersections filtered_inters;
    std::copy_if(inters.begin(), inters.end(),
                 std::back_inserter(filtered_inters),
                 [&beam](const Intersection &inters_p){
                   return beam.contains_intersection(inters_p);
                 });
    return filtered_inters;
  }

  double compute_chunk_area(const Segment2D &beam, const Rectangle &bnds,
                            bool is_occ,
                            const std::vector<Intersection> &inters) {
    if (inters.size() == 0) {
      // TODO: deal with line approx introduced by Bresenham
      return bnds.area() / 2;
    }

    assert(inters.size() == 2);
    double corner_x = 0, corner_y = 0, area = 0;
    double chunk_is_triangle = inters[0].is_horiz() ^ inters[1].is_horiz();
    if (chunk_is_triangle) {
      // determine "base" corner (corner of cell that
      // is also a corner of the triangle
      for (auto &inter : inters) {
        switch (inter.location) {
        case Intersection::Location::Bot: corner_y = bnds.bot(); break;
        case Intersection::Location::Top: corner_y = bnds.top(); break;
        case Intersection::Location::Left: corner_x = bnds.left(); break;
        case Intersection::Location::Right: corner_x = bnds.right(); break;
        default: assert(false && "Unexpected location type");
        }
      }
      // calculate triange area
      area = 0.5;
      for (auto &inter : inters) {
        if (inter.is_horiz()) {
          area *= std::abs(inter.x - corner_x);
        } else {
          area *= std::abs(inter.y - corner_y);
        }
      }
    } else {
      // chunk is a trapezoid
      // corner choise doesn't matter, so pick bottom-left one.
      corner_x = bnds.left(), corner_y = bnds.bot();
      double base_sum = 0;
      for (auto &inter : inters) {
        if (inter.is_horiz()) {
          base_sum += std::abs(inter.x - corner_x);
        } else {
          base_sum += std::abs(inter.y - corner_y);
        }
      }
      // NOTE: cell is supposed to be a square
      area = 0.5 * (bnds.top() - bnds.bot()) * base_sum;
    }
    assert(0 <= area && area <= bnds.area() && "BUG: AOE area estimation");
    if (is_occ &&
        are_on_the_same_side(inters[0], inters[1],
                             beam.beg(), {corner_x, corner_y})) {
      area = bnds.area() - area;
    }
    return area;
  }

  bool are_on_the_same_side(const Point2D &line_p1, const Point2D &line_p2,
                            const Point2D &p1, const Point2D &p2) {
    double dx = line_p2.x - line_p1.x, dy = line_p2.y - line_p1.y;
    return 0 < (dy*p1.y - dx*p1.x + dy*p1.x - dx*p1.y) *
               (dy*p2.y - dx*p2.x + dy*p2.x - dx*p2.y);
  }

  Occupancy estimate_occupancy(double chunk_area, double total_area,
                               bool is_occ) {
    double area_rate = chunk_area / total_area;
    if (is_occ) {
      // Far ToDo: think about experiment quality metric for an occupied case.
      return Occupancy{std::max(area_rate, base_empty().prob_occ),
                       base_occupied().estimation_quality};
    } else {
      // TODO: fix scale
      if (0.5 < area_rate) {
        area_rate = 1 - area_rate;
      }
      return Occupancy{base_empty().prob_occ,
                       base_empty().estimation_quality * area_rate};
    }
  }
private:
  double _low_qual;
  double _unknown_qual;
};

#endif
