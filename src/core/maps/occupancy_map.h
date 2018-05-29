#ifndef SLAM_CTOR_CORE_OCCUPANCY_MAP_H
#define SLAM_CTOR_CORE_OCCUPANCY_MAP_H

#include "../states/state_data.h"

/*! \brief A base class for occupancy maps
 *  \tparam AreaId a map area identifier (e.g. coordinate: DiscretePoint2D)
 *  \tparam OccupancyInfo a type of occupancy information stored in a map
 */
template <typename AreaId, typename OccupancyInfo>
class OccupancyMap {
  // TODO: [API Clean Up] Rename, e.g. add_observation.
  /*! \brief Updates area with a given observation. */
  virtual void update(const AreaId &, const AreaOccupancyObservation &) = 0;
  /*! \brief Returns known information about the occupancy of a given area. */
  virtual OccupancyInfo occupancy(const AreaId &) const = 0;
};

#endif
