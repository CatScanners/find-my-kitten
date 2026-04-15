#ifndef INPUTPOINT_H
#define INPUTPOINT_H

#include "vector2D.hpp"
#include <cmath>

struct inputPoint {
  int trackID;
  vector2D point;
};

// The center of image is (0,0)
// With Distance of 1 from focal point to screene point (0,0)
// the distance from (1,0) forms a right triangle with side lengths of 1,1,sqrt(2)
// On camera IMX219-170 the 170 is the cameras diagonal fov,
// which is the degree angle between two diagonal corners.


inline inputPoint convertToUsableForm(const int frame_dimension_x,
                                      const int frame_dimension_y,
                                      const float camera_fov_diagonal,
                                      const int id, const float x,
                                      const float y, const bool degrees) {
  float fov_diagonal = degrees ? std::tan(camera_fov_diagonal / 2 / 180)
                               : std::tan(camera_fov_diagonal / 2);
  float scale = fov_diagonal / std::sqrt(frame_dimension_x * frame_dimension_x +
                                         frame_dimension_y * frame_dimension_y);
  return {id, (x - frame_dimension_x / 2) * scale,
          (y - frame_dimension_y / 2) * scale};
}

#endif
