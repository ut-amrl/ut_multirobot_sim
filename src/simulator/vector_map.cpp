//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    vector_map.cc
\brief   Vector map representation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "stdio.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "vector_map.h"

using math_util::AngleMod;
using math_util::RadToDeg;
using geometry::Cross;
using geometry::Line;
using geometry::Line2f;
using std::string;
using std::vector;
using Eigen::Vector2f;
using std::swap;

#define PRINT_LINE(LINE) \
    (LINE).p0.x(), (LINE).p0.y(), \
    (LINE).p1.x(), (LINE).p1.y()

#define PRINT_VEC2(V) (V).x(), (V).y()

namespace vector_map {

struct AngleInterval {
  float start;
  float end;
  bool wraps;
};

AngleInterval NormalizeAngleInterval(float a0, float a1) {
  a0 = AngleMod(a0);
  a1 = AngleMod(a1);
  const bool wraps = (a1 < a0);
  return {a0, a1, wraps};
}

bool AngleIntervalContains(float a, const AngleInterval& interval) {
  a = AngleMod(a);
  if (!interval.wraps) {
    return (a >= interval.start && a <= interval.end);
  }
  return (a >= interval.start || a <= interval.end);
}

bool AngleIntervalsOverlap(const AngleInterval& a, const AngleInterval& b) {
  const auto contains = [](const AngleInterval& i, float angle) {
    return AngleIntervalContains(angle, i);
  };
  // Check endpoints; sufficient for convex intervals on circle.
  return contains(a, b.start) || contains(a, b.end) ||
         contains(b, a.start) || contains(b, a.end);
}

void TrimOcclusion(const Vector2f& loc,
                   const Line2f& test_line,
                   Line2f* trim_line_ptr,
                   vector<Line2f>* scene_lines_ptr) {
  // return TrimOcclusionInt(loc, test_line, trim_line_ptr, scene_lines_ptr);
  static const bool kDebug = false;
  Line2f& trim_line = *trim_line_ptr;
  if (kDebug) {
    printf("%s:\n", __FUNCTION__);
    printf("TestLine: %f,%f %f,%f\n", PRINT_LINE(test_line));
    printf("TrimLine: %f,%f %f,%f\n", PRINT_LINE(trim_line));
    printf("Loc: %f, %f\n", PRINT_VEC2(loc));
  }
  vector<Line2f>& scene_lines = *scene_lines_ptr;
  static const float sqeps = 1e-8;

  // test_line.p0
  Vector2f l1_p0 = test_line.p0;
  // test_line.p1
  Vector2f l1_p1 = test_line.p1;
  // test_line.p0 - loc
  Vector2f l1_r0 = l1_p0 - loc;
  // test_line.p1 - loc
  Vector2f l1_r1 = l1_p1 - loc;
  // trim_line.p0
  Vector2f l2_p0 = trim_line.p0;
  // trim_line.p1
  Vector2f l2_p1 = trim_line.p1;
  // trim_line.p0 - loc
  Vector2f l2_r0 = l2_p0 - loc;
  // trim_line.p1 - loc
  Vector2f l2_r1 = l2_p1 - loc;

  //Ensure that r0 vector to r1 vector is in the positive right-handed order
  if (Cross<float>(l1_r0, l1_r1) < 0.0) {
    swap(l1_r0,l1_r1);
    swap(l1_p0,l1_p1);
  }
  if (Cross<float>(l2_r0, l2_r1) < 0.0 ) {
    swap(l2_r0,l2_r1);
    swap(l2_p0,l2_p1);
  }

  if (kDebug) {
    printf("l1_r0:%f,%f l1_r1:%f,%f\nl2_r0:%f,%f l2_r1:%f,%f\n",
           PRINT_VEC2(l1_r0),
           PRINT_VEC2(l1_r1),
           PRINT_VEC2(l2_r0),
           PRINT_VEC2(l2_r1));
  }
  if ((Cross(l1_r0, l2_r0) >= 0.0 && Cross(l1_r1, l2_r0) >= 0.0) ||
      (Cross(l2_r0, l1_r0) >= 0.0 && Cross(l2_r1, l1_r0) >= 0.0)) {
    // No Line interaction.
    if (kDebug) {
      printf("No line interaction\n");
    }
    return;
  }

  // The semi-infinite ray from loc and passing through test_line.p0 intersects
  // the trim_line.
  const bool rayOcclusion0 = trim_line.RayIntersects(loc, l1_r0);
  // The semi-infinite ray from loc and passing through test_line.p1 intersects
  // trim_line.
  const bool rayOcclusion1 = trim_line.RayIntersects(loc, l1_r1);

  if (kDebug) {
    printf("rayOcclusion0:%d rayOcclusion1:%d\n",
           rayOcclusion0,
           rayOcclusion1);
  }

  // Vector2f p;
  const bool completeOcclusion =
      test_line.Intersects(loc, l2_p0) && test_line.Intersects(loc, l2_p1);

  // test_line.p0 is in front of, and occludes trim_line.
  const bool occlusion0 = rayOcclusion0 &&
      (trim_line.Touches(loc) || trim_line.Touches(l1_p0) ||
      !trim_line.Intersects(loc, l1_p0));

  // test_line.p1 is in front of, and occludes trim_line.
  const bool occlusion1 = rayOcclusion1 &&
      (trim_line.Touches(loc) || trim_line.Touches(l1_p1) ||
      !trim_line.Intersects(loc, l1_p1));

  if (kDebug) {
    printf("completeOcclusion:%d occlusion0:%d occlusion1:%d\n",
            completeOcclusion,
            occlusion0,
            occlusion1);
    printf("crosses0:%d crosses1:%d\n",
            trim_line.Crosses(loc, l1_p0),
            trim_line.Crosses(loc, l1_p1));
  }
  if (completeOcclusion) {
    if (kDebug) {
      printf("Case 2: trim line completely occluded by test line\n");
    }
    // Trim the line to zero length.
    trim_line.Set(Vector2f(0.1, 0.1), Vector2f(0.1, 0.1));
    return;
  } else if (occlusion0 && occlusion1) {
    if (kDebug) printf("Case 3: partial occlusion in middle\n");
    // trim_line is partially occluded in the middle by test_line. Break up
    // into 2 segments, make trim_line one segment, push back the other
    // segment to the sceneLines list.
    const Vector2f right_section_end = trim_line.RayIntersection(loc, l1_r0);
    const Vector2f left_section_end = trim_line.RayIntersection(loc, l1_r1);
    if (kDebug) {
      printf("Right: %f,%f  Left: %f,%f\n",
              right_section_end.x(),
              right_section_end.y(),
              left_section_end.x(),
              left_section_end.y());
    }
    trim_line.Set(l2_p0, right_section_end);
    // save the unoccluded part of trim_line at its left hand end, if any
    if ((left_section_end - l2_p1).squaredNorm() > sqeps) {
      scene_lines.push_back(Line2f(left_section_end, l2_p1));
    }
  } else if (occlusion0) {
    if (kDebug) printf("Case 5: left end occluded\n");
    //The left hand end of trim_line is occluded, trim it
    Vector2f right_section_end = trim_line.RayIntersection(loc, l1_r0);
    trim_line.Set(l2_p0, right_section_end);
  } else if (occlusion1) {
    if (kDebug) printf("Case 6: right end occluded\n");
    //The right hand end of trim_line is occluded, trim it
    Vector2f left_section_end = trim_line.RayIntersection(loc, l1_r1);
    trim_line.Set(left_section_end, l2_p1);
  } else {
    if (kDebug) printf("Case 7\n");
  }
}


void VectorMap::GetSceneLines(const Vector2f& loc,
                              float max_range,
                              vector<Line2f>* lines_list) const {
  const float x_min = loc.x() - max_range;
  const float y_min = loc.y() - max_range;
  const float x_max = loc.x() + max_range;
  const float y_max = loc.y() + max_range;
  lines_list->clear();
  if (grid_valid_ && !grid_cells_.empty()) {
    const auto clamp_index = [](int v, int lo, int hi) {
      return std::min(std::max(v, lo), hi);
    };
    const int x0 = clamp_index(
        static_cast<int>(std::floor((x_min - grid_min_.x()) / grid_cell_size_)),
        0, grid_cols_ - 1);
    const int x1 = clamp_index(
        static_cast<int>(std::floor((x_max - grid_min_.x()) / grid_cell_size_)),
        0, grid_cols_ - 1);
    const int y0 = clamp_index(
        static_cast<int>(std::floor((y_min - grid_min_.y()) / grid_cell_size_)),
        0, grid_rows_ - 1);
    const int y1 = clamp_index(
        static_cast<int>(std::floor((y_max - grid_min_.y()) / grid_cell_size_)),
        0, grid_rows_ - 1);
    std::unordered_set<int> line_indices;
    line_indices.reserve(static_cast<size_t>((x1 - x0 + 1) * (y1 - y0 + 1)) * 4);
    for (int y = y0; y <= y1; ++y) {
      const int row_offset = y * grid_cols_;
      for (int x = x0; x <= x1; ++x) {
        const auto& cell = grid_cells_[row_offset + x];
        for (int idx : cell) {
          line_indices.insert(idx);
        }
      }
    }
    lines_list->reserve(line_indices.size());
    for (int idx : line_indices) {
      const Line2f& l = lines[idx];
      if (l.p0.x() < x_min && l.p1.x() < x_min) continue;
      if (l.p0.y() < y_min && l.p1.y() < y_min) continue;
      if (l.p0.x() > x_max && l.p1.x() > x_max) continue;
      if (l.p0.y() > y_max && l.p1.y() > y_max) continue;
      lines_list->push_back(l);
    }
  } else {
    lines_list->reserve(lines.size());
    for (const Line2f& l : lines) {
      if (l.p0.x() < x_min && l.p1.x() < x_min) continue;
      if (l.p0.y() < y_min && l.p1.y() < y_min) continue;
      if (l.p0.x() > x_max && l.p1.x() > x_max) continue;
      if (l.p0.y() > y_max && l.p1.y() > y_max) continue;
      lines_list->push_back(l);
    }
  }
  // Add object lines
  for (const Line2f& l : object_lines){
    if (l.p0.x() < x_min && l.p1.x() < x_min) continue;
    if (l.p0.y() < y_min && l.p1.y() < y_min) continue;
    if (l.p0.x() > x_max && l.p1.x() > x_max) continue;
    if (l.p0.y() > y_max && l.p1.y() > y_max) continue;
    lines_list->push_back(l);
  }
}

void VectorMap::SceneRender(const Vector2f& loc,
                            float max_range,
                            float angle_min,
                            float angle_max,
                            vector<Line2f>* render) const {
  static const float eps = 0.0001;
  static const unsigned int MaxLines = 2000;
  vector<Line2f> scene;
  vector<Line2f> lines_list;
  GetSceneLines(loc, max_range, &lines_list);
  render->clear();
  if (lines_list.empty()) {
    return;
  }
  // Only cull by FOV if it is narrower than a full circle; otherwise keep all.
  const float fov_span = std::fabs(angle_max - angle_min);
  const bool full_circle = fov_span >= (static_cast<float>(M_2PI) - 1e-3f);
  if (!full_circle) {
    const AngleInterval fov = NormalizeAngleInterval(angle_min, angle_max);
    vector<Line2f> fov_lines;
    fov_lines.reserve(lines_list.size());
    for (const Line2f& l : lines_list) {
      const Vector2f r0 = l.p0 - loc;
      const Vector2f r1 = l.p1 - loc;
      if (r0.squaredNorm() < eps || r1.squaredNorm() < eps) {
        fov_lines.push_back(l);
        continue;
      }
      const float a0 = atan2(r0.y(), r0.x());
      const float a1 = atan2(r1.y(), r1.x());
      if (std::fabs(a0 - a1) < 0.0001f) {
        if (AngleIntervalContains(a0, fov)) {
          fov_lines.push_back(l);
        }
        continue;
      }
      const AngleInterval line_interval = NormalizeAngleInterval(a0, a1);
      if (AngleIntervalsOverlap(line_interval, fov)) {
        fov_lines.push_back(l);
      }
    }
    lines_list.swap(fov_lines);
    if (lines_list.empty()) {
      return;
    }
  }
  scene.reserve(lines_list.size());
  render->reserve(lines_list.size());

  for(size_t i = 0; i < lines_list.size() && i < MaxLines; ++i) {
    Line2f cur_line = lines_list[i];
    // Check if any part of cur_line is unoccluded by present list of lines,
    // as seen from loc.
    for(size_t j = 0; j < scene.size() && cur_line.SqLength() >= eps; ++j) {
      if (scene[j].SqLength() < eps) continue;
      TrimOcclusion(loc, scene[j], &cur_line, &lines_list);
    }

    if (cur_line.SqLength() > eps) { //At least part of cur_line is unoccluded
      for(size_t j = 0; j < scene.size(); ++j) {
        if (scene[j].SqLength() < eps) continue;
        TrimOcclusion(loc, cur_line, &scene[j], &lines_list);
      }
      // Add the visible part of cur_line.
      scene.push_back(cur_line);
    }
  }

  if (lines_list.size() >= MaxLines) {
    fprintf(stderr,
            "Runaway Analytic Scene Render at %.30f,%.30f, %.3f : %.3f\u00b0\n",
            loc.x(), loc.y(),
            RadToDeg(angle_min),
            RadToDeg(angle_max));
  }
  for(const Line2f& l : scene) {
    if (l.SqLength() > eps) render->push_back(l);
  }
}

int GetRayIntersection(const Vector2f& loc,
                       const size_t skip_line_idx,
                       const vector<Line2f>& lines_list,
                       Vector2f* ray_end) {
  Vector2f intersection(0, 0);
  int intersecting_line_idx = -1;
  for (size_t i = 0; i < lines_list.size(); ++i) {
    if (i == skip_line_idx) continue;
    const Line2f& l = lines_list[i];
    if (l.Intersection(loc, *ray_end, &intersection)) {
      *ray_end = intersection;
      intersecting_line_idx = i;
    }
  }
  return intersecting_line_idx;
}

void VectorMap::RayCast(const Vector2f& loc,
                        float max_range,
                        vector<Line2f>* render) const {
  static const float kEpsilon = 1e-4;

  // Small optimization: ignore all lines not within max_range.
  vector<Line2f> lines_list;
  GetSceneLines(loc, max_range, &lines_list);

  // NOTE(joydeep): In this function, "iidx" refers to the index of
  // the line segment from lines_list that intersects with the associated
  // ray.

  struct RayCastRay {
    Vector2f ray_end;
    int iidx;
    RayCastRay(const Vector2f& ray_end, int iidx) :
        ray_end(ray_end), iidx(iidx) {}
  };
  // Go through all lines, and check for intersection of rays.
  vector<RayCastRay> ray_cast_rays;
  for (size_t i = 0; i < lines_list.size(); ++i) {
    const Line2f& l = lines_list[i];
    const Vector2f dir = kEpsilon * (l.p1 - l.p0).normalized();

    // Add rays from loc to just inside of the line segment.
    Vector2f r0 = l.p0 + dir;
    Vector2f r1 = l.p1 - dir;
    int r0_iidx = GetRayIntersection(loc, i, lines_list, &r0);
    int r1_iidx = GetRayIntersection(loc, i, lines_list, &r1);
    if (r0_iidx < 0) r0_iidx = i;
    if (r1_iidx < 0) r1_iidx = i;
    ray_cast_rays.push_back(RayCastRay(r0, r0_iidx));
    ray_cast_rays.push_back(RayCastRay(r1, r1_iidx));

    // Add rays from loc to max_range just past the line segment.
    Vector2f end_p0 = loc + (l.p0 - dir - loc).normalized() * max_range;
    Vector2f end_p1 = loc + (l.p1 + dir - loc).normalized() * max_range;
    const int end_p0_iidx = GetRayIntersection(loc, i, lines_list, &end_p0);
    const int end_p1_iidx = GetRayIntersection(loc, i, lines_list, &end_p1);
    if (end_p0_iidx >= 0) {
      ray_cast_rays.push_back(RayCastRay(end_p0, end_p0_iidx));
    }
    if (end_p1_iidx >= 0) {
      ray_cast_rays.push_back(RayCastRay(end_p1, end_p1_iidx));
    }
  }

  if (true) {
    struct Comparator{
      bool operator() (RayCastRay r1, RayCastRay r2) {
        // if (r1.iidx <= r2.iidx) return true;
        const Vector2f l1 = r1.ray_end - loc;
        const Vector2f l2 = r2.ray_end - loc;
        return (Cross(l1, l2) > 0.0);
        return false;
      };
      Vector2f loc;
    };
    Comparator comparator;
    comparator.loc = loc;
    /*
    const auto comparator = [loc](const RayCastRay& r1, const RayCastRay& r2) {
      if (r1.iidx < r2.iidx) return true;
      const Vector2f l1 = r1.ray_end - loc;
      const Vector2f l2 = r2.ray_end - loc;
      return (Cross(l1, l2) > 0.0);
      return false;
    };
    */
    sort(ray_cast_rays.begin(), ray_cast_rays.end(), comparator);
  }

  if (ray_cast_rays.size() < 2) return;
  for (size_t i = 0; i < ray_cast_rays.size(); ++i) {
    if (ray_cast_rays[i].ray_end != ray_cast_rays[i - 1].ray_end) {
      render->push_back(Line2f(loc, ray_cast_rays[i].ray_end));
    }
  }
}


void ShrinkLine(float distance, Line2f* line) {
  const float len = line->Length();
  const Vector2f dir = line->Dir();
  if (len < 2.0 * distance) return;
  line->p0 += distance * dir;
  line->p1 -= distance * dir;
}

void VectorMap::Cleanup() {
  const float kShrinkDistance = 1e-4;
  // const float kMinLineLength = 2.0 * kShrinkDistance;
  const float kMinLineLength = 0.05;
  vector<Line2f> new_lines;
  for (size_t i = 0; i < lines.size(); ++i) {
    const Line2f l1 = lines[i];
    if (l1.Length() < kMinLineLength) continue;
    // Check if l1 intersects with any line in new lines.
    Vector2f p;
    bool intersection = false;
    for (const Line2f l2 : new_lines) {
      if (l2.Intersection(l1, &p)) {
        const Vector2f shrink = kShrinkDistance * l1.Dir();
        lines.push_back(Line2f(l1.p0, p - shrink));
        lines.push_back(Line2f(p + shrink, l1.p1));
        intersection = true;
        break;
      }
    }
    // No intersection, add it!
    if (!intersection) new_lines.push_back(l1);
  }

  for (Line2f& l : new_lines) {
    ShrinkLine(kShrinkDistance, &l);
  }
  lines = new_lines;
}

void VectorMap::BuildSpatialIndex() {
  grid_valid_ = false;
  grid_cells_.clear();
  if (lines.empty()) {
    return;
  }
  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  for (const Line2f& l : lines) {
    const float lmin_x = std::min(l.p0.x(), l.p1.x());
    const float lmax_x = std::max(l.p0.x(), l.p1.x());
    const float lmin_y = std::min(l.p0.y(), l.p1.y());
    const float lmax_y = std::max(l.p0.y(), l.p1.y());
    min_x = std::min(min_x, lmin_x);
    min_y = std::min(min_y, lmin_y);
    max_x = std::max(max_x, lmax_x);
    max_y = std::max(max_y, lmax_y);
  }
  const float width = std::max(1.0f, max_x - min_x);
  const float height = std::max(1.0f, max_y - min_y);
  const float area = width * height;
  const float avg_area = area / static_cast<float>(std::max<size_t>(1, lines.size()));
  float cell_size = std::sqrt(avg_area);
  if (!std::isfinite(cell_size) || cell_size <= 0.0f) {
    cell_size = 1.0f;
  }
  cell_size = math_util::Clamp(cell_size, 1.0f, 10.0f);
  grid_cell_size_ = cell_size;
  grid_min_ = Vector2f(min_x, min_y);
  grid_cols_ = static_cast<int>(std::ceil(width / cell_size)) + 1;
  grid_rows_ = static_cast<int>(std::ceil(height / cell_size)) + 1;
  grid_cells_.assign(static_cast<size_t>(grid_cols_ * grid_rows_), {});
  for (size_t i = 0; i < lines.size(); ++i) {
    const Line2f& l = lines[i];
    const float lmin_x = std::min(l.p0.x(), l.p1.x());
    const float lmax_x = std::max(l.p0.x(), l.p1.x());
    const float lmin_y = std::min(l.p0.y(), l.p1.y());
    const float lmax_y = std::max(l.p0.y(), l.p1.y());
    int x0 = static_cast<int>(std::floor((lmin_x - min_x) / cell_size));
    int x1 = static_cast<int>(std::floor((lmax_x - min_x) / cell_size));
    int y0 = static_cast<int>(std::floor((lmin_y - min_y) / cell_size));
    int y1 = static_cast<int>(std::floor((lmax_y - min_y) / cell_size));
    x0 = std::max(0, std::min(grid_cols_ - 1, x0));
    x1 = std::max(0, std::min(grid_cols_ - 1, x1));
    y0 = std::max(0, std::min(grid_rows_ - 1, y0));
    y1 = std::max(0, std::min(grid_rows_ - 1, y1));
    for (int y = y0; y <= y1; ++y) {
      const int row_offset = y * grid_cols_;
      for (int x = x0; x <= x1; ++x) {
        grid_cells_[row_offset + x].push_back(static_cast<int>(i));
      }
    }
  }
  grid_valid_ = true;
}

void VectorMap::Load(const string& file) {
  FILE* fid = fopen(file.c_str(), "r");
  if (fid == NULL) {
    fprintf(stderr, "ERROR: Unable to load map %s\n", file.c_str());
    exit(1);
  }
  lines.clear();
  float x1(0), y1(0), x2(0), y2(0);
  while (fscanf(fid, "%f,%f,%f,%f", &x1, &y1, &x2, &y2) == 4) {
    lines.push_back(Line2f(Vector2f(x1, y1), Vector2f(x2, y2)));
  }
  fclose(fid);
  Cleanup();
  BuildSpatialIndex();
  file_name = file;
}

bool VectorMap::Intersects(const Vector2f& v0, const Vector2f& v1) const {
  for (const Line2f& l : lines) {
    if (l.Intersects(v0, v1)) return true;
  }
  return false;
}

void VectorMap::GetPredictedScan(const Vector2f& loc,
                                 float range_min,
                                 float range_max,
                                 float angle_min,
                                 float angle_max,
                                 int num_rays,
                                 vector<float>* scan_ptr) const {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  vector<float>& scan = *scan_ptr;
  vector<Line2f> lines_list;
  GetSceneLines(loc, range_max, &lines_list);
  scan.resize(num_rays);
  std::fill(scan.begin(), scan.end(), range_max);
  if (lines_list.empty()) {
    return;
  }
  // Cull by FOV if needed.
  const float fov_span = std::fabs(angle_max - angle_min);
  const bool full_circle = fov_span >= (static_cast<float>(M_2PI) - 1e-3f);
  if (!full_circle) {
    const AngleInterval fov = NormalizeAngleInterval(angle_min, angle_max);
    vector<Line2f> fov_lines;
    fov_lines.reserve(lines_list.size());
    for (const Line2f& l : lines_list) {
      const Vector2f r0 = l.p0 - loc;
      const Vector2f r1 = l.p1 - loc;
      if (r0.squaredNorm() < 1e-6f || r1.squaredNorm() < 1e-6f) {
        fov_lines.push_back(l);
        continue;
      }
      const float a0 = atan2(r0.y(), r0.x());
      const float a1 = atan2(r1.y(), r1.x());
      if (std::fabs(a0 - a1) < 0.0001f) {
        if (AngleIntervalContains(a0, fov)) {
          fov_lines.push_back(l);
        }
        continue;
      }
      const AngleInterval line_interval = NormalizeAngleInterval(a0, a1);
      if (AngleIntervalsOverlap(line_interval, fov)) {
        fov_lines.push_back(l);
      }
    }
    lines_list.swap(fov_lines);
    if (lines_list.empty()) {
      return;
    }
  }

  const float da = (angle_max - angle_min) / static_cast<float>(num_rays);
  float a = angle_min;
  float cos_a = std::cos(a);
  float sin_a = std::sin(a);
  const float cos_da = std::cos(da);
  const float sin_da = std::sin(da);
  const bool use_grid = grid_valid_ && !grid_cells_.empty();
  auto intersect_segment = [&](const Line2f& l,
                               const Vector2f& ray_end,
                               float* best_ptr) {
    Vector2f intersection;
    if (l.Intersection(loc, ray_end, &intersection)) {
      const float dist = (intersection - loc).norm();
      if (dist >= range_min && dist < *best_ptr) {
        *best_ptr = dist;
      }
    }
  };

  for (int i = 0; i < num_rays; ++i) {
    const Vector2f ray_dir(cos_a, sin_a);
    float best = range_max;
    if (use_grid) {
      const float inv_dx = (std::abs(ray_dir.x()) < 1e-6f) ? 0.0f : 1.0f / ray_dir.x();
      const float inv_dy = (std::abs(ray_dir.y()) < 1e-6f) ? 0.0f : 1.0f / ray_dir.y();
      int cx = static_cast<int>(std::floor((loc.x() - grid_min_.x()) / grid_cell_size_));
      int cy = static_cast<int>(std::floor((loc.y() - grid_min_.y()) / grid_cell_size_));
      int step_x = (ray_dir.x() >= 0.0f) ? 1 : -1;
      int step_y = (ray_dir.y() >= 0.0f) ? 1 : -1;
      float next_boundary_x = grid_min_.x() + (static_cast<float>(cx + (step_x > 0 ? 1 : 0)) * grid_cell_size_);
      float next_boundary_y = grid_min_.y() + (static_cast<float>(cy + (step_y > 0 ? 1 : 0)) * grid_cell_size_);
      float t_max_x = (inv_dx == 0.0f) ? std::numeric_limits<float>::infinity()
                                       : (next_boundary_x - loc.x()) * inv_dx;
      float t_max_y = (inv_dy == 0.0f) ? std::numeric_limits<float>::infinity()
                                       : (next_boundary_y - loc.y()) * inv_dy;
      const float t_delta_x = (inv_dx == 0.0f) ? std::numeric_limits<float>::infinity()
                                               : grid_cell_size_ * std::abs(inv_dx);
      const float t_delta_y = (inv_dy == 0.0f) ? std::numeric_limits<float>::infinity()
                                               : grid_cell_size_ * std::abs(inv_dy);
      float traveled = 0.0f;
      // DDA through grid until we exceed best or range_max.
      while (traveled <= best && traveled <= range_max) {
        if (cx >= 0 && cx < grid_cols_ && cy >= 0 && cy < grid_rows_) {
          const auto& cell = grid_cells_[cy * grid_cols_ + cx];
          if (!cell.empty()) {
            const Vector2f ray_end = loc + ray_dir * best;
            for (int idx : cell) {
              intersect_segment(lines[idx], ray_end, &best);
            }
          }
        }
        if (t_max_x < t_max_y) {
          traveled = t_max_x;
          t_max_x += t_delta_x;
          cx += step_x;
        } else {
          traveled = t_max_y;
          t_max_y += t_delta_y;
          cy += step_y;
        }
        if (cx < 0 || cx >= grid_cols_ || cy < 0 || cy >= grid_rows_) {
          break;
        }
      }
      // Also consider dynamic object lines (not in grid).
      if (!object_lines.empty()) {
        const Vector2f ray_end = loc + ray_dir * best;
        for (const Line2f& l : object_lines) {
          intersect_segment(l, ray_end, &best);
        }
      }
    } else {
      const Vector2f ray_end = loc + ray_dir * range_max;
      for (const Line2f& l : lines_list) {
        intersect_segment(l, ray_end, &best);
      }
    }
    scan[i] = best;
    // Advance trig using recurrence.
    const float next_cos = (cos_a * cos_da) - (sin_a * sin_da);
    const float next_sin = (sin_a * cos_da) + (cos_a * sin_da);
    cos_a = next_cos;
    sin_a = next_sin;
  }
}

}  // namespace vector_map
