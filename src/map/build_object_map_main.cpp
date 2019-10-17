#include <algorithm>
#include <dirent.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <glog/logging.h>
#include <list>
#include <map>
#include <ros/package.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <termios.h>
#include <utility>
#include <vector>

#include "CImg/CImg.h"
#include "cobot_msgs/CobotLocalizationMsg.h"
#include "cobot_msgs/LidarDisplayMsg.h"
#include "cobot_msgs/LocalizationGuiCaptureSrv.h"
#include "configreader.h"
#include "connected_components/connected_components.h"
#include "eigen_helper.h"
#include "gui_publisher_helper.h"
#include "helpers.h"
#include "kdtree.h"
#include "maximal_cliques/graph.hpp"
#include "maximal_cliques/maximalcliquegenerator.h"
#include "non_markov_localization.h"
#include "object_map.h"
#include "openmp_utils.h"
#include "perception_tools/perception_2d.h"
#include "popt_pp.h"
#include "timer.h"


using cobot_gui::DrawLine;
using cobot_gui::DrawPoint;
using cobot_gui::DrawText;
using cobot_gui::ClearDrawingMessage;
using connected_components::compute_scc;
using connected_components::get_largest_components;
using Eigen::Affine2d;
using Eigen::Affine2f;
using Eigen::Matrix2d;
using Eigen::Matrix2f;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Perp2;
using Eigen::Rotation2Dd;
using Eigen::Rotation2Df;
using Eigen::ScalarCross;
using Eigen::Translation2d;
using Eigen::Translation2f;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector3d;
using EnmlMaps::GridObject;
using EnmlMaps::LoadPersistentObjects;
using EnmlMaps::SavePersistentObjects;
using EnmlMaps::PersistentObject;
using perception_2d::NormalCloudf;
using perception_2d::Pose2Df;
using perception_2d::PointCloudf;
using ros::ServiceServer;
using ros::Subscriber;
using std::fwrite;
using std::fprintf;
using std::list;
using std::make_pair;
using std::map;
using std::pair;
using std::size_t;
using std::sort;
using std::string;
using std::vector;

typedef KDNodeValue<float, 2> KDNodeValue2f;

int debug_level_ = -1;

bool model_accuracy_test_ = false;

bool display_cliques_ = false;

unsigned int max_num_objects_ = 1000;

// Display message for drawing debug vizualizations on the localization_gui.
cobot_msgs::LidarDisplayMsg display_message_;

// Service client to ask localization_gui to save images.
ros::ServiceClient gui_capture_client;

// ROS publisher to publish display_message_ to the ROS topic
// Cobot/VectorLocalization/Gui
ros::Publisher display_publisher_;

// ROS publisher to publish the latest robot localization estimate to
// Cobot/Localization
ros::Publisher localization_publisher_;

// The path of the cobot_linux ROS stack.
static const string kCobotStackPath(ros::package::getPath("cobot_linux"));

// Visualization colour of points belonging to STFs.
// const uint32_t kStfPointColor = 0x1F994CD9;
const uint32_t kStfPointColor = 0xFFFF0000;

// The timestamp of the current log, in epoch time.
double log_timestamp_ = 0.0;

// List of persistent objects.
vector<PersistentObject> persistent_objects_;

static const bool kUseKlDivergence = false;
static const bool kUseOverlap = false;
static const bool kUseNewMetric = true;

EnmlMaps::ObjectMapOptions map_options_;

#ifdef FREIBURG_DATASET
static const string kMapName = "freiburg_parkinglot";
static const string kObjectMapDir =
    kCobotStackPath + "/../maps/" + kMapName + "/objects";
static const float kLaserAngularResolution = RAD(0.5);
static const float kImageResolution = 0.04;
static const float kImageBorder = 5.0 * kImageResolution;
static const float kMinSdfValue = -10.0 * kImageResolution;
static const float kMaxSdfValue = 5.0 * kImageResolution;
static const float kMinSdfWeight = 0.2;
static const float kMaxSdfWeight = 0.7;
static const float kMinSdfInferenceWeight = 0.4;
static const bool kGenerateSdf = false;
static const float kMistrustWeight = 1.0;
static const float kAngularResolution = RAD(10.0);
static const float kMaxDeltaLoc = 0.5;
static const float kLocResolution = 0.1;
#endif

string kMapName = "GHC6";
string kObjectMapDir =
    kCobotStackPath + "/../maps/" + kMapName + "/objects";

void PublishDisplay() {
  for (int i = 0; i < 20; ++i) {
    display_publisher_.publish(display_message_);
    ros::spinOnce();
    Sleep(0.01);
  }
  ClearDrawingMessage(&display_message_);
}

bool LoadConfiguration() {
  WatchFiles watch_files;
  ConfigReader config((kCobotStackPath + "/").c_str());

  config.init(watch_files);
  config.addFile("../robot.cfg");
  config.addFile("config/non_markov_localization.cfg");
  if (!config.readFiles()) return false;
  bool error = false;
  ConfigReader::SubTree c(config,"ProbabilisticObjectMaps");
  error = error || !c.getReal("object_distance_threshold",
                              map_options_.object_distance_threshold);
  error = error || !c.getUInt("min_object_points",
                              map_options_.min_object_points);
  error = error || !c.getReal("laser_angular_resolution",
                              map_options_.laser_angular_resolution);
  error = error || !c.getReal("image_resolution",
                              map_options_.image_resolution);
  error = error || !c.getReal("image_border",
                              map_options_.image_border);
  error = error || !c.getReal("min_sdf_value",
                              map_options_.min_sdf_value);
  error = error || !c.getReal("max_sdf_value",
                              map_options_.max_sdf_value);
  error = error || !c.getReal("min_sdf_weight",
                              map_options_.min_sdf_weight);
  error = error || !c.getReal("max_sdf_weight",
                              map_options_.max_sdf_weight);
  error = error || !c.getReal("min_sdf_inference_weight",
                              map_options_.min_sdf_inference_weight);
  error = error || !c.getBool("generate_sdf",
                              map_options_.generate_sdf);
  error = error || !c.getReal("sdf_mistrust_weight",
                              map_options_.sdf_mistrust_weight);
  error = error || !c.getReal("matching_angular_resolution",
                              map_options_.matching_angular_resolution);
  error = error || !c.getReal("matching_delta_loc",
                              map_options_.matching_delta_loc);
  error = error || !c.getReal("matching_loc_resolution",
                              map_options_.matching_loc_resolution);
  error = error || !c.getReal("max_clique_kld_value",
                              map_options_.max_clique_kld_value);
  error = error || !c.getReal("min_clique_overlap_value",
                              map_options_.min_clique_overlap_value);
  error = error || !c.getReal("laser_std_dev",
                              map_options_.laser_std_dev);
  error = error || !c.getUInt("max_num_objects", max_num_objects_);
  error = error || !c.getReal("epsilon_occupancy",
                              map_options_.epsilon_occupancy);
  error = error || !c.getReal("occupancy_threshold",
                              map_options_.occupancy_threshold);
  error = error || !c.getReal("good_match_threshold",
                              map_options_.good_match_threshold);
  return (!error);
}

// Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
}

void LoadStfs(
    const string& stfs_file, PointCloudf* point_cloud_ptr,
    NormalCloudf* normal_cloud_ptr, vector<Pose2Df>* poses_ptr,
    double* timestamp) {
  PointCloudf& point_cloud = *point_cloud_ptr;
  NormalCloudf& normal_cloud = *normal_cloud_ptr;
  vector<Pose2Df>& poses = *poses_ptr;
  ScopedFile fid(stfs_file, "r");
  CHECK_NOTNULL(fid());
  static const int kNumFields = 7;
  Vector2f point(0, 0);
  Vector2f normal(0, 0);
  Pose2Df pose(0, 0, 0);
  char map_name_str[64];
  if (fscanf(fid(), "%s\n", map_name_str) != 1) {
    fprintf(stderr, "ERROR: Unable to read map name.\n");
    exit(1);
  }
  if (fscanf(fid(), "%lf\n", timestamp) != 1) {
    fprintf(stderr, "ERROR: Unable to read map timeStamp.\n");
    exit(1);
  }
  kMapName = string(map_name_str);
  kObjectMapDir = kCobotStackPath + "/../maps/" + kMapName + "/objects";
  printf("Map name: %s\n", kMapName.c_str());
  while (fscanf(fid(), "%f,%f,%f, %f,%f, %f,%f\n",
    &(pose.translation.x()), &(pose.translation.y()), &(pose.angle),
    &(point.x()), &(point.y()), &(normal.x()), &(normal.y())) == kNumFields) {
    point_cloud.push_back(point);
    normal_cloud.push_back(normal);
    poses.push_back(pose);
  }
}

void PublishStfs(const vector<vector<KDNodeValue2f> >& C) {
  // Reset localization_gui to correct map.
  cobot_msgs::CobotLocalizationMsg localization_msg_;
  localization_msg_.timeStamp = GetTimeSec();
  localization_msg_.map = kMapName;
  localization_msg_.x = 0;
  localization_msg_.y = 0;
  localization_msg_.angle = 0;
  localization_publisher_.publish(localization_msg_);
  for (size_t i = 0; i < C.size(); ++i) {
    Vector2f object_origin(FLT_MAX, FLT_MAX);
    for (size_t j = 0; j < C[i].size(); ++j) {
      DrawPoint(C[i][j].point, kStfPointColor, &display_message_);
      object_origin.x() = min(object_origin.x(), C[i][j].point.x());
      object_origin.y() = min(object_origin.y(), C[i][j].point.y());
    }
    DrawText(object_origin,
             StringPrintf("%d", static_cast<int>(i)),
             0xFF000000,
             0.5,
             false,
             &display_message_);
  }
  PublishDisplay();
}

inline Vector2f MarchingSquaresEdge(const float v0, const float v1,
                         const Vector2f& p0, const Vector2f& p1) {
  return ((v0 * p0 - v1 * p1) / (v0 - v1));
}

void MarchingSquares(const cimg_library::CImg<float>& sdf_image,
                     const cimg_library::CImg<float>& weights_image,
                     const Vector2f& image_origin,
                     const float image_resolution,
                     const float kMinWeight,
                     vector<pair<Vector2f, Vector2f> >* contour_ptr) {
  vector<pair<Vector2f, Vector2f> >& contour = *contour_ptr;
  // Convention of vertex locations 0-3, where origin is bottom left:
  //
  // 3 * * * 2
  // *       *
  // *       *
  // *       *
  // 0 * * * 1
  //
  // Convention of bitmask of zero-crossing locations using bits 0-3:
  //
  // * * 4 * *
  // *       *
  // 8       2
  // *       *
  // * * 1 * *
  CHECK_EQ(sdf_image.width(), weights_image.width());
  CHECK_EQ(sdf_image.height(), weights_image.height());
  for (int x = 0; x < sdf_image.width() - 1; ++x) {
    for (int y = 0; y < sdf_image.height() - 1; ++y) {
      if (weights_image(x, y) < kMinWeight ||
          weights_image(x + 1, y) < kMinWeight ||
          weights_image(x + 1, y + 1) < kMinWeight ||
          weights_image(x, y + 1) < kMinWeight) {
        continue;
      }
      const float& v0 = sdf_image(x, y);
      const float& v1 = sdf_image(x + 1, y);
      const float& v2 = sdf_image(x + 1, y + 1);
      const float& v3 = sdf_image(x, y + 1);
      const Vector2f p0 =
          image_origin + image_resolution * Vector2f(x, y);
      const Vector2f p1 =
          image_origin + image_resolution * Vector2f(x + 1, y);
      const Vector2f p2 =
          image_origin + image_resolution * Vector2f(x + 1, y+ 1);
      const Vector2f p3 =
          image_origin + image_resolution * Vector2f(x, y + 1);

      const Vector2f e1 = MarchingSquaresEdge(v0, v1, p0, p1);
      const Vector2f e2 = MarchingSquaresEdge(v1, v2, p1, p2);
      const Vector2f e4 = MarchingSquaresEdge(v2, v3, p2, p3);
      const Vector2f e8 = MarchingSquaresEdge(v3, v0, p3, p0);
      const uint8_t case_number =
          ((v0 < 0.0) ? 1 : 0) |
          ((v1 < 0.0) ? 2 : 0) |
          ((v2 < 0.0) ? 4 : 0) |
          ((v3 < 0.0) ? 8 : 0);
      switch (case_number) {
        case 0: case 15: {
          // No edges.
        } break;

        case 1: case 14: {
          // Edge from 1 - 8.
          contour.push_back(make_pair(e1, e8));
        } break;

        case 2: case 13: {
          // Edge from 1 - 2.
          contour.push_back(make_pair(e1, e2));
        } break;

        case 3: case 12: {
          // Edge from 2 - 8.
          contour.push_back(make_pair(e2, e8));
        } break;

        case 4: case 11: {
          // Edge from 2 - 4.
          contour.push_back(make_pair(e2, e4));
        } break;

        case 5: {
          // Edge from 1 - 8.
          contour.push_back(make_pair(e1, e8));
          // Edge from 2 - 4
          contour.push_back(make_pair(e2, e4));
        } break;

        case 10: {
          // Edge from 1 - 2.
          contour.push_back(make_pair(e1, e2));
          // Edge from 4 - 8.
          contour.push_back(make_pair(e4, e8));
        } break;

        case 6: case 9: {
          // Edge from 1 - 4.
          contour.push_back(make_pair(e1, e4));
        } break;

        case 7: case 8: {
          // Edge from 4 - 8.
          contour.push_back(make_pair(e4, e8));
        } break;

        default: {
          // Cannot happen unless there's a bug.
          static const bool kMarchingSquaresBug = false;
          CHECK(kMarchingSquaresBug);
        }
      }
    }
  }
}

void BuildSdfObjects(const vector<vector<KDNodeValue2f> >& objects,
                     const vector<Pose2Df>& poses,
                     vector<GridObject>* grd_objects_ptr) {

  //if (!map_options_.generate_sdf) return; ##########################
  vector<GridObject>& grid_objects = *grd_objects_ptr;
  const float pixel_half_width = sqrt(2.0) * map_options_.image_resolution;
  grid_objects.resize(objects.size());

  printf("Building SDF objects...\n");
  // For every cluster:
  for (size_t i = 0; i < objects.size(); ++i) {
    const vector<KDNodeValue2f>& object = objects[i];
    GridObject& grid_object = grid_objects[i];
    for (size_t j = 0; j < object.size(); ++j) {
      grid_object.points.push_back(object[j].point);
    }
    float min_x(FLT_MAX), min_y(FLT_MAX);
    float max_x(-FLT_MAX), max_y(-FLT_MAX);
    for (size_t j = 0; j < object.size(); ++j) {
      min_x = min(min_x, object[j].point.x());
      max_x = max(max_x, object[j].point.x());
      min_y = min(min_y, object[j].point.y());
      max_y = max(max_y, object[j].point.y());
    }
    const float width = max_x - min_x + 2.0 * map_options_.image_border;
    const float height = max_y - min_y + 2.0 * map_options_.image_border;
    printf("\rObject %2d: width %5.2f height %5.2f",
           static_cast<int>(i), width, height);
    fflush(stdout);
    const unsigned int image_width =
        ceil(width / map_options_.image_resolution);
    const unsigned int image_height =
        ceil(height / map_options_.image_resolution);
    // Initialize an empty SDF image and weights image.
    cimg_library::CImg<float> sdf_value_image(image_width, image_height);
    cimg_library::CImg<float> sdf_weights_image(image_width, image_height);
    cimg_library::CImg<uint8_t> sdf_display_image(
        image_width, image_height, 1, 3, 0);
    const Vector2f image_origin(
        min_x - map_options_.image_border, min_y - map_options_.image_border);
    grid_object.location = image_origin;
    grid_object.angle = 0.0;
    grid_object.image_width = image_width;
    grid_object.image_height = image_height;
    grid_object.image_resolution = map_options_.image_resolution;
    grid_object.pose_transform = Translation2f(image_origin);
    // For every pixel in the SDF image:
    OMP_PARALLEL_FOR
    for (size_t x = 0; x < image_width; ++x) {
      for (size_t y = 0; y < image_height; ++y) {
        //if (map_options_.generate_sdf) { ######################
          sdf_weights_image(x, y) = 0.0;
          // Unseen pixels are assumed to be occuped.
          sdf_value_image(x, y) = map_options_.min_sdf_value;
        //} ####################################################
        const Vector2f pixel_loc =
            image_origin + map_options_.image_resolution * Vector2f(x, y);
        // For every observed point:
        for (size_t j = 0; j < object.size(); ++j) {
          const size_t point_index = object[j].index;
          const Vector2f& point = object[j].point;
          const Vector2f& source = poses[point_index].translation;
          const Vector2f line_dir = (point - source).normalized();
          const Vector2f line_perp = Perp2(line_dir);
          const bool within_angle_tolerance =
              (fabs(line_perp.dot(point - pixel_loc)) /
              (point - source).norm() < 0.5 *
              map_options_.laser_angular_resolution);
          const bool along_viewing_ray =
              (fabs(line_perp.dot(pixel_loc - point)) < pixel_half_width);
          // printf("%d %d\n", along_viewing_ray, within_angle_tolerance);
          if (!along_viewing_ray && !within_angle_tolerance) continue;
          // Locations farther than the observed point along the viewing ray
          // from the source have a negative SDF value since they are
          // "occupied".
          const float sdf_value = line_dir.dot(point - pixel_loc);
          // If pixel is along visible ray and between the point and its
          // source:
          // Update SDF
          const float tsdf_value = min(sdf_value, map_options_.max_sdf_value);
          if (sdf_value >= map_options_.min_sdf_value) {
            float sdf_weight = ramp<float>(
                sdf_value, map_options_.min_sdf_value,
                map_options_.min_sdf_weight, map_options_.max_sdf_value,
                map_options_.max_sdf_weight);
            if (map_options_.sdf_mistrust_weight < 1.0 &&
                sdf_weights_image(x, y) > 0.0f){
              if (sdf_value_image(x, y) < 0.0 && tsdf_value > 0.0) {
                // Older observations say this pixel is occupied, but the
                // current observation says it's unoccupied.
                sdf_weight = map_options_.sdf_mistrust_weight * sdf_weight;
              } else if (sdf_value_image(x, y) > 0.0 && tsdf_value < 0.0) {
                // Older observations say this pixel is unoccupied, but the
                // current observation says it's occupied.
                sdf_weights_image(x, y) = map_options_.sdf_mistrust_weight *
                    sdf_weights_image(x, y);
              }
            }
            sdf_value_image(x, y) =
                (sdf_value_image(x, y) * sdf_weights_image(x, y) +
                sdf_weight * tsdf_value) /
                (sdf_weights_image(x, y) + sdf_weight);
            // Update weights
            sdf_weights_image(x, y) += sdf_weight;
          }
          if (sdf_weights_image(x, y) > map_options_.min_sdf_inference_weight) {
            if (sdf_value_image(x, y) >= 0.0f) {
              sdf_display_image(x, y, 0, 0) = 0;
              sdf_display_image(x, y, 0, 1) = static_cast<uint8_t>(
                  sdf_value_image(x, y) / map_options_.max_sdf_value * 255.0);
            } else {
              sdf_display_image(x, y, 0, 0) = static_cast<uint8_t>(
                  sdf_value_image(x, y) / map_options_.min_sdf_value * 255.0);
              sdf_display_image(x, y, 0, 1) = 0;
            }
            sdf_display_image(x, y, 0, 2) = 0;
          } else {
            sdf_display_image(x, y, 0, 0) = 0;
            sdf_display_image(x, y, 0, 1) = 0;
            sdf_display_image(x, y, 0, 2) = 0xFF;
          }
        }
      }
    }
    const string weights_image_file = kObjectMapDir +
        StringPrintf("/%03d_sdf_weights.png", static_cast<int>(i));
    sdf_weights_image.save_png(weights_image_file.c_str());
    const string sdf_image_file = kObjectMapDir +
        StringPrintf("/%03d_sdf_values.png", static_cast<int>(i));
    sdf_display_image.save_png(sdf_image_file.c_str());
    vector<pair<Vector2f, Vector2f> > contour;
    MarchingSquares(
        sdf_value_image, sdf_weights_image, image_origin,
        map_options_.image_resolution, map_options_.min_sdf_weight, &contour);
  }
  printf("\n");
}

void BuildGridObjects(const vector<vector<KDNodeValue2f> >& objects,
                      const vector<Pose2Df>& poses,
                      vector<GridObject>* grd_objects_ptr) {
  vector<GridObject>& grid_objects = *grd_objects_ptr;
  const float pixel_half_width = sqrt(2.0) * map_options_.image_resolution;
  grid_objects.resize(objects.size());

  // For every cluster:
  printf("Building grid objects...\n");
  for (size_t i = 0; i < objects.size(); ++i) {
    const vector<KDNodeValue2f>& object = objects[i];
    GridObject& grid_object = grid_objects[i];
    for (size_t j = 0; j < object.size(); ++j) {
      grid_object.points.push_back(object[j].point);
    }
    float min_x(FLT_MAX), min_y(FLT_MAX);
    float max_x(-FLT_MAX), max_y(-FLT_MAX);
    for (size_t j = 0; j < object.size(); ++j) {
      min_x = min(min_x, object[j].point.x());
      max_x = max(max_x, object[j].point.x());
      min_y = min(min_y, object[j].point.y());
      max_y = max(max_y, object[j].point.y());
    }
    grid_object.width = max_x - min_x + 2.0 * map_options_.image_border;
    grid_object.height = max_y - min_y + 2.0 * map_options_.image_border;
    printf("\rObject %2d: width %5.2f height %5.2f",
           static_cast<int>(i), grid_object.width, grid_object.height);
    fflush(stdout);
    grid_object.image_width =
        ceil(grid_object.width / map_options_.image_resolution);
    grid_object.image_height =
        ceil(grid_object.height / map_options_.image_resolution);
    cimg_library::CImg<float> occupancy_image(
        grid_object.image_width, grid_object.image_height);
    cimg_library::CImg<float> occupancy_counts(
        grid_object.image_width, grid_object.image_height);
    const Vector2f image_origin(
        min_x - map_options_.image_border, min_y - map_options_.image_border);
    grid_object.location = image_origin;
    grid_object.angle = 0.0;
    grid_object.image_resolution = map_options_.image_resolution;
    grid_object.pose_transform = Translation2f(image_origin);
    OMP_PARALLEL_FOR
    for (size_t x = 0; x < grid_object.image_width; ++x) {
      for (size_t y = 0; y < grid_object.image_height; ++y) {
        occupancy_image(x, y) = 0.0;
        occupancy_counts(x, y) = 0.0;
        const Vector2f pixel_loc =
            image_origin + map_options_.image_resolution * Vector2f(x, y);
        // For every observed point:
        for (size_t j = 0; j < object.size(); ++j) {
          const size_t point_index = object[j].index;
          const Vector2f& point = object[j].point;
          const Vector2f& source = poses[point_index].translation;
          const Vector2f line_dir = (point - source).normalized();
          const Vector2f line_perp = Perp2(line_dir);
          const bool within_angle_tolerance =
              (fabs(line_perp.dot(point - pixel_loc)) /
              (point - source).norm() < 0.5 *
              map_options_.laser_angular_resolution);
          const bool along_viewing_ray =
              (fabs(line_perp.dot(pixel_loc - point)) < pixel_half_width);
          if (!along_viewing_ray && !within_angle_tolerance) continue;
          const float sdf_value = line_dir.dot(point - pixel_loc);
          if (sdf_value > 10.0 * map_options_.min_sdf_value &&
              sdf_value < map_options_.max_sdf_value) {
            occupancy_counts(x, y) += 1.0f;
            const float weight = exp(-sdf_value * sdf_value /
                sq(map_options_.laser_std_dev));
            occupancy_image(x, y) += weight;
          }
        }
      }
    }

    OMP_PARALLEL_FOR
    for (size_t x = 0; x < grid_object.image_width; ++x) {
      for (size_t y = 0; y < grid_object.image_height; ++y) {
        const bool valid = (occupancy_counts(x, y) > 5.0f);
        if (valid) {
          occupancy_image(x, y) =
              occupancy_image(x, y) / occupancy_counts(x, y);
        } else {
          occupancy_image(x, y) = 0.0;
        }
      }
    }
    float max_occupancy = 0.0;
    for (size_t x = 0; x < grid_object.image_width; ++x) {
      for (size_t y = 0; y < grid_object.image_height; ++y) {
        max_occupancy = max(max_occupancy, occupancy_image(x, y));
      }
    }
    OMP_PARALLEL_FOR
    for (size_t x = 0; x < grid_object.image_width; ++x) {
      for (size_t y = 0; y < grid_object.image_height; ++y) {
        occupancy_image(x, y) = occupancy_image(x, y) / max_occupancy;
      }
    }
    // Compute the gradients of the occupancy map.
    cimg_library::CImg<float> occupancy_gradients(
        grid_object.image_width, grid_object.image_height, 1, 2);
    cimg_library::CImg<uint8_t> gradient_image(
        grid_object.image_width, grid_object.image_height, 1, 3);
    OMP_PARALLEL_FOR
    for (size_t x = 0; x < grid_object.image_width; ++x) {
      for (size_t y = 0; y < grid_object.image_height; ++y) {
        if (x == 0 || y == 0 ||
            x == grid_object.image_width - 1 ||
            y == grid_object.image_height - 1) {
          occupancy_gradients(x, y, 0, 0) = 0.0f;
          occupancy_gradients(x, y, 0, 1) = 0.0f;
        } else {
          const float gradient_x =
              (occupancy_image(x + 1, y - 1) - occupancy_image(x - 1, y - 1) +
              occupancy_image(x + 1, y) - occupancy_image(x - 1, y) +
              occupancy_image(x + 1, y + 1) - occupancy_image(x - 1, y + 1)) /
              (6.0f * map_options_.image_resolution);
          const float gradient_y =
              (occupancy_image(x - 1, y + 1) - occupancy_image(x - 1, y - 1) +
              occupancy_image(x, y + 1) - occupancy_image(x, y - 1) +
              occupancy_image(x + 1, y + 1) - occupancy_image(x + 1, y - 1)) /
              (6.0f * map_options_.image_resolution);
          occupancy_gradients(x, y, 0, 0) = gradient_x;
          occupancy_gradients(x, y, 0, 1) = gradient_y;
        }
        gradient_image(x, y, 0, 0) = static_cast<uint8_t>(
            occupancy_gradients(x, y, 0, 0) * map_options_.image_resolution *
            255.0 + 127.0);
        gradient_image(x, y, 0, 1) = static_cast<uint8_t>(
            occupancy_gradients(x, y, 0, 0) * map_options_.image_resolution *
            255.0 + 127.0);
        gradient_image(x, y, 0, 2) = 255;
      }
    }
    grid_object.occupancy_gradients = occupancy_gradients;
    grid_object.occupancy_image = occupancy_image;
    const string object_map_file =
        kObjectMapDir + StringPrintf("/%03d.dat", static_cast<int>(i));
    grid_object.Save(object_map_file);
    const string gradients_image_file = kObjectMapDir +
      StringPrintf("/%03d_gradients.png", static_cast<int>(i));
    gradient_image.save_png(gradients_image_file.c_str());
    occupancy_image = 255.0 * occupancy_image;
    const string occupancy_image_file = kObjectMapDir +
        StringPrintf("/%03d_occupancy.png", static_cast<int>(i));
    occupancy_image.save_png(occupancy_image_file.c_str());
  }
  printf("\n");
}

float OverlapScore(int index1, int index2, const GridObject& object1,
                   const GridObject& object2, const Affine2f& transform,
                   bool persistent) {
  cimg_library::CImg<uint8_t> overlap_image(
      object2.image_width, object2.image_height, 1, 3, 0);
  const string debug_obj1 = (persistent) ?
      StringPrintf("results/object_maps/alignments/persistent/%d_%d_1.txt",
                   index1, index2) :
      StringPrintf("results/object_maps/alignments/%d_%d_1.txt",
                   index1, index2);
  const string debug_obj2 = (persistent) ?
      StringPrintf("results/object_maps/alignments/persistent/%d_%d_2.txt",
                   index1, index2) :
      StringPrintf("results/object_maps/alignments/%d_%d_2.txt",
                   index1, index2);
  ScopedFile fid_obj1(debug_obj1, "w");
  ScopedFile fid_obj2(debug_obj2, "w");
  CHECK_NOTNULL(fid_obj1());
  CHECK_NOTNULL(fid_obj2());
  overlap_image = cimg_library::CImg<uint8_t>(
      object1.image_width, object1.image_height, 1, 3, 0);
  float overlap_score = 0.0;
  float num_valid_pixels = 0.0;
  for (unsigned int x = 0; x < object1.image_width; ++x) {
    for (unsigned int y = 0; y < object1.image_height; ++y) {
      const float& occupancy1 = object1.occupancy_image(x, y);
      overlap_image(x, y, 0, 0) = static_cast<uint8_t>(occupancy1 * 255.0);
      overlap_image(x, y, 0, 1) = 0;
      overlap_image(x, y, 0, 2) = 0;
      if (occupancy1 > map_options_.occupancy_threshold) {
        num_valid_pixels += 1.0;
      }
      const Vector2f p1 =
          object1.location + object1.image_resolution * Vector2f(x, y);
      const Vector2f p2 = transform * p1;
      const Vector2f p2_image = p2 / object2.image_resolution;
      const int x2 = rint(p2_image.x());
      const int y2 = rint(p2_image.y());
      if (x2 < 0 || y2 < 0 ||
          x2 > static_cast<int>(object2.image_width - 1) ||
          y2 > static_cast<int>(object2.image_height - 1)) {
        fprintf(fid_obj2(), "%.3f ", 0.0);
      } else {
        const float& occupancy2 = object2.occupancy_image(x2, y2);
        fprintf(fid_obj2(), "%.3f ", occupancy2);
        overlap_image(x, y, 0, 1) = static_cast<uint8_t>(
              object2.occupancy_image(x2, y2) * 255.0);
        if (occupancy1 > map_options_.occupancy_threshold &&
            occupancy2 > map_options_.occupancy_threshold) {
          overlap_score += 1.0;
        }
      }
      fprintf(fid_obj1(), "%.3f ", occupancy1);
    }
    fprintf(fid_obj1(), "\n");
    fprintf(fid_obj2(), "\n");
  }
  overlap_score = overlap_score / num_valid_pixels;
  const string image_name = (persistent) ?
      StringPrintf("results/object_maps/alignments/persistent/%d_%d.png",
                   index1, index2) :
      StringPrintf("results/object_maps/alignments/%d_%d.png",
                   index1, index2);
  overlap_image.save_png(image_name.c_str());
  return overlap_score;
}

double AlignGridObjects(const GridObject& object1, const GridObject& object2,
                        Affine2f* best_transform_ptr) {
  Affine2f& best_transform = *best_transform_ptr;
  const Vector2f centroid1 = object1.location + object1.image_resolution * 0.5 *
      Vector2f(object1.image_width, object1.image_height);
  const Vector2f centroid2 = object2.location + object2.image_resolution * 0.5 *
      Vector2f(object2.image_width, object2.image_height);

  static const size_t kNumAngularBins =
      ceil(2.0 * M_PI / map_options_.matching_angular_resolution);
  static const size_t kNumLocBins =
      ceil(2.0 * map_options_.matching_delta_loc /
      map_options_.matching_loc_resolution);
  vector<vector<vector<double> > > overlap_values(
      kNumAngularBins, vector<vector<double> >(
          kNumLocBins, vector<double>(kNumLocBins, 0.0)));
  vector<vector<vector<Affine2f> > > transforms(
      kNumAngularBins, vector<vector<Affine2f> >(
          kNumLocBins, vector<Affine2f>(kNumLocBins)));

  OMP_PARALLEL_FOR
  for (size_t i = 0; i < kNumAngularBins; ++i) {
    const float a =
        static_cast<float>(i) * map_options_.matching_angular_resolution;
    for (size_t j = 0; j < kNumLocBins; ++j) {
      const float dx = -map_options_.matching_delta_loc +
          static_cast<float>(j) * map_options_.matching_loc_resolution;
      for (size_t k = 0; k < kNumLocBins; ++k) {
        const float dy = -map_options_.matching_delta_loc +
            static_cast<float>(k) * map_options_.matching_loc_resolution;
        double& overlap = overlap_values[i][j][k];
        double total_occupancy = 0.0;
        overlap = 0.0;
        // Transform to convert points from object1's coordinates to
        // object2's coordinates.
        Affine2f& transform = transforms[i][j][k];
        transform = Translation2f(centroid2 - object2.location) *
            Translation2f(Vector2f(dx, dy)) *
            Rotation2Df(a) * Translation2f(-centroid1);
        for (unsigned int x = 0; x < object1.image_width; ++x) {
          for (unsigned int y = 0; y < object1.image_height; ++y) {
            const float& occupancy1 = object1.occupancy_image(x, y);
            const Vector2f p1 =
                object1.location + object1.image_resolution * Vector2f(x, y);
            const Vector2f p2 = transform * p1;
            const Vector2f p2_image = p2 / object2.image_resolution;
            const int p2_image_x = rint(p2_image.x());
            const int p2_image_y = rint(p2_image.y());
            if (p2_image_x < 0 || p2_image_y < 0 ||
                p2_image_x > static_cast<int>(object2.image_width - 1) ||
                p2_image_y > static_cast<int>(object2.image_height - 1)) {
              if (occupancy1 > map_options_.epsilon_occupancy) {
                total_occupancy += 1.0;
                overlap += occupancy1;
              }
              continue;
            }
            const float& occupancy2 =
                object2.occupancy_image(p2_image_x, p2_image_y);
            if (occupancy1 > map_options_.epsilon_occupancy ||
                occupancy2 > map_options_.epsilon_occupancy) {
              overlap += fabs(occupancy1 - occupancy2);
              total_occupancy += 1.0;
            }
          }
        }
        overlap = 1.0 - overlap / total_occupancy;
      }
    }
  }

  double max_overlap = 0.0;
  for (size_t i = 0; i < overlap_values.size(); ++i) {
    for (size_t j = 0; j < overlap_values[i].size(); ++j) {
      for (size_t k = 0; k < overlap_values[i][j].size(); ++k) {
        if (max_overlap < overlap_values[i][j][k]) {
          max_overlap = overlap_values[i][j][k];
          best_transform = transforms[i][j][k];
        }
      }
    }
  }
  return max_overlap;
}

void DisplayPersistentObjects() {
  static const size_t kMinInstances = 2;
  size_t num_common_models = 0;
  ScopedFile fid("results/object_maps/persistent_objects.txt", "w");
  for (size_t i = 0; i < persistent_objects_.size(); ++i) {
    fprintf(fid, "%lu\n", persistent_objects_[i].instance_poses.size());
    if (persistent_objects_[i].instance_poses.size() < kMinInstances) continue;
    vector<Vector2f> object_points;
    ++num_common_models;
    for (unsigned int x = 0; x < persistent_objects_[i].image_width; ++x) {
      for (unsigned int y = 0; y < persistent_objects_[i].image_height; ++y) {
        if (persistent_objects_[i].occupancy_image(x, y) <= 0.4) {
          continue;
        }
        const Vector2f p =
            persistent_objects_[i].image_resolution * Vector2f(x, y);
        object_points.push_back(p);
      }
    }
    double t_oldest_instance = FLT_MAX;
    double t_newest_instance = -FLT_MAX;
    CHECK_EQ(persistent_objects_[i].instance_timestamps.size(),
             persistent_objects_[i].instance_poses.size());
    for (size_t j = 0; j < persistent_objects_[i].instance_poses.size(); ++j) {
      t_newest_instance =
          max(t_newest_instance, persistent_objects_[i].instance_timestamps[j]);
      t_oldest_instance =
          min(t_oldest_instance, persistent_objects_[i].instance_timestamps[j]);
    }
    const double object_age = t_newest_instance - t_oldest_instance;
    const PersistentObject& persistent_object =
        persistent_objects_[i];
    printf("Object Model %3d: %4d Instances, age:%.1f\n",
           static_cast<int>(i),
           static_cast<int>(persistent_object.instance_poses.size()),
           object_age);
    for (size_t j = 0; j < persistent_object.instance_poses.size(); ++j) {
      const Vector2f p1(persistent_object.instance_poses[j].translation);
      const Vector2f p2 =
          p1 + Rotation2Df(persistent_object.instance_poses[j].angle) *
          Vector2f(persistent_object.width, 0.0);
      const Vector2f p3 =
          p1 + Rotation2Df(persistent_object.instance_poses[j].angle) *
          Vector2f(persistent_object.width, persistent_object.height);
      const Vector2f p4 =
          p1 + Rotation2Df(persistent_object.instance_poses[j].angle) *
          Vector2f(0.0, persistent_object.height);
      const Affine2f pose =
          Translation2f(persistent_object.instance_poses[j].translation) *
          Rotation2Df(persistent_object.instance_poses[j].angle);
      DrawLine(p1, p2, 0xFFc0c0c0, &display_message_);
      DrawLine(p2, p3, 0xFFc0c0c0, &display_message_);
      DrawLine(p3, p4, 0xFFc0c0c0, &display_message_);
      DrawLine(p4, p1, 0xFFc0c0c0, &display_message_);
      /*
      const double age_fraction =
          (persistent_object.instance_timestamps[j] - t_oldest_instance) /
          object_age;
      const uint32_t instance_color =
          0xFF000000lu |
          (static_cast<uint32_t>(age_fraction * 255.0) << 16) |
          (static_cast<uint32_t>((1.0 - age_fraction) * 255.0) << 0);
      */
      const uint32_t instance_color = 0xFFFF0000;
      for (size_t j = 0; j < object_points.size(); ++j) {
        DrawPoint((pose * object_points[j]), instance_color, &display_message_);
      }
    }
    if (persistent_objects_[i].instance_poses.size() > 4) {
      if (true) {
        cimg_library::CImg<float> occupancy_image =
            255.0 * persistent_object.occupancy_image;
        occupancy_image.save_png("/tmp/model.png");
      }
      PublishDisplay();
      Sleep(1.0);
    } else {
      ClearDrawingMessage(&display_message_);
    }
  }
  printf("%lu common object models\n", num_common_models);
}

void MergePersistentObjects(
    const EnmlMaps::ObjectMapOptions& map_options,
    vector<PersistentObject>* new_objects_ptr,
    vector<PersistentObject>* old_objects_ptr) {
  vector<PersistentObject>& new_objects = *new_objects_ptr;
  vector<PersistentObject>& old_objects = *old_objects_ptr;
  vector<bool> unique(new_objects.size(), true);
  for (size_t i = 0; i < new_objects.size(); ++i) {
    GridObject object_i(new_objects[i]);
    vector<float> scores_ij;
    vector<float> scores_ji;
    vector<Affine2f> transforms_ij;
    vector<Affine2f> transforms_ji;
    vector<size_t> matching_models;
    for (size_t j = 0; j < old_objects.size(); ++j) {
      GridObject object_j(old_objects[j]);
      Affine2f transform_ij = Affine2f::Identity();
      Affine2f transform_ji = Affine2f::Identity();
      AlignGridObjects(object_i, object_j, &transform_ij);
      const float score_ij = OverlapScore(
          i, j, object_i, object_j, transform_ij, true);
      AlignGridObjects(object_j, object_i, &transform_ji);
      const float score_ji = OverlapScore(
        j, i, object_j, object_i, transform_ji, true);
      if (score_ij > map_options.min_clique_overlap_value &&
          score_ji > map_options.min_clique_overlap_value) {
        matching_models.push_back(j);
        scores_ij.push_back(score_ij);
        scores_ji.push_back(score_ji);
        transforms_ij.push_back(transform_ij);
        transforms_ji.push_back(transform_ji);
      }
    }
    if (matching_models.size() < 1) {
      unique[i] = true;
      continue;
    }
    // Merge the new persistent object with the first matching old model.
    old_objects[matching_models[0]].Merge(
        new_objects[i], transforms_ij[0]);
    // Merge remaining matching old models with the first matching old model
    // since this new persistent object bridges both their models.
    for (size_t j = 1; j < matching_models.size(); ++j) {
      const Affine2f relative_transform =
          transforms_ij[0] * transforms_ij[j].inverse(Eigen::Affine);
      old_objects[matching_models[0]].Merge(
        old_objects[matching_models[j]], relative_transform);
    }
    // Finally, delete the duplicate old models.
    for (size_t j = matching_models.size() - 1; j > 0; --j) {
      old_objects.erase(
          old_objects.begin() + matching_models[j]);
    }
    unique[i] = false;
  }
  for (size_t i = 0; i < new_objects.size(); ++i) {
    if (unique[i]) {
      old_objects.push_back(new_objects[i]);
    }
  }
}

void FindUniqueObjects(const vector<GridObject>& objects,
                       const vector<vector<double> >& error_matrix,
                       const vector<vector<Affine2f> >& transforms,
                       vector<PersistentObject>* new_persistent_objects_ptr) {
  printf("Finding strongly connected components...\n");
  // Generate the sequential graph representation from the error matrix.
  vector<vector<long> > graph(objects.size());
  for (size_t i = 0; i < error_matrix.size(); ++i) {
    for (size_t j = 0; j < error_matrix.size(); ++j) {
      if (i != j &&
          error_matrix[i][j] > map_options_.min_clique_overlap_value) {
        graph[i].push_back(j);
      }
    }
  }
  // Compute the strongly-connected components
  int num_objects = 0;
  map<long, vector<long> > scc = compute_scc(graph);
  vector<PersistentObject>& new_persistent_objects =
      *new_persistent_objects_ptr;
  for (size_t i = 0; i < scc.size(); ++i) {
    const vector<long>& component = scc[i];
    if (component.size() < 1) continue;
    vector<size_t> cluster(component.size());
    for (size_t j = 0; j < component.size(); ++j) {
      cluster[j] = component[j];
    }
    new_persistent_objects.push_back(PersistentObject(
        i, cluster, objects, error_matrix, transforms, log_timestamp_));
    if (component.size() > 1) {
      ++num_objects;
      const PersistentObject& persistent_object =
          new_persistent_objects.back();
      for (size_t j = 0; j < persistent_object.instance_poses.size(); ++j) {
        const Vector2f p1(persistent_object.instance_poses[j].translation);
        const Vector2f p2 =
            p1 + Rotation2Df(persistent_object.instance_poses[j].angle) *
            Vector2f(persistent_object.width, 0.0);
        const Vector2f p3 =
            p1 + Rotation2Df(persistent_object.instance_poses[j].angle) *
            Vector2f(persistent_object.width, persistent_object.height);
        const Vector2f p4 =
            p1 + Rotation2Df(persistent_object.instance_poses[j].angle) *
            Vector2f(0.0, persistent_object.height);
        if (debug_level_ > 0) {
          DrawLine(p1, p2, 0xFFc0c0c0, &display_message_);
          DrawLine(p2, p3, 0xFFc0c0c0, &display_message_);
          DrawLine(p3, p4, 0xFFc0c0c0, &display_message_);
          DrawLine(p4, p1, 0xFFc0c0c0, &display_message_);
        }
      }
      if (debug_level_ > 0) {
        for (size_t j = 0; j < component.size(); ++j) {
          for (size_t k = 0; k < objects[component[j]].points.size(); ++k) {
            DrawPoint(objects[component[j]].points[k], kStfPointColor,
                      &display_message_);
          }
          DrawText(objects[component[j]].location,
                   StringPrintf("%d", static_cast<int>(component[j])),
                   0xFF000000,
                   0.5,
                   false,
                   &display_message_);
        }
        PublishDisplay();
      }
    }
  }
  printf("%d objects found.\n", num_objects);
}

void FindReplace(
    const string& oldString, const string& newString, string* line) {
  const size_t oldSize = oldString.length();

  // do nothing if line is shorter than the string to find
  if( oldSize > line->length() ) return;

  const size_t newSize = newString.length();
  for( size_t pos = 0; ; pos += newSize ) {
    // Locate the substring to replace
    pos = line->find( oldString, pos );
    if( pos == string::npos ) return;
    if( oldSize == newSize ) {
      // if they're same size, use std::string::replace
      line->replace( pos, oldSize, newString );
    } else {
      // if not same size, replace by erasing and inserting
      line->erase( pos, oldSize );
      line->insert( pos, newString );
    }
  }
}

void GenerateHtml(
    size_t num_objects, const vector<vector<double> >& error_matrix) {
  string html;
  {
    // The header of the html file is fixed: just read it from the header file.
    ScopedFile fid_header("results/object_maps/html_header.html", "r");
    CHECK_NOTNULL(fid_header());
    fseek(fid_header(), 0L, SEEK_END);
    const size_t file_size = ftell(fid_header());
    rewind(fid_header());
    char* header = new char[file_size + 1];
    CHECK_EQ(fread(header, 1, file_size, fid_header()), file_size);
    header[file_size] = 0;
    string header_string(header);
    delete [] header;

    const string table_size = StringPrintf(
        "%d", static_cast<int>(100 * (num_objects + 1)));
    FindReplace("kTableSize", table_size, &header_string);
    html = header_string + "\n";
  }

  // Start a table.
  html += "<table cellpadding=\"0\" cellspacing=\"0\" border=\"0\""
          "class=\"display\" id=\"example\">\n";

  {
    html += "<thead>\n";
    // Create a fixed top row with the object models.
    html += "\t<tr> \n\t\t<th></th>\n";
    for (size_t i = 0; i < num_objects; ++i) {
      const string object_image = StringPrintf(
          "maps/%s/objects/%03d_occupancy.png",
          kMapName.c_str(),  static_cast<int>(i));
      html += "\t\t<th> <img src=\"" + object_image + "\"> <br>" +
              StringPrintf("%d", static_cast<int>(i)) + "</th>\n";
    }
    html += "\t\t<th></th>\n\t</tr>\n";
    html += "</thead>\n";
  }

  {
    html += "<tbody>\n";
    // The remaining rows are the entries of the error matrix.
    for (size_t i = 0; i < num_objects; ++i) {
      // First entry in every row is the object model.
      const string object_image = StringPrintf(
          "maps/%s/objects/%03d_occupancy.png",
          kMapName.c_str(), static_cast<int>(i));
      html += "\t<tr class=\"gradeA\">\n" + StringPrintf(
          "\t\t<td class=\"left_cell\"> <img src=\"%s\"> <br> %d </td>\n",
          object_image.c_str(), static_cast<int>(i));
      // The remaining entries of the row are the matrix entries with the
      // overlap images.
      // The overlap images will just have a placeholder to begin with, and the
      // actual images will be loaded by jquery when clicked.
      /*
      static const string kImagePlaceholder =
          "\t\t<td class=\"center\"> <img class=\"lazy img-responsive\""
          " data-original=\"";
      */
      for (size_t j = 0; j < num_objects; ++j) {
        const float match_error =
            error_matrix[i][j] - map_options_.good_match_threshold;
        const string match_class =
            (match_error > 0.15 * map_options_.good_match_threshold) ?
            "GoodMatch" :
            ((match_error < -0.15 * map_options_.good_match_threshold) ?
            "BadMatch" : "PoorMatch");
        const string image_placeholder =
            "\t\t<td class=\"" + match_class + "\"> <img src=\"";
        const string overlap_image = StringPrintf(
            "alignments/%d_%d.png", static_cast<int>(i), static_cast<int>(j));
        html += image_placeholder + overlap_image +
            StringPrintf("\"> <br> %d,%d : %.2f </td>\n",
            static_cast<int>(i), static_cast<int>(j), error_matrix[i][j]);
      }
      html += "\t\t<td class=\"right_cell\"></td>\n\t</tr>\n";
    }
    html += "</tbody>\n";
  }
  // Finish the table, and the HTML document.
  html += "</table>\n</body>\n</html>\n";
  ScopedFile fid("/home/joydeepb/public_html/objects.html", "w");
  fprintf(fid(), "%s", html.c_str());
}

void SaveErrorMatrix(const vector<vector<double> > error_matrix) {
  ScopedFile fid("results/object_maps/error_matrix.txt", "w");
  for (size_t i = 0; i < error_matrix.size(); ++i) {
    for (size_t j = 0; j < error_matrix[i].size(); ++j) {
      fprintf(fid(), "%.6f ", error_matrix[i][j]);
    }
    fprintf(fid(), "\n");
  }
  printf("\n");
}

void FindSimilarObjects(const vector<GridObject>& grid_objects,
                        vector<vector<double> >* error_matrix_ptr,
                        vector<vector<Affine2f> >* transforms_ptr) {
  vector<vector<double> >& error_matrix = *error_matrix_ptr;
  vector<vector<Affine2f> >& transforms = *transforms_ptr;
  error_matrix = vector<vector<double> >(
      grid_objects.size(), vector<double>(grid_objects.size(), 0.0));
  transforms = vector<vector<Affine2f> >(
      grid_objects.size(), vector<Affine2f>(
      grid_objects.size(), Affine2f::Identity()));
  printf("\n");
  for (size_t i = 0; i < grid_objects.size(); ++i) {
    printf("\rComparing object %d", static_cast<int>(i));
    fflush(stdout);
    for (size_t j = 0; j < grid_objects.size(); ++j) {
      Affine2f& transform = transforms[i][j];
      AlignGridObjects(grid_objects[i], grid_objects[j], &transform);
      error_matrix[i][j] = OverlapScore(
          i, j, grid_objects[i], grid_objects[j], transform, false);
    }
  }
  printf("\n");
}

void DisplayCliques(const vector<vector<KDNodeValue2f> >& objects,
                    const vector<Pose2Df>& poses) {
  ScopedFile fid("results/cliques.txt", "r");
  CHECK_NOTNULL(fid());
  cimg_library::CImg<int> cliques;
  cliques.load_dlm(fid());
  printf("Displaying Maximal Cliques\n");
  CHECK_EQ(cliques.height(), objects.size());
  vector<GridObject> persistent_objects;
  for (int i = 0; i < cliques.width(); ++i) {
    int num_objects = 0;
    vector<vector<KDNodeValue2f> > instances;
    for (int j = 0; j < cliques.height(); ++j) {
      if (cliques(i, j) == 1) {
        ++num_objects;
        instances.push_back(objects[i]);
        for (size_t k = 0; k < objects[j].size(); ++k) {
          DrawPoint(objects[j][k].point, kStfPointColor, &display_message_);
        }
      }
    }
    if (num_objects > 1) {
      PublishDisplay();
      // Build models of all the instances.
      printf("Building model for cluster %d\n", i);
      vector<GridObject> instance_models;
      BuildGridObjects(instances, poses, &instance_models);
      vector<vector<double> > error_matrix;
      vector<vector<Affine2f> > transforms;
      FindSimilarObjects(instance_models, &error_matrix, &transforms);
      double max_score = 0.0;
      int best_model = -1;
      for (size_t j = 0; j < instance_models.size(); ++j) {
        double score = 0.0;
        for (size_t k = 0; k < instance_models.size(); ++k) {
          if (j == k) continue;
          score += error_matrix[j][k] * error_matrix[k][j];
        }
        if (score > max_score) {
          max_score = score;
          best_model = j;
        }
      }
      if (best_model > -1) {
        persistent_objects.push_back(instance_models[best_model]);
      } else {
        printf("ERROR: no best model found for clique %d\n", i);
      }
    } else {
      ClearDrawingMessage(&display_message_);
    }
  }

  // Compare persistent_objects to existing persistent_objects, and update
  // instance counts.
}

void TestSdfObjects(const vector<GridObject>& objects,
                    const vector<vector<KDNodeValue2f> >& test_obs,
                    vector<float>* test_scores_ptr) {
  vector<float>& test_scores = *test_scores_ptr;
  CHECK_EQ(test_obs.size(), objects.size());
  test_scores.resize(test_obs.size(), 0.0);
  for (size_t i = 0; i < test_obs.size(); ++i) {
    for (size_t j = 0; j < test_obs[i].size(); ++j) {
      const Vector2f p = Rotation2Df(-objects[i].angle) *
          (test_obs[i][j].point - objects[i].location);
      const Vector2f p_image = p / objects[i].image_resolution;
      const int p_x = static_cast<int>(p_image.x());
      const int p_y = static_cast<int>(p_image.y());
      if (p_x >= 0 && p_x < static_cast<int>(objects[i].image_width) &&
          p_y >= 0 && p_y < static_cast<int>(objects[i].image_height)) {
        test_scores[i] += objects[i].occupancy_image(p_x, p_y);
      }
    }
    test_scores[i] = test_scores[i] / static_cast<float>(test_obs[i].size());
  }
}

void TestModelAccuracy(const vector<vector<KDNodeValue2f> >& objects,
                       const vector<Pose2Df>& poses) {
  static const float kFractionIncrement = 0.01;
  ScopedFile fid("results/object_maps/model_accuracy_test.txt", "w");
  for (float f = kFractionIncrement; f < 1.0; f += kFractionIncrement) {
    printf("Testing models with %.1f%% training data\n", f * 100.0);
    fprintf(fid(), "%.2f ", f);

    // Subsample objects.
    vector<vector<KDNodeValue2f> > training_obs(objects.size());
    vector<vector<KDNodeValue2f> > testing_obs(objects.size());
    for (size_t i = 0; i < objects.size(); ++i) {
      for (size_t j = 0; j < objects[i].size(); ++j) {
        const float random_number = frand<float>(0.0, 1.0);
        if (random_number > f) {
          testing_obs[i].push_back(objects[i][j]);
        } else {
          training_obs[i].push_back(objects[i][j]);
        }
      }
    }

    // Train using subsampled objects.
    vector<GridObject> grid_objects;
    BuildGridObjects(training_obs, poses, &grid_objects);

    // Test remaining observations.
    vector<float> test_scores;
    TestSdfObjects(grid_objects, testing_obs, &test_scores);

    for (size_t i = 0; i < test_scores.size(); ++i) {
      fprintf(fid(), "%.2f ", test_scores[i]);
    }
    fprintf(fid(), "\n");
  }
}

int kbhit() {
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &fds);
}

static const int NB_ENABLE = 1;
static const int NB_DISABLE = 0;

void nonblock(int state) {
  struct termios ttystate;

  //get the terminal state
  tcgetattr(STDIN_FILENO, &ttystate);

  if (state==NB_ENABLE)
  {
    //turn off canonical mode
    ttystate.c_lflag &= ~ICANON;
    //minimum of number input read.
    ttystate.c_cc[VMIN] = 1;
  }
  else if (state==NB_DISABLE)
  {
    //turn on canonical mode
    ttystate.c_lflag |= ICANON;
  }
  //set the terminal attributes.
  tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}


void DissectCluster(const vector<KDNodeValue2f>& cluster,
                    const vector<Pose2Df>& poses) {
  vector<Vector2f> seen_poses;
  nonblock(NB_ENABLE);
  for (size_t i = 0; i < cluster.size(); ++i) {
    const Vector2f& current_pose = poses[cluster[i].index].translation;
    if (std::find(seen_poses.begin(), seen_poses.end(), current_pose) !=
        seen_poses.end()) {
      // Already visualized this pose.
      continue;
    }
    seen_poses.push_back(current_pose);
    for (size_t j = i; j < cluster.size(); ++j) {
      if (poses[cluster[j].index].translation != current_pose) continue;
      DrawLine(cluster[j].point, poses[cluster[j].index].translation,
              0xFFc0c0c0, &display_message_);
      DrawPoint(cluster[j].point, kStfPointColor, &display_message_);
    }
    PublishDisplay();
    while (kbhit() == 0) {
      Sleep(0.02);
    }
    fgetc(stdin);
    printf("\r");
    fflush(stdout);
  }
  nonblock(NB_DISABLE);
  exit(0);
}

void ClusterObjects(
    const vector<Pose2Df>& poses,
    const PointCloudf& point_cloud,
    const NormalCloudf& normal_cloud,
    vector<vector<KDNodeValue2f > >* clusters_ptr) {
  const double t_start = GetTimeSec();
  vector<KDNodeValue2f > P;
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    const KDNodeValue2f value(point_cloud[i], normal_cloud[i], P.size());
    P.push_back(value);
  }

  // Create a list of booleans of size ||P|| to track if the points in P have
  // been processed or not.
  vector<bool> P_processed(P.size(), false);

  // Create a Kd-tree representation for the input point cloud dataset P;
  KDTree<float, 2> kdtree(P);

  // Set up an empty list of clusters C, and a queue of the points that need
  // to be checked Q;
  vector<vector<KDNodeValue2f > >& C = *clusters_ptr;
  vector<KDNodeValue2f > Q;

  printf("Processing %d points\n", static_cast<int>(P.size()));
  printf("Clustering... %5.1f%% ", 0.0); fflush(stdout);
  // Then for every point \boldsymbol{p}_i \in P, perform the following steps:
  for (size_t i = 0; i < P.size(); ++i) {
    if (P_processed[i]) continue;

    KDNodeValue2f& p_i = P[i];
    const float progress =
        static_cast<float>(i) / static_cast<float>(P.size()) * 100.0;
    printf("\rClustering... %5.1f%% ", progress); fflush(stdout);

    // add \boldsymbol{p}_i to the current queue Q;
    Q.push_back(p_i);
    DCHECK_LT(p_i.index, P_processed.size());

    // for every point \boldsymbol{p}_i \in Q do:
    for (size_t j = 0; j < Q.size(); ++j) {
      KDNodeValue2f& q_i = Q[j];
      DCHECK_LT(q_i.index, P_processed.size());
      P_processed[q_i.index] = true;
      // search for the set P^i_k of point neighbors of \boldsymbol{p}_i in a
      // sphere with radius r < d_{th};
      vector<KDNodeValue2f > P_i_k;
      kdtree.FindNeighborPoints(
          q_i.point, map_options_.object_distance_threshold, &P_i_k);

      // for every neighbor \boldsymbol{p}^k_i \in P^k_i, check if the point has
      // already been processed, and if not add it to Q;
      for (size_t k = 0; k < P_i_k.size(); ++k) {
        if (!P_processed[P_i_k[k].index]) {
          DCHECK_LT(P_i_k[k].index, P_processed.size());
          Q.push_back(P_i_k[k]);
          P_processed[P_i_k[k].index] = true;
        }
      }
    }
    // when the list of all points in Q has been processed, add Q to the list of
    // clusters C, and reset Q to an empty list
    if (Q.size() > map_options_.min_object_points) {
      C.push_back(Q);
    }
    if (C.size() >= max_num_objects_) {
      printf("\r\nWARNING: Ignoring remaining objects!\n");
      break;
    }
    Q.clear();
  }
  // the algorithm terminates when all points \boldsymbol{p}_i \in P have been
  // processed and are now part of the list of point clusters C

  const double t_duration = GetTimeSec() - t_start;
  printf("\rClustering... Done in %fms\n", t_duration * 1000.0);
  printf("Found %d clusters.\n", static_cast<int>(C.size()));

  // DissectCluster(C[14], poses);
  if (display_cliques_) {
    DisplayCliques(C, poses);
    exit(0);
  }
  PublishStfs(C);
  if (model_accuracy_test_) {
    TestModelAccuracy(C, poses);
    return;
  }
}

void BuildObjectsMap(const string& stfs_file) {
  static const bool kDrawVisibilityLines = false;
  PointCloudf point_cloud;
  NormalCloudf normal_cloud;
  vector<Pose2Df> poses;
  LoadStfs(stfs_file, &point_cloud, &normal_cloud, &poses, &log_timestamp_);
  CHECK_GT(point_cloud.size(), 0);
  CHECK_EQ(point_cloud.size(), normal_cloud.size());
  CHECK_EQ(point_cloud.size(), poses.size());

  // Display the STFs.
  if (debug_level_ > 0) {
    for (size_t i = 0; i < point_cloud.size(); ++i) {
      DrawPoint(point_cloud[i], kStfPointColor, &display_message_);
      if (kDrawVisibilityLines) {
        DrawLine(point_cloud[i], poses[i].translation, 0xFFc0c0c0,
                &display_message_);
      }
    }
    PublishDisplay();
  }
  if (debug_level_ > 3) return;
  vector<vector<KDNodeValue2f > > C;
  ClusterObjects(poses, point_cloud, normal_cloud, &C);

  vector<GridObject> object_instances;
  vector<vector<double> > similarity_matrix;
  vector<vector<Affine2f> > optimal_transforms;
  vector<PersistentObject> new_object_models;
  BuildSdfObjects(C, poses, &object_instances);
  BuildGridObjects(C, poses, &object_instances);
  FindSimilarObjects(object_instances, &similarity_matrix, &optimal_transforms);
  SaveErrorMatrix(similarity_matrix);
  if (debug_level_ > 0) {
    GenerateHtml(object_instances.size(), similarity_matrix);
  }
  FindUniqueObjects(
      object_instances, similarity_matrix, optimal_transforms,
      &new_object_models);
  CHECK(LoadPersistentObjects(
      kObjectMapDir + "/persistent", &persistent_objects_));
  MergePersistentObjects(
      map_options_, &new_object_models, &persistent_objects_);
  if (debug_level_ > 1) DisplayPersistentObjects();
  CHECK(SavePersistentObjects(
      kObjectMapDir + "/persistent/", persistent_objects_));
}

int main(int argc, char** argv) {
  printf("Probabilistic Object Map Builder\n");
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();

  char* stfs_file = NULL;
  char* map_name = NULL;
  bool view_object_map = false;
  static struct poptOption options[] = {
    { "debug" , 'd', POPT_ARG_INT, &debug_level_, 0, "Debug level", "NUM" },
    { "stfs-file", 'f', POPT_ARG_STRING, &stfs_file, 0, "STFs bagfile to use",
        "STRING"},
    { "cliques", 'c', POPT_ARG_NONE, &display_cliques_, 0, "Display cliques",
        "NONE"},
    { "test-accuracy", 't', POPT_ARG_NONE, &model_accuracy_test_, 0,
        "Test model accuracy", "NONE"},
    { "view_map", 'v', POPT_ARG_NONE, &view_object_map, 0, "View Object Map",
        "NONE"},
    { "map_name", 'm', POPT_ARG_STRING, &map_name, 0, "Map Name", "STRING"},
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0){
  }

  const bool config_loading_success = LoadConfiguration();
  CHECK(config_loading_success);

  const string node_name =
      view_object_map ? "Object_Map_Viewer" : "Object_Map_Builder";

  ros::init(argc, argv, node_name,
            ros::init_options::NoSigintHandler);

  ros::NodeHandle ros_node;
  localization_publisher_ =
      ros_node.advertise<cobot_msgs::CobotLocalizationMsg>(
      "Cobot/Localization", 1, true);
  display_publisher_ =
      ros_node.advertise<cobot_msgs::LidarDisplayMsg>(
      "Cobot/VectorLocalization/Gui",1,true);
  gui_capture_client =
      ros_node.serviceClient<cobot_msgs::LocalizationGuiCaptureSrv>(
      "VectorLocalization/Capture");

  if (view_object_map) {
    if (map_name == NULL) {
      printf("ERROR: Map name not specified.\n");
      exit(1);
    }
    kMapName = string(map_name);
    kObjectMapDir = kCobotStackPath + "/../maps/" + kMapName + "/objects";
    CHECK(LoadPersistentObjects(
        kObjectMapDir + "/persistent", &persistent_objects_));
    printf("Loaded %d persistent objects.\n",
         static_cast<int>(persistent_objects_.size()));
    DisplayPersistentObjects();
    return 0;
  }

  if (stfs_file == NULL) {
    printf("ERROR: STFs file not specified.\n");
    exit(1);
  }
  BuildObjectsMap(stfs_file);
  return 0;
}
