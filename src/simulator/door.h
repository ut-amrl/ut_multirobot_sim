#ifndef SRC_SIMULATOR_DOOR_H_
#define SRC_SIMULATOR_DOOR_H_

#include <eigen3/Eigen/Eigen>
#include <vector>

#include "config_reader/config_reader.h"
#include "simulator/entity_base.h"
#include "shared/math/line2d.h"

namespace door {

enum DoorState {
  OPEN, CLOSED,
  OPENING, CLOSING
};

class Door : public EntityBase {

 protected:
  float length_;
  Eigen::Vector2f axis_position_;
  float open_angle_;
  float closed_angle_;
  float angle_;
  DoorState state_;

  config_reader::ConfigReader config_reader_;

 public:
  Door(const std::vector<std::string>& config_file);
  void UpdateROS() const;
  void Transform();
  void Open();
  void Close();
  void SetState(DoorState state);

  void Step(const double& dt) override;
  void SetPose(const Pose2Df& pose) override;
  Pose2Df GetPose() override;
  std::vector<geometry::Line2f> GetLines() override;
  std::vector<geometry::Line2f> GetTemplateLines() override;

  float GetLength() const;
  Eigen::Vector2f GetAxisPosition() const;
  float GetOpenAngle() const;
  float GetClosedAngle() const;
  float GetAngle() const;
  DoorState GetState() const;
};

} // namespace door

#endif // SRC_SIMULATOR_DOOR_H_