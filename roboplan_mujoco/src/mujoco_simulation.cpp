#include <cassert>

#include <roboplan_mujoco/mujoco_simulation.hpp>

namespace roboplan {
void MujocoSpecDeleter::operator()(mjSpec* spec) const {
  if (spec != nullptr) {
    mj_deleteSpec(spec);
  }
}

void MujocoModelDeleter::operator()(mjModel* model) const {
  if (model != nullptr) {
    mj_deleteModel(model);
  }
}

void MujocoDataDeleter::operator()(mjData* data) const {
  if (data != nullptr) {
    mj_deleteData(data);
  }
}

MujocoSimulation::MujocoSimulation(MujocoModelPtr model, MujocoDataPtr data)
    : model_(std::move(model)), data_(std::move(data)) {
  assert(this->model_ != nullptr);
  assert(this->data_ != nullptr);
}

const mjModel& MujocoSimulation::model() const {
  assert(model_ != nullptr);
  return *model_;
}

mjData& MujocoSimulation::data() {
  assert(data_ != nullptr);
  return *data_;
}

const mjData& MujocoSimulation::data() const {
  assert(data_ != nullptr);
  return *data_;
}

void MujocoSimulation::reset() {
  assert(model_ != nullptr);
  assert(data_ != nullptr);
  mj_resetData(model_.get(), data_.get());
  mj_forward(model_.get(), data_.get());
}

void MujocoSimulation::forward() {
  assert(model_ != nullptr);
  assert(data_ != nullptr);
  mj_forward(model_.get(), data_.get());
}

void MujocoSimulation::step(std::size_t substeps) {
  assert(model_ != nullptr);
  assert(data_ != nullptr);
  assert(substeps > 0);
  for (std::size_t i = 0; i < substeps; ++i) {
    mj_step(model_.get(), data_.get());
  }
}

double MujocoSimulation::time() const {
  assert(data_ != nullptr);
  return data_->time;
}
}  // namespace roboplan