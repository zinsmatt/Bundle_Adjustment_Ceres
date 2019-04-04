#include "ba_problem.h"

#include <fstream>

bool BA_Problem::load_file(const std::string& filename)
{
  std::ifstream file(filename);

  if (!file.is_open())
  {
    return false;
  }

  file >> num_cameras_ >> num_points_ >> num_observations_;
  point_index_.resize(num_observations_);
  camera_index_.resize(num_observations_);
  observations_.resize(2 * num_observations_);

  num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
  parameters_.resize(num_parameters_);

  for (int i = 0; i < num_observations_; ++i)
  {
    file >> camera_index_[i] >> point_index_[i]
         >> observations_[i * 2] >> observations_[i * 2 + 1];
  }

  for (int i = 0; i < num_cameras_; ++i)
  {
    int idx = i * 9;
    file >> parameters_[idx + 0] >> parameters_[idx + 1] >> parameters_[idx + 2]
         >> parameters_[idx + 3] >> parameters_[idx + 4] >> parameters_[idx + 5]
         >> parameters_[idx + 6] >> parameters_[idx + 7] >> parameters_[idx + 8];
  }

  for (int i = 0; i < num_points_; ++i)
  {
    int idx = num_cameras_ * 9 + i * 3;
    file >> parameters_[idx] >> parameters_[idx + 1] >> parameters_[idx + 2];
  }
}
