#ifndef BALPROBLEM_H
#define BALPROBLEM_H

#include <string>
#include <vector>

class BA_Problem
{
public:

  int num_observations() const { return num_observations_; }

  const double* observations() const { return observations_.data(); }

  double* mutable_cameras() { return parameters_.data(); }

  double* mutable_points() { return parameters_.data() + 9 * num_cameras_; }


  double* mutable_camera_for_observation(int i)
  {
    return mutable_cameras() + camera_index_[i] * 9;
  }

  double* mutable_point_for_observation(int i)
  {
    return mutable_points() + point_index_[i] * 3;
  }

  bool load_file(const std::string& filename);


private:
  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;

  std::vector<int> point_index_;
  std::vector<int> camera_index_;
  std::vector<double> observations_;
  std::vector<double> parameters_;
};

#endif // BALPROBLEM_H
