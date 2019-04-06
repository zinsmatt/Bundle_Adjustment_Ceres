#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "ba_problem.h"

struct snavely_reprojection_error
{
  snavely_reprojection_error(double obs_x, double obs_y)
    : observed_x(obs_x), observed_y(obs_y)
  {}

  template <class T>
  bool operator ()(const T* camera, const T* point, T* residuals) const
  {
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);
    p[0] += camera[3]; p[1] += camera[4]; p[2] += camera[5];


    T xp = - p[0] / p[2];
    T yp = - p[1] / p[2];

    const T& l1 = camera[7];
    const T& l2 = camera[8];

    T r2 = xp * xp + yp * yp;
    T distortion = T(1.0) + r2 * l1 + r2 * r2 * l2;

    const T& focal = camera[6];
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;

    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);
    return true;
  }

  static ceres::CostFunction* Create(double observed_x, double observed_y)
  {
    return (new ceres::AutoDiffCostFunction<snavely_reprojection_error, 2, 9, 3>(
              new snavely_reprojection_error(observed_x, observed_y)));
  }

  double observed_x, observed_y;
};

int main()
{

  google::InitGoogleLogging("losc");

  BA_Problem ba_problem;
  if (!ba_problem.load_file("../problem-126-40037-pre.txt"))
  {
    std::cerr << "Error : unable to open file\n";
  }



  const double* observations = ba_problem.observations();

  ceres::Problem problem;

  for (int i = 0; i < ba_problem.num_observations(); ++i)
  {
    ceres::CostFunction* cost_function =
        snavely_reprojection_error::Create(observations[2 * i],
                                           observations[2 * i + 1]);

    problem.AddResidualBlock(cost_function, nullptr,
                             ba_problem.mutable_camera_for_observation(i),
                             ba_problem.mutable_point_for_observation(i));
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);



  std::cout << summary.FullReport() << "\n";


  return 0;
}
