#include <QCoreApplication>
#include "osqp_problem.h"
#include <math.h>
#include <iostream>
#include <array>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <algorithm>
#include <functional>
#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>



using namespace  std;
using PathBoundary = std::vector<std::pair<double, double> >;

namespace {
// PathBoundPoint contains: (s, l_min, l_max).
using PathBoundPoint = std::tuple<double, double, double>;
// PathBound contains a vector of PathBoundPoints.
using PathBound = std::vector<PathBoundPoint>;
// ObstacleEdge contains: (is_start_s, s, l_min, l_max, obstacle_id).
using ObstacleEdge = std::tuple<int, double, double, double, std::string>;
}  // namespace

bool UpdatePathBoundaryAndCenterLine(
    size_t idx, double left_bound, double right_bound,
    PathBound* const path_boundaries, double* const center_line) {
  // Update the right bound (l_min):
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]),
                right_bound + 1.0);
  // Update the left bound (l_max):
  double new_l_max = std::fmin(std::get<2>((*path_boundaries)[idx]),
                               left_bound - 1.0);

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max)
  {
    std::cout << "Path is blocked at idx = " << idx;
    return false;
  }

  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  *center_line = (std::get<1>((*path_boundaries)[idx]) +
                  std::get<2>((*path_boundaries)[idx])) /
                 2.0;
  return true;
}

//std::vector<ObstacleEdge> SortObstaclesForSweepLine(
//    const vector<std::string, Obstacle>& indexed_obstacles) {
//  std::vector<ObstacleEdge> sorted_obstacles;

//  // Go through every obstacle and preprocess it.
//  for (const auto* obstacle : indexed_obstacles.Items()) {
//    // Only focus on those within-scope obstacles.
//    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
//      continue;
//    }
//    // Only focus on obstacles that are ahead of ADC.
//    if (obstacle->PerceptionSLBoundary().end_s() < adc_frenet_s_) {
//      continue;
//    }
//    // Decompose each obstacle's rectangle into two edges: one at
//    // start_s; the other at end_s.
//    const auto obstacle_sl = obstacle->PerceptionSLBoundary();
//    sorted_obstacles.emplace_back(
//        1, obstacle_sl.start_s() - FLAGS_obstacle_lon_start_buffer,
//        obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,
//        obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, obstacle->Id());
//    sorted_obstacles.emplace_back(
//        0, obstacle_sl.end_s() + FLAGS_obstacle_lon_end_buffer,
//        obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,
//        obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, obstacle->Id());
//  }

//  // Sort.
//  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
//            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
//              if (std::get<1>(lhs) != std::get<1>(rhs)) {
//                return std::get<1>(lhs) < std::get<1>(rhs);
//              } else {
//                return std::get<0>(lhs) > std::get<0>(rhs);
//              }
//            });

//  return sorted_obstacles;
//}

bool GetBoundaryFromStaticObstacles(PathBound* const path_boundaries,
    std::string* const blocking_obstacle_id, const std::vector<ObstacleEdge>& sorted_obstacles) {
  // Preprocessing.

  double center_line = 0.0; //reference line
  size_t obs_idx = 0;
  int path_blocked_idx = -1;

  std::multiset<double, std::greater<double>> right_bounds;
  right_bounds.insert(std::numeric_limits<double>::lowest());
  std::multiset<double> left_bounds;
  left_bounds.insert(std::numeric_limits<double>::max());

  // Maps obstacle ID's to the decided ADC pass direction, if ADC should
  // pass from left, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_direction;
  // Maps obstacle ID's to the decision of whether side-pass on this obstacle
  // is allowed. If allowed, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_sidepass_decision;

  // Step through every path point.
  for (size_t i = 1; i < path_boundaries->size(); ++i) {
    double curr_s = std::get<0>((*path_boundaries)[i]);
    // Check and see if there is any obstacle change:
    if (obs_idx < sorted_obstacles.size() && std::get<1>(sorted_obstacles[obs_idx]) < curr_s)
    {

      while (obs_idx < sorted_obstacles.size() && std::get<1>(sorted_obstacles[obs_idx]) < curr_s)
      {

        const auto& curr_obstacle = sorted_obstacles[obs_idx];
        double curr_obstacle_l_min = std::get<2>(curr_obstacle);
        double curr_obstacle_l_max = std::get<3>(curr_obstacle);
        std::string curr_obstacle_id = std::get<4>(curr_obstacle);

        if (std::get<0>(curr_obstacle) == 1)
        {
          // A new obstacle enters into our scope:
          //   - Decide which direction for the ADC to pass.
          //   - Update the left/right bound accordingly.
          //   - If boundaries blocked, then decide whether can side-pass.
          //   - If yes, then borrow neighbor lane to side-pass.
          if (curr_obstacle_l_min + curr_obstacle_l_max < center_line * 2) {
            // Obstacle is to the right of center-line, should pass from left.
            obs_id_to_direction[curr_obstacle_id] = true;
            right_bounds.insert(curr_obstacle_l_max);

            if (!UpdatePathBoundaryAndCenterLine(i, *left_bounds.begin(), *right_bounds.begin(),
                    path_boundaries, &center_line)) {
              path_blocked_idx = static_cast<int>(i);
              *blocking_obstacle_id = curr_obstacle_id;
              break;
            }
          } else {
            // Obstacle is to the left of center-line, should pass from right.
            obs_id_to_direction[curr_obstacle_id] = false;
            left_bounds.insert(curr_obstacle_l_min);
            if (!UpdatePathBoundaryAndCenterLine(
                    i, *left_bounds.begin(), *right_bounds.begin(),
                    path_boundaries, &center_line)) {
              path_blocked_idx = static_cast<int>(i);
              *blocking_obstacle_id = curr_obstacle_id;
              break;
            }
          }
        }
        else
        {
          // An existing obstacle exits our scope.
          if (obs_id_to_direction[curr_obstacle_id]) {
            right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
          } else {
            left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
          }
          obs_id_to_direction.erase(curr_obstacle_id);
        }

        // Update the bounds and center_line.
        std::get<1>((*path_boundaries)[i]) = std::fmax(
            std::get<1>((*path_boundaries)[i]),
            *right_bounds.begin() + 1.0);
        std::get<2>((*path_boundaries)[i]) = std::fmin(
            std::get<2>((*path_boundaries)[i]),
            *left_bounds.begin() - 1.0);
        if (std::get<1>((*path_boundaries)[i]) >
            std::get<2>((*path_boundaries)[i])) {
          path_blocked_idx = static_cast<int>(i);
          break;
        } else {
          center_line = (std::get<1>((*path_boundaries)[i]) +
                         std::get<2>((*path_boundaries)[i])) /
                        2.0;
        }

        ++obs_idx;
      }

    } else {
      // If no obstacle change, update the bounds and center_line.
      std::get<1>((*path_boundaries)[i]) =
          std::fmax(std::get<1>((*path_boundaries)[i]),
                    *right_bounds.begin() + 1.0);
      std::get<2>((*path_boundaries)[i]) =
          std::fmin(std::get<2>((*path_boundaries)[i]),
                    *left_bounds.begin() - 1.0);
      if (std::get<1>((*path_boundaries)[i]) >
          std::get<2>((*path_boundaries)[i])) {
        path_blocked_idx = static_cast<int>(i);
        if (!obs_id_to_direction.empty()) {
          *blocking_obstacle_id = obs_id_to_direction.begin()->first;
        }
      } else {
        center_line = (std::get<1>((*path_boundaries)[i]) +
                       std::get<2>((*path_boundaries)[i])) /
                      2.0;
      }

    }

    // Early exit if path is blocked.
    if (path_blocked_idx != -1) {
      break;
    }
  }

  return true;
}


class PathBoundaryInfo {
 public:
  inline void set_start_s(double start_s) { start_s_ = start_s; }
  inline void set_delta_s(double delta_s) { delta_s_ = delta_s; }
  inline void set_boundary(const PathBoundary& boundary) { boundary_ = boundary; }

  inline double delta_s() { return delta_s_; }
  inline const PathBoundary& boundary() { return boundary_; }
  PathBoundary boundary_;

 private:
  double start_s_;
  double delta_s_;
};

void build_path_boundary(PathBoundaryInfo& bound_info, double planning_length,
                         double delta_s, double start_s) {
  bound_info.set_start_s(start_s);
  bound_info.set_delta_s(delta_s);

  int num = floor(planning_length / delta_s);
  PathBoundary bound;

  for (int i = 0; i < num; i++)
  {
    bound.emplace_back(std::make_pair(-2.0, 3.0));
  }

   int start = floor(num / 3.0);
   int end = floor(num / 2.0);

   for (int i = start; i < end; i++) {
     bound[i].first = -1.0;
   }

  bound_info.set_boundary(bound);
}


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    std::cout<<"OSQP RUNNING..."<<std::endl;
    PathBoundaryInfo bound_info;
    std::string blocking_obstacle_id = "";
    //boundary
    PathBound path_boundaries;

    for (size_t i = 0; i < 100; ++i)
    {
        path_boundaries.push_back(std::make_tuple(i*0.5,-3.5,3.5));
    }

    bound_info.set_start_s(0.0);
    bound_info.set_delta_s(0.5);

    std::vector<ObstacleEdge> sorted_obstacles;
    sorted_obstacles.push_back(std::make_tuple(1, 8.0, 0, 1, "1"));
    sorted_obstacles.push_back(std::make_tuple(1, 10.5, -4, -3, "2"));
    sorted_obstacles.push_back(std::make_tuple(0, 13.0, 0, 1, "1"));
    sorted_obstacles.push_back(std::make_tuple(0, 15.0, -4, -3, "2"));

    bool status = GetBoundaryFromStaticObstacles(&path_boundaries, &blocking_obstacle_id, sorted_obstacles);

    PathBoundary bound_result;
    for (size_t i = 0; i < 100; ++i)
    {
        bound_result.emplace_back(std::make_pair(std::get<1>(path_boundaries.at(i)), std::get<2>(path_boundaries.at(i))));
        std::cout<<"start_s: "<< std::get<0>(path_boundaries.at(i)) <<" start_l: "<< std::get<1>(path_boundaries.at(i)) <<" end_l: "<< std::get<2>(path_boundaries.at(i)) <<std::endl;
    }

    bound_info.set_boundary(bound_result);

    //build_path_boundary(bound_info, 50.0, 0.5, 0.0);//build the boundary we need to change according to the distrubiation of obstacle

    std:array<double, 3> init_state = {0.0, 0.0, 0.0};

    OSQPProblem prob(bound_info.boundary().size(), bound_info.delta_s(),
                     init_state);

    std::array<double, 3> end_state = {-1, 0.0, 0.0};

    prob.set_end_state_ref({1000.0, 10.0, 10.0}, end_state);

    prob.set_weight_x(4.0);
    prob.set_weight_dx(20.0);
    prob.set_weight_ddx(1000.0);
    prob.set_weight_dddx(50000.0);

    prob.set_scale_factor({1.0, 10.0, 100.0});
    prob.set_x_bounds(bound_info.boundary());
    prob.set_dx_bounds(-2.0, 2.0);
    prob.set_ddx_bounds(-3.0, 3.0);
    prob.set_dddx_bounds(-4.0, 4.0);

    ofstream mycout;
    time_t nowtime = time(NULL);
     struct tm *p;
     p = gmtime(&nowtime);
     char filename[256] = {0};
     char timeinfo[256] = {0};
     sprintf(timeinfo,"%d-%d-%d %d-%02d-osqp-traj",1900+p->tm_year,1+p->tm_mon,p->tm_mday,8+p->tm_hour,p->tm_min);
     mycout.open(timeinfo);

    if(prob.Optimize(1000)){
      std::cout<<"Optimize successful!!"<<std::endl;

      for (int i = 0; i < prob.x_.size(); ++i)
      {
          mycout<<"x: "<<prob.x_.at(i)<<" dx: "<<prob.dx_.at(i)<<" ddx: "<<prob.ddx_.at(i)<<" left_edge: "<<bound_info.boundary_.at(i).second<<" right_edge: "<<bound_info.boundary_.at(i).first<<std::endl;
      }

    }else{
      std::cout<<"Optimize failed!!"<<std::endl;
    }
    std::cout<<"OSQP END!!!"<<std::endl;

    return a.exec();
}
