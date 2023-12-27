
#pragma once

#include "math.h"
#include <iostream>

namespace apollo
{

struct Position2D
{
    double x;
    double y;
};

struct Pose2D
{
    Position2D pos;
    /* theta of head against east (unit: r) [-pi,pi)*/
    double theta;
};

/* Calculate distance from two points */
#define calc_point2d_dist(p1, p2)                             \
    std::sqrt(((p1)->x - (p2)->x) * ((p1)->x - (p2)->x) + \
              ((p1)->y - (p2)->y) * ((p1)->y - (p2)->y))

#define calc_point2d_dist_sq(p1, p2)                  \
    (((p1)->x - (p2)->x) * ((p1)->x - (p2)->x) + \
     ((p1)->y - (p2)->y) * ((p1)->y - (p2)->y))

#define calc_point2d_origin_dist(p) std::sqrt((p)->x *(p)->x + (p)->y * (p)->y)

#define calc_point2d_origin_dist_sq(p) ((p)->x * (p)->x + (p)->y * (p)->y)


/* Check if two points are same */
#define two_points_are_same(pt1, pt2) \
    (apollo_fequal((pt1)->x, (pt2)->x) && (apollo_fequal((pt1)->y, (pt2)->y)))

/* Unify theta to [-base, -base + 2*apollo_PI) */
double apollo_unify_theta(double theta, double base);

/* Get difference of two theta, unify it to [-PI, PI) */
#define apollo_theta_diff(theta1, theta2) \
    apollo_unify_theta((theta1) - (theta2), apollo_PI)

/* Compare two angles (radian) are equal (unified to [0, 2*PI) */
#define apollo_theta_equal(theta1, theta2) \
    (apollo_fequal(apollo_unify_theta((theta1) - (theta2), apollo_PI), 0.0))

#define apollo_theta_equal_within_thres(theta1, theta2, thres) \
    (!apollo_fgreater(std::fabs(apollo_theta_diff((theta1), (theta2))), (thres)))
/*
 * Get theta orientation (angle from east) of a line
 * specified by (curr_pos, next_pos). clockwise is negative.
 * result range: (-PI, PI]
 */
#define apollo_calc_theta(curr_pos, next_pos) \
    apollo_atan2((next_pos)->y - (curr_pos)->y, (next_pos)->x - (curr_pos)->x)

/*
 * Get theta orientation (angle from east) of a line
 * specified by (x1, y1, x2, y2). clockwise is negative.
 * result range: (-PI, PI]
 */
#define apollo_calc_theta_by_x_y(x1, y1, x2, y2) \
    apollo_atan2((y2) - (y1), (x2) - (x1))

/*
 * Vector cross product of ab and ac
 */
#define cross_product_2d_by_vector(a, b) (((a)->x * (b)->y) - (a)->y * (b)->x)

/*
 * Vector cross product of a and b
 */
#define cross_product_2d_by_points(a, b, c)  \
    (((b)->x - (a)->x) * ((c)->y - (a)->y) - \
     ((c)->x - (a)->x) * ((b)->y - (a)->y))

int init_pose(Pose2D *pose, double x, double y, double theta);

int cvt_pos_global_to_local(Position2D *local_pos, const Position2D *global_pos,
                            const Pose2D *base_pose);

void cvt_pos_global_to_local_fast(Position2D *local_pos,
                                  const Position2D *global_pos,
                                  const Pose2D *base_pose,
                                  const double sin_theta,
                                  const double cos_theta);

int cvt_pos_local_to_global(Position2D *global_pos, const Position2D *local_pos,
                            const Pose2D *base_pose);

void cvt_pos_local_to_global_fast(Position2D *global_pos,
                                  const Position2D *local_pos,
                                  const Pose2D *base_pose,
                                  const double sin_theta,
                                  const double cos_theta);

int cvt_theta_global_to_local(double *local_theta, const double global_theta,
                              const double base_theta);

int cvt_theta_local_to_global(double *global_theta, const double local_theta,
                              const double base_theta);

int cvt_pose_global_to_local(Pose2D *local_pose, const Pose2D *global_pose,
                             const Pose2D *base_pose);

void cvt_pose_global_to_local_fast(Pose2D *local_pose,
                                   const Pose2D *global_pose,
                                   const Pose2D *base_pose,
                                   const double sin_theta,
                                   const double cos_theta);

int cvt_pose_local_to_global(Pose2D *global_pose, const Pose2D *local_pose,
                             const Pose2D *base_pose);

void cvt_pose_local_to_global_fast(Pose2D*global_pose,
                                          const Pose2D*local_pose,
                                          const Pose2D*base_pose,
                                          const double sin_theta,
                                          const double cos_theta);

int is_line_segment2d_intersection(bool *is,
                                         const Position2D *p1,
                                         const Position2D *p2,
                                         const Position2D *p3,
                                         const Position2D *p4);


}  // namespace apollo
