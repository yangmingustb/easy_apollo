
#include "pose.h"
#include <iostream>
#include "math.h"

namespace apollo
{

int init_pose(Pose2D*pose, double x, double y, double theta)
{

    pose->pos.x = x;
    pose->pos.y = y;
    pose->theta = theta;

    return 1;
}

int cvt_pos_global_to_local(Position2D *local_pos, const Position2D *global_pos,
                            const Pose2D*base_pose)
{
    double dx, dy, theta;


    dx = global_pos->x - base_pose->pos.x;
    dy = global_pos->y - base_pose->pos.y;
    theta = base_pose->theta;

    local_pos->x = apollo_sin(theta) * dx - apollo_cos(theta) * dy;
    local_pos->y = apollo_cos(theta) * dx + apollo_sin(theta) * dy;

    return 1;
}

double apollo_unify_theta(double theta, double base)
{
    if (((theta) + (base)) >= 0.0)
    {
        return apollo_fmod((theta) + (base), apollo_PI * 2) - (base);
    }
    else
    {
        if (apollo_fequal(apollo_fmod(theta + base, apollo_PI * 2), 0.0))
        {
            return apollo_fmod((theta) + (base), apollo_PI * 2) - (base);
        }
        else
        {
            return apollo_fmod((theta) + (base), apollo_PI * 2) + apollo_PI * 2 - base;
        }
    }

    return theta;
}

void cvt_pos_global_to_local_fast(Position2D *local_pos,
                                  const Position2D *global_pos,
                                  const Pose2D*base_pose,
                                  const double sin_theta,
                                  const double cos_theta)
{
    double dx, dy;

    dx = global_pos->x - base_pose->pos.x;
    dy = global_pos->y - base_pose->pos.y;

    local_pos->x = sin_theta * dx - cos_theta * dy;
    local_pos->y = cos_theta * dx + sin_theta * dy;

    return;
}

int cvt_pos_local_to_global(Position2D *global_pos, const Position2D *local_pos,
                            const Pose2D*base_pose)
{
    double lx, ly, theta;


    lx = local_pos->x;
    ly = local_pos->y;
    theta = base_pose->theta;

    global_pos->x = base_pose->pos.x;
    global_pos->x += apollo_sin(theta) * lx + apollo_cos(theta) * ly;
    global_pos->y = base_pose->pos.y;
    global_pos->y += apollo_sin(theta) * ly - apollo_cos(theta) * lx;

    return 1;
}

void cvt_pos_local_to_global_fast(Position2D *global_pos,
                                  const Position2D *local_pos,
                                  const Pose2D*base_pose,
                                  const double sin_theta,
                                  const double cos_theta)
{
    double lx, ly;

    lx = local_pos->x;
    ly = local_pos->y;

    global_pos->x = base_pose->pos.x;
    global_pos->x += sin_theta * lx + cos_theta * ly;
    global_pos->y = base_pose->pos.y;
    global_pos->y += sin_theta * ly - cos_theta * lx;

    return;
}

int cvt_theta_global_to_local(double *local_theta, const double global_theta,
                              const double base_theta)
{

    *local_theta =
            apollo_unify_theta(apollo_PI / 2.0 + global_theta - base_theta, 0.0);

    return 1;
}

int cvt_theta_local_to_global(double *global_theta, const double local_theta,
                              const double base_theta)
{

    *global_theta =
            apollo_unify_theta(local_theta + base_theta - apollo_PI / 2.0, 0.0);

    return 1;
}

int cvt_pose_global_to_local(Pose2D*local_pose, const Pose2D*global_pose,
                             const Pose2D*base_pose)
{
    double dx, dy, theta;


    dx = global_pose->pos.x - base_pose->pos.x;
    dy = global_pose->pos.y - base_pose->pos.y;
    theta = base_pose->theta;

    local_pose->pos.x = apollo_sin(theta) * dx - apollo_cos(theta) * dy;
    local_pose->pos.y = apollo_cos(theta) * dx + apollo_sin(theta) * dy;

    local_pose->theta = apollo_PI / 2.0 + global_pose->theta - theta;
    local_pose->theta = apollo_unify_theta(local_pose->theta, 0.0);

    return 1;
}

void cvt_pose_global_to_local_fast(Pose2D*local_pose,
                                   const Pose2D*global_pose,
                                   const Pose2D*base_pose,
                                   const double sin_theta,
                                   const double cos_theta)
{
    double dx, dy, theta;

    dx = global_pose->pos.x - base_pose->pos.x;
    dy = global_pose->pos.y - base_pose->pos.y;
    theta = base_pose->theta;

    local_pose->pos.x = sin_theta * dx - cos_theta * dy;
    local_pose->pos.y = cos_theta * dx + sin_theta * dy;

    local_pose->theta = apollo_PI / 2.0 + global_pose->theta - theta;
    local_pose->theta = apollo_unify_theta(local_pose->theta, 0.0);

    return;
}

int cvt_pose_local_to_global(Pose2D*global_pose, const Pose2D*local_pose,
                             const Pose2D*base_pose)
{
    double lx, ly, theta;


    lx = local_pose->pos.x;
    ly = local_pose->pos.y;
    theta = base_pose->theta;

    global_pose->pos.x = base_pose->pos.x;
    global_pose->pos.x += apollo_sin(theta) * lx + apollo_cos(theta) * ly;
    global_pose->pos.y = base_pose->pos.y;
    global_pose->pos.y += apollo_sin(theta) * ly - apollo_cos(theta) * lx;

    global_pose->theta = local_pose->theta + theta - apollo_PI / 2.0;
    global_pose->theta = apollo_unify_theta(global_pose->theta, 0.0);

    return 1;
}

void cvt_pose_local_to_global_fast(Pose2D*global_pose,
                                   const Pose2D*local_pose,
                                   const Pose2D*base_pose,
                                   const double sin_theta,
                                   const double cos_theta)
{
    double lx, ly, theta;

    lx = local_pose->pos.x;
    ly = local_pose->pos.y;
    theta = base_pose->theta;

    global_pose->pos.x = base_pose->pos.x;
    global_pose->pos.x += sin_theta * lx + cos_theta * ly;
    global_pose->pos.y = base_pose->pos.y;
    global_pose->pos.y += sin_theta * ly - cos_theta * lx;

    global_pose->theta = local_pose->theta + theta - apollo_PI / 2.0;
    global_pose->theta = apollo_unify_theta(global_pose->theta, 0.0);

    return;
}


int is_line_segment2d_intersection(bool *is,
                                         const Position2D *p1,
                                         const Position2D *p2,
                                         const Position2D *p3,
                                         const Position2D *p4)
{
    Position2D p3p1, p3p4, p3p2, p1p3, p1p2, p1p4;
    double cross_result1, cross_result2;


    p3p1.x = p1->x - p3->x;
    p3p1.y = p1->y - p3->y;

    p3p4.x = p4->x - p3->x;
    p3p4.y = p4->y - p3->y;

    p3p2.x = p2->x - p3->x;
    p3p2.y = p2->y - p3->y;

    p1p3.x = -p3p1.x;
    p1p3.y = -p3p1.y;

    p1p2.x = p2->x - p1->x;
    p1p2.y = p2->y - p1->y;

    p1p4.x = p4->x - p1->x;
    p1p4.y = p4->y - p1->y;

    if (!apollo_fgreater(apollo_min(p1->x, p2->x), apollo_max(p3->x, p4->x)) &&
        !apollo_fgreater(apollo_min(p3->x, p4->x), apollo_max(p1->x, p2->x)) &&
        !apollo_fgreater(apollo_min(p1->y, p2->y), apollo_max(p3->y, p4->y)) &&
        !apollo_fgreater(apollo_min(p3->y, p4->y), apollo_max(p1->y, p2->y)))
    {
        cross_result1 = cross_product_2d_by_vector(&p3p1, &p3p4);
        cross_result1 *= cross_product_2d_by_vector(&p3p2, &p3p4);
        cross_result2 = cross_product_2d_by_vector(&p1p3, &p1p2);
        cross_result2 *= cross_product_2d_by_vector(&p1p4, &p1p2);

        if (!apollo_fgreater(cross_result1, 0) &&
            !apollo_fgreater(cross_result2, 0))
        {
            *is = true;
            return 0;
        }
    }

    *is = false;

    return 0;
}

}  // namespace apollo
