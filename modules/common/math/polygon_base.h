
#pragma once

#include <iostream>
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "pose.h"

namespace apollo
{

/* Indicate the max num of vertex of polygon*/
#define MAX_POLYGON_VERTEX_NUM (12)

#define POLYGON_MAX_RADIUS (1000000.0)

/* Check whether polygon is legal */
#define is_polygon_legal(poly)                                              \
    ((poly) == nullptr ? false                                              \
                       : (((poly)->vertex_num <= MAX_POLYGON_VERTEX_NUM) && \
                          ((poly)->vertex_num >= 1)))

enum Polygon_Type
{
    POLYGON_ARBITRARY,
    POLYGON_RECTANGLE,
};

struct Polygon2D
{
    Polygon_Type type;
    int vertex_num;
    Position2D vertexes[MAX_POLYGON_VERTEX_NUM];
    Position2D axises[MAX_POLYGON_VERTEX_NUM];
    Position2D center_pt;
    double radius;
    double min_tangent_radius;

};

 int dump_polygon(const Polygon2D *polygon);

 int print_polygon_array(FILE *fp, const Polygon2D *poly_array, int size);

 int print_polygon(FILE *fp, const Polygon2D *poly,
                         int polygon_array_idx);

 int init_polygon(Polygon2D *polygon);

 int polygon_assign(Polygon2D *des_poly, const Polygon2D *src_poly);

 int update_polygon_value(Polygon2D *polygon, const Pose2D*center_pose,
                                bool use_center_pose, bool radius_known,
                                double radius);

 int generate_rect_polygon(Polygon2D *polygon, double min_x, double min_y,
                                 double max_x, double max_y);


/**
 * @brief
 *
 *
 *
 *                 1  ---------    point id:0
 *                    *       *
 *                    *       *
 *                    *       *
 *                    *       *
 *                 2  *-------*     3
 * @note
 * @param  &veh_params:
 * @retval
 */
Polygon2D init_adv_box(const apollo::common::VehicleParam &veh_params);

Polygon2D *get_obs_local_polygon();

int cvt_local_polygon_to_global(Polygon2D *poly_global,
                                const Polygon2D *poly_local,
                                const Pose2D*global_pose);

int cvt_global_polygon_to_local(Polygon2D *poly_local,
                                const Polygon2D *poly_global,
                                const Pose2D*global_pose);

const apollo::common::VehicleParam &get_static_vehicle_params();

int extend_adv_box_by_width(Polygon2D *poly, double w,
                            const Polygon2D *adc_local_polygon);

int extend_adv_box_by_length(Polygon2D *poly, double h,
                             const Polygon2D *adc_local_polygon);

int extend_adv_box_by_width_and_length(Polygon2D *poly, double left_w,
                                       double right_w, double l,
                                       const Polygon2D *adc_local_polygon);

}  // namespace apollo
