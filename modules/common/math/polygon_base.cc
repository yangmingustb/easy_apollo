#include "polygon_base.h"

namespace apollo
{
Polygon2D static_obs_veh_local_polygon;
apollo::common::VehicleParam static_veh_params;

Polygon2D *get_obs_local_polygon() { return &static_obs_veh_local_polygon; }

const apollo::common::VehicleParam &get_static_vehicle_params()
{
    return static_veh_params;
}

int dump_polygon(const Polygon2D *polygon)
{
    int i;

    if (nullptr == polygon) return 1;

    printf("poly_type %d poly_vertex_num %d polygon_radius %.2f\n",
           polygon->type, polygon->vertex_num, polygon->radius);

    for (i = 0; i < polygon->vertex_num; i++)
    {
        printf("%lf\t%lf\n", polygon->vertexes[i].x, polygon->vertexes[i].y);
    }

    return 1;
}

int print_polygon_array(FILE *fp, const Polygon2D *poly_array, int size)
{
    int j = 0;
    const Polygon2D *poly;


    for (j = 0; j < size; j++)
    {
        poly = &poly_array[j];
        print_polygon(fp, poly, j);
    }

    return 1;
}

int print_polygon(FILE *fp, const Polygon2D *poly, int j)
{
    int i;

    if (fp != nullptr)
    {
        fprintf(fp, "polygon[%d]: type %d, vex_num %d\n", j, poly->type,
                poly->vertex_num);
        for (i = 0; i < poly->vertex_num; i++)
        {
            fprintf(fp, " [%.5lf, %.5lf,],\n", poly->vertexes[i].x,
                    poly->vertexes[i].y);
        }
    }
    else
    {
        printf("polygon%d = [ ", j);
        for (i = 0; i < poly->vertex_num; i++)
        {
            if (i < poly->vertex_num - 1)
            {
                printf("[%.5lf, %.5lf,], ", poly->vertexes[i].x,
                       poly->vertexes[i].y);
            }
            else
            {
                printf("[%.5lf, %.5lf, ] ] \n", poly->vertexes[i].x,
                       poly->vertexes[i].y);
            }
        }
    }

    return 1;
}

int init_polygon(Polygon2D *polygon)
{
    int i;

    if (polygon == nullptr)
    {
        return -1;
    }

    polygon->type = POLYGON_ARBITRARY;
    polygon->vertex_num = 0;
    polygon->radius = 0.0;
    polygon->center_pt.x = 0.0;
    polygon->center_pt.y = 0.0;

    for (i = 0; i < MAX_POLYGON_VERTEX_NUM; i++)
    {
        polygon->vertexes[i].x = 0.0;
        polygon->vertexes[i].y = 0.0;

        polygon->axises[i].x = 0.0;
        polygon->axises[i].y = 0.0;
    }

    return 1;
}

int polygon_assign(Polygon2D *des_poly, const Polygon2D *src_poly)
{
    int i;

    if (des_poly == nullptr || src_poly == nullptr)
    {
        return -1;
    }

    des_poly->type = src_poly->type;
    des_poly->vertex_num = src_poly->vertex_num;
    des_poly->radius = src_poly->radius;
    des_poly->center_pt = src_poly->center_pt;

    for (i = 0; i < des_poly->vertex_num; i++)
    {
        des_poly->vertexes[i] = src_poly->vertexes[i];
        des_poly->axises[i] = src_poly->axises[i];
    }

    return 1;
}

int update_polygon_value(Polygon2D *polygon, const Pose2D*center_pose,
                         bool use_center_pose, bool radius_known, double radius)
{
    int i;
    double tmp_dist;
    Position2D *curr, *next, *axis;

    if (polygon == nullptr || !is_polygon_legal(polygon))
    {
        return -1;
    }

    for (i = 0; i < polygon->vertex_num; i++)
    {
        curr = &polygon->vertexes[i];
        next = &polygon->vertexes[(i + 1) % polygon->vertex_num];

        axis = &polygon->axises[i];
        axis->x = -(next->y - curr->y);
        axis->y = next->x - curr->x;
    }

    if (POLYGON_RECTANGLE == polygon->type)
    {
        polygon->center_pt.x = polygon->vertexes[0].x;
        polygon->center_pt.x += polygon->vertexes[2].x;
        polygon->center_pt.x /= 2;

        polygon->center_pt.y = polygon->vertexes[0].y;
        polygon->center_pt.y += polygon->vertexes[2].y;
        polygon->center_pt.y /= 2;


        if (!radius_known)
        {
            polygon->radius = calc_point2d_dist(&polygon->vertexes[0],
                                            &polygon->vertexes[2]) /
                              2.0;
        }
        else
        {
            polygon->radius = radius;
        }
    }
    else
    {
        if (center_pose != nullptr && use_center_pose)
        {
            polygon->center_pt.x = center_pose->pos.x;
            polygon->center_pt.y = center_pose->pos.y;
        }
        else
        {
            polygon->center_pt.x = polygon->vertexes[0].x;
            polygon->center_pt.y = polygon->vertexes[0].y;
        }

        if (!radius_known)
        {
            polygon->radius = 0.0;
            for (i = 0; i < polygon->vertex_num; i++)
            {
                tmp_dist = calc_point2d_dist(&polygon->center_pt,
                                         &polygon->vertexes[i]);
                polygon->radius = std::fmax(polygon->radius, tmp_dist);
            }
        }
        else
        {
            polygon->radius = radius;
        }
    }

    return 1;
}

int generate_rect_polygon(Polygon2D *polygon, double min_x, double min_y,
                          double max_x, double max_y)
{
    int ret;

    if (polygon == nullptr)
    {
        return 0;
    }

    polygon->vertex_num = 4;
    polygon->type = POLYGON_RECTANGLE;

    polygon->vertexes[0].x = max_x;
    polygon->vertexes[0].y = max_y;

    polygon->vertexes[1].x = min_x;
    polygon->vertexes[1].y = max_y;

    polygon->vertexes[2].x = min_x;
    polygon->vertexes[2].y = min_y;

    polygon->vertexes[3].x = max_x;
    polygon->vertexes[3].y = min_y;

    ret = update_polygon_value(polygon, nullptr, false, false,
                               POLYGON_MAX_RADIUS);

    return 1;
}


Polygon2D init_adv_box(const apollo::common::VehicleParam &veh_params)
{
    int ret;
    Polygon2D *local_polygon, adv_local_poly;

    local_polygon = &adv_local_poly;
    local_polygon->vertexes[0].x = veh_params.right_edge_to_center();
    local_polygon->vertexes[0].y = veh_params.front_edge_to_center();

    local_polygon->vertexes[1].x = -veh_params.left_edge_to_center();
    local_polygon->vertexes[1].y = veh_params.front_edge_to_center();

    local_polygon->vertexes[2].x = -veh_params.left_edge_to_center();
    local_polygon->vertexes[2].y = -veh_params.back_edge_to_center();


    local_polygon->vertexes[3].x = veh_params.right_edge_to_center();
    local_polygon->vertexes[3].y = -veh_params.back_edge_to_center();

    local_polygon->vertex_num = 4;

    local_polygon->type = POLYGON_RECTANGLE;
    ret = update_polygon_value(local_polygon, NULL, 0, false,
                               POLYGON_MAX_RADIUS);

    local_polygon->min_tangent_radius = veh_params.left_edge_to_center();

    static_veh_params = veh_params;

    local_polygon = &static_obs_veh_local_polygon;
    local_polygon->vertexes[0].x = veh_params.right_edge_to_center();
    local_polygon->vertexes[0].y = veh_params.length() / 2;

    local_polygon->vertexes[1].x = -veh_params.left_edge_to_center();
    local_polygon->vertexes[1].y = veh_params.length() / 2;

    local_polygon->vertexes[2].x = -veh_params.left_edge_to_center();
    local_polygon->vertexes[2].y = -veh_params.length() / 2;


    local_polygon->vertexes[3].x = veh_params.right_edge_to_center();
    local_polygon->vertexes[3].y = -veh_params.length() / 2;

    local_polygon->vertex_num = 4;

    local_polygon->type = POLYGON_RECTANGLE;
    ret = update_polygon_value(local_polygon, NULL, 0, false, POLYGON_MAX_RADIUS);

    local_polygon->min_tangent_radius = veh_params.left_edge_to_center();

    return adv_local_poly;
}

int cvt_local_polygon_to_global(Polygon2D *poly_global,
                                const Polygon2D *poly_local,
                                const Pose2D*global_pose)
{
    int32_t i;
    double sin_theta, cos_theta;


    sin_theta = apollo_sin(global_pose->theta);
    cos_theta = apollo_cos(global_pose->theta);
    for (i = 0; i < poly_local->vertex_num; i++)
    {
        cvt_pos_local_to_global_fast(&poly_global->vertexes[i],
                                       &poly_local->vertexes[i], global_pose,
                                       sin_theta, cos_theta);
    }
    poly_global->vertex_num = poly_local->vertex_num;
    poly_global->radius = poly_local->radius;
    poly_global->min_tangent_radius = poly_local->min_tangent_radius;
    poly_global->type = poly_local->type;

    cvt_pos_local_to_global_fast(&poly_global->center_pt,
                                   &poly_local->center_pt, global_pose,
                                   sin_theta, cos_theta);

    return 1;
}

int cvt_global_polygon_to_local(Polygon2D *poly_local,
                                const Polygon2D *poly_global,
                                const Pose2D*global_pose)
{
    int32_t i;
    int ret;


    for (i = 0; i < poly_global->vertex_num; i++)
    {
        ret = cvt_pos_global_to_local(&poly_local->vertexes[i],
                                        &poly_global->vertexes[i], global_pose);
    }

    poly_local->vertex_num = poly_global->vertex_num;
    poly_local->radius = poly_global->radius;
    poly_local->min_tangent_radius = poly_global->min_tangent_radius;
    poly_local->type = poly_global->type;

    ret = cvt_pos_global_to_local(&poly_local->center_pt,
                                    &poly_global->center_pt, global_pose);

    return 1;
}


#define VEH_FRONT_RIGHT (0)
#define VEH_FRONT_LEFT (1)
#define VEH_BACK_LEFT (2)
#define VEH_BACK_RIGHT (3)
int extend_adv_box_by_width(Polygon2D *poly, double w,
                            const Polygon2D *adc_local_polygon)
{
    int ret;

    if (poly == NULL)
    {
        return 0;
    }

    *poly = *adc_local_polygon;
    poly->vertexes[VEH_FRONT_RIGHT].x += w;
    poly->vertexes[VEH_FRONT_LEFT].x -= w;
    poly->vertexes[VEH_BACK_LEFT].x -= w;
    poly->vertexes[VEH_BACK_RIGHT].x += w;
    ret = update_polygon_value(
            poly, NULL, false, false, 100000);

    return 0;
}

int extend_adv_box_by_length(Polygon2D *poly, double h,
                             const Polygon2D *adc_local_polygon)
{
    int ret;

    if (poly == NULL)
    {
        return 0;
    }

    *poly = *adc_local_polygon;
    poly->vertexes[VEH_FRONT_RIGHT].y += h;
    poly->vertexes[VEH_FRONT_LEFT].y += h;
    poly->vertexes[VEH_BACK_LEFT].y -= h;
    poly->vertexes[VEH_BACK_RIGHT].y -= h;
    ret = update_polygon_value(
            poly, NULL, false, false, 100000);

    return 0;
}

int extend_adv_box_by_width_and_length(Polygon2D *poly, double left_w,
                                       double right_w, double l,
                                       const Polygon2D *adc_local_polygon)
{
    int ret;

    if (poly == NULL)
    {
        return 0;
    }

    *poly = *adc_local_polygon;
    poly->vertexes[VEH_FRONT_RIGHT].x += right_w;
    poly->vertexes[VEH_FRONT_LEFT].x -= left_w;
    poly->vertexes[VEH_BACK_LEFT].x -= left_w;
    poly->vertexes[VEH_BACK_RIGHT].x += right_w;

    poly->vertexes[VEH_FRONT_RIGHT].y += l;
    poly->vertexes[VEH_FRONT_LEFT].y += l;
    poly->vertexes[VEH_BACK_LEFT].y -= l;
    poly->vertexes[VEH_BACK_RIGHT].y -= l;
    ret = update_polygon_value(
            poly, NULL, false, false, 1000000);

    return 0;
}

}  // namespace apollo