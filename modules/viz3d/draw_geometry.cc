

#include "modules/viz3d/draw_geometry.h"
#include "modules/viz3d/pcl_viz.h"

namespace apollo
{

int draw_local_polygon2d_frame(pcl::visualization::PCLVisualizer *viz,
                               Polygon2D *poly,
                               pcl_color_index color_index,
                               const Pose2D *veh_pose)
{
    pcl_color color;
    pcl_get_color(&color, color_index);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_polygon(
            new pcl::PointCloud<pcl::PointXYZ>);

    pcl_polygon->points.resize(poly->vertex_num);

    for (int i = 0; i < poly->vertex_num; i++)
    {
        (*pcl_polygon)[i].x = poly->vertexes[i].x;
        (*pcl_polygon)[i].y = poly->vertexes[i].y;
        (*pcl_polygon)[i].z = 0.0;
    }

    std::string obj_id = get_pcl_object_string_id();
    viz->addPolygon<pcl::PointXYZ>(pcl_polygon, color.r, color.g, color.b,
                                   obj_id);
    viz->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, obj_id);

    return 0;
}

int draw_global_polygon_frame(pcl::visualization::PCLVisualizer *viz,
                              Polygon2D *global_poly,
                              pcl_color_index color_index,
                              const Pose2D *veh_pose)
{
    pcl_color color;
    pcl_get_color(&color, color_index);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_polygon(
            new pcl::PointCloud<pcl::PointXYZ>);

    pcl_polygon->points.resize(global_poly->vertex_num);

    for (int i = 0; i < global_poly->vertex_num; i++)
    {
        (*pcl_polygon)[i].x = global_poly->vertexes[i].x;
        (*pcl_polygon)[i].y = global_poly->vertexes[i].y;
        (*pcl_polygon)[i].z = 0.0;
    }

    std::string obj_id = get_pcl_object_string_id();
    viz->addPolygon<pcl::PointXYZ>(pcl_polygon, color.r, color.g, color.b,
                                   obj_id);
    viz->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, obj_id);

    return 0;
}

int draw_global_polygon_frame(pcl::visualization::PCLVisualizer *viz,
                              Polygon2D *global_poly,
                              pcl_color color,
                              const Pose2D *veh_pose)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_polygon(
            new pcl::PointCloud<pcl::PointXYZ>);

    pcl_polygon->points.resize(global_poly->vertex_num);

    for (int i = 0; i < global_poly->vertex_num; i++)
    {
        (*pcl_polygon)[i].x = global_poly->vertexes[i].x;
        (*pcl_polygon)[i].y = global_poly->vertexes[i].y;
        (*pcl_polygon)[i].z = 0.0;
    }


    std::string obj_id = get_pcl_object_string_id();

    viz->addPolygon<pcl::PointXYZ>(pcl_polygon, color.r, color.g, color.b,
                                   obj_id);

    viz->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, obj_id);

    return 0;
}

int draw_global_polygon_plane(pcl::visualization::PCLVisualizer *viz,
                              Polygon2D *global_poly,
                              pcl_color color,
                              const Pose2D *veh_pose)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_polygon(
            new pcl::PointCloud<pcl::PointXYZ>);

    pcl_polygon->points.resize(global_poly->vertex_num);

    for (int i = 0; i < global_poly->vertex_num; i++)
    {
        (*pcl_polygon)[i].x = global_poly->vertexes[i].x;
        (*pcl_polygon)[i].y = global_poly->vertexes[i].y;
        (*pcl_polygon)[i].z = 0.0;
    }


    std::string obj_id = get_pcl_object_string_id();

    viz->addPolygon<pcl::PointXYZ>(pcl_polygon, color.r, color.g, color.b,
                                   obj_id);

    viz->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, obj_id);

    return 0;
}

int draw_coordinate_system(pcl::visualization::PCLVisualizer *viz,
                           pcl_color_index color_index, const Pose2D *pose)
{
    double len = 10.0;

    pcl_color color;
    pcl_get_color(&color, pcl_colors_red);


    pcl::PointXYZ start;
    pcl::PointXYZ end;


    // x

    start.x = 0;
    start.y = 0;
    start.z = 0;

    end.x = len;
    end.y = 0;
    end.z = 0;

    int64_t id = get_pcl_object_id();

    std::string obj_id = "arrow" + std::to_string(id);

    viz->addLine(start, end, color.r, color.g, color.b, obj_id);

    // y
    end.x = 0;
    end.y = len;
    end.z = 0;

    id = get_pcl_object_id();

    obj_id = "arrow" + std::to_string(id);
    pcl_get_color(&color, pcl_colors_green);
    viz->addLine(start, end, color.r, color.g, color.b, obj_id);

    // z
    end.x = 0;
    end.y = 0;
    end.z = len;

    obj_id = get_pcl_object_string_id();

    pcl_get_color(&color, pcl_colors_blue);
    viz->addLine(start, end, color.r, color.g, color.b, obj_id);

    return 0;
}

int draw_local_polygon3d_frame(pcl::visualization::PCLVisualizer *viz,
                              Polygon2D *poly,
                              double height,
                              pcl_color_index color_index,
                              const Pose2D *veh_pose)
{
    pcl_color color;
    pcl_get_color(&color, color_index);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_polygon(
            new pcl::PointCloud<pcl::PointXYZ>);

    pcl_polygon->points.resize(poly->vertex_num);

    // lower
    for (int i = 0; i < poly->vertex_num; i++)
    {
        (*pcl_polygon)[i].x = poly->vertexes[i].x;
        (*pcl_polygon)[i].y = poly->vertexes[i].y;
        (*pcl_polygon)[i].z = 0.0;
    }

    std::string obj_id = get_pcl_object_string_id();
    viz->addPolygon<pcl::PointXYZ>(pcl_polygon, color.r, color.g, color.b,
                                   obj_id);


    viz->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, obj_id);

    // upper
    for (int i = 0; i < poly->vertex_num; i++)
    {
        (*pcl_polygon)[i].z += height;
    }

    obj_id = get_pcl_object_string_id();
    viz->addPolygon<pcl::PointXYZ>(pcl_polygon, color.r, color.g, color.b,
                                   obj_id);

    viz->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, obj_id);


    // line
    pcl::PointXYZ start;
    pcl::PointXYZ end;

    for (int i = 0; i < poly->vertex_num; i++)
    {
        start.x = poly->vertexes[i].x;
        start.y = poly->vertexes[i].y;
        start.z = 0;

        end.x = start.x;
        end.y = start.y;
        end.z = start.z + height;

        obj_id = get_pcl_object_string_id();

        viz->addLine(start, end, color.r, color.g, color.b, obj_id);
    }

    return 0;
}


int draw_local_grid(
        pcl::visualization::PCLVisualizer *viz,
        double left ,double right, double front, double back,
        const Pose2D*base_pose, pcl_color_index color_index, int line_width)
{

    pcl_color color;
    pcl_get_color(&color, color_index);

    pcl::PointXYZ start;
    pcl::PointXYZ end;

    Position2D local_pose, global_pose;

    Position2D left_point;
    left_point.x = -left;
    left_point.y = -back;

    Position2D right_point;
    right_point.x = right;
    right_point.y = -back;

    double delta_y = 10;
    double delta_x = 10;

    std::string obj_id;

    while (left_point.y < front)
    {
        start.x = left_point.x;
        start.y = left_point.y;
        start.z = 0;

        end.x = right_point.x;
        end.y = right_point.y;
        end.z = 0;

        obj_id = get_pcl_object_string_id();

        viz->addLine(start, end, color.r, color.g, color.b, obj_id);

        left_point.y += delta_y;
        right_point.y+= delta_y;
    }



    Position2D front_point;
    front_point.x = -left;
    front_point.y = front;

    Position2D back_point;
    back_point.x = -left;
    back_point.y = -back;


    while (front_point.x < right)
    {
        start.x = front_point.x;
        start.y = front_point.y;
        start.z = 0;

        end.x = back_point.x;
        end.y = back_point.y;
        end.z = 0;

        obj_id = get_pcl_object_string_id();

        viz->addLine(start, end, color.r, color.g, color.b, obj_id);

        front_point.x+= delta_x;
        back_point.x += delta_x;
    }

    return 1;
}

}  // namespace apollo