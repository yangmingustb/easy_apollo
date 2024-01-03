

#include "modules/viz3d/draw_geometry.h"
#include "modules/viz3d/pcl_viz.h"

namespace apollo
{


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

}  // namespace apollo