#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <gtest/gtest.h>

using pcl::PointCloud;
using pcl::PointXYZ;

TEST(test_opencv3d, coordinatesystem)
{
    pcl::visualization::PCLVisualizer viz("Visualizator");

    viz.addCoordinateSystem(1.0);

    viz.setBackgroundColor(255, 255, 255);

    printf("viz coor\n");
    viz.spin();
}

// 测试cube
TEST(test_opencv3d, cube)
{
    pcl::visualization::PCLVisualizer viz("Visualizator");

    viz.addCoordinateSystem(1.0);

    viz.setBackgroundColor(255, 255, 255);

    float min[3] = {-1, -1, -1};
    float max[3] = {1, 1, 1};

    viz.addCube(min[0], max[0], min[1], max[1],min[2], max[2], 0, 255, 0);
    viz.setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"cube");

    printf("viz coor\n");
    viz.spin();
}

TEST(Viz, polygon2)
{
    pcl::visualization::PCLVisualizer viz("Visualizator");

    viz.addCoordinateSystem(1.0);

    viz.setBackgroundColor(255, 255, 255);

    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

    cloud->points.resize(4);

    (*cloud)[0].x = 0;
    (*cloud)[0].y = 0;
    (*cloud)[0].z = 0.0f;

    (*cloud)[1].x = 1;
    (*cloud)[1].y = 0;
    (*cloud)[1].z = 0.0f;

    (*cloud)[2].x = 1;
    (*cloud)[2].y = 1;
    (*cloud)[2].z = 0.0f;

    (*cloud)[3].x = 0;
    (*cloud)[3].y = 1;
    (*cloud)[3].z = 0.0f;

    viz.addPolygon<PointXYZ>(cloud, 1.0, 0.0, 0.0, "polygon");

    printf("viz coor\n");
    viz.spin();
}

TEST(Viz, grid)
{
    pcl::visualization::PCLVisualizer viz("Visualizator");

    viz.addCoordinateSystem(1.0);

    viz.setBackgroundColor(255, 255, 255);


    pcl::ModelCoefficients::Ptr plane_1 (new pcl::ModelCoefficients);
    plane_1->values.resize (4);
    plane_1->values[0] = 1;
    plane_1->values[1] = 0;
    plane_1->values[2] = 0;
    plane_1->values[3] = -1;


    viz.addPlane(*plane_1, 0, 0, 0, "grid");

    viz.setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
            "grid", 0);

    viz.spin();
}

TEST(Viz, camera_pose)
{
    pcl::visualization::PCLVisualizer viz("Visualizator");

    viz.addCoordinateSystem(1.0);

    viz.setBackgroundColor(255, 255, 255);

    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

    cloud->points.resize(4);

    (*cloud)[0].x = 10;
    (*cloud)[0].y = 100;
    (*cloud)[0].z = 0.0f;

    (*cloud)[1].x = 10;
    (*cloud)[1].y = 140;
    (*cloud)[1].z = 0.0f;

    (*cloud)[2].x = -10;
    (*cloud)[2].y = 140;
    (*cloud)[2].z = 0.0f;

    (*cloud)[3].x = -10;
    (*cloud)[3].y = 100;
    (*cloud)[3].z = 0.0f;

    viz.addPolygon<PointXYZ>(cloud, 1.0, 0.0, 0.0, "polygon");

    double camera_pos[3] = {0, 100, 10};
    double view_pos[3] = {0, 110, 0};

    viz.setCameraPosition(camera_pos[0], camera_pos[1], camera_pos[2],
                          view_pos[0], view_pos[1], view_pos[2], 0, 0, 0);

    printf("viz coor\n");
    viz.spin();
}

TEST(Viz, camera_pose2)
{
    pcl::visualization::PCLVisualizer viz("Visualizator");

    viz.addCoordinateSystem(1.0);

    viz.setBackgroundColor(255, 255, 255);

    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

    cloud->points.resize(4);

    (*cloud)[0].x = 10;
    (*cloud)[0].y = 100;
    (*cloud)[0].z = 0.0f;

    (*cloud)[1].x = 10;
    (*cloud)[1].y = 140;
    (*cloud)[1].z = 0.0f;

    (*cloud)[2].x = -10;
    (*cloud)[2].y = 140;
    (*cloud)[2].z = 0.0f;

    (*cloud)[3].x = -10;
    (*cloud)[3].y = 100;
    (*cloud)[3].z = 0.0f;

    viz.addPolygon<PointXYZ>(cloud, 1.0, 0.0, 0.0, "polygon");

    double camera_pos[3] = {0, 100, 30};
    double view_pos[3] = {0, 100, 10};

    view_pos[1] += 10;
    view_pos[2] -= 10;

    viz.setCameraPosition(camera_pos[0], camera_pos[1], camera_pos[2],
                          view_pos[0], view_pos[1], view_pos[2], 0, 0, 0);

    for (int i = 0; i < 10000; i++)
    {

        camera_pos[1] +=0.2;
        view_pos[1] += 0.2;


        viz.setCameraPosition(camera_pos[0], camera_pos[1], camera_pos[2],
                          view_pos[0], view_pos[1], view_pos[2], 0, 0, 0);

        // viz.updateCoordinateSystemPose();

        viz.spinOnce(100);

        printf("i %d \n", i);
    }
    
    viz.spin();

}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
