#include <gtest/gtest.h>

#include <string>

#include "opencv_viz.h"

using namespace apollo;

TEST(test_uviz, viz2d_start)
{
    int ret;
    viz2d_image image_handle;
    viz2d_font_setting font;
    int win_columns;
    int win_rows;
    int origin_columns_index;
    int origin_rows_index;
    double resolution;
    viz2d_color background_color;
    char win_name[MAX_WINDOW_NAME_LEN];
    IplImage *img;
    int i, j, B, G, R;
    CvScalar ele;

    ret = viz2d_set_default_font(&font);
    EXPECT_EQ(ret, 1);
    EXPECT_EQ(font.font_type, viz2d_font_hershey_simplex);

    strncpy(win_name, "", MAX_WINDOW_NAME_LEN);
    win_columns = 200;
    win_rows = 200;
    origin_columns_index = 100;
    origin_rows_index = 100;
    resolution = 0;
    background_color = viz2d_colors_white;

    ret = viz2d_init_image_handle(&image_handle, &font, win_name, win_columns,
                                 win_rows, origin_columns_index,
                                 origin_rows_index, resolution,
                                 background_color);
    EXPECT_EQ(ret, 0);

    strncpy(win_name, "viz2d_test", MAX_WINDOW_NAME_LEN);
    ret = viz2d_init_image_handle(&image_handle, &font, win_name, win_columns,
                                 win_rows, origin_columns_index,
                                 origin_rows_index, resolution,
                                 background_color);
    EXPECT_EQ(ret, 0);

    resolution = 0.05;
    viz2d_init_image_handle(&image_handle, &font, win_name, win_columns,
                           win_rows, origin_columns_index, origin_rows_index,
                           resolution, background_color);
    ret = viz2d_start(&image_handle);
    EXPECT_EQ(ret, 1);
    EXPECT_TRUE(nullptr != image_handle.image);
    EXPECT_EQ(image_handle.image->nChannels, 3);
    EXPECT_EQ(image_handle.image->depth, 8);
    EXPECT_EQ(image_handle.image->width, win_columns);
    EXPECT_EQ(image_handle.image->height, win_rows);
    img = image_handle.image;
    for (i = 0; i < img->height; i++)
    {
        for (j = 0; j < img->width; j++)
        {
            ele = cvGet2D(img, i, j);
            B = ele.val[0];
            G = ele.val[1];
            R = ele.val[2];
            EXPECT_EQ(255, B);
            EXPECT_EQ(255, G);
            EXPECT_EQ(255, R);
        }
    }
    viz2d_display(&image_handle);
    cvWaitKey(2000); /* wait 2 seconds for show */
    ret = viz2d_shutdown(&image_handle);
    EXPECT_EQ(ret, 1);
    EXPECT_TRUE(nullptr == image_handle.image);
}

TEST(test_uviz, viz2d_get_index)
{
    int ret;
    viz2d_image image_handle;
    int win_columns;
    int win_rows;
    int origin_columns_index;
    int origin_rows_index;
    double resolution;
    viz2d_font_setting font;
    viz2d_color background_color;
    CvPoint point_index;
    char win_name[] = "viz2d_get_index_test";
    double ego_frame_x, ego_frame_y;

    ret = viz2d_set_default_font(&font);
    EXPECT_EQ(ret, 1);
    EXPECT_EQ(font.font_type, viz2d_font_hershey_simplex);

    win_columns = 200;
    win_rows = 200;
    origin_columns_index = 100;
    origin_rows_index = 50;
    resolution = 0.05;
    background_color = viz2d_colors_white;

    ret = viz2d_init_image_handle(&image_handle, &font, win_name, win_columns,
                                 win_rows, origin_columns_index,
                                 origin_rows_index, resolution,
                                 background_color);
    EXPECT_EQ(ret, 1);

    ret = viz2d_start(&image_handle);
    EXPECT_EQ(ret, 1);

    ego_frame_x = 0.0;
    ego_frame_y = 0.0;
    ret = viz2d_get_index(&image_handle, &point_index, ego_frame_x, ego_frame_y);
    EXPECT_EQ(ret, 1);
    EXPECT_EQ(100, point_index.x);
    EXPECT_EQ(50, point_index.y);

    ego_frame_x = 1.0;
    ego_frame_y = -6.0;
    ret = viz2d_get_index(&image_handle, &point_index, ego_frame_x, ego_frame_y);
    EXPECT_EQ(ret, 1);
    EXPECT_EQ(0, point_index.x);
    EXPECT_EQ(70, point_index.y);

    ego_frame_x = -3.0;
    ego_frame_y = 1.0;
    ret = viz2d_get_index(&image_handle, &point_index, ego_frame_x, ego_frame_y);
    EXPECT_EQ(ret, 1);
    EXPECT_EQ(120, point_index.x);
    EXPECT_EQ(0, point_index.y);

    ret = viz2d_shutdown(&image_handle);
    EXPECT_EQ(ret, 1);
    EXPECT_TRUE(nullptr == image_handle.image);
}

TEST(test_uviz, multiply_image_windows)
{
    int ret;
    viz2d_image image_handle[3];
    int win_columns[3] = {200, 400, 600};
    int win_rows[3] = {400, 200, 600};
    int origin_columns_index[3] = {100, 200, 200};
    int origin_rows_index[3] = {200, 100, 200};
    double resolution;
    viz2d_font_setting font;
    viz2d_color background_color;
    char win_name[][MAX_WINDOW_NAME_LEN] = {"viz2d_windows_1", "viz2d_windows_2",
                                            "viz2d_windows_3"};
    int16_t i;

    resolution = 0.05;
    background_color = viz2d_colors_white;
    ret = viz2d_set_default_font(&font);
    EXPECT_EQ(ret, 1);
    EXPECT_EQ(font.font_type, viz2d_font_hershey_simplex);

    for (i = 0; i < 3; i++)
    {
        ret = viz2d_init_image_handle(
                &image_handle[i], &font, win_name[i], win_columns[i],
                win_rows[i], origin_columns_index[i], origin_rows_index[i],
                resolution, background_color);
        EXPECT_EQ(ret, 1);
        ret = viz2d_start(&image_handle[i]);
        EXPECT_EQ(ret, 1);
    }

    for (i = 0; i < 3; i++) viz2d_display(&image_handle[i]);
    cvWaitKey(3000);

    for (i = 0; i < 3; i++)
    {
        ret = viz2d_shutdown(&image_handle[i]);
        EXPECT_EQ(ret, 1);
        EXPECT_TRUE(nullptr == image_handle[i].image);
    }
}

int main(int argc, char *argv[])
{
    int ret;

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
