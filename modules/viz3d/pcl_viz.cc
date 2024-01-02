
#include "pcl_viz.h"

namespace apollo
{
    
static pcl_color pcl_color_list[pcl_colors_num] = {
        {0.0, 0.0, 0.0},       {255.0, 0.0, 0.0},     {42.0, 42.0, 165.0},
        {255.0, 255.0, 0.0},   {0.0, 215.0, 255.0},   {128.0, 128.0, 128.0},
        {0.0, 128.0, 0.0},     {0.0, 165.0, 255.0},   {0.0, 0.0, 255.0},
        {0.0, 255.0, 255.0},   {203.0, 192.0, 255.0}, {128.0, 0.0, 128.0},
        {192.0, 192.0, 192.0}, {255.0, 255.0, 255.0}, {70.0, 23.0, 11.0},
        {33.0, 145.0, 234.0},  {198.0, 238.0, 118.0}, {144.0, 238.0, 144.0},
        {178.0, 223.0, 238.0}, {255.0, 144.0, 30.0}, {28, 28, 28},
};

int pcl_get_color(pcl_color *Color, pcl_color_index color_index)
{
    if (color_index >= pcl_colors_num)
    {
        *Color = pcl_color_list[color_index];

        return 0;
    }

    *Color = pcl_color_list[color_index];
    
    return 0;
}

} // namespace apollo
