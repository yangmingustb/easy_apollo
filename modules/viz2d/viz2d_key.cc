
#include "viz2d_key.h"

namespace apollo
{

void cv_key_direction_init(cv_direction_key* key)
{
    key->left = false;
    key->right = false;
    key->up = false;
    key->low = false;

    return;
}

bool has_cv_direction_key(cv_direction_key* key)
{
    if (key->left || key->right || key->up || key->low)
    {
        return true;
    }

    return false;
}

}