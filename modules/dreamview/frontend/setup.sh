# link map_data to dist/assets
SYMLINK=./../modules/dreamview/frontend/dist/assets/map_data
if [ ! -e ${SYMLINK} ] ; then
    ln -sf ./../modules/map/data $SYMLINK
fi

# generate protobuf bundles
./gen_pbjs.sh
