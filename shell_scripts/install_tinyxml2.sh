

cd /tmp
rm -rf tinyxml2

git clone --depth 1  https://github.com/leethomason/tinyxml2.git

cd tinyxml2/

cmake .
make

./xmltest

sudo make install