# conda activate emernerf

cd plus_general/csrc

# build common protobuf
pip install protoletariat
git clone git@github-cn.plus.ai:PlusAI/common_protobuf.git
python ./build_proto.py -p torch

# build g2opy
apt-get update
apt-get -y install gfortran libatlas-base-dev libsuitesparse-dev liblz4-dev libeigen3-dev libcxsparse3
git clone git@github-cn.plus.ai:PlusAI/g2opy.git
cd g2opy
mkdir build
cd build
cmake .. -DPYTHON_EXECUTABLE=$(which python) -DBUILD_CSPARSE=OFF
make
cd ..
python setup.py install

# fastbag
apt-get -y install libpq-dev
pip install fastbag --extra-index-url http://dist-cn:3141/root/pypi --extra-index-url http://dist-cn:3141/plusai/stable --trusted-host dist-cn

pip install bagpy # for ros
pip install transformations scikit-video

cd ../..

