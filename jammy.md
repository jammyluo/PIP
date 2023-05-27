

docker run --name PIP -it --rm --cpus=8 -v /d/proj/xplan/PIP:/root/PIP -p 8888:8888 -p 8777:8777  pip_env


wget -O /etc/apt/sources.list https://mirrors.tencent.com/repo/ubuntu18_sources.list


sudo apt update -y
sudo apt upgrade -y


apt-get install -y iputils-ping net-tools wget vim cmake git-core libgl1-mesa-glx libeigen3-dev build-essential libboost-dev netcat-traditional

# Miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
chmod 777 Miniconda3-latest-Linux-x86_64.sh
./Miniconda3-latest-Linux-x86_64.sh -b
vim ~/.bashrc
export PATH=/root/miniconda3/bin:$PATH
source ~/.bashrc

conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge 
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/msys2/
conda config --add channels https://mirrors.bfsu.edu.cn/anaconda/pkgs/main
conda config --add channels https://mirrors.bfsu.edu.cn/anaconda/pkgs/free
conda config --add channels https://mirrors.bfsu.edu.cn/anaconda/pkgs/r
conda config --add channels https://mirrors.bfsu.edu.cn/anaconda/pkgs/pro
conda config --add channels https://mirrors.bfsu.edu.cn/anaconda/pkgs/msys2
conda config --set show_channel_urls yes

source ~/.bashrc
conda create --name PIPENV python=3.7 -y
conda activate PIPENV 

pip3 config set global.index-url https://mirrors.cloud.tencent.com/pypi/simple
pip3 install pip -U

pip3 install pygame cython numpy torch tqdm chumpy vctoolkit open3d pybullet qpsolvers cvxopt quadprog

git clone --recursive https://github.com/rbdl/rbdl
cd rbdl
git pull origin master
git submodule update --init
cd ..
mkdir rbdl-build
cd rbdl-build

RBDL_BUILD_ADDON_URDFREADER
RBDL_BUILD_PYTHON_WRAPPER

cmake ../rbdl  -DPYTHON_INCLUDE_DIR=/root/miniconda3/envs/PIP/include/python3.7m -DPYTHON_LIBRARY=/root/miniconda3/envs/PIP/lib
make

vim ~/.bashrc
export PYTHONPATH=$PYTHONPATH:/root/rbdl-build/python
source ~/.bashrc






