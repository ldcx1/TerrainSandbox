# Terrain sandbox
A project that uses a Kinect camera and a sandbox to allow editing of a terrain in a Urtho3D application.
The project is based mainly on the Oliver Kreylos work: https://web.cs.ucdavis.edu/~okreylos/ResDev/SARndbox/index.html

The `Kinect-3.0` and `SARndbox-2.8` direct copies from the Olives page, but I posted them here for easy access, and for the Kinect driver, to allow intern patches.

## Util links and tutorials
* A plugin for Urtho3D with a more advanced terrain editor: https://github.com/JTippetts/U3DTerrainEditor
* Urtho3D relevant info and tutorials: https://www.gamedesigning.org/engines/urho3d/
* Urtho3D tutorials: https://www.youtube.com/watch?v=2SHZwar9cmA
* Urtho3D docs: https://rebelfork.io/_building.html

## How to build
To run everything an Ubuntu 20+ flavored distribution is required, preferably Linux Mint.

### How to install the Kinect drivers
Follow these instructions: https://web.cs.ucdavis.edu/~okreylos/ResDev/SARndbox/SoftwareInstallation.html

The main commands are:
```shell
sudo apt update
sudo apt upgrade
sudo apt install mesa-utils

echo "Installing Vrui ..."
wget http://web.cs.ucdavis.edu/~okreylos/ResDev/Vrui/Build-Ubuntu.sh
bash Build-Ubuntu.sh
rm ~/Build-Ubuntu.sh

echo "Installing Kinect driver ..."
wget http://web.cs.ucdavis.edu/~okreylos/ResDev/Kinect/Kinect-3.10.tar.gz
tar xfz Kinect-3.10.tar.gz
cd Kinect-3.10
make
sudo make install
sudo make installudevrules
ls /usr/local/bin  # The show the created executables.
cd ..

echo "Installing SARndbox software ..."
wget http://web.cs.ucdavis.edu/~okreylos/ResDev/SARndbox/SARndbox-2.8.tar.gz
tar xfz SARndbox-2.8.tar.gz
cd SARndbox-2.8
make
ls ./bin
```

### How to install Urtho3D
Might not be the best method but it works.
Make sure that you have the last version of Doxgyen.
The commands are:
```shell
cd urtho3d/scripts
./cmake_generic.sh .. -DURTHO3D_LIB=shared
cd ..
make
make install

echo "Patching the headers for the third party tools that are integrated in Urtho3D but are hidden ..."
sudo cp /usr/local/share/Urtho3D/Modules/ThirdParty/* /usr/local/share/
```

### (Optional) How to install OpenCV 3.4
Follow this tutorial: https://gist.github.com/johanleroux/77ceffb2e809be2d918a0f3c0425c7e3
```shell
cd ~/
mkdir opencv
cd ~/opencv

sudo apt-get update
sudo apt-get install -y build-essential cmake
sudo apt-get install -y qt5-default libvtk6-dev
sudo apt-get install -y libgtk2.0-dev pkg-config
sudo apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev
sudo apt-get install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev
sudo apt-get install -y python-dev python-tk python-numpy python3-dev python3-tk python3-numpy

sudo apt-get install -y unzip wget
wget https://github.com/opencv/opencv/archive/3.4.0.zip
unzip 3.4.0.zip
rm 3.4.0.zip

cd opencv-3.4.0
mkdir build
cd build
cmake -DBUILD_EXAMPLES=OFF ..
make -j4
sudo make install
sudo ldconfig
```

### How to run the TerrainSandbox
Connect the camera and run:
```shell
cd TerrainAR
make
./bin/TerrainSandbox
```