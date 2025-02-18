---
parent: Development
---

# ZedSDK on Ubuntu22

Dependecies 
- Cuda 12
- zstd
- GPU with Nvidia compute capacity of =<5.3 (https://developer.nvidia.com/cuda-gpus#compute)

Dowload and run the installer

```
#!/bin/bash

installer="ZED_SDK_Ubuntu_22_V4.2.zstd.run"

wget -O "$installer" https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu22

chmod +x "$installer"

./"$installer" -- silent
```

## Python API Needs to be installed separately

Install dependencies and the API
```
python -m pip install cython numpy opencv-python pyopengl

sudo python3 /usr/local/zed/get_python_api.py
```
Note that since the API is installed with sudo you'll need to run python scripts as sudo too.
Else module pyzed.sl won't be found. 

Links
- https://www.stereolabs.com/docs/installation/linux
- https://www.stereolabs.com/en-fi/developers/release
- https://www.stereolabs.com/docs/app-development/python/install
- https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_network
- https://www.stereolabs.com/docs/installation/specifications
- https://github.com/stereolabs/zed-sdk

# DepthAI on Ubuntu22
Install dependencies and DepthAI
```
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
python3 -m pip install depthai
```

## Test installation

Here we'll do a test installation to check that DepthAI sdk is working. We'll use conda for convinience

Miniconda install
```
#!/bin/bash

installer="miniconda_installer.sh"

wget -O "$installer" https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

bash "$installer"
```
Miniconda enviroment creation and activation
```
conda create --name depthai python=3.10
conda activate depthai
```

Setup DepthAI enviroment:
```
git clone https://github.com/luxonis/depthai-python.git
cd depthai-python\examples\
python3 install_requirements.py
```
And finally test the RGB camera
```
python3 ColorCamera/rgb_preview.py
```
Links
- https://docs.luxonis.com/software/depthai/manual-install/
- https://docs.anaconda.com/miniconda/miniconda-install/
- https://docs.luxonis.com/software/api/python/
  
