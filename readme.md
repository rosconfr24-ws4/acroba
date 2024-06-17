# **ACROBA Platform**

## **0. Requirements** 

* Host operating system:
    - Linux
    - Windows 11 with WSL 2 
* Software: 
    - git: 
        https://git-scm.com/book/en/v2/Getting-Started-Installing-Git
    - docker:
        https://docs.docker.com/engine/install/
    -vscode

* Hardware: 
    - NVIDIA GPU (optional)

<br>

>[!WARNING]
> **Known Issues**<br>
> 
> **Docker**<br>
> it is required to use:
> - a docker version >= 24.x <br>
> older versions are not supported due to some issues with the path resolution of docker compose file extensions<br>
> - a docker compose v2 version >= 2.23.x ( :bangbang: uninstall older versions of docker-compose v1 ) <br>
> :warning: **DO NOT USE** docker compose 2.24.3, 2.24.4, 2.24.6 or 2.24.7, they have some bugs and cannot be used, cf:<br>
> https://github.com/docker/compose/issues/11394 <br>
> https://github.com/compose-spec/compose-go/pull/547 <br>
> https://github.com/docker/compose/issues/11544 <br>
> 
>
> **WSL 2**<br>
> When using WSL 2 as the host system, there are a couple of limitations in docker:  
> - OpenCL with Nvidia GPUs is not supported:
>       https://github.com/microsoft/WSL/issues/6951
> - The ports used by the services running in the docker container are not 
>   visible in WSL2 if the network host mode is used:  
>   https://github.com/docker/for-win/issues/6736 
> - The reverse, accessing a service running on WSL2 from a docker using 
>   host network is also not supported
>   https://github.com/docker/for-win/issues/9168#issuecomment-771971994 
> - the moveit setup assistant is not working under WSL with ros noetic. Some fixes are availabe here: 
>   https://answers.ros.org/question/394135/robot-meshes-not-visible-in-rviz-windows11-wsl2/
>
>
> **Linux**<br>
> - Docker desktop
>   * X11 forwarding does not seem to work properly with docker desktop under Ubuntu. 
>   * Some problems were encountered with the Nvidia GPU support while using docker desktop under Ubuntu 20.04 as the host OS: <br>
>     https://github.com/NVIDIA/nvidia-docker/issues/1711 <br>
>     The same issue seems to occur with Ubuntu 22.04 LTS: <br>
>     https://github.com/NVIDIA/nvidia-docker/issues/1652 <br>
> 
>   :white_check_mark: The solution is to desinstall Docker desktop and to use docker engine instead. 
>
> - When using an NVidia GPU with docker engine, docker could trigger the following error "docker: Error response from daemon: failed to create shim task: OCI runtime create failed: runc create failed". In this case, it is necessary to re-set up the container toolkit: <br>
> https://github.com/NVIDIA/nvidia-docker/issues/1648 <br>
> https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
> 
>
> **Windows**<br>
> - windows cannot be used as the host os, as teh host network mode (which is the mode used by the platform) is not available in docker for windows.


## **1. Setup**

### 1.1 NVIDIA GPU 

Some setup is necessary to be able to use the Nvidia GPUs inside the docker containers:

#### 1.1.1 Linux Host OS 

https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

#### 1.1.2 Windows 11 + WSL 2

https://learn.microsoft.com/en-us/windows/ai/directml/gpu-cuda-in-wsl<br>
https://docs.nvidia.com/cuda/wsl-user-guide/index.html


### 1.2 Git

#### 1.2.1 SSH 

To be able to checkout packages, you need to set up your git account to use SSH authentification. 

https://github.com/settings/keys
https://docs.github.com/en/authentication/connecting-to-github-with-ssh

>[!WARNING]
> Do not use a passphrase in your ssh keys (submodules checkout will fail)!

#### 1.2.2 Personal Acess Tokens  

The platform images are available as packages on github. 
To be able to pull or push/release packages from/to github it is necessary to set up PAT (Personal Access Token) authentification. 

##### *Generate a git token*

1. From the git user menu, go to Settings > Developper settings > Personal access tokens > Tokens (classic)
2. Click on Generate new token menu > Generate new token (classic)
3. check the boxes repo write:packages delete:packages
4. Generate token, put a name and save the token somewhere. 

    
##### *Set up a git credential helper*

On linux, one way is to use libsecret to store the PAT in an encrypted format: 
```
sudo apt-get install libsecret-1-0 libsecret-1-dev
sudo make --directory=/usr/share/doc/git/contrib/credential/libsecret
git config --global credential.helper /usr/share/doc/git/contrib/credential/libsecret/git-credential-libsecret
``` 

some other alternatives are described [here](https://stackoverflow.com/questions/46645843/where-to-store-my-git-personal-access-token)


##### *Test the access*
```
docker login ghcr.io -u <Your-GitHub-Username>
password: <enter your PAT>
```

>[!WARNING]
> if you do get some "permission denied error" with docker, you need to set docker up for non root user:<br>
> ```
> sudo usermod -aG docker $USER
> ```
> more information [here](https://docs.docker.com/engine/install/linux-postinstall/)

## **2. Installation**

```
git clone git@github.com:rosconfr24-ws4/acroba.git
cd acroba
make pull
```

### **2. Running *

To run the platform: 

- in "normal" mode: 
```
make run [<optional-arguments>]
```

- using devcontainers for given modules (you will be given the choice): 
```
make run-dev [<optional-arguments>]
```

Optional Arguments: 
*  `X11=(YES|NO) [default: YES]` Use X11 port forwarding or not. If not, a NoVnc server service will be started and the platform is accessible through the web browser at http://localhost:8080/

* `GPU=(YES|NO) [default: YES]` Enable the nvidia GPU support or not (i.e. use CPU only). 

* `VG=(DOC|WIN|WSL|HL|) [default: DOC]` Launch the virtual gym (unity app) on the given platform (DOC:from docker, WIN: from windows, WSL: from wsl2, HL: from docker in headless mode)

## **3. Folders** 

the platform is using the following folders across all containers:

```
/home/acroba/ros-workspaces/ros1-noetic/   # ros1 workspace will contain src build devel log subfolders
/home/acroba/logs                          # volume containing logs 
/home/acroba/data                          # volume for the acroba data
/home/acroba/shared                        # bind mount to the ./shared directory in the architecture repo
```
  
The "shared" folder can be used to hold any specific code file, any change in this folder will be reflected in platform and vice versa.

