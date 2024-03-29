# COSE-MAP
<p align="center">
    <img src="./icon.png" width="250"/>
</p>


## Setting up the COSE-MAP docker image
To set up the COSE-MAP docker image the only requirements needed are:
* GPU compatible with CUDA 12.1 and cuDNN 8 
* Docker
* MariaDB
### Create a workspace
First, we need to create a ROS2 galactic workspace that will be mounted inside the COSE-MAP container, and clone there the COSE-MAP repo:
```
mkdir -p cosemap_ws/src/
cd cosemap_ws/src/
git clone **TO DO**
```
### Build docker image
To build the cosemap docker image:
```
cd cosemap/docker
bash 0_build.sh
```
At the moment, before running the cosemap docker image for the first time, MariaDB needs to be installed in the host machine as well. This is due to errors in MariaDB socket ownership. To install MariaDB on the host machine, run :
```
bash scripts/install_mariadb_server_host.sh
```

To run the cosemap container and attach to it:
```
bash 2_run.sh
bash 3_attach.sh
```
To eventually kill the current you have two choices:
```
bash 4_kill.sh
```
or
```
docker kill cosemap
```

### Setup mariadb

From inside the cosemap container run the following command to setup mariadb server:
```
mysql_secure_installation
```
and follow the procedure to configure mariadb security settings. We suggest the following settings: 

> Enter current password for root (enter for none): *(choose a password for root user if you want)*

> Switch to unix_socket authentication [Y/n] ```n```

> Set root password? [Y/n] ```n```

> Remove anonymous users? [Y/n] ```n```

> press ```Y``` or ```ENTER``` for other settings

### Create the Semantic Database (Host machine only)

Enter inside mariadb console:
```
mariadb -u root
```
and create the Semantic Database:
```sh
CREATE DATABASE IF NOT EXISTS semantic_database;
```
to exit from mariadb console:
```
exit
```
If you want to delete a created database you can delete it following [delete database](#delete-database)
### Import Semantic Database dump from file (Host machine only)

Now you can create the Semantic Database. Change *create_database* params in *cosemap_launch/config/db_utils.yaml* and then run the script create_database.py:  

```
cd src/cosemap/
```

```
mariadb semantic_database < dump-semantic_database.sql
```

### Change bind-address (Host machine only)
To allow cooperation, the bind-address of the SQL server on the host machine has to be changed from 127.0.0.1 (localhost) to the static ip of the host machine. To do so, you need to edit mariadb server config file:
```
nano /etc/mysql/mariadb.conf.d/50-server.cnf
```
and change the line ```bind-address = 127.0.0.1``` to ```bind-address = your_static_ip```. Save and exit.
Now the SQL server in your host machine is able to accept connection requests coming to its static ip.

To make changes permanent, the docker image has to be committed. To do so, from outside the docker containter, run:
```sh
docker commit cosemap cosemap:latest
```
### Create a user of the semantic database
This procedure has to be followed for every user that wants to cooperate in the mapping phase. Note that, in order to use cooperative mapping, your machines need to have static ip address.
To create a user open the mariadb console:
```
mariadb
```
and type the following changing *user*, *static_ip* and *password*: 
```sql
$ CREATE USER 'username'@'static_ip' IDENTIFIED BY 'password';
```
to get all provileges on the database:
```sql
$ GRANT ALL PRIVILEGES ON semantic_database.* TO 'username'@'static_ip';
```
```
FLUSH PRIVILEGES;
```
To exit from mariadb console you can just type:
```
exit
```

If you want to delete a created user you can delete it following [delete user](#delete-user)

### Build COSE-MAP
From inside the cosemap docker container:
```
cd /home/cosemap/ws/
export MAKEFLAGS="-j 4"
colcon build --symlink-install --parallel-workers 1
source /home/cosemap/ws/install/setup.bash
```
## Install in ROS2 galactic workspace
Currently, COSE-MAP is being developed and tested within an isolated docker environment. It is also possible to use COSE-MAP in a ROS2 galactic environment (not tested) installing previously the following dependencies:
* CUDA 12.1 or higher
* cuDNN 8 or higher
* OpenCV 4.7 or higher
* PCL 1.13
* mariadb

If all the dependencies are met, clone the repository inside the *src/* directory of your ROS2 galactic workspace:
```
cd /path-to-your-workspace/src/
git clone **TO DO**
```
and then build the workspace:
```
cd ../
export MAKEFLAGS="-j 4"
colcon build --symlink-install --parallel-workers 1
source install/setup.bash
```

To setup mariadb and configure the Semantic Database and its users follow instructions starting from [here](#setup-mariadb).

## Usage
First, you need to provide a segmentation network to the system. We suggest to use YOLOv8m-seg, which is pre-trained on COCO dataset. To download it:
```
cd cosemap_launch/utils
python3 download_and_convert_network_to_onnx.py
```
You also need to provide a .txt file inside cosemap_launch/labels containing the semantic class names. We provide an example file containing the labels of COCO dataset.

To use COSE-MAP launch you need to configure parameters in ```cosemap_launch/config/cosemap.yaml``` and then launch the code with:
```
ros2 launch cosemap_launch cosemap.launch.py
```
To visualize the created map and the mapped objects you can launch:
```
ros2 launch cosemap_launch rviz.launch.py
```

## Utils
### Delete database
Enter mariadb console:
```
mariadb -u root
```
Delete database:
```sql
DROP DATABASE database_name;
```

### Delete user
Enter mariadb console:
```
mariadb -u root
```
Delete user:
```sql
DROP USER 'user'@'ip';
```
### Truncate tables

First, configure database settings in *cosemap_launch/config/db_utils.yaml*. Then run:
```
cd cosemap_launch/utils/
python3 truncate_table.py
```



<!-- # Editing this README

When you're ready to make this README your own, just edit this file and use the handy template below (or feel free to structure it however you want - this is just a starting point!). Thank you to [makeareadme.com](https://www.makeareadme.com/) for this template.

## Suggestions for a good README
Every project is different, so consider which of these sections apply to yours. The sections used in the template are suggestions for most open source projects. Also keep in mind that while a README can be too long and detailed, too long is better than too short. If you think your README is too long, consider utilizing another form of documentation rather than cutting out information.

## Name
Choose a self-explaining name for your project.

## Description
Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors.

## Badges
On some READMEs, you may see small images that convey metadata, such as whether or not all the tests are passing for the project. You can use Shields to add some to your README. Many services also have instructions for adding a badge.

## Visuals
Depending on what you are making, it can be a good idea to include screenshots or even a video (you'll frequently see GIFs rather than actual videos). Tools like ttygif can help, but check out Asciinema for a more sophisticated method.

## Installation
Within a particular ecosystem, there may be a common way of installing things, such as using Yarn, NuGet, or Homebrew. However, consider the possibility that whoever is reading your README is a novice and would like more guidance. Listing specific steps helps remove ambiguity and gets people to using your project as quickly as possible. If it only runs in a specific context like a particular programming language version or operating system or has dependencies that have to be installed manually, also add a Requirements subsection.

## Usage
Use examples liberally, and show the expected output if you can. It's helpful to have inline the smallest example of usage that you can demonstrate, while providing links to more sophisticated examples if they are too long to reasonably include in the README.

## Support
Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc.

## Roadmap
If you have ideas for releases in the future, it is a good idea to list them in the README.

## Contributing
State if you are open to contributions and what your requirements are for accepting them.

For people who want to make changes to your project, it's helpful to have some documentation on how to get started. Perhaps there is a script that they should run or some environment variables that they need to set. Make these steps explicit. These instructions could also be useful to your future self.

You can also document commands to lint the code or run tests. These steps help to ensure high code quality and reduce the likelihood that the changes inadvertently break something. Having instructions for running tests is especially helpful if it requires external setup, such as starting a Selenium server for testing in a browser.

## Authors and acknowledgment
Show your appreciation to those who have contributed to the project.

## License
For open source projects, say how it is licensed.

## Project status
If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers. -->
