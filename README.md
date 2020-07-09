# Blender AMBF Add-on
Blender Add-on for creating and loading AMBF yaml config files

# Introduction:
This is a plugin to ease the creation of AF MultiBodies using Blender and load existing ambf config files in blender.

#### Author:
Adnan Munawar

Email: amunawar@wpi.edu

#### Decription:
1. The blender plugin is to easy the creation of multi-body config files that are used in AMBF Framework.
AMBF stands for (Asynchoronous Multi-Body Framework). AMBF is real-time dynamics engine
based on Bullet and CHAI-3D with ROS Support on Linux. The source code is located at:
"https://github.com/WPI-AIM/ambf"
This plugin helps in generation of both high and low resolution files (for collision) and subsequently
generate the ambf config file which is based on YAML.

2. AMBF Config files are akin to URDF or SDF but are written in YAML rather than XML. AMBF also supports
soft bodies as well as multiple unconnected, semi-connected and fully connected dynamic bodies in simulation.

3. AMBF files allow multiple parents as well as cyclical interconnection which is not possible with URDF and SDF.

4. Joints are considered independent objects, similar to the bodies in the environment. Joints can easily be ignored,
added and modified in the AMBF Yaml config files.

5. Because of the underlying philosophy of treating joints as independent objects, the AMBF yaml config files can seperate out joints from the bodies in differnet files. E.g. one config file can contain information about the bodies only and another config file can contain information about the joints. In addition to this features, the joints can be added at run-time for any dynamic ridig body in simulation.

#### Usage:

Please head over to the Youtube channel with the basic tutorials to get you started.

https://www.youtube.com/playlist?list=PLKH7Q-IzaPumDw77qQzF8deR1l4LgbWeP

#### Known Issues:
1. The **yaml** modules is usually not installed alongside Blenders python interpreter, therefore, while trying to load the plugin, you may encounter an issue saying **No Module Names 'yaml'**.
This can be solved by installing `pyyaml` for the Python interpreter used by Blender. In your terminal navigate to the folder where blender resides. If you downloaded blender
from its website to your downloads folder, after extracting the zipped file, you should have a folder with the following naming convention `blender-<version>-<os>'.

```bash
cd ~/Downloads/blender-<version>-<os>
cd ./<version>/python/bin
```
Then install pip for the python interpreter
```
./python<version> -m ensurepip
```
and finally
```
./pip<version> install pyyaml
```

If for example, you downloaded blender 2.83.1 from its website, the above commands will become

```bash
cd ~/Downloads/blender-2.83.1-linux64
cd ./2.83/python/bin/
./python3.7m -m ensurepip
./pip3 install pyyaml
```
2. The simulation key-frame must be at 0 while saving the ADF files to ensure proper world transforms.
