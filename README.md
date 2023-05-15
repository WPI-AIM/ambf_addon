# Blender AMBF Add-on
Blender addon for creating and loading AMBF Description Files (ADF) for [AMBF](https://github.com/WPI-AIM/ambf).

# Introduction:
This addon can be used to create ADFs.

![blender-galen](https://user-images.githubusercontent.com/5005445/235729465-ac421bcf-4f6f-4edb-9604-c15d98fee6b2.gif)


#### Author:
Adnan Munawar

#### Description:
1. [AMBF](https://github.com/WPI-AIM/ambf) uses ADF files to represent scene objects (that may model robots, mechanisms, or environments).  ADFs are thus similar to URDFs or SDFs. While ADFs can be constructed by hand, this addon can help create them with an intuitive graphical interface to prevent mistakes, and save time and effort.

#### Usage:

Please check the Youtube playlist with the basic tutorials to get you started.

https://www.youtube.com/playlist?list=PLKH7Q-IzaPumDw77qQzF8deR1l4LgbWeP

#### Known Issues:
1. The **pyyaml** package is usually not installed alongside Blender's Python interpreter, therefore, while trying to load the plugin, you may encounter an error saying **No Module Names 'yaml'**.
This can be solved by installing `pyyaml` for the Blender's Python interpreter. 
In your terminal navigate to the Blender's folder. If you downloaded Blender from its website to your downloads folder, after extracting the zipped file, you should have a folder with the following naming convention `blender-<version>-<os>'.

```bash
cd ~/Downloads/blender-<version>-<os>
cd ./<version>/python/bin
```
Then install pip for the Python interpreter
```
./python<version> -m ensurepip
```
If you get a message such as:
```bash
Requirement already satisfied: <Any path that is not the current working directory path>
```
Then install pip using the 'get-pip' method defined here (https://pip.pypa.io/en/stable/installation/#get-pip-py)

Finally
```
./pip<version> install pyyaml
```

If for example, you downloaded Blender 3.5.0 from its website, the above commands will become

```bash
cd ~/Downloads/blender-3.5.0-linux64/3.5/python/bin/
./python3.10 -m ensurepip
./pip3 install pyyaml
```
2. The simulation key-frame must be at 0 while saving the ADF files to ensure proper world transforms.

##### Windows-specific instructions for the above
Once you have installed Blender, you can find the python interpreter in in e.g. "C:\Program Files\Blender Foundation\Blender 3.5\3.5\python\bin"
Using powershell, you will still follow through the 'get-pip' instructions above (They have ones for windows). The difference is that instead of running "python XYZ" in these examples, you will directly invoke the python.exe file in this directory:
For example:
```bash
.\python.exe "get-pip.py"
```
To install pyyaml, run:
```bash
.\python.exe -m pip install pyyaml
```

