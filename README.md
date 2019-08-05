# moveit_bot_kinematics_plugin
This is a custom plugin for moveit aplicable for bots of any degrees of freedom. Useful if you have your own closed form ik solution of your system and does not want to use ikfast or other plugins.

Simply add this repository to your ros workspace src and hit catkin_make.

```bash
#open workspace src folder
cd catkin_ws/src/
git clone https://github.com/SentryGA/moveit_bot_kinematics_plugin.git
cd ..
catkin_make
```

the portions to change/modify for your system:
1. include/bot_kinematics/bot_kinematics.h:
   the parameters used , the naming scheme are all set here. read the comments for further info.
2. src/moveit_bot_kinematics_plugin.cpp:
   the setBotParameter function also be needed to modify according to the dh parameters you use.

How to use the plugin:

When you create the moveit package for your bot select the new plugin when you create the planning group.

Since urdf does not provide the dh parameters, they have to be given seperately in the config/kinematics.yaml file of the moveit package you crearted for your bot add the following for each planning group:
```yaml
planning_group:
   kinematics_solver_dh_parameters:
     a1: 0.09
     a2: 0.88
     l2: 0.07
     t1: 0.002
```
The parameters specified above are arbitrary, and is your choice.
After the above procedure now you can run the demo.launch or corresponding launch files

Thanks to Jeroen(https://github.com/JeroenDM), we developed this from his repository https://github.com/JeroenDM/moveit_opw_kinematics_plugin.
Also thanks to Jonathan Meyer(https://github.com/Jmeyer1292) for his repository https://github.com/Jmeyer1292/opw_kinematics which was used to develop the include files.
