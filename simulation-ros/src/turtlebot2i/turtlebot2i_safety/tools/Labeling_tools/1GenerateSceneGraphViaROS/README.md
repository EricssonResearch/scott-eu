# Scene graph generator (ROS)

Please note that "sg_generator_for_labeling" is basically same as "sg_generator", but the generating frequency was set to 2Hz (fixed).

The max. frequency on Enyu's machine is 3 Hz without running navigation node.

The frequency of "sg_generator" should be configured carefully, but here "sg_generator_for_labeling" don't need to be real-time.

- This package is created by Shaolei Wang and Alberto Hata.
- Enyu just edited the files, so that it can work under ROS.
- Please run sg_generator.py
- Dependency:  
1. vrep.py
2. vrepConst.py
3. the appropriate remote API library: "remoteApi.so" (Linux)

When you detect any error, first make sure you have all files, especially "remoteApi.so". Becasue of gitignore, one have to manually add this file. When you copy the files, it is very likely to make such a mistake.

