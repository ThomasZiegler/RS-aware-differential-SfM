# Synthetic data creation with Matlab renderer

## Download
The renderer can be downloaded here: https://www.openu.ac.il/home/hassner/projects/poses/ 

Please read also the given readme file carefully to understand how the renderer works.
https://www.openu.ac.il/home/hassner/projects/poses/readme.txt

We used Ubuntu 16.04 LTS and Matlab 2014a (thus following the instructions given for R2014a). We worked with the precompiled version of the renderer and did not use the calibration part of the code.

We had some problem to load the texture. What helped was using a .osg model and installin the libjpeg62 package.
```
$ sudo apt install apt-get install libjpeg62
```


## HOW-TO:

Run the file castle.m with the desired motion parameters. By default it will generate the data used for figure 7 in the project report (parameter sweeps).

### Note:
- Your MATLAB path is placed in the installtion directory of the renderer (where the .mexa64 file is).
- The .osg model should be in the same directory.
- You can adjust the output folder in the file start_generating.m
- Setting the flag "take_video" to 1 in the file start_generating.m the renderer will store every frame and not just the RS frames
- You can adjust the intial pose of the renderer in the file setup_renderer.m. By default the virtual camera is positioned in front of the castle gate. 
- Using the setting "vertical_lines" it will generate the images shown in figure 4 (results on synthetic data)



## Data set

We used the "castle" 3D mesh which can be downloaded from the following site: https://cvg.ethz.ch/research/rolling-shutter-stereo/
