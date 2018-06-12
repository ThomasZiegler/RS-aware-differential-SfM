
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

Run the file castle.m with the desired motion parameters while your MATLAB path is 
placed in the installtion directory of the renderer (where the .mexa64 file is).
The .osg model should be in the same directory.
