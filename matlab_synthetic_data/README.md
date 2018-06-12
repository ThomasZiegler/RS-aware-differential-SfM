Installation guide to use the MATLAB code to generate synthetic data

https://www.openu.ac.il/home/hassner/projects/poses/ and https://www.openu.ac.il/home/hassner/projects/poses/readme.txt

We used Ubuntu 16.04 LTS and Matlab 2014a (thus following the instructions given for R2014a). 
We worked with the precompiled version of the renderer and did not use the calibration part of the code.
Getting the texture to load was a bit tricky for us. Using an .osg model and running "apt-get install libjpeg62" worked for us.

Read the given readme carefully to understand how the renderer works.


HOW-TO:

Run the file castle.m with the desired motion parameters while your MATLAB path is 
placed in the installtion directory of the renderer (where the .mexa64 file is).
The .osg model should be in the same directory.