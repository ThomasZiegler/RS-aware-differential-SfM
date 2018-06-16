# Example Data Sets

We provide some sample data which both real world and synthetic with which the functionality of the algorithm can be tested. 
**Note that the files need to be extracted before usage.**

# Usage

## Real World Data
The following samples can be used in the *evaluateSingleRun()* method by uncomment the corresponding lines in the code. 

- Example 1: Images used for figure 6 in report. Recorded at Buecheggplatz in Zuerich. Rectification works. 
- Example 2: Images used for figure 1 in report. Recorded at ETH Höngerberg in Zuerich. Rectification works.
- Example 3: Images used for figure 7 in report. Recorded at ETH Zentrum in Zuerich. Rectification works.
- Example 4: Images taken at ETH Höngerberg in Zuerich. Rectification **does not** work due to wrong flow estimate from Deep Flow.
- Example 5: Images taken at ETH Höngerberg in Zuerich. Rectification works partly with some artifacts due to wrong seperation between foreground and background in the flow estimate.

## Synthetic Data
The following samples can be used in the *evaluateSingleRun()* method by uncomment the corresponding lines in the code. 

- Example 1: Images with the following motion parameters: v=[0.03;0.03;0]; w=[0;0;0.5]; k=0; gamma=0.8
- Example 2: Images with the following motion parameters: v=[0.01;0.01;0]; w=[0;0;0.5]; k=0; gamma=0.8
- Example 3: Images with the following motion parameters: v=[0.07;0.07;0]; w=[0;0;0.5]; k=0; gamma=0.8
- Example 4: Images with the following motion parameters: v=[0.03;0.03;0]; w=[0;0;1]; k=0; gamma=0.8
- Example 5: Images with the following motion parameters: v=[0.03;0.03;0]; w=[0;0;2]; k=0; gamma=0.8

The following parameter sweep is used as default in the *evaluateParameterSweep()* method. 
- Example sweep: Sweep over the k values from 0 to 1.25 in 0.25 steps. The remaining motion parameters are: w=[0;0;0.5]; k=0; gamma=0.8
