2D CFAR Algorithm Description
1.Select a number of training cells and guard cells around each Cell Under Test (CUT).

2.For each CUT, estimate the average noise level by summing the signal levels of all training cells, after converting the values from dB to linear scale using db2pow.

3.Exclude guard cells and the CUT itself when estimating the noise level.

4.After averaging the noise, convert the value back to dB using pow2db and add an offset (in dB) to set the detection threshold.

5.Compare the CUT value to the threshold:

6.If the CUT value is greater than the threshold, declare a detection (set to 1).

7.Otherwise, no detection.
