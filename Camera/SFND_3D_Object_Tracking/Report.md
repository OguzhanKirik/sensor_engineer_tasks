Task1: 
Implemented the method "matchBoundingBoxes" in the  camFusion_Student.cpp file.

Task2:
Implemented the method "computeTTCLidar"  in the  camFusion_Student.cpp file.

Task3:
Implemented the method "clusterKptMatchesWithROI"  in the  camFusion_Student.cpp file.

Task4:
Implemented the method "computeTTCCamera" in the  camFusion_Student.cpp file.



Evaluation 1
Comparing these two images, the shortest collision time occurs when there are more points concentrated around the middle of the car. In contrast, the other image shows more points distributed towards the edges of the car. This discrepancy can be compensated for by calculating the median.

Challenges of the estimating collusion time based on  Lidar data:
In real-world driving scenarios, vehicle motion is rarely perfectly linear or constant; actions such as acceleration, braking, or lane changes frequently violate the assumptions of the constant velocity model, leading to inaccurate Time-to-Collision (TTC) estimates. This model is particularly sensitive to measurement noise—minor errors in bounding box tracking or keypoint correspondences can significantly distort TTC calculations. Additionally, its reliability is heavily dependent on frame rate: at lower frame rates, the assumption of constant velocity becomes less valid, as there is more time for velocity changes to occur between frames, further reducing estimation accuracy.

<img src="images/Final Results : TTC_screenshot_23.05.2025.png" width="779" height="414" />
<img src="images/Final Results : TTC_screenshot_23.05.2025_2.png" width="779" height="414" />


Evaluation 2
Conducted all experimetns with the detector and descriptors from the pervious exercise.


-> for results, check the report.xlsx file
