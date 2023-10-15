# similarity_rate_based_switching
A similarity rate-based switching decision mechanism for docking problem of autonomous mobile robots

In this repository a multi-stage localization framework including delivery and docking stages is proposed. An AMR can be localized using SA-MCL algorithm in the delivery stage and scan matching based localization algorithm in the docking stage. The switching point can be automatically determined using correntropy based decision mechanism. 

The following SIMULINK blocks are given in this repository:

_1) Correntropy based similarity rate estimator_

![](https://github.com/yilmazabdurrah/similarity_rate_based_switching/blob/master/figures/SimilarityRateEstimatorSIMULINK_v02.png?raw=true)

_2) Active localizer decision mechanism_

![](https://github.com/yilmazabdurrah/similarity_rate_based_switching/blob/master/figures/DecisionMechanismSIMULINK_v02.png?raw=true)

_3) Precise localization for docking stage_

![](https://github.com/yilmazabdurrah/similarity_rate_based_switching/blob/master/figures/SimilarityRateEstimatorSIMULINK_v02.png?raw=true)

PS: These blocks can be directly used as ROS nodes or employed to generate standalone ROS nodes from SIMULINK.

_4) Coarse localization for delivery stage_ 

![](https://github.com/yilmazabdurrah/similarity_rate_based_switching/blob/master/figures/SA-MCL.png?raw=true)
