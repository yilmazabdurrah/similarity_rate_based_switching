# similarity_rate_based_switching
A similarity rate-based switching decision mechanism for the docking problem of autonomous mobile robots (AMRs)

In this repository, a ROS-compatible multi-stage localization framework including delivery and docking stages is proposed. An AMR can be localized using the SA-MCL algorithm in the delivery stage and the scan-matching-based localization algorithm in the docking stage. The switching point can be automatically determined using a correntropy-based decision mechanism. 

The overall framework can be described as follows:

![](https://github.com/yilmazabdurrah/similarity_rate_based_switching/blob/master/figures/Framework.png?raw=true)

The following ROS-compatible SIMULINK blocks and ROS packages are given in this repository:

_1) Correntropy based similarity rate estimator_

![](https://github.com/yilmazabdurrah/similarity_rate_based_switching/blob/master/figures/SimilarityRateEstimatorSIMULINK_v02.png?raw=true)

_2) Active localizer decision mechanism_

![](https://github.com/yilmazabdurrah/similarity_rate_based_switching/blob/master/figures/DecisionMechanismSIMULINK_v02.png?raw=true)

_3) Precise localization for docking stage_

![](https://github.com/yilmazabdurrah/similarity_rate_based_switching/blob/master/figures/ScanMatcher_v21.png?raw=true)

PS: These blocks can be directly used as ROS nodes or employed to generate standalone ROS nodes from SIMULINK.

_4) Coarse localization for delivery stage_ 

![](https://github.com/yilmazabdurrah/similarity_rate_based_switching/blob/master/figures/SA-MCL.png?raw=true)

Please refer to the following [manuscript](https://doi.org/10.1017/S0263574724000602) for further details and cite it for academic works

```
@article{yilmaz2024robotica,
  title={A Multi-stage Localization Framework for Accurate and Precise Docking of Autonomous Mobile Robots (AMRs)},
  author={Yilmaz, Abdurrahman and Temeltas, Hakan},
  journal={Robotica},
  year={2024},
  publisher={Cambridge University Press},
  doi={10.1017/S0263574724000602}
}
```


