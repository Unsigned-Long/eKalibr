<div style="text-align: center;">
    <img src="../img/logo.svg" style="width: 100vw; height: auto;">
</div>

---

<h3 align="center">Tutorial: General Calibration Procedure in eKalibr</h3>
<p align="center">
    <a href="https://github.com/Unsigned-Long"><strong>Author » Shuolong Chen</strong></a>
</p>

---

<p align="left">
    <a><strong>Focus Adjustment and Create Circle Grid Board »</strong></a>
</p> 



Firstly, please adjust the focus of your event camera according to your preferences, ensuring it is properly focused. We recommend using a special pattern, such as the [Back Focus Pattern](../img/backfocus.pdf), for this purpose.

Next, please create a circular grid pattern. We provide a [program](../../script/gen_pattern.py) and [example](../../script/example.sh) for generating the circular grid pattern in `eKalibr`; feel free to use it as needed. The most important consideration is to ensure that the printed size of the circular grid pattern matches the actual dimensions (consistent with the settings used during the generation process).

**Attention:** Note that although both the asymmetric and symmetric circle grids are supported in `eKalibr`, only the asymmetric circle grid works in multi-camera spatiotemporal calibration, as it does not exhibit 180-degree ambiguity (see [here](https://ww2.mathworks.cn/help/vision/ug/calibration-patterns.html) for more explanations). For event-based intrinsic calibration, both the asymmetric and symmetric circle grids work.



<p align="left">
    <a><strong>Collect Dynamic Sensor Data »</strong></a>
</p> 


An event camera generates events only when there is a change in luminosity. Therefore, to calibrate the event camera, the user needs to position the camera towards a calibration board and move it to capture data. It is recommended to use visualization software compatible with the event camera during data collection to monitor the quality of the data in real-time (ensuring that the target remains within the camera’s field of view). We suggest that users refer to our [open-source data](https://drive.google.com/drive/folders/13Qd08xRzj5DgvpTplmUMfo9EgQ6P_WzM?usp=sharing) or the demo [video](https://www.bilibili.com/video/BV1yKc1e8Edc) we have created to replicate the procedures for data collection.

**Attention:** Excited motion (dynamic motion) is also required in event-based multi-camera spatiotemporal calibration for spatiotemporal observability. Please ensure that the camera motion is sufficiently excited (6-DoF) during data collection. However, avoid excessive movements that may cause difficulties in keeping the checkerboard pattern within the camera's field of view.



<p align="left">
    <a><strong>Write An Adaptable Configure File »</strong></a>
</p> 


A template of configure file has been provided [here](../../config/ekalibr-config.yaml). The detailed notes are also provided. We highly recommend you to read it before performing configuring and further solving, if it's your first time.



<p align="left">
    <a><strong>Perform Calibration Using eKalibr »</strong></a>
</p> 
Finally, we can launch the `ekalibr` ros node to perform intrinsic calibration for a given event camera, or multi-camera spatiotemporal calibration if multiple event cameras are integrated. Just run the following command:

```sh
roslaunch ekalibr ekalibr-prog.launch config_path:="path_of_your_config_file"
```
