# armor_processor

- [armor_processor](#armor_processor)
  - [ArmorProcessorNode](#armorprocessornode)
  - [Tracker](#tracker)
  - [KalmanFilter](#kalmanfilter)

## ArmorProcessorNode
装甲板处理节点

装甲板处理节点订阅识别节点发布的装甲板目标及机器人的坐标转换信息，将装甲板目标通过 `tf` 其变换到世界坐标系下，然后将目标送入跟踪器中得到最终目标在世界坐标系下的位置及速度并发布出来。

包含[Tracker](#tracker)

订阅：
- 已识别到的装甲板 `/detector/armors`
- 机器人的坐标转换信息
  - `/tf`
  - `/tf_static`

发布：
- 最终锁定的目标 `/processor/target`

参数：
- 跟踪器参数 tracker
  - 两帧间目标可匹配的最大距离 max_match_distance
  - `detecting` 状态进入 `tracking` 状态的阈值 tracking_threshold
  - `tracking` 状态进入 `no_found` 状态的阈值 lost_threshold

## Tracker
跟踪器

包含[KalmanFilter](#kalmanfilter)

## KalmanFilter
卡尔曼滤波器
