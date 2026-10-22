# MPPI measurement delegate

仅在实验 Nav2 overlay 中，将 `FollowPath.plugin` 指向本插件。它加载已安装的 `nav2_mppi_controller::MPPIController`，使用相同父节点、插件名 `FollowPath`、TF、costmap、路径、速度、goal checker 和参数调用原实现，原样返回命令及异常。其余生命周期、setPlan、setSpeedLimit 同样委托。

新增观测：setPlan 返回后的完整路径、每次实际输入位姿/速度、steady-clock 原 MPPI 计算耗时。发布观测引入少量实验开销；计时区间不含前置输入发布和后置计时发布。正常导航不加载本插件。后续新控制器比较应使用相同测量边界与反馈/底盘链。
