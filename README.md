## 使用方法

### 依赖
- ROS2：建议使用鱼香ros，选择ros2-humble安装
- rosbridge
- opencv3.5.4
- jsoncpp
- pybind11
- python flask

### 编译
```sh
mkdir build && cd build
cmake ..
make -j8
```

### 运行
首次运行需要在AutoAim_HUST_2024目录下执行


```sh
chmod 777 ./start_sim.sh
```


给启动脚本设置权限
```sh
sudo chmod 777 ../start_sim.sh
```

在build文件内启动

```sh
../start_sim.sh
```

新建一个终端启动rosbridge
```sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml 
```

## TODO

- [X] 静态文件相对路径部署

- [ ] tracker多目标时会出现Segmentation fault，暂时未定位问题源

- [ ] 打开监视网页后程序会报错，不打开网页正常

- [ ] predictor和driver未完成

- [ ] 参数读取json未添加

- [X] tracker的运动状态分类器数据集未制作
