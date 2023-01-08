# Tracker
---

## 介绍
    
&emsp; 本仓库主要实现了基于Sort的MOT的Tracking模块 ,并留下MOT的通用接口，待实现DeepSort等常用MOT算法

## 项目结构

    ├── demo      ： 测试
    │   ├── CMakeLists.txt
    │   └── TrackingDemo.cpp
    ├── include   
    │   ├── BatchTracker.hpp  : 基础单个跟踪器，由kf实现，可更改（比如改成kf+kcf的组合跟踪器）
    │   ├── Common.hpp        ：包含匈牙利算法（引用），和多个iou计算函数
    │   └── Tracking.hpp      ： 定义 Tracking 基类
    ├── README.md
    └── src
        ├── BatchTracker.cpp
        ├── Common.cpp
        └── Tracking.cpp

## 依赖环境

1. C++11

2. OpenCV >=4.4.0

## Build

    $  cd ./demo 
    $  mkdir build && cd build
    $  cmake ..
    $  make -j
    $  ./demo ../test_video.mp4

## 算法原理 (TODO)

- Sort

    1. 匈牙利算法

    2. kalman Filter

- DeepSort

    1. ReID

    2. 匈牙利算法

    3. kalman Filter

- ...

## Reference:

[1]. abewley/sort,https://github.com/abewley/sort

[2]. mcximing/sort-cpp,https://github.com/mcximing/sort-cpp


