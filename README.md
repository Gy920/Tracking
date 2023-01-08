# Tracker

## 文件目录

            ├── demo
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