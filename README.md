# 机器人/嵌入式系统/人工智能开源项目列表

- 本页面收录优秀的机器人、嵌入式系统、人工智能相关的开源项目。
- 目前仅收录中文开源及华人发起的项目。
- 本列表由 云飞机器人实验室 ( [B站](https://space.bilibili.com/493264461) | [知乎](https://www.zhihu.com/column/yfworld) | [博客](https://yfrobotics.github.io/) | [YouTube](https://www.youtube.com/@yfrobotics) | [Instagram](https://www.instagram.com/yfrobotics/) ) 长期维护与更新。
- 欢迎贡献：如果你希望推荐别的项目加入该列表，可以在项目的 [GitHub](https://github.com/yfrobotics/awesome-robotics-ee-opensource) 上 [commit issue](https://github.com/yfrobotics/awesome-robotics-ee-opensource/issues) 或者 clone & pull request。


## 1. 机器人项目 | Robots

| 项目名称                      | 发起人/作者             | 项目地址                                                     | 项目介绍                                                     |
| ----------------------------- | ----------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| XR-1 VLA模型                  | 北京人形机器人创新中心  | [GitHub](https://github.com/Open-X-Humanoid/XR-1)            | 中国首个国标级VLA大模型，支持跨机器人平台操作，包含RoboMIND 2.0数据集和ArtVIP资产。 |
| DexVLA                        | 多机构联合             | [GitHub](https://github.com/juruobenruo/DexVLA)              | 基于Qwen2-VL的视觉-语言-动作模型，支持单臂、双臂和灵巧手等多种机器人形态的通用控制。 |
| ManiFoundation                | 新加坡国立大学/清华大学 | [GitHub](https://github.com/NUS-LinS-Lab/ManiFM)             | 通用机器人操作基础模型，通过接触合成实现对刚性、铰接和可变形物体的操作。 |
| AgiBot X1开源人形机器人        | 智元机器人 (AgibotTech)  | [推理](https://github.com/AgibotTech/agibot_x1_infer) \| [训练](https://github.com/AgibotTech/agibot_x1_train) \| [硬件](https://github.com/AgibotTech/agibot_x1_hardware) | 智元机器人X1完整开源人形机器人项目，包含推理模块、强化学习训练代码和全套硬件设计资料。 |
| AgiBot-World                  | OpenDriveLab (上海AI实验室) | [GitHub](https://github.com/OpenDriveLab/AgiBot-World)      | IROS 2025最佳论文候选，面向可扩展和智能具身系统的大规模操控平台。 |
| unitree_rl_gym                | 宇树科技 (Unitree)     | [GitHub](https://github.com/unitreerobotics/unitree_rl_gym)   | 宇树科技四足/人形机器人强化学习训练框架，基于Isaac Gym。      |
| ManiSkill                     | haosulab (Hillbot)     | [GitHub](https://github.com/haosulab/ManiSkill)               | SAPIEN操控技能框架，GPU并行化机器人仿真器和基准测试平台。     |
| Awesome-Robotics-Foundation-Models | robotics-survey    | [GitHub](https://github.com/robotics-survey/Awesome-Robotics-Foundation-Models) | 机器人基础模型研究论文和项目汇总，包括RT-1、RT-2、OpenVLA等。 |
| awesome-3dcv-papers-daily     | 3D视觉工坊              | [GitHub](https://github.com/qxiaofan/awesome-3dcv-papers-daily) | 主要记录计算机视觉、VSLAM、点云、结构光、机械臂抓取、三维重建、深度学习、自动驾驶等前沿paper与文章。 |
| 视觉SLAM十四讲                | 高翔                    | [GitHub](https://github.com/gaoxiang12/slambook2)             | SLAM领域经典中文教程及配套代码，视觉SLAM入门必读。           |
| VINS-Mono                     | 香港科技大学            | [GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) | 鲁棒通用的单目视觉惯性状态估计器，VIO/SLAM领域经典项目。     |
| VINS-Fusion                   | 香港科技大学            | [GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | 基于优化的多传感器状态估计器，支持单/双目相机+IMU融合。       |
| LIO-SAM                       | TixiaoShan             | [GitHub](https://github.com/TixiaoShan/LIO-SAM)              | 紧耦合激光惯性里程计（通过平滑和建图），被广泛引用的LiDAR SLAM方案。 |
| FAST_LIO                      | 港大MARS实验室          | [GitHub](https://github.com/hku-mars/FAST_LIO)               | 计算高效且鲁棒的LiDAR惯性里程计，港大MARS实验室代表作。      |
| FAST-LIVO2                    | 港大MARS实验室          | [GitHub](https://github.com/hku-mars/FAST-LIVO2)             | 快速、直接的LiDAR-惯性-视觉里程计，多传感器紧耦合方案。      |
| R3LIVE                        | 港大MARS实验室          | [GitHub](https://github.com/hku-mars/r3live)                 | 鲁棒、实时的RGB彩色LiDAR-惯性-视觉紧耦合状态估计与建图。    |
| Apollo自动驾驶平台             | 百度                    | [GitHub](https://github.com/ApolloAuto/apollo)               | 百度Apollo开源自动驾驶平台，国内最大的自动驾驶开源生态系统。  |
| UniAD                         | OpenDriveLab (上海AI实验室) | [GitHub](https://github.com/OpenDriveLab/UniAD)             | CVPR 2023最佳论文，面向规划的统一自动驾驶框架，整合感知、预测和规划。 |
| BEVFormer                     | 上海AI实验室/南京大学   | [GitHub](https://github.com/fundamentalvision/BEVFormer)      | ECCV 2022，基于纯相机的BEV感知框架，用于3D目标检测和语义地图分割。 |
| 超迷你机械臂机器人项目        | 稚晖君 (peng-zhihui)    | [GitHub](https://github.com/peng-zhihui/Dummy-Robot)         | 视频介绍：[【自制】我造了一台 钢 铁 侠 的 机 械 臂 ！【硬核】](https://www.bilibili.com/video/BV12341117rG) |
| MiniRover火星车               | 稚晖君 (peng-zhihui)    | [GitHub](https://github.com/peng-zhihui/MiniRover-Hardware)  | 自制火星车的开源资料。                                       |
| X-Bot智能机械臂写字机器人     | 稚晖君 (peng-zhihui)    | [GitHub](https://github.com/peng-zhihui/X-Bot)               | 基于CoreXY结构的机械臂。                                     |
| ONE-Robot独轮机器人           | 稚晖君 (peng-zhihui)    | [GitHub](https://github.com/peng-zhihui/ONE-Robot)           | 基于IMU和STM32的独轮自平衡机器人。                           |
| ElectronBot迷你桌面机器人     | 稚晖君 (peng-zhihui)    | [项目主页](https://github.com/peng-zhihui/ElectronBot)       | 非常小巧的桌面机器人。                                       |
| 解魔方机器人                  | 动力老男孩              | [项目主页](http://www.diy-robots.com/?page_id=46)            | 基于乐高的解魔方机器人。                                     |
| RoboWiki (云飞机器人中文百科) | 云飞机器人实验室        | [GitHub](https://github.com/yfrobotics/robowiki)             | 机器人领域的维基百科（公共知识编辑）。                       |
| 基于树莓派的目标识别与追踪    | 云飞机器人实验室        | [GitHub](https://github.com/automaticdai/rpi-object-detection) | 基于树莓派 + Web Camera的视觉追踪项目。                      |
| OpenCat                       | Petoi                   | [GitHub](https://github.com/PetoiCamp/OpenCat)               | 开源四足机器人平台                                           |
| vlm_arm                       | 同济子豪兄 (TommyZihao) | [GitHub](https://github.com/TommyZihao/vlm_arm)              | 机械臂+大模型+多模态                                         |
| 小觅双目相机系列               | 小觅智能 (MYNTAI)      | [GitHub](https://github.com/slightech/MYNT-EYE-S-SDK)          | 小觅双目相机系列，提供完整的SLAM和视觉算法解决方案。       |
| 大疆Tello无人机SDK             | 大疆创新 (DJI)         | [GitHub](https://github.com/dji-sdk/Tello-Python)              | 大疆Tello系列无人机的Python SDK，支持编程控制和图像处理。  |
| Prometheus自主无人机系统       | 阿木实验室 (amov-lab)   | [GitHub](https://github.com/amov-lab/Prometheus)               | 面向自主无人机的开源软件系统，支持目标检测、SLAM导航、编队控制等。 |
| EGO-Planner                   | 浙江大学FAST实验室      | [GitHub](https://github.com/ZJU-FAST-Lab/ego-planner)         | 高效的无人机梯度引导在线局部规划器。                         |
| XTDrone无人机仿真平台          | robin-shaun            | [GitHub](https://github.com/robin-shaun/XTDrone)               | 基于PX4、ROS和Gazebo的无人机仿真平台，支持集群仿真。        |
| Unitree Qmini开源双足机器人    | 宇树科技 (Unitree)     | [GitHub](https://github.com/unitreerobotics/Qmini)             | 开源双足平台，提供全套BOM/装配指南、RoboTamer4Qmini控制框架与URDF模型。 |
| 宇树科技四足机器人             | 宇树科技 (Unitree)     | [GitHub](https://github.com/unitreerobotics/unitree_ros)       | 宇树科技四足机器人Go1/Go2的ROS驱动包。                     |
| 小米CyberDog开源四足机器人     | 小米科技               | [GitHub](https://github.com/MiRoboticsLab/cyberdog_ros2)       | 小米CyberDog四足机器人的开源软件和硬件资料。               |
| 浙江大学FAST实验室无人机项目    | 浙江大学FAST实验室     | [GitHub](https://github.com/ZJU-FAST-Lab/Fast-Drone-250)       | 250mm自主无人机的硬件和软件设计。                           |
| 香港科技大学空中机器人项目      | 香港科技大学           | [GitHub](https://github.com/HKUST-Aerial-Robotics/FIESTA)      | 空中机器人在线运动规划的快速增量欧几里得距离场。           |
| Genesis                        | 胡渊鸣等 (CMU)           | [GitHub](https://github.com/Genesis-Embodied-AI/Genesis)        | 面向机器人与具身AI的通用物理仿真引擎，纯Python实现，速度可达传统引擎数万倍，支持多种材料仿真。 |
| RoboticsDiffusionTransformer (RDT-1B) | 清华大学          | [GitHub](https://github.com/thu-ml/RoboticsDiffusionTransformer) | 双臂机器人操控基础模型，采用扩散Transformer架构，在多种双臂操控任务上取得SOTA效果。 |
| Open-TeleVision                | 多校联合                  | [GitHub](https://github.com/OpenTeleVision/TeleVision)           | 基于VR头显的沉浸式机器人遥操作系统，操作者通过第一视角实时控制机器人双臂完成灵巧操作。 |
| AnyGrasp                       | 上海AI实验室              | [GitHub](https://github.com/graspnet/anygrasp_sdk)               | 高效通用的6自由度抓取位姿估计算法，支持任意物体的机器人抓取检测。 |
| Humanoid-Gym                   | 多校联合 (RoboterAX等)    | [GitHub](https://github.com/roboterax/humanoid-gym)              | 基于Isaac Gym的人形机器人强化学习训练框架，支持零样本迁移至真实机器人。 |

## 2. 嵌入式系统项目 | Embedded System

| 项目名称                            | 发起人/作者                 | 项目地址                                                     | 项目介绍                                                     |
| ----------------------------------- | --------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| PocketLCD: 带充电宝功能的便携显示器 | 稚晖君 (peng-zhihui)        | [GitHub](https://github.com/peng-zhihui/PocketLCD)           | 介绍视频：[【自制】你的下一个显示器，可能是个充电宝？？](https://www.bilibili.com/video/BV17D4y1X7AT) |
| L-ink电子墨水屏NFC智能卡片          | 稚晖君 (peng-zhihui)        | [GitHub](https://github.com/peng-zhihui/L-ink_Card)          | 为了解决个人使用IC卡时遇到的一些痛点设计的一个迷你NFC智能卡片，基于STM32L051和ST25DV。 |
| 低成本激光投射虚拟键盘的设计制作    | 陈世凯 (CSK)                |                                                              | - [低成本激光投射虚拟键盘的设计制作-上(原理和硬件)](http://www.csksoft.net/blog/post/lowcost.laserkbd_part1.html) <br />- [低成本激光投射虚拟键盘的设计制作-下(算法与实现)](http://www.csksoft.net/blog/post/lowcost.laserkbd_part2.html) |
| 自制低成本3D激光扫描测距仪          | 陈世凯 (CSK)                | [Google Code](https://code.google.com/archive/p/rp-3d-scanner/) | - [自制低成本3D激光扫描测距仪(3D激光雷达)，第一部分](http://www.csksoft.net/blog/post/lowcost_3d_laser_ranger_1.html) <br />- [自制低成本3D激光扫描测距仪(3D激光雷达)，第二部分](http://www.csksoft.net/blog/post/lowcost_3d_laser_ranger_2.html) |
| NixieClock辉光管时钟                | Blanboom                    | [GitHub](https://github.com/blanboom/NixieClock)             | 支持蓝牙 4.0 的辉光管时钟。                                  |
| 3D8光立方                           | 官微宏 (aGuegu)             | [项目主页](http://aguegu.net/?page_id=99)                    | 8 x 8 LED光立方。                                            |
| Gameduino 2/3                       | ExCamera / 云飞机器人实验室 | [Gameduino 2 (KS)](https://www.kickstarter.com/projects/2084212109/gameduino-2-this-time-its-personal?ref=discovery&term=Gameduino) \| [Gameduino 3 (KS)](https://www.kickstarter.com/projects/2084212109/gameduino-3?ref=discovery&term=Gameduino) | Gameduino是基于Arduino的图形交互和游戏扩展版。它是目前Arduino平台上性能最好的图形协处理器。它由ExCamera在Kickstarter上成功众筹。云飞实验室参与了工具链开发、中文手册 [(点击这里下载)](http://excamera.com/files/gd2book_cn.pdf) 以及中文推广。 |
| 妖姬 – 增强现实电子植物             | 云飞机器人实验室            | [GitHub](https://github.com/automaticdai/arduino-yaoji)      | 妖姬是云飞实验室在极客大赛中的48小时极限创作作品。妖姬是一款概念式的互动电子植物，采用了Arduino + Android的方案，融合了信息与物理的概念式作品。 |
| YF Smart Home                       | 云飞机器人实验室            | [GitHub](https://github.com/yfrobotics/yf-home-iot)          | 云飞智能家居项目旨在探索新的智能家居系统解决方案。           |
| 树莓派温湿度气象站                  | 云飞机器人实验室            | [GitHub](https://github.com/automaticdai/rpi-environmental-sensing) | 基于树莓派的开源温湿度气象站。                               |
| ESP32智能家居开发板                | 乐鑫科技 (Espressif)      | [GitHub](https://github.com/espressif/esp-idf)                | ESP32系列芯片的官方开发框架和示例项目。                     |
| LicheeRV开发板项目                 | 矽递科技 (Sipeed)        | [GitHub](https://github.com/sipeed/LicheeRV-Nano-Build)       | LicheeRV-Nano的构建项目和开发工具。                         |
| MaixPy                             | 矽递科技 (Sipeed)        | [GitHub](https://github.com/sipeed/MaixPy)                    | 基于RISC-V平台的MicroPython实现，支持K210/K230等芯片上的嵌入式机器视觉与AI推理。  |

## 3. 处理器架构及操作系统 | Arch & OS


| 项目名称                    | 发起人/作者   | 项目地址                                             | 项目介绍                                                     |
| --------------------------- | ------------- | ---------------------------------------------------- | ------------------------------------------------------------ |
| 香山（XiangShan）开源处理器 | 中科院计算所  | [GitHub](https://github.com/OpenXiangShan/XiangShan) | 香山是一款开源的高性能 RISC-V 处理器。                       |
| RT-Thread                   | Bernard Xiong | [GitHub](https://github.com/RT-Thread/rt-thread)     | RT-Thread诞生于2006年，是一款以开源、中立、社区化发展起来的物联网操作系统。 |
| TencentOS Tiny              | 腾讯          | [GitHub](https://github.com/Tencent/TencentOS-tiny)  | 腾讯物联网终端操作系统（TencentOS tiny）是腾讯面向物联网领域开发的实时操作系统，具有低功耗，低资源占用，模块化，安全可靠等特点，可有效提升物联网终端产品开发效率。TencentOS tiny 提供精简的 RTOS 内核，内核组件可裁剪可配置，可快速移植到多种主流 MCU 及模组芯片上。而且，基于RTOS内核提供了丰富的物联网组件，内部集成主流物联网协议栈（如 CoAP/MQTT/TLS/DTLS/LoRaWAN/NB-IoT 等），可助力物联网终端设备及业务快速接入腾讯云物联网平台。 |
| AimRT | AimRT | [GitHub](https://github.com/AimRT/AimRT) | AimRT 是一个面向现代机器人领域的运行时开发框架。 它基于 Modern C++ 开发，轻量且易于部署，在资源管控、异步编程、部署配置等方面具有更现代的设计。AimRT 致力于整合机器人端侧、边缘端、云端等各种部署场景的研发。 它服务于现代基于人工智能和云的机器人应用，提供完善的调试和性能分析工具链，以及良好的可观测性支持。AimRT 还提供了全面的插件开发接口，具有高度可扩展性。 它与 ROS2、HTTP、Grpc 等传统机器人生态系统或云服务生态系统兼容，并支持对现有系统的逐步升级。|
| OpenHarmony | 华为/开放原子基金会 | [Gitee](https://gitee.com/openharmony) | 面向全场景智能终端的开源分布式操作系统，支持从嵌入式设备到手机等多种形态，已广泛应用于IoT和机器人产品。 |

## 4. 机器学习项目 | ML

| 项目名称     | 发起人/作者           | 项目地址                                                     | 项目介绍                                                     |
| ------------ | --------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| DeepSeek     | DeepSeek-AI           | [DeepSeek-V3](https://github.com/deepseek-ai/DeepSeek-V3) \| [DeepSeek-R1](https://github.com/deepseek-ai/DeepSeek-R1) | DeepSeek 模型是近年来在自然语言处理（NLP）领域备受瞩目的开源大规模语言模型系列。其最新版本 DeepSeek-V3 采用了混合专家（Mixture-of-Experts，MoE）架构，拥有 6710 亿个参数，每个词元（token）激活 370 亿个参数。该模型在多项基准测试中表现出色，性能媲美 GPT-4 和 Claude 等领先的闭源模型。 |
| PaddleOCR    | 百度                  | [GitHub](https://github.com/PaddlePaddle/PaddleOCR)          | 支持100+语言的OCR工具包，提供文字检测、识别、版面分析等全流程能力，是GitHub上最受欢迎的中国AI项目之一。 |
| ncnn         | 腾讯                  | [GitHub](https://github.com/Tencent/ncnn)                    | 高性能神经网络推理框架，针对移动端和嵌入式设备优化，广泛应用于机器人端侧AI推理。 |
| MNN          | 阿里巴巴              | [GitHub](https://github.com/alibaba/MNN)                     | 轻量级深度学习推理引擎，支持多种硬件后端，适用于移动端和边缘设备部署。 |
| MiniCPM-o    | 面壁智能 (OpenBMB)    | [GitHub](https://github.com/OpenBMB/MiniCPM-o)               | Gemini 2.5 Flash级别的多模态语言模型，支持视觉、语音和全双工多模态直播，可在手机端运行。 |
| YOLOX        | 旷视科技 (Megvii)     | [GitHub](https://github.com/Megvii-BaseDetection/YOLOX)      | 旷视科技开源的高性能无锚点YOLO目标检测器。                   |
| InternVL     | 上海AI实验室          | [GitHub](https://github.com/OpenGVLab/InternVL)              | 开源多模态视觉-语言模型，对标GPT-4o，支持图像理解和多模态推理。 |
| Yi           | 零一万物 (01.AI)      | [GitHub](https://github.com/01-ai/Yi)                        | 零一万物开源大语言模型系列，从头训练，支持中英双语。          |
| CogVLM2      | 智谱AI                | [GitHub](https://github.com/THUDM/CogVLM2)                   | GPT4V级别的开源多模态视觉语言模型。                          |
| Step1X-Edit  | 阶跃星辰 (StepFun)    | [GitHub](https://github.com/stepfun-ai/Step1X-Edit)          | SOTA开源图像编辑模型，性能对标GPT-4o和Gemini 2 Flash。       |
| TVM          | 陈天奇                | [GitHub](https://github.com/apache/tvm)                      | *Apache TVM* 是一个用于CPU、GPU 和机器学习加速器的开源机器学习编译器框架，旨在让机器学习工程师能够在任何硬件后端上高效地优化和运行计算。 |
| MxNet        | 李沐 (Mu Li)          | [GitHub](https://github.com/apache/incubator-mxnet)          | 深度学习编程框架，支持C++/Python/R。                         |
| Caffe2       | 贾扬清 (Yangqing Jia) | [GitHub](https://github.com/facebookarchive/caffe2)          | 深度学习编程框架，支持C++/Python/Matlab。现已与Pytorch合并。 |
| PaddlePaddle | 百度                  | [GitHub](https://github.com/PaddlePaddle/Paddle)             | 飞桨（PaddlePaddle）以百度多年的深度学习技术研究和业务应用为基础，集深度学习核心训练和推理框架、基础模型库、端到端开发套件、丰富的工具组件于一体，是中国首个自主研发、功能丰富、开源开放的产业级深度学习平台。 |
| DeepVision   | 稚晖君 (peng-zhihui)  | [GitHub](https://github.com/peng-zhihui/DeepVision)          | 本项目实现了移动端CV算法快速验证框架，旨在提供一套通用的CV算法验证框架。框架经过本人一年多的开发和维护，目前已经完成绝大部分API的开发，实现包括实时视频流模块、单帧图像处理模块、3D场景模块、云端推理模块等众多功能。 |
| MMDetection  | 商汤科技              | [GitHub](https://github.com/open-mmlab/mmdetection)          | 商汤的目标检测工具箱及基准测试                               |
| ChatGLM      | 智谱AI                | [GitHub](https://github.com/THUDM/ChatGLM-6B)                | 开源双语对话语言模型，支持中英文对话。                       |
| Qwen         | 阿里云                | [GitHub](https://github.com/QwenLM/Qwen)                     | 阿里云通义千问大语言模型系列。                               |
| Baichuan     | 百川智能              | [GitHub](https://github.com/baichuan-inc/Baichuan2)          | 百川智能开源大语言模型。                                     |
| InternLM     | 上海AI实验室          | [GitHub](https://github.com/InternLM/InternLM)               | 上海AI实验室开源的大语言模型。                               |
| Qwen2-VL     | 阿里云                | [GitHub](https://github.com/QwenLM/Qwen2-VL)                 | 阿里云通义千问多模态视觉-语言模型，广泛应用于VLA模型开发。   |
| Wan2.1       | 阿里巴巴              | [GitHub](https://github.com/Wan-Video/Wan2.1)                 | 阿里巴巴开源的高质量视频生成大模型，支持文生视频，性能对标商业顶级模型。            |
| HunyuanVideo | 腾讯                  | [GitHub](https://github.com/Tencent-Hunyuan/HunyuanVideo)     | 腾讯开源的高分辨率视频生成模型，支持文生视频和图生视频，视频质量业界领先。          |
| ControlNet   | 张吕敏 (Lvmin Zhang)  | [GitHub](https://github.com/lllyasviel/ControlNet)            | 为扩散模型添加条件控制的神经网络结构，支持姿态、深度、边缘等多种控制信号，极具影响力。 |
| AnimateDiff  | 郭宇威等              | [GitHub](https://github.com/guoyww/AnimateDiff)               | 即插即用的视频动画模块，无需额外训练即可将现有图像扩散模型转化为视频生成器。        |
| CogVideoX    | 智谱AI/清华大学        | [GitHub](https://github.com/THUDM/CogVideo)                   | 开源视频生成大模型，生成质量优秀，支持文生视频和图生视频。                        |
| Janus        | DeepSeek              | [GitHub](https://github.com/deepseek-ai/Janus)                | 统一多模态理解与生成的框架，通过解耦视觉编码解决理解与生成任务之间的冲突。          |
| GroundingDINO | IDEA Research        | [GitHub](https://github.com/IDEA-Research/GroundingDINO)      | 开放集目标检测框架，通过自然语言描述实现任意类别目标的定位，零样本检测能力强。       |
| FunASR       | 阿里达摩院            | [GitHub](https://github.com/modelscope/FunASR)                | 工业级端到端语音识别工具链，支持语音识别、标点恢复、说话人分离等任务。              |
| ChatTTS      | 2noise                | [GitHub](https://github.com/2noise/ChatTTS)                   | 专为对话设计的高质量中英文语音合成模型，支持细粒度韵律控制（停顿、笑声等）。        |

---

**其他优秀项目更多项目持续收集中... 欢迎在issue中投稿!**

---

本项目由 [云飞机器人实验室](https://yfrobotics.github.io/) 长期维护与更新。最后更新时间： 2026年2月20日
