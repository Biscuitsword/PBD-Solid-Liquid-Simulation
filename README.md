# PBD-Solid-Liquid-Simulation
Simulating the physical effects of solid-liquid coupling based on the project https://github.com/Scrawk/PBD-Fluid-in-Unity

本项目基于 https://github.com/Scrawk/PBD-Fluid-in-Unity项目中的流体系统，实现了基于PBD算法的固液双向耦合系统。

受Scrawk项目启发，基于Unity中的ComputeShader，实现了在GPU上的运算。

实现效果：

1. 固体对液体的单向作用：通过操作移动刚体，能够推动或截断水流![单向](https://github.com/Biscuitsword/PBD-Solid-Liquid-Simulation/blob/main/Demonstration/%E5%8D%95%E5%90%91.png)
2. 固液双向耦合作用：将刚体抛入水中，溅起水花且刚体在水面沉浮![双向]([D:\大四上\课程设计\PBD-Solid-Liquid-Simulation\Demonstration\双向.png](https://github.com/Biscuitsword/PBD-Solid-Liquid-Simulation/blob/main/Demonstration/%E5%8F%8C%E5%90%91.png))



代码使用：

​	直接下载本项目，使用Unity(2020.3.13f1c1)打开即可



工程简介：

在流体计算框架的基础之上，自行开发实现了双向流固耦合系统，增添了刚体粒子数据结构、刚体耦合求解器、刚体GPU运算；修改了网格空间哈希、耦合流体求解器、流体GPU计算，以及上层控制脚本。主要结构示意图如下。

![工程]([D:\大四上\课程设计\PBD-Solid-Liquid-Simulation\Demonstration\工程.png](https://github.com/Biscuitsword/PBD-Solid-Liquid-Simulation/blob/main/Demonstration/%E5%B7%A5%E7%A8%8B.png))



存在问题：

流固耦合体系中，存在一定的抖动和不合理的翻滚现象，推测原因有:

（1）刚体ShapeMatching算法修正姿态后,仍与流体粒子存在较多穿透，导致持续受力且与重力产生的约束处于动态均衡。

（2）刚体粒子穿透流体粒子时，设计的恢复系数不够合理，对所有粒子使用均一的恢复系数导致修正不符合常见规律。

（3）PBD仿真本身是物理不完备的仿真算法，因而对参数调整要求较高，在参数设计上需要更多尝试。

