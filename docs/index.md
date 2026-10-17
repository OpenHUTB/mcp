title: 主页



# [OpenHUTB Page 模板](https://github.com/OpenHUTB/template)

欢迎使用 OpenHUTB 的 Github Page 模板文档 [@he2024dynsyn]。





- [简介](#introduction)
    - [mkdocs简介](#mkdocs)
    - [播放视频](#videos)
- [表格](#table)
    - [普通表格](#普通表格)
    - [跨越多行和多列](#跨越多行和多列)
    
    

---


## 1. 简介 <span id="introduction"></span>


### 1.1 [__mkdocs简介__](./mkdocs.md) <span id="mkdocs"></span>

* mermaid 图表渲染支持

```mermaid
graph LR
    K[键盘输入] -->|pynput 监听| T[drone_ros_teleop 键盘发布节点]
    T -->|/drone/cmd_vel Twist 10 Hz| B[drone_ros_node 桥接控制节点]
    B -->|AirSim RPC 41451| A[Windows 宿主机 AirSim 模拟器]
```

* [播放视频](videos.md) <span id="videos"></span>

* 显示图片 

```
![](./img/action_page.jpg)
```

![](./img/action_page.jpg)



## 2. 表格 <span id='table'></span>

### 普通表格

| 命令  | 描述 |
|-------|--------|
| `make help`     | 打印所有可用的命令。  |

### 跨越多行和多列

```text
+---------+----------+------+
| 控制方案|    值    | 动作 |
+=========+==========+======+
|         |   `[0]`  |向前走|
+         +----------+------+
|  离散   |   `[1]`  |向后走|
+  (`0`)  +----------+------+
|         |  `[2]`   | 右转 |
+         +----------+------+
|         |   `[3]`  | 左转 |
+---------+----------+------+
|   连续  | `forward_speed`,|
|  (`1`)  |   `rot_speed`   |
+---------+----------+------+
```
渲染为：

+---------+----------+------+
| 控制方案|    值    | 动作 |
+=========+==========+======+
|         |   `[0]`  |向前走|
+         +----------+------+
|  离散   |   `[1]`  |向后走|
+  (`0`)  +----------+------+
|         |  `[2]`   | 右转 |
+         +----------+------+
|         |   `[3]`  | 左转 |
+---------+----------+------+
|   连续  | `forward_speed`,|
|  (`1`)  |   `rot_speed`   |
+---------+----------+------+

```text
+-------+----------+------+
| Table Headings   | Here |
+-------+----------+------+
| Sub   | Headings | Too  |
+=======+==========+======+
| cell  | column spanning |
+ spans +----------+------+
| rows  | normal   | cell |
+-------+----------+------+
| multi | cells can be    |
| line  | *formatted*     |
|       | **paragraphs**  |
| cells |                 |
| too   |                 |
+-------+-----------------+
```

渲染为：

+-------+----------+------+
| Table Headings   | Here |
+-------+----------+------+
| Sub   | Headings | Too  |
+=======+==========+======+
| cell  | column spanning |
+ spans +----------+------+
| rows  | normal   | cell |
+-------+----------+------+
| multi | cells can be    |
| line  | *formatted*     |
|       | **paragraphs**  |
| cells |                 |
| too   |                 |
+-------+-----------------+





___

如果对文档中的任何问题可以在 [本文档的源码仓库](https://github.com/OpenHUTB/templte) 中的 [问题](https://github.com/OpenHUTB/templte/issues) 页面讨论或者提交 [拉取请求](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md) 直接修改文档。

## 参考文献
