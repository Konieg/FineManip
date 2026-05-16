# Pinocchio 功能展示 Demo

这个 demo 用一条完整链路展示 Pinocchio 的核心能力：

1. 加载 URDF / 内置样例机械臂
2. 读取模型信息：`nq`、`nv`、joint、frame
3. 正运动学 FK：计算末端位姿
4. 雅可比 Jacobian：查看末端速度映射和奇异值
5. 动力学：质量矩阵、重力补偿力矩、RNEA
6. 逆运动学 IK：用 CLIK 从 A 点求到 B 点
7. 插值轨迹，并用 Meshcat 可视化播放

## 安装依赖

```bash
pip install numpy meshcat
# Pinocchio 推荐用 conda-forge 安装：
conda install pinocchio -c conda-forge
```

## 用你的机械臂 URDF 运行

```bash
python pinocchio_feature_demo.py --urdf "D:/example_urdf/manipulator.urdf" --package-dir "D:/example_urdf" --ee-frame link6
```

## 不打开可视化，只看终端输出

```bash
python pinocchio_feature_demo.py --urdf "D:/example_urdf/manipulator.urdf" --package-dir "D:/example_urdf" --ee-frame link6 --no-viewer
```

## 没有 URDF 时，先用内置样例跑通

```bash
python pinocchio_feature_demo.py --sample --no-viewer
```

## demo

* - Pinocchio 先把 URDF 解析成 `model`、`collision_model`、`visual_model`。

- `model` 负责运动学和动力学计算，`data` 是每次计算时复用的缓存。
- FK 用来从关节角得到末端位姿。
- Jacobian 连接关节速度和末端空间速度，是 IK 和控制的基础。
- RNEA / CRBA 展示动力学能力，可以算重力补偿、逆动力学和质量矩阵。
- 最后用 IK 求出到目标点的关节角，再用 Meshcat 播放 A → B 轨迹。
