"""
Pinocchio 功能展示 Demo
=======================

展示内容：
1. 加载 URDF 或内置样例机械臂
2. 打印模型信息 nq / nv / joints / frames
3. 正运动学 FK：计算末端位姿
4. 雅可比 Jacobian：计算末端速度映射与奇异值
5. 动力学 Dynamics：质量矩阵、重力补偿力矩、逆动力学 RNEA
6. 带速度/加速度的逆动力学：验证 tau = M(q)a + nle(q,v)，并用 ABA 反验证
7. 逆运动学 IK：用 CLIK 从 A 姿态求解到 B 姿态
8. A -> B 轨迹插值，并可选用 Meshcat 播放

运行示例：
python pinocchio_feature_demo.py --urdf "D:/example_urdf/manipulator.urdf" --package-dir "D:/example_urdf" --ee-frame link6

"""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path
from typing import Iterable, Optional, Tuple

import numpy as np
import pinocchio as pin


def section(title: str) -> None:
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


def short_vec(x: np.ndarray, digits: int = 4) -> str:
    return np.array2string(np.asarray(x), precision=digits, suppress_small=True)



def print_check(name: str, err: float, tol: float) -> bool:
    """统一打印验证结果。"""
    ok = err <= tol
    mark = "PASS" if ok else "WARN"
    print(f"[{mark}] {name}: error = {err:.6e}, tol = {tol:.1e}")
    return ok


def deterministic_vector(n: int, scale: float, phase: float = 0.0) -> np.ndarray:
    """生成确定性的测试向量，避免 demo 每次输出不同。"""
    if n <= 0:
        return np.array([])
    x = np.arange(1, n + 1, dtype=float)
    return scale * np.sin(0.73 * x + phase)


def load_robot(
    urdf_path: Optional[str],
    package_dirs: Iterable[str],
    use_sample: bool,
) -> Tuple[pin.Model, Optional[pin.GeometryModel], Optional[pin.GeometryModel]]:
    """加载自定义 URDF；如果指定 --sample，则加载 Pinocchio 内置样例机械臂。"""
    if use_sample:
        model = pin.buildSampleModelManipulator()
        return model, None, None

    if not urdf_path:
        raise ValueError("请提供 --urdf 路径，或使用 --sample 运行内置样例。")

    urdf = Path(urdf_path).expanduser().resolve()
    if not urdf.exists():
        raise FileNotFoundError(f"URDF 不存在：{urdf}")

    pkg_dirs = [str(Path(p).expanduser().resolve()) for p in package_dirs]
    if not pkg_dirs:
        # 常见情况：mesh 使用 package:// 或相对路径时，把 URDF 所在目录作为搜索目录
        pkg_dirs = [str(urdf.parent)]

    model, collision_model, visual_model = pin.buildModelsFromUrdf(str(urdf), pkg_dirs)
    return model, collision_model, visual_model


def select_end_effector_frame(model: pin.Model, preferred: Optional[str]) -> int:
    """优先使用用户指定 frame；否则尽量自动选择末端 frame。"""
    if preferred:
        if model.existFrame(preferred):
            return model.getFrameId(preferred)

        section("没有找到指定末端 frame，下面是可用 frame 名称")
        for i, frame in enumerate(model.frames):
            print(f"[{i:03d}] {frame.name}  type={frame.type}")
        raise ValueError(f"Frame '{preferred}' 不存在，请把 --ee-frame 改成上面列表中的名称。")

    # 自动猜测：优先选择名字里像末端执行器的 frame
    candidates = ["tool", "tcp", "ee", "end", "effector", "link6", "flange"]
    for key in candidates:
        for i, frame in enumerate(model.frames):
            if key.lower() in frame.name.lower() and i != 0:
                return i

    # 兜底：取最后一个非 universe frame
    for i in range(len(model.frames) - 1, 0, -1):
        return i
    raise RuntimeError("模型里没有可用 frame。")


def update_frame_pose(model: pin.Model, data: pin.Data, q: np.ndarray, frame_id: int) -> pin.SE3:
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacement(model, data, frame_id)
    return data.oMf[frame_id].copy()


def make_goal_configuration(model: pin.Model, q_start: np.ndarray) -> np.ndarray:
    """从 A 点生成一个温和的 B 点，保证 B 是由模型自身积分得到的可达姿态。"""
    dq = np.zeros(model.nv)
    # 前 6 个自由度给一点扰动；如果机械臂自由度少，也能兼容
    pattern = np.array([0.35, -0.25, 0.28, -0.20, 0.22, -0.18])
    dq[: min(model.nv, len(pattern))] = pattern[: min(model.nv, len(pattern))]
    return pin.integrate(model, q_start, dq)


def print_model_info(model: pin.Model, frame_id: int) -> None:
    section("1. 模型信息")
    print(f"Pinocchio version: {pin.__version__}")
    print(f"Model name: {model.name}")
    print(f"nq = {model.nq}  # configuration 维度")
    print(f"nv = {model.nv}  # velocity/tangent 维度")
    print(f"njoints = {model.njoints}")
    print(f"nframes = {len(model.frames)}")
    print(f"End-effector frame: {model.frames[frame_id].name}  id={frame_id}")

    print("\nJoints:")
    for i, name in enumerate(model.names):
        print(f"  [{i:02d}] {name}")


def run_fk_and_jacobian(model: pin.Model, q: np.ndarray, frame_id: int) -> None:
    section("2. 正运动学 FK + 末端雅可比 Jacobian")
    data = model.createData()

    M = update_frame_pose(model, data, q, frame_id)
    pin.computeJointJacobians(model, data, q)
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    print("q_A =", short_vec(q))
    print("End-effector position p_A =", short_vec(M.translation))
    print("End-effector rotation R_A =\n", short_vec(M.rotation))
    print(f"Jacobian shape = {J.shape}  # 6 x nv")

    # 奇异值可以直观看机械臂在当前位姿附近的可动性/病态程度
    s = np.linalg.svd(J, compute_uv=False)
    print("Jacobian singular values =", short_vec(s))
    print(f"Estimated Jacobian rank = {np.linalg.matrix_rank(J, tol=1e-6)}")


def run_dynamics(model: pin.Model, q: np.ndarray) -> None:
    section("3. 动力学：质量矩阵 / 重力补偿 / RNEA")
    data = model.createData()
    v0 = np.zeros(model.nv)
    a0 = np.zeros(model.nv)

    # RNEA(q, 0, 0) 常用于得到静态重力补偿项
    tau_g = pin.rnea(model, data, q, v0, a0)

    # CRBA 计算关节空间质量矩阵
    M = pin.crba(model, data, q)
    M = 0.5 * (M + M.T)  # 数值对称化，方便展示

    # 非线性项 nle = C(q,v)v + g(q)，当 v=0 时等价于 g(q)
    nle = pin.nonLinearEffects(model, data, q, v0)

    print("Gravity compensation tau_g =", short_vec(tau_g))
    print("nonLinearEffects(q, 0) =", short_vec(nle))
    print(f"Mass matrix M shape = {M.shape}")
    print("M top-left block =\n", short_vec(M[: min(6, model.nv), : min(6, model.nv)]))


def make_symmetric_mass_matrix(M_raw: np.ndarray) -> np.ndarray:
   
    M_raw = np.asarray(M_raw).copy()
    lower_norm = np.linalg.norm(np.tril(M_raw, -1))
    upper_norm = np.linalg.norm(np.triu(M_raw, 1))

    if upper_norm > 0 and lower_norm < 1e-12:
        return np.triu(M_raw) + np.triu(M_raw, 1).T

    return 0.5 * (M_raw + M_raw.T)


def run_inverse_dynamics_with_velocity(model: pin.Model, q: np.ndarray) -> None:
    """
    展示非零速度/加速度下的逆动力学，并做两个独立验证：
    1. RNEA(q,v,a) == M(q) @ a + nonLinearEffects(q,v)
    2. ABA(q,v,tau_rnea) == a
    """
    section("4. 带速度/加速度的逆动力学 RNEA + 验证")

    if model.nv == 0:
        print("模型没有速度自由度，跳过动力学验证。")
        return

    v_dyn = deterministic_vector(model.nv, scale=0.35, phase=0.2)
    a_cmd = deterministic_vector(model.nv, scale=0.80, phase=1.1)

    data_rnea = model.createData()
    data_nle = model.createData()
    data_M = model.createData()
    data_aba = model.createData()

    tau_rnea = pin.rnea(model, data_rnea, q, v_dyn, a_cmd)

    M_raw = pin.crba(model, data_M, q)
    M = make_symmetric_mass_matrix(M_raw)
    nle = pin.nonLinearEffects(model, data_nle, q, v_dyn)
    tau_formula = M @ a_cmd + nle

    a_aba = pin.aba(model, data_aba, q, v_dyn, tau_rnea)

    print("q_dyn =", short_vec(q))
    print("v_dyn =", short_vec(v_dyn))
    print("a_cmd =", short_vec(a_cmd))
    print("tau_rnea =", short_vec(tau_rnea))
    print("tau_formula = M(q) @ a_cmd + nonLinearEffects(q,v) =", short_vec(tau_formula))
    print("a_aba = ABA(q, v, tau_rnea) =", short_vec(a_aba))

    tau_l2_err = float(np.linalg.norm(tau_rnea - tau_formula))
    tau_max_err = float(np.max(np.abs(tau_rnea - tau_formula)))
    aba_l2_err = float(np.linalg.norm(a_aba - a_cmd))
    aba_max_err = float(np.max(np.abs(a_aba - a_cmd)))

    print("\nInverse dynamics verification:")
    ok1 = print_check("RNEA(q,v,a) == M(q)@a + nle(q,v), L2", tau_l2_err, 1e-8)
    ok2 = print_check("RNEA(q,v,a) == M(q)@a + nle(q,v), max abs", tau_max_err, 1e-8)
    ok3 = print_check("ABA(q,v,tau_rnea) == a_cmd, L2", aba_l2_err, 1e-8)
    ok4 = print_check("ABA(q,v,tau_rnea) == a_cmd, max abs", aba_max_err, 1e-8)

    if ok1 and ok2 and ok3 and ok4:
        print("结论：带速度/加速度的逆动力学与质量矩阵公式、正动力学 ABA 反推结果一致。")
    else:
        print("提示：如果出现 WARN，通常是数值精度、模型惯量尺度或 CRBA 矩阵填充差异导致；可适当放宽 tol 到 1e-7。")

def solve_ik_clik(
    model: pin.Model,
    frame_id: int,
    target: pin.SE3,
    q_init: np.ndarray,
    max_iter: int = 800,
    eps: float = 1e-5,
    dt: float = 0.15,
    damp: float = 1e-6,
) -> Tuple[np.ndarray, bool, list[float]]:
    """基于 Pinocchio 的闭环逆运动学 CLIK。"""
    data = model.createData()
    q = q_init.copy()
    history: list[float] = []

    for _ in range(max_iter):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, frame_id)

        current = data.oMf[frame_id]
        iMd = current.actInv(target)
        err = pin.log(iMd).vector
        err_norm = float(np.linalg.norm(err))
        history.append(err_norm)

        if err_norm < eps:
            return q, True, history

        J = pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL)

        # 与 SE(3) log 误差一致的雅可比修正；老版本若没有 Jlog6，则使用常见阻尼最小二乘近似
        try:
            J = -pin.Jlog6(iMd.inverse()) @ J
            v = -J.T @ np.linalg.solve(J @ J.T + damp * np.eye(6), err)
        except Exception:
            v = J.T @ np.linalg.solve(J @ J.T + damp * np.eye(6), err)

        q = pin.integrate(model, q, v * dt)

    return q, False, history


def run_ik_demo(model: pin.Model, q_start: np.ndarray, q_goal_ref: np.ndarray, frame_id: int) -> np.ndarray:
    section("5. 逆运动学 IK：从 A 求到 B")
    data = model.createData()
    target = update_frame_pose(model, data, q_goal_ref, frame_id)

    print("IK target position p_B =", short_vec(target.translation))
    q_ik, ok, hist = solve_ik_clik(model, frame_id, target, q_start)

    solved = update_frame_pose(model, data, q_ik, frame_id)
    pos_err = np.linalg.norm(solved.translation - target.translation)
    rot_err = np.linalg.norm(pin.log3(target.rotation.T @ solved.rotation))

    print(f"Converged: {ok}")
    print(f"Iterations: {len(hist)}")
    print(f"Initial 6D error: {hist[0]:.6e}")
    print(f"Final 6D error:   {hist[-1]:.6e}")
    print(f"Position error:   {pos_err:.6e} m")
    print(f"Rotation error:   {rot_err:.6e} rad")
    print("q_IK =", short_vec(q_ik))
    return q_ik


def interpolate_trajectory(model: pin.Model, q0: np.ndarray, q1: np.ndarray, steps: int) -> list[np.ndarray]:
    return [pin.interpolate(model, q0, q1, u) for u in np.linspace(0.0, 1.0, steps)]


def run_trajectory_demo(
    model: pin.Model,
    collision_model: Optional[pin.GeometryModel],
    visual_model: Optional[pin.GeometryModel],
    q0: np.ndarray,
    q1: np.ndarray,
    frame_id: int,
    no_viewer: bool,
    steps: int,
    play_dt: float,
) -> None:
    section("6. A -> B 轨迹插值 + 可视化")
    data = model.createData()
    qs = interpolate_trajectory(model, q0, q1, steps)

    print(f"Trajectory steps: {len(qs)}")
    for idx in [0, len(qs) // 2, len(qs) - 1]:
        M = update_frame_pose(model, data, qs[idx], frame_id)
        print(f"step {idx:03d}: p = {short_vec(M.translation)}")

    if no_viewer:
        print("已跳过 Meshcat 播放；去掉 --no-viewer 可打开浏览器查看。")
        return

    try:
        from pinocchio.visualize import MeshcatVisualizer

        if collision_model is not None and visual_model is not None:
            viz = MeshcatVisualizer(model, collision_model, visual_model)
        else:
            viz = MeshcatVisualizer(model)

        viz.initViewer(open=True)
        viz.loadViewerModel()
        print("Meshcat viewer opened. 正在播放 A -> B 轨迹……")
        for q in qs:
            viz.display(q)
            time.sleep(play_dt)
        print("播放完成。")
    except ImportError:
        print("未安装 Meshcat，无法可视化。安装：pip install meshcat")
    except Exception as exc:
        print(f"Meshcat 可视化失败：{exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pinocchio 功能展示 demo")
    parser.add_argument("--urdf", type=str, default=None, help="URDF 文件路径")
    parser.add_argument("--package-dir", action="append", default=[], help="mesh/package 搜索目录，可重复传入")
    parser.add_argument("--ee-frame", type=str, default=None, help="末端执行器 frame 名称，例如 link6/tool0")
    parser.add_argument("--sample", action="store_true", help="不加载 URDF，使用 Pinocchio 内置样例机械臂")
    parser.add_argument("--no-viewer", action="store_true", help="不打开 Meshcat 浏览器可视化")
    parser.add_argument("--steps", type=int, default=80, help="轨迹插值步数")
    parser.add_argument("--play-dt", type=float, default=0.03, help="Meshcat 每帧播放间隔秒数")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    model, collision_model, visual_model = load_robot(args.urdf, args.package_dir, args.sample)
    frame_id = select_end_effector_frame(model, args.ee_frame)

    q_start = pin.neutral(model)
    q_goal_ref = make_goal_configuration(model, q_start)

    print_model_info(model, frame_id)
    run_fk_and_jacobian(model, q_start, frame_id)
    run_dynamics(model, q_start)
    # 使用 B 点附近姿态做非零速度/加速度逆动力学，更接近运动过程中的动力学计算
    run_inverse_dynamics_with_velocity(model, q_goal_ref)
    q_ik = run_ik_demo(model, q_start, q_goal_ref, frame_id)
    run_trajectory_demo(
        model,
        collision_model,
        visual_model,
        q_start,
        q_ik,
        frame_id,
        no_viewer=args.no_viewer,
        steps=args.steps,
        play_dt=args.play_dt,
    )

    section("Demo 结束")


if __name__ == "__main__":
    main()
