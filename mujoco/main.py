import threading
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


def _actuator_for_joint(model: mujoco.MjModel, joint_name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f"joint not found: {joint_name}")

    for act_id in range(model.nu):
        if model.actuator_trntype[act_id] != mujoco.mjtTrn.mjTRN_JOINT:
            continue
        if model.actuator_trnid[act_id, 0] == joint_id:
            return act_id

    raise ValueError(f"no actuator bound to joint: {joint_name}")


def _joint_meta(model: mujoco.MjModel, joint_names):
    joint_ids = []
    dof_ids = []
    qpos_ids = []
    ranges = []

    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"joint not found: {name}")

        joint_ids.append(joint_id)
        dof_ids.append(int(model.jnt_dofadr[joint_id]))
        qpos_ids.append(int(model.jnt_qposadr[joint_id]))
        ranges.append(model.jnt_range[joint_id].copy())

    return joint_ids, np.array(dof_ids), np.array(qpos_ids), np.array(ranges)


def _input_loop(state, lock, joint_count: int) -> None:
    help_text = (
        "输入关节角或末端位置：\n"
        "- 关节角（弧度）: j a1 a2 a3 a4 a5 a6\n"
        "- 关节角（角度）: deg a1 a2 a3 a4 a5 a6\n"
        "- 末端位置（米）: p x y z\n"
        "- 直接输入 {n} 个数，默认弧度\n"
        "- 输入 q 退出输入线程\n"
    ).format(n=joint_count)
    print(help_text)

    while True:
        try:
            line = input("> ").strip()
        except EOFError:
            return

        if not line:
            continue
        if line.lower() in {"q", "quit", "exit"}:
            return
        if line.lower() in {"h", "help", "?"}:
            print(help_text)
            continue

        tokens = line.split()
        head = tokens[0].lower()

        if head in {"p", "pos", "position"}:
            tokens = tokens[1:]
            if len(tokens) != 3:
                print("位置需要 3 个数：x y z")
                continue
            try:
                pos = [float(x) for x in tokens]
            except ValueError:
                print("解析失败，请输入数字。")
                continue
            with lock:
                state["target_pos"] = np.array(pos, dtype=float)
            continue

        unit = "rad"
        if head in {"j", "joint", "rad", "deg"}:
            if head == "deg":
                unit = "deg"
            tokens = tokens[1:]

        if len(tokens) != joint_count:
            print(f"需要 {joint_count} 个关节角，当前是 {len(tokens)} 个。")
            continue

        try:
            values = [float(x) for x in tokens]
        except ValueError:
            print("解析失败，请输入数字。")
            continue

        if unit == "deg":
            values = [v * np.pi / 180.0 for v in values]

        with lock:
            state["q_target"] = np.array(values, dtype=float)
            state["target_pos"] = None


def _ik_step(model, data, body_id, dof_ids, qpos_ids, q_target, target_pos, joint_ranges):
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))

    for _ in range(15):
        for i, qpos_id in enumerate(qpos_ids):
            data.qpos[qpos_id] = q_target[i]
        mujoco.mj_forward(model, data)

        cur_pos = data.xpos[body_id].copy()
        err = target_pos - cur_pos
        if np.linalg.norm(err) < 1e-4:
            break

        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
        J = jacp[:, dof_ids]

        damping = 0.05
        A = J @ J.T + (damping**2) * np.eye(3)
        dq = J.T @ np.linalg.solve(A, err)

        q_target[:] = q_target + 0.5 * dq
        q_target[:] = np.clip(q_target, joint_ranges[:, 0], joint_ranges[:, 1])


def main() -> None:
    xml_path = Path(__file__).parent.joinpath("assets/scene-4.xml")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    ik_data = mujoco.MjData(model)

    arm_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    arm_actuators = [_actuator_for_joint(model, name) for name in arm_joints]

    _, dof_ids, qpos_ids, joint_ranges = _joint_meta(model, arm_joints)

    ee_body = "link_hand"
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_body)
    if body_id < 0:
        raise ValueError(f"end-effector body not found: {ee_body}")

    state = {
        "q_target": np.zeros(len(arm_joints), dtype=float),
        "target_pos": None,
    }
    lock = threading.Lock()

    input_thread = threading.Thread(
        target=_input_loop, args=(state, lock, len(arm_joints)), daemon=True
    )
    input_thread.start()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            with lock:
                q_target = state["q_target"].copy()
                target_pos = None if state["target_pos"] is None else state["target_pos"].copy()

            if target_pos is not None:
                _ik_step(model, ik_data, body_id, dof_ids, qpos_ids, q_target, target_pos, joint_ranges)
                with lock:
                    state["q_target"] = q_target

            for idx, act_id in enumerate(arm_actuators):
                data.ctrl[act_id] = q_target[idx]

            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
