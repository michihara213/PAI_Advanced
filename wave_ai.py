#!/usr/bin/env python3
"""CRANE+ Gazebo: phase-conditioned RBF imitation from ROS 2 sqlite3 bags.

No ROS messages are published by prepare/train. run is simulation-only.
"""
import argparse
import csv
import json
import sqlite3
import sys
import time
from pathlib import Path

import numpy as np

ARM = [f"crane_plus_joint{i}" for i in range(1, 5)]
ARM_TOPIC = "/crane_plus_arm_controller/joint_trajectory"
GRIP_TOPIC = "/crane_plus_gripper_controller/joint_trajectory"


def named_positions(names, values, wanted):
    if len(names) != len(values) or len(set(names)) != len(names):
        raise ValueError("Invalid/duplicate joint names or position count")
    lookup = dict(zip(names, values))
    if not set(wanted).issubset(lookup):
        raise ValueError(f"Missing joints: {set(wanted) - set(lookup)}")
    result = np.array([lookup[n] for n in wanted], dtype=float)
    if not np.isfinite(result).all():
        raise ValueError("Non-finite joint position")
    return result


def hold(t, q, query):
    """Causal zero-order hold: never use a future command."""
    if np.min(query) < t[0] or np.max(query) > t[-1]:
        raise ValueError("Resampling outside recorded interval")
    return q[np.searchsorted(t, query, side="right") - 1]


def clock_times(records, clock):
    rt = np.array([r[0] for r in records], dtype=np.int64)
    ct = np.array([r[0] for r in clock], dtype=np.int64)
    sim = np.array([r[1] for r in clock], dtype=float)
    if len(ct) < 2 or not np.isfinite(sim).all():
        raise ValueError("Missing or invalid /clock")
    monotonic_sim = np.maximum.accumulate(sim)
    rewind = float(np.max(monotonic_sim - sim))
    if rewind > 0.010 + 1e-9:
        raise ValueError(f"Simulation clock moved backwards by {rewind:.6f} s (limit 0.010 s)")
    # Repair only small reversals; retain message order and leave the bag untouched.
    sim = monotonic_sim
    # Subtract integer origin before converting nanoseconds to floating point.
    times = np.interp((rt - ct[0]) * 1e-9, (ct - ct[0]) * 1e-9, sim)
    values = np.stack([r[1] for r in records])
    keep = np.r_[np.diff(times) > 1e-9, True]  # last command at same sim time
    return times[keep], values[keep]


def read_episode(folder):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    wanted = {ARM_TOPIC, GRIP_TOPIC, "/joint_states", "/clock"}
    records = {k: [] for k in wanted}
    files = sorted(folder.glob("*.db3"))
    if not files:
        raise ValueError(f"No .db3 files in {folder}")
    for file in files:
        with sqlite3.connect(file.resolve().as_uri() + "?mode=ro", uri=True) as db:
            topics = {}
            for tid, name, typename, fmt in db.execute(
                "SELECT id,name,type,serialization_format FROM topics"
            ):
                if name in wanted:
                    if fmt != "cdr":
                        raise ValueError(f"Unsupported serialization: {fmt}")
                    topics[tid] = (name, get_message(typename))
            if not topics:
                continue
            sql = "SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ("
            sql += ",".join("?" for _ in topics) + ") ORDER BY timestamp"
            for tid, stamp, raw in db.execute(sql, list(topics)):
                name, cls = topics[tid]
                records[name].append((stamp, deserialize_message(raw, cls)))
    for name in wanted:
        records[name].sort(key=lambda r: r[0])
        if not records[name]:
            raise ValueError(f"Missing/empty topic: {name}")
    grip_names = list(records[GRIP_TOPIC][0][1].joint_names)
    if len(grip_names) != 1 or set(grip_names) & set(ARM):
        raise ValueError(f"Expected one distinct gripper joint: {grip_names}")
    names = ARM + grip_names
    clock = [(t, m.clock.sec + m.clock.nanosec * 1e-9)
             for t, m in records["/clock"]]
    # Avoid extrapolating simulation time outside recorded /clock coverage.
    lo, hi = clock[0][0], clock[-1][0]
    streams = []
    for topic, joints in [(ARM_TOPIC, ARM), (GRIP_TOPIC, grip_names),
                          ("/joint_states", names)]:
        decoded = []
        for stamp, msg in records[topic]:
            if not lo <= stamp <= hi:
                continue
            if topic == "/joint_states":
                q = named_positions(msg.name, msg.position, joints)
            else:
                if len(msg.points) != 1:
                    raise ValueError("Expected single-point teleop commands; multi-point trajectory found")
                q = named_positions(msg.joint_names, msg.points[0].positions, joints)
            decoded.append((stamp, q))
        if len(decoded) < 3:
            raise ValueError(f"Too few messages: {topic}")
        streams.append(clock_times(decoded, clock))
    return streams, names, {k: len(v) for k, v in records.items()}


def resample_episode(streams, threshold=0.01, trim=True):
    start = max(t[0] for t, _ in streams)
    end = min(t[-1] for t, _ in streams)
    if end - start < 1:
        raise ValueError("Common simulation-time interval shorter than 1 s")
    grid = np.arange(start, end, 0.05)
    commands = np.column_stack([hold(t, q, grid) for t, q in streams[:2]])
    changed = np.flatnonzero(np.max(np.abs(np.diff(commands, axis=0)), axis=1) > threshold)
    if not len(changed):
        raise ValueError("No detectable command movement (>0.01 rad)")
    a, b = 0, len(grid) - 1
    if trim:
        a = max(0, int(changed[0]) - 10)  # 0.5 s before first change
        b = min(len(grid) - 1, int(changed[-1]) + 21)  # 1 s after last change
    target_t = np.linspace(grid[a], grid[b], 201)
    y = np.column_stack([hold(t, q, target_t) for t, q in streams[:2]])
    st, sq = streams[2]
    actual = np.column_stack([np.interp(target_t, st, sq[:, j]) for j in range(sq.shape[1])])
    report = {
        "recorded_sim_s": round(end - start, 3),
        "active_sim_s": round(grid[b] - grid[a], 3),
        "trim_start_s": round(grid[a] - start, 3),
        "trim_end_s": round(end - grid[b], 3),
        "target_state_rmse_rad": float(np.sqrt(np.mean((y - actual) ** 2))),
        "max_command_gap_sim_s": float(max(np.diff(t).max() for t, _ in streams[:2])),
    }
    return y, actual, report


def prepare(args):
    root, out = Path(args.data).expanduser(), Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    episodes, states, durations, rows, joint_names = [], [], [], [], None
    for i in range(1, 11):
        name = f"ep{i:02d}"
        streams, names, counts = read_episode(root / name)
        if joint_names is not None and names != joint_names:
            raise ValueError(f"Joint names differ in {name}")
        joint_names = names
        q, state, report = resample_episode(streams, trim=not args.keep_idle)
        row = {"episode": name, **report,
               "arm_messages": counts[ARM_TOPIC], "joint_state_messages": counts["/joint_states"]}
        rows.append(row)
        episodes.append(q)
        states.append(state)
        durations.append(report["active_sim_s"])
        print(f"{name}: active={report['active_sim_s']:.2f} sim-s, "
              f"trim={report['trim_start_s']:.2f}/{report['trim_end_s']:.2f} s", flush=True)
    np.savez_compressed(out / "dataset.npz", targets=np.stack(episodes),
                        states=np.stack(states), durations=durations,
                        names=joint_names, episodes=[f"ep{i:02d}" for i in range(1, 11)])
    with (out / "episode_report.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    plot(out / "episodes.png", np.stack(episodes), joint_names)
    print(f"PREPARE OK: {out.resolve()}/dataset.npz")


def features(phase, centers):
    p = np.asarray(phase).reshape(-1, 1)
    width = 1.5 / (len(centers) - 1)
    return np.column_stack([np.ones(len(p)), np.exp(-0.5 * ((p - centers) / width) ** 2)])


def fit(targets):
    centers = np.linspace(0, 1, 32)
    x = features(np.linspace(0, 1, targets.shape[1]), centers)
    # Equal weight per demonstration; minimizer equals fitting their mean.
    penalty = np.eye(x.shape[1]) * 0.001
    penalty[0, 0] = 0
    weights = np.linalg.solve(x.T @ x + penalty, x.T @ targets.mean(axis=0))
    return centers, weights


def predict(phase, centers, weights, low, high):
    p = np.asarray(phase)
    if not np.isfinite(p).all() or np.any(p < 0) or np.any(p > 1):
        raise ValueError("Phase must be finite and within [0,1]")
    return np.clip(features(p, centers) @ weights, low, high)


def plot(path, curves, names, learned=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Plot skipped: matplotlib is not installed (data/model still saved)")
        return
    fig, axes = plt.subplots(len(names), 1, figsize=(10, 11), sharex=True)
    phase = np.linspace(0, 1, curves.shape[1])
    for j, ax in enumerate(axes):
        for i, q in enumerate(curves):
            ax.plot(phase, q[:, j], alpha=0.5, lw=1, label=f"ep{i+1:02d}")
        if learned is not None:
            ax.plot(phase, learned[:, j], "k", lw=2.5, label="RBF (train ep01-08)")
        ax.set_ylabel(names[j] + "\n(rad)")
        ax.grid(alpha=0.3)
    axes[0].legend(ncol=4, fontsize=8)
    axes[-1].set_xlabel("Normalized progress (phase)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def train(args):
    out = Path(args.out).expanduser()
    with np.load(out / "dataset.npz", allow_pickle=False) as data:
        y, names, durations = data["targets"], data["names"], data["durations"]
    if y.shape != (10, 201, 5) or not np.isfinite(y).all():
        raise ValueError("Expected 10 valid episodes with shape (10,201,5)")
    centers, weights = fit(y[:8])
    low, high = y[:8].min(axis=(0, 1)), y[:8].max(axis=(0, 1))
    prediction = predict(np.linspace(0, 1, 201), centers, weights, low, high)
    baseline = y[:8].mean(axis=(0, 1))
    report = {"method": "Phase-conditioned Gaussian RBF ridge regression",
              "train": [f"ep{i:02d}" for i in range(1, 9)], "test": ["ep09", "ep10"],
              "test_rmse_per_joint_rad": np.sqrt(np.mean((y[8:] - prediction) ** 2, axis=(0, 1))).tolist(),
              "constant_baseline_test_rmse_per_joint_rad": np.sqrt(np.mean((y[8:] - baseline) ** 2, axis=(0, 1))).tolist(),
              "train_rmse_rad": float(np.sqrt(np.mean((y[:8] - prediction) ** 2))),
              "test_rmse_rad": float(np.sqrt(np.mean((y[8:] - prediction) ** 2))),
              "names": names.tolist(), "duration_sim_s": float(np.median(durations[:8])),
              "note": "Offline target error, NOT Gazebo success rate. Test episodes were not fitted."}
    np.savez_compressed(out / "model.npz", centers=centers, weights=weights,
                        low=low, high=high, names=names, duration=report["duration_sim_s"])
    (out / "metrics.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    plot(out / "learned.png", y, names, prediction)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"TRAIN OK: {out.resolve()}/model.npz")


def make_path(current, learned, duration, max_speed=0.3):
    """Prepend slow positioning and stretch motion to bound point-to-point speed."""
    if not np.isfinite(current).all() or not np.isfinite(learned).all():
        raise ValueError("Non-finite path")
    if duration <= 0 or max_speed <= 0:
        raise ValueError("Duration/speed must be positive")
    setup = max(3.0, float(np.max(np.abs(learned[0] - current))) / max_speed)
    required = float(np.max(np.abs(np.diff(learned, axis=0)))) * (len(learned) - 1) / max_speed
    duration = max(duration, required)
    points = np.vstack([current, learned])
    times = np.r_[0.1, 0.1 + setup + np.linspace(0, duration, len(learned))]
    return points, times


def run(args):
    if not args.sim:
        raise ValueError("Gazebo only: pass --sim. This program is not validated for real hardware.")
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock

    with np.load(Path(args.out).expanduser() / "model.npz", allow_pickle=False) as model:
        names = model["names"].tolist()
        learned = predict(np.linspace(0, 1, 201), model["centers"], model["weights"],
                          model["low"], model["high"])
        duration = float(model["duration"])
    if names[:4] != ARM or len(names) != 5:
        raise ValueError("Unexpected model joint names")
    # Keep the context alive during Python's KeyboardInterrupt so finally can cancel.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("wave_rbf_inference",
                            parameter_overrides=[Parameter("use_sim_time", value=True)])
    latest, clock = {}, {}
    handles = []
    def state_cb(msg):
        try:
            latest.update(q=named_positions(msg.name, msg.position, names), wall=time.monotonic())
        except ValueError:
            pass
    def clock_cb(msg):
        value = msg.clock.sec + msg.clock.nanosec * 1e-9
        if value > clock.get("value", -1):
            clock.update(value=value, wall=time.monotonic())
    node.create_subscription(JointState, "/joint_states", state_cb, qos_profile_sensor_data)
    node.create_subscription(Clock, "/clock", clock_cb, qos_profile_sensor_data)
    clients = [ActionClient(node, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")
               for controller in ["crane_plus_arm_controller", "crane_plus_gripper_controller"]]
    try:
        for client in clients:
            if not client.wait_for_server(timeout_sec=10):
                raise RuntimeError("Controller action unavailable. Start Gazebo first.")
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest and clock and time.monotonic() - latest["wall"] < 1:
                break
        if not latest or not clock or time.monotonic() - clock["wall"] > 2:
            raise RuntimeError("Fresh /joint_states and advancing /clock required")
        if node.count_publishers(ARM_TOPIC) or node.count_publishers(GRIP_TOPIC):
            raise RuntimeError("Another command publisher is active. Stop keyboard teleop (ESC), then retry.")
        points, times = make_path(latest["q"], learned, duration)
        print(f"Gazebo motion: slow start positioning + learned wave, {times[-1]:.1f} sim-s. Ctrl+C cancels.")
        for client, selection in zip(clients, [slice(0, 4), slice(4, 5)]):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = names[selection]
            for q, stamp in zip(points[:, selection], times):
                point = JointTrajectoryPoint()
                point.positions = q.tolist()
                ns = round(float(stamp) * 1e9)
                point.time_from_start.sec = ns // 1000000000
                point.time_from_start.nanosec = ns % 1000000000
                goal.trajectory.points.append(point)
            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done() or future.result() is None:
                raise RuntimeError("Goal response timed out")
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError("Controller rejected goal")
            handles.append(handle)
        results = [h.get_result_async() for h in handles]
        deadline = time.monotonic() + 10 * times[-1] + 60
        while not all(f.done() for f in results):
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() > deadline:
                raise RuntimeError("Motion timed out")
            if time.monotonic() - latest["wall"] > 5 or time.monotonic() - clock["wall"] > 5:
                raise RuntimeError("Simulation stopped updating; cancelling goals")
            for f in results:
                if f.done() and (f.result().status != 4 or f.result().result.error_code != 0):
                    raise RuntimeError(f"Controller failed: {f.result().result.error_string}")
        for f in results:
            if f.result().status != 4 or f.result().result.error_code != 0:
                raise RuntimeError(f"Controller failed: {f.result().result.error_string}")
        print("RUN OK: both controllers reported success. Check the visible wave in Gazebo.")
    finally:
        if rclpy.ok():
            for handle in handles:
                f = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, f, timeout_sec=2)
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("prepare", help="Read all 10 bags; no robot motion")
    p.add_argument("--data", default="~/dataset/rosbags/wave_full")
    p.add_argument("--keep-idle", action="store_true", help="Disable automatic leading/trailing trim")
    p.add_argument("--out", default="results")
    p = sub.add_parser("train", help="Fit ep01-08; evaluate ep09-10; no robot motion")
    p.add_argument("--out", default="results")
    p = sub.add_parser("run", help="Execute once in Gazebo; stop teleop first")
    p.add_argument("--out", default="results")
    p.add_argument("--sim", action="store_true")
    args = parser.parse_args()
    try:
        {"prepare": prepare, "train": train, "run": run}[args.command](args)
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, FileNotFoundError, ImportError, sqlite3.Error) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
