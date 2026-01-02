from typing import Optional, Callable, Any

import os
import re
from math import cos, sin
from multiprocessing import Process, Manager

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

RESULTS_ROOTDIR = "results"
LOG_ROOTDIR = "../log"

ROS2_LOG_RE = re.compile(r"^\[(?:INFO|ERROR)\] \[.+\]: .*$")
NODE_LOG_RE = re.compile(r"^\[.+\] \[(?:INFO|ERROR)\] \[(?P<ts>\d+\.\d+)\] \[(?P<nn>.+)\]: (?P<msg>.*)$")
VALUE_RE = re.compile(r"(\w+)=([-\d\.]+)")

TIME_LIMIT = 10.0
DISTANCE_LIMIT = 0.1

CTRL_ERROR_THRESHOLD = 0.2
CTRL_ERROR_OVER_THRESHOLD_LIMIT = 10


def log_to_csv(log_file: str, output_file: Optional[str] = None):
    logs_list = []
    with open(log_file, "r", encoding="UTF-8") as f:
        for line in f:
            if m := NODE_LOG_RE.match(line):
                timestamp = int(m["ts"].replace(".", ""))
                node_name = m["nn"]
                message = m["msg"].strip()
                logs_list.append({"timestamp": timestamp, "node_name": node_name, "message": message})

    logs_df = pd.DataFrame(logs_list)

    # get init time
    filtered_logs = logs_df[
        (logs_df["node_name"] == "turtle2_controller") & (logs_df["message"] == '"turtle2" successfully spawned.')
    ]
    if filtered_logs.shape[0] == 0:
        raise ValueError("Cannot find turtle2 spawn log.")
    init_time = filtered_logs["timestamp"].iloc[0]

    logs_df: pd.DataFrame = logs_df[logs_df["timestamp"] > init_time]
    logs_df["timestamp"] = (logs_df["timestamp"] - init_time) / 1e9

    # parse logs
    parsed_logs = []
    for _, row in logs_df.iterrows():
        for v in VALUE_RE.findall(row["message"]):
            parsed_logs.append({"timestamp": row["timestamp"], "variable": v[0], "value": v[1]})
    logs = pd.DataFrame(parsed_logs)

    # save as csv
    if output_file is None:
        output_file = log_file.replace(".log", ".csv")
    logs.to_csv(output_file, index=False)


def check_0(kd: float, ka: float, time_limit: float, distance_limit: float) -> dict[str, Any]:
    csv_file = f"../log/myturtlesim_tf2_launch_{str(kd).replace('.', '')}_{str(ka).replace('.', '')}.csv"
    # print(csv_file)
    df = pd.read_csv(csv_file)

    distance_df = df[df["variable"] == "distance"].reset_index(drop=True)
    distance_dt = distance_df["timestamp"].diff().dropna()
    distance_dd = distance_df["value"].diff().dropna()
    # angle_df = df[df['variable'] == 'angle'].reset_index(drop=True)
    # angle_dt = angle_df['timestamp'].diff().dropna()
    # angle_da = angle_df['value'].diff().dropna()
    cmd_linear_vel_df = df[df["variable"] == "lin_vel_x_"].reset_index(drop=True)

    result_distance = pd.DataFrame()
    result_distance["timestamp"] = distance_df["timestamp"][1:]
    result_distance["actual_linear"] = -distance_dd / distance_dt
    # result_distance['cmd_linear_vel'] = -distance_df['value'][1:] * kd * np.cos(angle_df['value'][1:])
    result_distance["cmd_linear_vel"] = cmd_linear_vel_df["value"][1:]
    result_distance["ctrl_error"] = (result_distance["actual_linear"] - result_distance["cmd_linear_vel"]).abs()
    result_distance["vel_lower_bound"] = (distance_df["value"][1:] - distance_limit).apply(
        lambda x: x if x > 0 else 0
    ) / (time_limit - distance_df["timestamp"].apply(lambda x: x if x < time_limit else np.nan))
    # result_angle = pd.DataFrame()
    # result_angle['actual'] = angle_da/angle_dt
    # result_angle['command_angular'] = -angle_df['value'][1:] * ka

    actual_success = not (distance_df[distance_df["timestamp"] > TIME_LIMIT]["value"] > DISTANCE_LIMIT).any()

    vel_lower_bound_check_success = not (result_distance["vel_lower_bound"] > 4.0).any()
    vel_lower_bound_timestamp_check_success = (
        result_distance["timestamp"][result_distance[result_distance["vel_lower_bound"] > 4.0].index[0]]
        if not vel_lower_bound_check_success
        else 0
    )

    ctrl_error_check_success = (
        result_distance["ctrl_error"][result_distance["timestamp"] <= TIME_LIMIT] > CTRL_ERROR_THRESHOLD
    ).sum() < CTRL_ERROR_OVER_THRESHOLD_LIMIT

    check_result = {}
    check_result["kd"] = str(kd).replace(".", "")
    check_result["ka"] = str(ka).replace(".", "")
    check_result["success"] = actual_success
    check_result["vel_lower_bound"] = vel_lower_bound_check_success
    check_result["vel_lower_bound_timestamp"] = vel_lower_bound_timestamp_check_success
    check_result["ctrl_error"] = ctrl_error_check_success

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(result_distance["timestamp"], result_distance["actual_linear"], marker=".", label="actual")
    ax.plot(result_distance["timestamp"], result_distance["cmd_linear_vel"], marker=".", label="cmd_linear_vel")
    ax.plot(result_distance["timestamp"], result_distance["ctrl_error"], marker=".", label="ctrl_error")
    ax.set_ylim(ax.get_ylim())
    ax.plot(result_distance["timestamp"], result_distance["vel_lower_bound"], marker=".", label="vel_lower_bound")
    ax.legend()
    title_suffix = (
        f"(Failed at {vel_lower_bound_timestamp_check_success:.4f})"
        if not vel_lower_bound_check_success
        else "(Success)"
    )
    ax.set_title(f"kd = {kd}, ka = {ka} {title_suffix}")
    fig.tight_layout(w_pad=0, h_pad=0)
    fig.savefig(f"../log/myturtlesim_tf2_launch_{str(kd).replace('.', '')}_{str(ka).replace('.', '')}_check_0.png")
    plt.close(fig)

    return check_result


def physical_model(e_d: float, e_a: float, u_d: float, u_a: float, delta_t: float = 0.5) -> tuple[float, float]:
    dt = 0.001
    if e_d == 0:
        return e_d, e_a
    for _ in range(int(delta_t / dt)):
        e_d = e_d - u_d * cos(e_a) * dt
        e_a = e_a - (u_a - u_d * sin(e_a) / e_d) * dt
    return e_d, e_a


def computational_model(e_d: float, e_a: float, k_d: float, k_a: float) -> tuple[float, float]:
    u_d = k_d * e_d
    u_a = k_a * e_a
    if u_d > 4:
        u_d = 4
    elif u_d < -4:
        u_d = -4
    if u_a > 2:
        u_a = 2
    elif u_a < -2:
        u_a = -2
    return u_d, u_a


def objective_function(e_d: float, timestamp: float, time_limit: float, distance_limit: float) -> bool:
    return not (e_d > distance_limit and timestamp > time_limit)


def check_1(kd: float, ka: float, time_limit: float, distance_limit: float) -> dict[str, Any]:
    log_csv_file = f"{LOG_ROOTDIR}/{str(kd).replace('.', '')}_{str(ka).replace('.', '')}/launch.csv"
    result_csv_file = f"{LOG_ROOTDIR}/{str(kd).replace('.', '')}_{str(ka).replace('.', '')}/check_1.csv"

    df = pd.read_csv(log_csv_file)

    df = df[df["variable"].isin(["distance", "angle"])].reset_index(drop=True)
    df["timestamp"] = df["timestamp"].round(3)
    df = df.pivot(index="timestamp", columns="variable", values="value").reset_index()

    def _predict(ed: float, ea: float, kd: float, ka: float, delta_ts: pd.Series) -> pd.DataFrame:
        pa_name = "predicted_angle"
        pd_name = "predicted_distance"
        result = {
            pa_name: [],
            pd_name: [],
        }
        for delta_t in delta_ts[:]:
            vd, va = computational_model(ed, ea, kd, ka)
            ed, ea = physical_model(ed, ea, vd, va, delta_t)
            result[pa_name].append(round(ea, 3))
            result[pd_name].append(round(ed, 3))

        result_df = pd.DataFrame(result, index=delta_ts.index)
        return result_df

    results_df = df.copy()
    results_df["objective"] = results_df.apply(
        lambda x: objective_function(x["distance"], x["timestamp"], time_limit, distance_limit), axis=1
    )
    actual_success = results_df["objective"].all()

    delta_ts = df["timestamp"].diff()
    prediction_starting_indices = []
    for row in df.itertuples():
        i, timestamp, ea, ed = row
        prediction_starting_index = i + 1
        prediction_starting_indices.append(prediction_starting_index)
        result_df = _predict(ed, ea, kd, ka, delta_ts[prediction_starting_index:])
        check_success = (
            pd.concat([df["timestamp"], result_df], axis=1)
            .apply(
                lambda x: objective_function(x["predicted_distance"], x["timestamp"], time_limit, distance_limit),
                axis=1,
            )
            .all()
        )
        result_df.columns = [f"{col}_{prediction_starting_index}" for col in result_df.columns]
        results_df = pd.concat([results_df, result_df], axis=1)
        # if not check_success:
        #     break
        if timestamp > 10:
            break

    results_df.to_csv(result_csv_file, index=False)

    for i in prediction_starting_indices:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(df["timestamp"], df["distance"], label="distance", marker="o")
        ax.plot(
            results_df["timestamp"], results_df[f"predicted_distance_{i}"], label=f"predicted_distance_{i}", marker="."
        )
        ax.legend(fontsize=8, loc="upper right")
        ax.set_title(f"kd = {kd}, ka = {ka} Actual: {actual_success} Predicted: {check_success}")
        fig.tight_layout(w_pad=0, h_pad=0)
        fig.savefig(f"{LOG_ROOTDIR}/{str(kd).replace('.', '')}_{str(ka).replace('.', '')}/check_1_prediction_{i}.png")
        plt.close(fig)

    check_result = {}
    check_result["kd"] = str(kd).replace(".", "")
    check_result["ka"] = str(ka).replace(".", "")
    check_result["actual_success"] = actual_success
    check_result["check_success"] = check_success
    return check_result


def run(i: int, j: int, results: list, check_func: Callable):
    file_path_prefix = f"{LOG_ROOTDIR}/{i + 5:02d}_{j + 5:02d}/launch"
    log_file = f"{file_path_prefix}.log"
    csv_file = f"{file_path_prefix}.csv"
    if os.path.exists(log_file) and (
        not os.path.exists(csv_file) or os.path.getmtime(log_file) > os.path.getmtime(csv_file)
    ):
        log_to_csv(log_file)
    check_result = check_func((i + 5) / 10, (j + 5) / 10, TIME_LIMIT, DISTANCE_LIMIT)
    results.append(check_result)


if __name__ == "__main__":
    os.makedirs("results", exist_ok=True)

    parallel = True
    check_func = check_1

    if parallel:
        manager = Manager()
        check_results = manager.list()

        processes: list[Process] = []
        for i in range(0, 80, 5):
            for j in range(0, 80, 5):
                process = Process(target=run, args=(i, j, check_results, check_func))
                processes.append(process)
                process.start()

        for process in processes:
            process.join()
    else:
        check_results = []
        for i in range(0, 80, 5):
            for j in range(0, 80, 5):
                run(i, j, check_results, check_func)

    if check_func is check_0:
        check_results_df = pd.DataFrame(list(check_results))
        check_results_df.sort_values(by=["kd", "ka"], inplace=True)
        check_results_df.to_csv(os.path.join(RESULTS_ROOTDIR, "check_0.csv"), index=False)

        vel_lower_bound_check_missing_cases = check_results_df[
            check_results_df["vel_lower_bound"] != check_results_df["success"]
        ]
        vel_lower_bound_check_missing_cases.to_csv("check_0_vel_lower_bound.csv", index=False)

        ctrl_error_missing_cases = check_results_df[check_results_df["ctrl_error"] != check_results_df["success"]]
        ctrl_error_missing_cases.to_csv("check_0_ctrl_error.csv", index=False)
    elif check_func is check_1:
        check_results_df = pd.DataFrame(list(check_results))
        check_results_df.sort_values(by=["kd", "ka"], inplace=True)
        check_results_df.to_csv(os.path.join(RESULTS_ROOTDIR, "check_1.csv"), index=False)

        check_success_missing_cases = check_results_df[
            check_results_df["check_success"] != check_results_df["actual_success"]
        ]
        check_success_missing_cases.to_csv("check_1_check_success.csv", index=False)
