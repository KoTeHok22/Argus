import itertools
import json
import os
from pathlib import Path
import subprocess
import time

import pytest

try:
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from argus_msgs.msg import AnomalySet, CleanCloud, Diagnostics, GaugeState, ObstacleArray
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
except ModuleNotFoundError as error:
    if error.name != "rclpy":
        raise
    pytest.skip("ROS 2 runtime is unavailable", allow_module_level=True)

DOMAINS = itertools.count(80 + os.getpid() % 100)


class RuntimeProbe:
    def __init__(self, node):
        self.node = node
        self.executor = SingleThreadedExecutor(context=node.context)
        self.executor.add_node(node)
        self.decisions = []
        self.gauges = []
        self.frame = 0
        self.cloud = node.create_publisher(CleanCloud, "/argus/clean", qos_profile_sensor_data)
        self.geometry = node.create_publisher(AnomalySet, "/argus/anom_g", qos_profile_sensor_data)
        self.no_return = node.create_publisher(
            AnomalySet, "/argus/anom_nr", qos_profile_sensor_data)
        self.sensor = node.create_publisher(Diagnostics, "/argus/diagnostics", 10)
        self.health = node.create_publisher(String, "/argus/system_health", 10)
        self.sub_decisions = node.create_subscription(
            ObstacleArray, "/argus/obstacles", self.decisions.append, 10)
        self.sub_gauges = node.create_subscription(
            GaugeState, "/argus/gauge_state", self.gauges.append, 10)

    def wait(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        pytest.fail("ROS 2 output did not arrive before timeout")

    def send(self, distance=200.0, sensor=True, health="OK", obstacle=False):
        if sensor is not None:
            self.sensor.publish(Diagnostics(sensor_ok=sensor))
        if health is not None:
            self.health.publish(String(data=health))
        if sensor is not None or health is not None:
            time.sleep(0.1)
        self.frame += 1
        cloud = CleanCloud()
        cloud.header.stamp.sec = self.frame
        cloud.header.frame_id = "lidar"
        cloud.n_raw = 16
        cloud.rings = 2
        cloud.az_steps = 8
        cloud.x = [0.02 * i if obstacle else 0.0 for i in range(16)]
        cloud.y = [-distance] * 16
        cloud.z = [0.5] * 16
        cloud.intensity = [1.0] * 16
        cloud.raw_idx = list(range(16))
        cloud.ground_mask = [0] * 16
        geometry = AnomalySet()
        geometry.header = cloud.header
        if obstacle:
            geometry.indices = list(range(16))
            geometry.scores = [1.0] * 16
        no_return = AnomalySet()
        no_return.header = cloud.header
        self.cloud.publish(cloud)
        self.geometry.publish(geometry)
        self.no_return.publish(no_return)
        stamp = self.frame
        self.wait(lambda: any(d.header.stamp.sec == stamp for d in self.decisions)
                  and any(g.header.stamp.sec == stamp for g in self.gauges))
        decision = next(d for d in self.decisions if d.header.stamp.sec == stamp)
        gauge = next(g for g in self.gauges if g.header.stamp.sec == stamp)
        return decision, gauge


@pytest.fixture
def probe(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(next(DOMAINS)))
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node("supervision_probe", context=context)
    executable = Path(get_package_prefix("argus_node_fusion"))
    executable /= "lib/argus_node_fusion/argus_fusion_node"
    command = [str(executable), "--ros-args", "-p", "gauge.forward_axis:=-y",
               "-p", "gauge.sensor_height:=1.075", "-p", "fusion.stale_timeout_s:=0.5",
               "-p", "clustering.adaptive_scaling:=false"]
    process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    runtime = RuntimeProbe(node)
    try:
        publishers = [runtime.cloud, runtime.geometry, runtime.no_return,
                      runtime.sensor, runtime.health]
        runtime.wait(lambda: all(p.get_subscription_count() for p in publishers)
                     and node.count_publishers("/argus/obstacles")
                     and node.count_publishers("/argus/gauge_state"))
        time.sleep(0.1)
        yield runtime
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        runtime.executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


@pytest.mark.parametrize("distance", [20.0, 200.0])
def test_distant_return_never_certifies_free_gauge(probe, distance):
    decision, gauge = probe.send(distance=distance)
    assert decision.status == ObstacleArray.STATUS_UNKNOWN
    assert gauge.observed_range_m == pytest.approx(distance)
    assert gauge.clear_range_m == 0.0
    assert gauge.speed_limit_mps == 0.0
    assert "свободный габарит не подтверждён" in decision.explain


@pytest.mark.parametrize("sensor,health", [
    (False, "OK"), (True, "DEAD:argus_detector"), (None, None)])
def test_missing_or_failed_health_never_allows_motion(probe, sensor, health):
    decision, gauge = probe.send(sensor=sensor, health=health)
    assert decision.status == ObstacleArray.STATUS_DEGRADED
    assert gauge.clear_range_m == 0.0
    assert gauge.speed_limit_mps == 0.0


@pytest.mark.parametrize("expired", ["sensor", "health"])
def test_health_expiry_is_not_hidden_by_continuing_frames(probe, expired):
    probe.send()
    time.sleep(0.6)
    decision, gauge = probe.send(sensor=None if expired == "sensor" else True,
                                 health=None if expired == "health" else "OK")
    assert decision.status == ObstacleArray.STATUS_DEGRADED
    assert gauge.speed_limit_mps == 0.0


def test_confirmed_obstacle_keeps_alert_when_health_fails(probe):
    for _ in range(6):
        decision, gauge = probe.send(distance=17.0, obstacle=True)
    assert decision.status == ObstacleArray.STATUS_BLOCKED
    decision, gauge = probe.send(distance=17.0, obstacle=True, sensor=False)
    assert decision.status == ObstacleArray.STATUS_BLOCKED
    assert decision.obstacles_detected > 0
    assert gauge.speed_limit_mps == 0.0


def test_stopped_frames_clear_observation_and_speed(probe):
    probe.send()
    probe.decisions.clear()
    probe.gauges.clear()
    probe.wait(lambda: any(d.status == ObstacleArray.STATUS_DEGRADED for d in probe.decisions)
               and any(g.observed_range_m == 0.0 for g in probe.gauges))
    assert probe.gauges[-1].clear_range_m == 0.0
    assert probe.gauges[-1].speed_limit_mps == 0.0


def test_recorder_preserves_unknown_status(probe, tmp_path):
    executable = Path(get_package_prefix("argus_node_recorder"))
    executable /= "lib/argus_node_recorder/argus_recorder_node"
    command = [str(executable), "--ros-args", "-p", f"recorder.output_dir:={tmp_path}",
               "-p", "recorder.flush_period_s:=0.1"]
    process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        probe.wait(lambda: probe.node.count_subscribers("/argus/obstacles") >= 2
                   and probe.node.count_subscribers("/argus/gauge_state") >= 2)
        probe.send()

        def recorded_unknown():
            rows = [json.loads(line) for path in tmp_path.glob("*.jsonl")
                    for line in path.read_text().splitlines() if line.endswith("}")]
            return (any(r.get("type") == "decision" and r.get("status") == "UNKNOWN"
                        for r in rows)
                    and any(r.get("type") == "gauge" and r.get("observed_m") == 200.0
                            and r.get("clear_m") == 0.0 and r.get("limit_mps") == 0.0
                            for r in rows))

        probe.wait(recorded_unknown)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
