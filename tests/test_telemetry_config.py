"""Validate the checked-in network routes without ROS or aircraft hardware."""
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("generator", ROOT / "scripts/generate_swarm_config.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class TelemetryConfigTests(unittest.TestCase):
    def setUp(self):
        self.configs = {
            name: yaml.safe_load((ROOT / "config" / f"ros_topics_{name}.yaml").read_text(encoding="utf-8"))
            for name in ("gcs", "drone1", "drone2", "drone3")
        }

    def test_fly_telemetry_routes_use_local_mavros_and_numbered_gcs_topics(self):
        fly, gcs = self.configs["drone2"], self.configs["gcs"]
        self.assertEqual(gcs["IP"]["robot2"], "192.168.0.195")
        for port, suffix, msg_type in (
            (5013, "battery", "sensor_msgs/BatteryState"),
            (5014, "local_position/pose", "geometry_msgs/PoseStamped"),
        ):
            source = next(t for t in fly["send_topics"] if t["srcPort"] == port)
            destination = next(t for t in gcs["recv_topics"]
                               if t["srcIP"] == "robot2" and t["srcPort"] == port)
            self.assertEqual(source["topic_name"], f"/mavros/{suffix}")
            self.assertEqual(destination["topic_name"], f"/uav2/mavros/{suffix}")
            self.assertEqual(source["msg_type"], msg_type)
            self.assertEqual(destination["msg_type"], msg_type)

    def test_fly_exploration_subscribes_to_send_topics_and_keeps_cloud_rate(self):
        fly = self.configs["drone2"]
        gcs = self.configs["gcs"]
        for port in range(5001, 5009):
            source = next(t for t in fly["send_topics"] if t["srcPort"] == port)
            destination = next(t for t in gcs["recv_topics"]
                               if t["srcIP"] == "robot2" and t["srcPort"] == port)
            self.assertEqual(source["topic_name"].replace("_send_2", "_recv_2"),
                             destination["topic_name"])
            self.assertTrue(source["topic_name"].endswith("_send_2"))
        cloud = next(t for t in fly["send_topics"] if t["srcPort"] == 5010)
        self.assertEqual(cloud["topic_name"], "/sdf_map/occupancy_all_2")
        self.assertEqual(cloud["max_freq"], 2)

    def test_every_receiver_matches_a_sender_and_types_agree(self):
        senders = {}
        for name, config in self.configs.items():
            alias = "gcs" if name == "gcs" else name.replace("drone", "robot")
            address = config["IP"][alias]
            for topic in config["send_topics"]:
                endpoint = (address, topic["srcPort"])
                self.assertNotIn(endpoint, senders)
                senders[endpoint] = topic
        for config in self.configs.values():
            for topic in config["recv_topics"]:
                endpoint = (config["IP"][topic["srcIP"]], topic["srcPort"])
                self.assertIn(endpoint, senders)
                self.assertEqual(topic["msg_type"], senders[endpoint]["msg_type"])

    def test_launch_files_load_deployed_yaml_and_allow_mavros_namespace(self):
        for drone_id in range(1, 4):
            launch = ET.parse(ROOT / "launch" / f"bridge_drone{drone_id}.launch").getroot()
            self.assertEqual(launch.find("arg[@name='mavros_ns']").get("default"), "/mavros")
            node = launch.find("node")
            config = node.find("rosparam").get("file").replace("$(find swarm_ros_bridge)", str(ROOT))
            self.assertTrue(Path(config).is_file())
            remaps = {r.get("from"): r.get("to") for r in node.findall("remap")}
            self.assertEqual(remaps["/mavros/battery"], "$(arg mavros_ns)/battery")
            self.assertEqual(remaps["/mavros/local_position/pose"], "$(arg mavros_ns)/local_position/pose")

    def test_regeneration_preserves_deployed_files_and_snapshots(self):
        gcs = self.configs["gcs"]
        ips = [gcs["IP"][f"robot{i}"] for i in range(1, 4)]
        generated = {"ros_topics_gcs.yaml": generator.generate_gcs_config(ips, gcs["IP"]["gcs"])}
        for drone_id in range(1, 4):
            generated[f"ros_topics_drone{drone_id}.yaml"] = generator.generate_drone_config(drone_id, ips, gcs["IP"]["gcs"])
            generated[f"bridge_drone{drone_id}.launch"] = generator.generate_launch(drone_id)
        for filename, content in generated.items():
            folder = "launch" if filename.endswith(".launch") else "config"
            self.assertEqual((ROOT / folder / filename).read_text(encoding="utf-8"), content)
            self.assertEqual((ROOT / "scripts/temp" / filename).read_text(encoding="utf-8"), content)


if __name__ == "__main__":
    unittest.main()
