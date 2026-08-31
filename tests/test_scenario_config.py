import json

from controllers.simulation_scenario.scenario_config import (
    DEFAULT_BIRDS,
    DEFAULT_TRAFFIC_DRONES,
    MAX_BIRDS,
    MAX_TRAFFIC_DRONES,
    ScenarioConfig,
    load_scenario_config,
)


def test_missing_scenario_config_uses_defaults(tmp_path):
    assert load_scenario_config(tmp_path / "missing.json") == ScenarioConfig()


def test_scenario_config_reads_counts_inside_limits(tmp_path):
    config_path = tmp_path / "scenario.json"
    config_path.write_text(
        json.dumps({"traffic_drones": {"count": 5}, "birds": {"count": 3}}),
        encoding="utf-8",
    )

    assert load_scenario_config(config_path) == ScenarioConfig(
        traffic_drone_count=5,
        bird_count=3,
    )


def test_scenario_config_caps_counts_to_supported_actor_pool(tmp_path):
    config_path = tmp_path / "scenario.json"
    config_path.write_text(
        json.dumps({"traffic_drones": {"count": 99}, "birds": {"count": 99}}),
        encoding="utf-8",
    )

    assert load_scenario_config(config_path) == ScenarioConfig(
        traffic_drone_count=MAX_TRAFFIC_DRONES,
        bird_count=MAX_BIRDS,
    )


def test_scenario_config_clamps_negative_counts(tmp_path):
    config_path = tmp_path / "scenario.json"
    config_path.write_text(
        json.dumps({"traffic_drones": {"count": -2}, "birds": {"count": -7}}),
        encoding="utf-8",
    )

    assert load_scenario_config(config_path) == ScenarioConfig(
        traffic_drone_count=0,
        bird_count=0,
    )


def test_scenario_config_uses_defaults_for_bad_values(tmp_path):
    config_path = tmp_path / "scenario.json"
    config_path.write_text(
        json.dumps({"traffic_drones": {"count": True}, "birds": {"count": "bad"}}),
        encoding="utf-8",
    )

    assert load_scenario_config(config_path) == ScenarioConfig(
        traffic_drone_count=DEFAULT_TRAFFIC_DRONES,
        bird_count=DEFAULT_BIRDS,
    )


def test_invalid_scenario_json_uses_defaults(tmp_path):
    config_path = tmp_path / "scenario.json"
    config_path.write_text("{", encoding="utf-8")

    assert load_scenario_config(config_path) == ScenarioConfig()
