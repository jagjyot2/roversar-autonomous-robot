"""Shared pytest fixtures for RoverSAR test suite."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from simulator.config import RoverConfig
from simulator.engine import RoverSimulationEngine
from simulator.rover_sim_server import create_app


@pytest.fixture()
def rover_config() -> RoverConfig:
    return RoverConfig()


@pytest.fixture()
def sim_engine(rover_config: RoverConfig) -> RoverSimulationEngine:
    return RoverSimulationEngine(config=rover_config, sensor_noise=False)


@pytest.fixture()
def api_client(rover_config: RoverConfig):
    app = create_app(config=rover_config)
    with TestClient(app) as client:
        yield client
