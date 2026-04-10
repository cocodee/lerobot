#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest

from lerobot.teleoperators.switchable import SwitchableTeleoperator, SwitchableTeleoperatorConfig, TeleopEvent
from lerobot.teleoperators.utils import make_teleoperator_from_config
from tests.mocks.mock_teleop import MockTeleopConfig


class FakeEventSource:
    def __init__(self, events=None):
        self._events = list(events or [])
        self.bindings = {"1": "activate_primary", "2": "activate_secondary"}
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def drain_events(self):
        events = list(self._events)
        self._events.clear()
        return events


def make_switchable_config(primary_values, secondary_values):
    return SwitchableTeleoperatorConfig(
        id="switchable",
        default_active="primary",
        keyboard_bindings={"1": "activate_primary", "2": "activate_secondary"},
        primary=MockTeleopConfig(id="primary", random_values=False, static_values=primary_values),
        secondary=MockTeleopConfig(id="secondary", random_values=False, static_values=secondary_values),
    )


def test_switchable_routes_action_based_on_events():
    teleop = SwitchableTeleoperator(make_switchable_config([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]))
    fake_event_source = FakeEventSource([TeleopEvent("activate_secondary"), TeleopEvent("activate_primary")])
    teleop._event_source = fake_event_source

    teleop.connect()

    action = teleop.get_action()
    assert action == {"motor_1.pos": 1.0, "motor_2.pos": 2.0, "motor_3.pos": 3.0}

    fake_event_source._events.append(TeleopEvent("activate_secondary"))
    action = teleop.get_action()
    assert action == {"motor_1.pos": 4.0, "motor_2.pos": 5.0, "motor_3.pos": 6.0}

    teleop.disconnect()
    assert fake_event_source.started is True
    assert fake_event_source.stopped is True


def test_switchable_validates_action_features():
    with pytest.raises(ValueError, match="action_features"):
        SwitchableTeleoperator(
            SwitchableTeleoperatorConfig(
                id="switchable",
                default_active="primary",
                keyboard_bindings={"1": "activate_primary", "2": "activate_secondary"},
                primary=MockTeleopConfig(id="primary", n_motors=2),
                secondary=MockTeleopConfig(id="secondary", n_motors=3),
            )
        )


def test_switchable_factory_builds_runtime():
    teleop = make_teleoperator_from_config(make_switchable_config([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]))
    assert isinstance(teleop, SwitchableTeleoperator)


def test_switchable_ignores_unknown_events():
    teleop = SwitchableTeleoperator(make_switchable_config([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]))
    teleop.handle_event(TeleopEvent("toggle_active"))

    assert teleop.active_teleoperator is teleop.primary
