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

import logging
from typing import Any

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..teleoperator import Teleoperator
from ..utils import make_teleoperator_from_config
from .config_switchable import SwitchableTeleoperatorConfig
from .keyboard_events import KeyboardEventSource, TeleopEvent

logger = logging.getLogger(__name__)


class SwitchableTeleoperator(Teleoperator):
    config_class = SwitchableTeleoperatorConfig
    name = "switchable"

    def __init__(self, config: SwitchableTeleoperatorConfig):
        super().__init__(config)
        self.config = config
        self.primary = make_teleoperator_from_config(config.primary)
        self.secondary = make_teleoperator_from_config(config.secondary)
        self._event_source = KeyboardEventSource(config.keyboard_bindings)
        self._active_name = config.default_active
        self._validate_compatibility()

    @property
    def active_teleoperator(self) -> Teleoperator:
        return self.primary if self._active_name == "primary" else self.secondary

    @property
    def action_features(self) -> dict:
        return self.active_teleoperator.action_features

    @property
    def feedback_features(self) -> dict:
        return self.active_teleoperator.feedback_features

    @property
    def is_connected(self) -> bool:
        return self.primary.is_connected and self.secondary.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.primary.connect(calibrate)
        try:
            self.secondary.connect(calibrate)
            self._event_source.start()
        except Exception:
            if self.secondary.is_connected:
                self.secondary.disconnect()
            if self.primary.is_connected:
                self.primary.disconnect()
            raise

    @property
    def is_calibrated(self) -> bool:
        return self.primary.is_calibrated and self.secondary.is_calibrated

    def calibrate(self) -> None:
        self.primary.calibrate()
        self.secondary.calibrate()

    def configure(self) -> None:
        self.primary.configure()
        self.secondary.configure()

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        for event in self._event_source.drain_events():
            self.handle_event(event)
        return self.active_teleoperator.get_action()

    def handle_event(self, event: TeleopEvent) -> None:
        if event.type == "activate_primary":
            self._active_name = "primary"
        elif event.type == "activate_secondary":
            self._active_name = "secondary"
        else:
            logger.warning("Ignoring unsupported teleop event '%s'.", event.type)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.active_teleoperator.send_feedback(feedback)

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        stop_error = None
        if self._event_source.bindings:
            try:
                self._event_source.stop()
            except DeviceNotConnectedError:
                pass
            except Exception as exc:
                stop_error = exc

        disconnect_error = None
        for teleop in (self.secondary, self.primary):
            if teleop.is_connected:
                try:
                    teleop.disconnect()
                except Exception as exc:
                    disconnect_error = disconnect_error or exc

        if stop_error is not None:
            raise stop_error
        if disconnect_error is not None:
            raise disconnect_error

    def _validate_compatibility(self) -> None:
        primary_action = self.primary.action_features
        secondary_action = self.secondary.action_features
        if primary_action != secondary_action:
            raise ValueError("Switchable teleoperators must expose identical action_features.")

        primary_feedback = self.primary.feedback_features
        secondary_feedback = self.secondary.feedback_features
        if primary_feedback != secondary_feedback:
            raise ValueError("Switchable teleoperators must expose identical feedback_features.")
