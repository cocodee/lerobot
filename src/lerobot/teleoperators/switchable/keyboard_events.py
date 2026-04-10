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
import os
import sys
from dataclasses import dataclass, field
from queue import Empty, Queue
from typing import Any

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

logger = logging.getLogger(__name__)

PYNPUT_AVAILABLE = True
try:
    if ("DISPLAY" not in os.environ) and ("linux" in sys.platform):
        raise ImportError("pynput blocked intentionally due to no display.")

    from pynput import keyboard
except Exception:
    keyboard = None
    PYNPUT_AVAILABLE = False


@dataclass(frozen=True)
class TeleopEvent:
    type: str
    payload: dict[str, Any] = field(default_factory=dict)


class KeyboardEventSource:
    def __init__(self, bindings: dict[str, str]):
        self.bindings = bindings
        self._listener = None
        self._event_queue: Queue[TeleopEvent] = Queue()

    @property
    def is_running(self) -> bool:
        return (
            self._listener is not None
            and keyboard is not None
            and isinstance(self._listener, keyboard.Listener)
            and self._listener.is_alive()
        )

    def start(self) -> None:
        if not self.bindings:
            return

        if self.is_running:
            raise DeviceAlreadyConnectedError("Keyboard event source is already running.")

        if not PYNPUT_AVAILABLE or keyboard is None:
            raise RuntimeError("Keyboard bindings were configured but pynput is not available in this environment.")

        self._listener = keyboard.Listener(on_press=self._on_press)
        self._listener.start()

    def _on_press(self, key) -> None:
        if not hasattr(key, "char") or key.char is None:
            return

        command = self.bindings.get(key.char)
        if command is None:
            return

        self._event_queue.put(TeleopEvent(type=command))

    def drain_events(self) -> list[TeleopEvent]:
        events: list[TeleopEvent] = []
        while True:
            try:
                events.append(self._event_queue.get_nowait())
            except Empty:
                return events

    def stop(self) -> None:
        if not self.bindings:
            return

        if self._listener is None:
            raise DeviceNotConnectedError("Keyboard event source is not running.")

        self._listener.stop()
        self._listener = None
