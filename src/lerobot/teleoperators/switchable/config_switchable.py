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

from dataclasses import dataclass, field

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("switchable")
@dataclass
class SwitchableTeleoperatorConfig(TeleoperatorConfig):
    primary: TeleoperatorConfig
    secondary: TeleoperatorConfig
    keyboard_bindings: dict[str, str] = field(default_factory=dict)
    default_active: str = "primary"

    def __post_init__(self):
        valid_targets = {"primary", "secondary"}

        if self.default_active not in valid_targets:
            raise ValueError(f"default_active must be one of {sorted(valid_targets)}")

        for key, command in self.keyboard_bindings.items():
            if not key:
                raise ValueError("keyboard binding keys must be non-empty strings")
            if not command:
                raise ValueError("keyboard binding commands must be non-empty strings")
