# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

"""
Emergency stop mechanism for robot safety.

Monitors various conditions and can trigger immediate halt
of robot motion when unsafe conditions are detected.
"""

import logging
import threading
import time
from enum import Enum
from typing import Optional, Callable, List
from dataclasses import dataclass

from .safety_config import EmergencyStopConfig

logger = logging.getLogger(__name__)


class EmergencyStopState(Enum):
    """States for emergency stop state machine."""
    NORMAL = "normal"
    TRIGGERED = "triggered"
    RECOVERING = "recovering"


@dataclass
class EmergencyStopEvent:
    """Record of an emergency stop event."""
    timestamp: float
    trigger_reason: str
    trigger_source: str  # 'collision', 'velocity', 'force', 'watchdog', 'manual'
    state_before: dict
    recovered: bool = False
    recovery_time: Optional[float] = None


class EmergencyStopController:
    """
    Manages emergency stop functionality for robot safety.

    Monitors various conditions and can trigger immediate halt
    of robot motion when unsafe conditions are detected.

    Args:
        config: Emergency stop configuration.
        on_stop_callback: Optional callback function called when emergency stop is triggered.
            Receives an EmergencyStopEvent as argument.

    Example:
        ```python
        def on_stop(event):
            logger.critical(f"EMERGENCY STOP: {event.trigger_reason}")
            # Send zero torque to motors, etc.

        controller = EmergencyStopController(
            config=EmergencyStopConfig(enabled=True, watchdog_timeout=0.5),
            on_stop_callback=on_stop
        )

        # Feed watchdog in control loop
        while running:
            controller.feed_watchdog()
            # ... control code ...

        # Trigger manually if needed
        controller.trigger_stop("Manual stop requested", "manual", current_state)
        ```
    """

    def __init__(
        self,
        config: EmergencyStopConfig,
        on_stop_callback: Optional[Callable[[EmergencyStopEvent], None]] = None,
    ):
        self.config = config
        self.on_stop_callback = on_stop_callback

        self.state = EmergencyStopState.NORMAL
        self.last_watchdog_feed = time.time()
        self.stop_events: List[EmergencyStopEvent] = []

        # Lock for thread-safe state updates
        self._lock = threading.Lock()

        # Watchdog thread
        self._watchdog_thread = None
        self._watchdog_running = False

        if self.config.enabled:
            self._start_watchdog()
            logger.info(
                f"EmergencyStopController initialized (timeout: {config.watchdog_timeout}s, "
                f"auto_recovery: {config.auto_recovery})"
            )

    def feed_watchdog(self):
        """
        Feed the watchdog to prevent timeout.

        Call this in your control loop to indicate communication is healthy.
        If this is not called within watchdog_timeout seconds, emergency stop
        will be triggered automatically.
        """
        with self._lock:
            self.last_watchdog_feed = time.time()

    def trigger_stop(
        self,
        reason: str,
        source: str,
        current_state: dict,
    ) -> bool:
        """
        Trigger emergency stop.

        Args:
            reason: Human-readable reason for the stop.
            source: Source of the trigger ('collision', 'velocity', 'force', 'watchdog', 'manual').
            current_state: Current robot state for logging.

        Returns:
            True if stop was triggered, False if already stopped.
        """
        with self._lock:
            if self.state == EmergencyStopState.TRIGGERED:
                return False

            self.state = EmergencyStopState.TRIGGERED

            # Record event
            event = EmergencyStopEvent(
                timestamp=time.time(),
                trigger_reason=reason,
                trigger_source=source,
                state_before=current_state.copy(),
            )
            self.stop_events.append(event)

            logger.critical(
                f"EMERGENCY STOP TRIGGERED: {reason} (source: {source}, "
                f"events: {len(self.stop_events)})"
            )

            # Execute callback
            if self.on_stop_callback:
                try:
                    self.on_stop_callback(event)
                except Exception as e:
                    logger.error(f"Error in emergency stop callback: {e}")

            return True

    def is_stopped(self) -> bool:
        """
        Check if emergency stop is currently active.

        Returns:
            True if in TRIGGERED state, False otherwise.
        """
        with self._lock:
            return self.state == EmergencyStopState.TRIGGERED

    def can_proceed(self) -> bool:
        """
        Check if robot can proceed with normal operation.

        Returns:
            True if in NORMAL state, False otherwise.
        """
        with self._lock:
            return self.state == EmergencyStopState.NORMAL

    def reset(self, manual: bool = False) -> bool:
        """
        Attempt to reset emergency stop.

        If auto_recovery is enabled, this will automatically attempt recovery.
        If auto_recovery is disabled, manual=True must be passed to reset.

        Args:
            manual: Set to True to manually force reset (only works if auto_recovery=False).

        Returns:
            True if reset successful, False otherwise.
        """
        with self._lock:
            if self.state == EmergencyStopState.NORMAL:
                return True

            if not self.config.auto_recovery and not manual:
                logger.warning("Auto-recovery disabled, manual reset required (use reset(manual=True))")
                return False

            if self.state == EmergencyStopState.TRIGGERED:
                self.state = EmergencyStopState.RECOVERING
                logger.info("Attempting emergency stop recovery...")

                # Mark last event as recovered
                if self.stop_events:
                    self.stop_events[-1].recovered = True
                    self.stop_events[-1].recovery_time = time.time()

                # Transition to normal state
                self.state = EmergencyStopState.NORMAL
                self.last_watchdog_feed = time.time()

                logger.info("Emergency stop recovered - robot can proceed")
                return True

            return False

    def get_state(self) -> EmergencyStopState:
        """Get current emergency stop state."""
        with self._lock:
            return self.state

    def get_event_count(self) -> int:
        """Get number of emergency stop events recorded."""
        with self._lock:
            return len(self.stop_events)

    def get_last_event(self) -> Optional[EmergencyStopEvent]:
        """Get the most recent emergency stop event."""
        with self._lock:
            return self.stop_events[-1] if self.stop_events else None

    def _start_watchdog(self):
        """Start watchdog thread to monitor communication health."""
        self._watchdog_running = True

        def watchdog_loop():
            while self._watchdog_running:
                time.sleep(0.1)  # Check at 10Hz

                with self._lock:
                    if self.state == EmergencyStopState.TRIGGERED:
                        continue

                    time_since_feed = time.time() - self.last_watchdog_feed

                    if time_since_feed > self.config.watchdog_timeout:
                        self.state = EmergencyStopState.TRIGGERED

                        event = EmergencyStopEvent(
                            timestamp=time.time(),
                            trigger_reason=f"Watchdog timeout: {time_since_feed:.2f}s",
                            trigger_source="watchdog",
                            state_before={},
                        )
                        self.stop_events.append(event)

                        logger.critical(
                            f"EMERGENCY STOP: Watchdog timeout ({time_since_feed:.2f}s > "
                            f"{self.config.watchdog_timeout}s)"
                        )

                        # Execute callback
                        if self.on_stop_callback:
                            try:
                                self.on_stop_callback(event)
                            except Exception as e:
                                logger.error(f"Error in emergency stop callback: {e}")

        self._watchdog_thread = threading.Thread(target=watchdog_loop, daemon=True)
        self._watchdog_thread.start()

    def stop(self):
        """Stop the watchdog thread. Call this when shutting down."""
        self._watchdog_running = False
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=1.0)
