from __future__ import annotations

import select
import sys
import termios
import threading
import time
import tty
from typing import Any

import numpy as np

from .config import KeyboardKeyMapping
from .input_base import TeleopInput
from .types import CommandFrame, TwistCommand


class KeyboardInput(TeleopInput):
    """Keyboard teleop input with viser-hook preference and terminal fallback."""

    def __init__(
        self,
        viewer: Any | None,
        key_mapping: KeyboardKeyMapping,
        key_hold_s: float = 0.12,
        default_frame: CommandFrame = "ee",
    ):
        self.key_mapping = key_mapping
        self.key_hold_s = key_hold_s
        self.default_frame = default_frame

        self._lock = threading.Lock()
        self._pressed_keys: set[str] = set()
        self._terminal_last_press: dict[str, int] = {}
        self._last_event_stamp_ns = time.monotonic_ns()
        self._pause_toggle_pending = False
        self._reset_home_pending = False
        self._reset_target_pending = False
        self._quit_requested = False
        self._key_event_count = 0

        self._terminal_thread: threading.Thread | None = None
        self._terminal_running = False
        self._old_term_attrs = None

        self._backend = "none"
        self._backend_note = ""

        self._viewer = viewer
        self._try_enable_viser_hooks(viewer)
        if self._backend == "none":
            self._start_terminal_fallback()

    @property
    def backend_name(self) -> str:
        return self._backend

    @property
    def backend_note(self) -> str:
        return self._backend_note

    @property
    def key_event_count(self) -> int:
        return self._key_event_count

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    def consume_pause_toggle(self) -> bool:
        with self._lock:
            result = self._pause_toggle_pending
            self._pause_toggle_pending = False
            return result

    def consume_reset_home(self) -> bool:
        with self._lock:
            result = self._reset_home_pending
            self._reset_home_pending = False
            return result

    def consume_reset_target(self) -> bool:
        with self._lock:
            result = self._reset_target_pending
            self._reset_target_pending = False
            return result

    def _normalize_key(self, key: str) -> str:
        key = key.lower().strip()
        if key == " ":
            return "space"
        return key

    def _register_key_press(self, key: str) -> None:
        key = self._normalize_key(key)
        now_ns = time.monotonic_ns()

        with self._lock:
            self._key_event_count += 1
            self._last_event_stamp_ns = now_ns
            self._pressed_keys.add(key)
            self._terminal_last_press[key] = now_ns

            if key == self.key_mapping.pause_toggle:
                self._pause_toggle_pending = True
            elif key == self.key_mapping.reset_home:
                self._reset_home_pending = True
            elif key == self.key_mapping.reset_target_to_current:
                self._reset_target_pending = True
            elif key == "x":
                self._quit_requested = True

    def _register_key_release(self, key: str) -> None:
        key = self._normalize_key(key)
        with self._lock:
            self._pressed_keys.discard(key)
            self._last_event_stamp_ns = time.monotonic_ns()

    def _extract_event_key(self, event: Any) -> str | None:
        if isinstance(event, str):
            return event
        for attr in ["key", "value", "name"]:
            if hasattr(event, attr):
                value = getattr(event, attr)
                if isinstance(value, str):
                    return value
        return None

    def _try_attach_client_key_hooks(self, client: Any) -> bool:
        down_name = None
        up_name = None
        for name in ["on_key_down", "on_keydown"]:
            if hasattr(client, name):
                down_name = name
                break
        for name in ["on_key_up", "on_keyup"]:
            if hasattr(client, name):
                up_name = name
                break
        if down_name is None or up_name is None:
            return False

        down_reg = getattr(client, down_name)
        up_reg = getattr(client, up_name)

        @down_reg
        def _on_key_down(event):
            key = self._extract_event_key(event)
            if key is not None:
                self._register_key_press(key)

        @up_reg
        def _on_key_up(event):
            key = self._extract_event_key(event)
            if key is not None:
                self._register_key_release(key)

        return True

    def _try_enable_viser_hooks(self, viewer: Any | None) -> None:
        if viewer is None:
            return

        attached_existing = 0
        try:
            clients = viewer.get_clients()
        except Exception:
            clients = {}
        for client in clients.values():
            if self._try_attach_client_key_hooks(client):
                attached_existing += 1

        if hasattr(viewer, "on_client_connect"):

            @viewer.on_client_connect
            def _on_client_connect(client):
                self._try_attach_client_key_hooks(client)

        if attached_existing > 0:
            self._backend = "viser"
            self._backend_note = "Using viser keyboard callbacks."
        else:
            self._backend_note = "Viser key callbacks unavailable in this environment; using terminal keyboard fallback."

    def _start_terminal_fallback(self) -> None:
        if not sys.stdin.isatty():
            if self._backend == "none":
                raise RuntimeError(
                    "No viser key callbacks were found and stdin is not a TTY for terminal fallback."
                )
            return

        self._old_term_attrs = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self._terminal_running = True
        self._terminal_thread = threading.Thread(
            target=self._terminal_loop, daemon=True
        )
        self._terminal_thread.start()
        self._backend = "terminal"

    def _terminal_loop(self) -> None:
        while self._terminal_running:
            ready, _, _ = select.select([sys.stdin], [], [], 0.01)
            if not ready:
                continue
            char = sys.stdin.read(1)
            if not char:
                continue
            self._register_key_press(char)

    def _axis_value_from_keys(self, pos_key: str, neg_key: str, now_ns: int) -> float:
        with self._lock:
            if self._backend == "terminal":
                pos_stamp = self._terminal_last_press.get(pos_key)
                neg_stamp = self._terminal_last_press.get(neg_key)
                hold_ns = int(self.key_hold_s * 1e9)
                pos_on = pos_stamp is not None and (now_ns - pos_stamp) <= hold_ns
                neg_on = neg_stamp is not None and (now_ns - neg_stamp) <= hold_ns
            else:
                # For callback-based backends we can rely on key down/up bookkeeping.
                pos_on = pos_key in self._pressed_keys
                neg_on = neg_key in self._pressed_keys

        if pos_on and not neg_on:
            return 1.0
        if neg_on and not pos_on:
            return -1.0
        return 0.0

    def read(self) -> TwistCommand:
        now_ns = time.monotonic_ns()

        if self._backend == "terminal":
            # Keep timestamp table bounded in long sessions.
            stale_ns = int(3.0 * self.key_hold_s * 1e9)
            with self._lock:
                remove_keys = [
                    key
                    for key, stamp in self._terminal_last_press.items()
                    if (now_ns - stamp) > stale_ns
                ]
                for key in remove_keys:
                    self._terminal_last_press.pop(key, None)

        tx = self._axis_value_from_keys(
            self.key_mapping.tx_pos, self.key_mapping.tx_neg, now_ns
        )
        ty = self._axis_value_from_keys(
            self.key_mapping.ty_pos, self.key_mapping.ty_neg, now_ns
        )
        tz = self._axis_value_from_keys(
            self.key_mapping.tz_pos, self.key_mapping.tz_neg, now_ns
        )
        rx = self._axis_value_from_keys(
            self.key_mapping.rx_pos, self.key_mapping.rx_neg, now_ns
        )
        ry = self._axis_value_from_keys(
            self.key_mapping.ry_pos, self.key_mapping.ry_neg, now_ns
        )
        rz = self._axis_value_from_keys(
            self.key_mapping.rz_pos, self.key_mapping.rz_neg, now_ns
        )
        gripper = self._axis_value_from_keys(
            self.key_mapping.gripper_open,
            self.key_mapping.gripper_close,
            now_ns,
        )

        with self._lock:
            stamp_ns = self._last_event_stamp_ns

        return TwistCommand(
            linear_xyz=np.array([tx, ty, tz], dtype=float),
            angular_xyz=np.array([rx, ry, rz], dtype=float),
            gripper=gripper,
            frame=self.default_frame,
            stamp_ns=stamp_ns,
        )

    def is_active(self) -> bool:
        if self._backend == "viser":
            return True
        if self._backend == "terminal":
            return self._terminal_running
        return False

    def close(self) -> None:
        self._terminal_running = False
        if self._terminal_thread is not None and self._terminal_thread.is_alive():
            self._terminal_thread.join(timeout=0.5)
        if self._old_term_attrs is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term_attrs)
            self._old_term_attrs = None
