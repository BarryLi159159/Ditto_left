#!/usr/bin/env python3
"""
Hand config tuning GUI.

Load a hand_config YAML, edit per-motor parameters, then use keyboard shortcuts:

  u — save tuned copy (*_tuned.yaml) and set that file for the next teleop run
  f — start / stop teleop (uses last saved tuned config after u; stop before re-saving)
  q — quit
"""

from __future__ import annotations

import argparse
import copy
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import yaml
from hydra import compose, initialize_config_dir
from omegaconf import DictConfig, OmegaConf

_DITTO_ROOT = Path(__file__).resolve().parent.parent
_CONF_DIR = _DITTO_ROOT / "hand_interfaces" / "conf"
_HAND_CONFIG_DIR = _CONF_DIR / "hand_config"

if str(_DITTO_ROOT) not in sys.path:
    sys.path.insert(0, str(_DITTO_ROOT))

from controllers.teleop_controller import TeleopHand


@dataclass(frozen=True)
class FloatParam:
    key: str
    label: str
    minimum: float
    maximum: float
    resolution: float = 0.01


LEADER_FORCE_PARAMS: Tuple[FloatParam, ...] = (
    FloatParam("force_rendering_gain", "Force gain", 0.0, 2.0, 0.01),
    FloatParam("force_rendering_alpha", "Force alpha", 0.0, 1.0, 0.01),
    FloatParam("force_rendering_threshold", "Force threshold (mA)", 0.0, 300.0, 1.0),
    FloatParam("force_rendering_max_current", "Force max current (mA)", 0.0, 2300.0, 10.0),
)

FOLLOWER_CHAIN_PARAMS: Tuple[FloatParam, ...] = (
    FloatParam("velocity_limit", "Velocity limit", 0.0, 500.0, 1.0),
    FloatParam("p_gain", "P gain", 0.0, 4000.0, 10.0),
    FloatParam("i_gain", "I gain", 0.0, 2000.0, 10.0),
    FloatParam("d_gain", "D gain", 0.0, 500.0, 5.0),
)

FOLLOWER_MOTOR_PARAMS: Tuple[FloatParam, ...] = (
    FloatParam("p_gain", "P gain (override)", 0.0, 4000.0, 10.0),
    FloatParam("i_gain", "I gain (override)", 0.0, 2000.0, 10.0),
    FloatParam("d_gain", "D gain (override)", 0.0, 500.0, 5.0),
)


def _deep_copy_dict(data: Any) -> Any:
    return copy.deepcopy(data)


def _load_yaml(path: Path) -> Tuple[Dict[str, Any], Optional[str]]:
    """Return (hand_config dict, optional @package header line)."""
    text = path.read_text(encoding="utf-8")
    header = None
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("# @package"):
            header = line.rstrip("\n")
            break
        if stripped and not stripped.startswith("#"):
            break
    raw = yaml.safe_load(text) or {}
    if "hand_config" in raw:
        return _deep_copy_dict(raw["hand_config"]), header
    return _deep_copy_dict(raw), header


def _tuned_output_path(source: Path) -> Path:
    """Path for a tuned copy; never overwrites the source file."""
    stem = source.stem
    if stem.endswith("_tuned"):
        return source.parent / f"{stem}{source.suffix}"
    return source.parent / f"{stem}_tuned{source.suffix}"


def _hand_config_hydra_override(path: Path) -> str:
    """Map filesystem path to Hydra hand_config=group/file override."""
    try:
        rel = path.resolve().relative_to(_HAND_CONFIG_DIR.resolve())
    except ValueError:
        raise ValueError(
            f"Hand config must live under {_HAND_CONFIG_DIR} for teleop; got {path}"
        ) from None
    return str(rel.with_suffix("")).replace(os.sep, "/")


def _motor_ids(chain: Dict[str, Any]) -> List[int]:
    ids = chain.get("motor_ids") or []
    return [int(x) for x in ids]


def _find_joint(chain: Dict[str, Any], motor_id: int) -> Dict[str, Any]:
    joints = chain.setdefault("joint_settings", [])
    for entry in joints:
        if int(entry.get("motor_id", -1)) == motor_id:
            return entry
    entry = {"motor_id": motor_id}
    joints.append(entry)
    return entry


def _get_joint(chain: Dict[str, Any], motor_id: int) -> Optional[Dict[str, Any]]:
    for entry in chain.get("joint_settings") or []:
        if int(entry.get("motor_id", -1)) == motor_id:
            return entry
    return None


def _chain_general(chain: Dict[str, Any]) -> Dict[str, Any]:
    return chain.setdefault("general_settings", {})


def _leader_current_defaults() -> Dict[str, Any]:
    return {
        "enable_force_rendering": False,
        "gradual_rendering": False,
        "force_rendering_gain": 0.0,
        "force_rendering_alpha": 0.95,
        "force_rendering_threshold": 50.0,
        "force_rendering_max_current": 200.0,
    }


def _follower_chain_defaults(chain: Dict[str, Any]) -> Dict[str, float]:
    """Chain-level follower PID defaults from YAML or sensible fallbacks."""
    general = chain.get("general_settings") or {}
    return {
        "velocity_limit": float(general.get("velocity_limit", 200)),
        "p_gain": float(general.get("p_gain", 1500)),
        "i_gain": float(general.get("i_gain", 0)),
        "d_gain": float(general.get("d_gain", 75)),
    }


def _effective_follower_pid(
    chain: Dict[str, Any], motor_id: int
) -> Tuple[Dict[str, float], bool]:
    general = _chain_general(chain)
    base = {
        "velocity_limit": float(general.get("velocity_limit", 150)),
        "p_gain": float(general.get("p_gain", 500)),
        "i_gain": float(general.get("i_gain", 0)),
        "d_gain": float(general.get("d_gain", 0)),
    }
    joint = _get_joint(chain, motor_id)
    if joint and "motor_settings" in joint:
        ms = joint["motor_settings"]
        for key in ("p_gain", "i_gain", "d_gain"):
            if key in ms:
                base[key] = float(ms[key])
        return base, True
    return base, False


class HandConfigTuneGUI:
    """Tkinter GUI for editing hand_config motor parameters."""

    COLORS = {
        "bg_dark": "#1a1a2e",
        "bg_medium": "#16213e",
        "bg_light": "#0f3460",
        "accent": "#e94560",
        "accent_hover": "#ff6b6b",
        "success": "#00bf63",
        "warning": "#ffc107",
        "text_primary": "#eaeaea",
        "text_secondary": "#9a9a9a",
        "border": "#2d3a4f",
    }

    LOG_TAGS = {
        "info": "#9a9a9a",
        "success": "#00bf63",
        "warning": "#ffc107",
        "error": "#ef4444",
    }

    def __init__(self, initial_path: Optional[str] = None, u2d2: str = "dual", fake_u2d2: bool = False):
        self.source_config_path: Optional[Path] = (
            Path(initial_path).resolve() if initial_path else None
        )
        self.run_config_path: Optional[Path] = None
        self.package_header: Optional[str] = "# @package hand_config"
        self.hand_config: Dict[str, Any] = {}
        self.dirty = False

        self.u2d2_profile = u2d2
        self.fake_u2d2 = fake_u2d2
        self.teleop_hand: Optional[TeleopHand] = None
        self.teleop_thread: Optional[threading.Thread] = None
        self.teleop_running = False

        self._motor_vars: Dict[Tuple[str, int], Dict[str, tk.Variable]] = {}
        self._chain_vars: Dict[str, Dict[str, tk.Variable]] = {}
        self._motor_listboxes: Dict[str, tk.Listbox] = {}
        self._detail_frames: Dict[str, ttk.Frame] = {}
        self._selected_motor: Dict[str, int] = {}
        self._suppress_config_sync = False

        self.root = tk.Tk()
        self.root.title("Hand Config Tuner")
        self.root.configure(bg=self.COLORS["bg_dark"])
        try:
            sw = self.root.winfo_screenwidth()
            w, h = 1280, 820
            self.root.geometry(f"{w}x{h}+{max(0, sw - w - 20)}+40")
            self.root.minsize(1000, 640)
        except Exception:
            self.root.geometry("1280x820")

        self.u2d2_var = tk.StringVar(value=self.u2d2_profile)
        self.fake_u2d2_var = tk.BooleanVar(value=self.fake_u2d2)
        self.status_var = tk.StringVar(value="No config loaded")

        self._configure_styles()
        self._build_ui()

        if self.source_config_path and self.source_config_path.is_file():
            self._load_config_file(self.source_config_path)
        elif self.source_config_path is None:
            default = _HAND_CONFIG_DIR / "7dof" / "7dof.yaml"
            if default.is_file():
                self._load_config_file(default)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._bind_keyboard_shortcuts()
        self._poll_log_queue()

    # ------------------------------------------------------------------ UI chrome
    def _configure_styles(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Dark.TFrame", background=self.COLORS["bg_dark"])
        style.configure("Medium.TFrame", background=self.COLORS["bg_medium"])
        style.configure(
            "Title.TLabel",
            background=self.COLORS["bg_dark"],
            foreground=self.COLORS["text_primary"],
            font=("JetBrains Mono", 16, "bold"),
        )
        style.configure(
            "Header.TLabel",
            background=self.COLORS["bg_medium"],
            foreground=self.COLORS["accent"],
            font=("JetBrains Mono", 12, "bold"),
        )
        style.configure(
            "Normal.TLabel",
            background=self.COLORS["bg_medium"],
            foreground=self.COLORS["text_primary"],
            font=("JetBrains Mono", 10),
        )
        style.configure(
            "Secondary.TLabel",
            background=self.COLORS["bg_medium"],
            foreground=self.COLORS["text_secondary"],
            font=("JetBrains Mono", 9),
        )

    def _create_button(
        self, parent, text: str, command: Callable, style: str = "neutral"
    ) -> tk.Button:
        palettes = {
            "start": (self.COLORS["success"], "#0ea55a"),
            "stop": (self.COLORS["accent"], self.COLORS["accent_hover"]),
            "neutral": (self.COLORS["bg_light"], "#1a4a7a"),
        }
        bg, hover = palettes.get(style, palettes["neutral"])
        btn = tk.Button(
            parent,
            text=text,
            command=command,
            bg=bg,
            fg=self.COLORS["text_primary"],
            activebackground=hover,
            activeforeground=self.COLORS["text_primary"],
            font=("JetBrains Mono", 10, "bold"),
            relief=tk.FLAT,
            padx=10,
            pady=4,
            cursor="hand2",
        )
        btn.bind("<Enter>", lambda _e: btn.configure(bg=hover))
        btn.bind("<Leave>", lambda _e: btn.configure(bg=bg))
        return btn

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, style="Dark.TFrame", padding=14)
        main.pack(fill=tk.BOTH, expand=True)

        title_row = ttk.Frame(main, style="Dark.TFrame")
        title_row.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(title_row, text="Hand Config Tuner", style="Title.TLabel").pack(side=tk.LEFT)

        self._create_button(title_row, "Load…", self._load_dialog).pack(side=tk.RIGHT, padx=(6, 0))

        toolbar = ttk.Frame(main, style="Dark.TFrame")
        toolbar.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(toolbar, text="Source:", style="Secondary.TLabel").pack(side=tk.LEFT)
        self.path_label = ttk.Label(toolbar, text="—", style="Normal.TLabel")
        self.path_label.pack(side=tk.LEFT, padx=(6, 8))
        ttk.Label(toolbar, text="Save:", style="Secondary.TLabel").pack(side=tk.LEFT)
        self.save_path_label = ttk.Label(toolbar, text="—", style="Secondary.TLabel")
        self.save_path_label.pack(side=tk.LEFT, padx=(6, 16))
        ttk.Label(toolbar, text="u2d2:", style="Secondary.TLabel").pack(side=tk.LEFT)
        u2d2_combo = ttk.Combobox(
            toolbar,
            textvariable=self.u2d2_var,
            values=self._discover_u2d2_profiles(),
            width=10,
            state="readonly",
        )
        u2d2_combo.pack(side=tk.LEFT, padx=(4, 12))
        tk.Checkbutton(
            toolbar,
            text="fake_u2d2",
            variable=self.fake_u2d2_var,
            bg=self.COLORS["bg_dark"],
            fg=self.COLORS["text_primary"],
            selectcolor=self.COLORS["bg_medium"],
            activebackground=self.COLORS["bg_dark"],
            activeforeground=self.COLORS["text_primary"],
            font=("JetBrains Mono", 9),
        ).pack(side=tk.LEFT)
        ttk.Label(toolbar, textvariable=self.status_var, style="Secondary.TLabel").pack(
            side=tk.RIGHT
        )

        keys_row = ttk.Frame(main, style="Dark.TFrame")
        keys_row.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(
            keys_row,
            text="Keys:  u = save tuned YAML (update settings)   f = start/stop teleop   q = quit",
            style="Secondary.TLabel",
        ).pack(anchor=tk.W)
        ttk.Label(
            keys_row,
            text="Run: tune sliders → u → f (start) → f (stop) → tune → u → f …",
            style="Secondary.TLabel",
        ).pack(anchor=tk.W, pady=(2, 0))

        content = ttk.Frame(main, style="Dark.TFrame")
        content.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(content, style="Dark.TFrame")
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

        self.notebook = ttk.Notebook(left)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        for chain_name, title in (("leader", "Leader (current / force)"), ("follower", "Follower (position PID)")):
            tab = ttk.Frame(self.notebook, style="Medium.TFrame", padding=8)
            self.notebook.add(tab, text=title.split()[0])
            self._build_chain_tab(tab, chain_name)

        right = ttk.Frame(content, style="Dark.TFrame")
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)
        ttk.Label(right, text="LOG", style="Title.TLabel").pack(anchor=tk.W)
        log_frame = tk.Frame(right, bg=self.COLORS["bg_medium"], padx=8, pady=8)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(
            log_frame,
            width=42,
            height=36,
            bg=self.COLORS["bg_medium"],
            fg=self.COLORS["text_primary"],
            font=("JetBrains Mono", 9),
            wrap=tk.WORD,
            state=tk.DISABLED,
            relief=tk.FLAT,
            highlightthickness=1,
            highlightbackground=self.COLORS["border"],
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        for tag, color in self.LOG_TAGS.items():
            self.log_text.tag_configure(tag, foreground=color)
        self._log_queue: List[Tuple[str, str]] = []

    def _discover_u2d2_profiles(self) -> List[str]:
        u2d2_dir = _CONF_DIR / "u2d2"
        if not u2d2_dir.is_dir():
            return ["default", "dual"]
        names = sorted(p.stem for p in u2d2_dir.glob("*.yaml"))
        return names or ["default", "dual"]

    def _build_chain_tab(self, parent: ttk.Frame, chain_name: str) -> None:
        if chain_name == "leader":
            global_row = ttk.Frame(parent, style="Medium.TFrame")
            global_row.pack(fill=tk.X, pady=(0, 8))
            self._disable_force_var = tk.BooleanVar(value=False)

            def _on_disable_force(*_args):
                if self._suppress_config_sync:
                    return
                self.hand_config.setdefault("leader", {})[
                    "disable_force_rendering_global"
                ] = bool(self._disable_force_var.get())
                self._mark_dirty()

            tk.Checkbutton(
                global_row,
                text="disable_force_rendering_global",
                variable=self._disable_force_var,
                command=_on_disable_force,
                bg=self.COLORS["bg_medium"],
                fg=self.COLORS["text_primary"],
                selectcolor=self.COLORS["bg_light"],
                activebackground=self.COLORS["bg_medium"],
                font=("JetBrains Mono", 9),
            ).pack(side=tk.LEFT)

        if chain_name == "follower":
            chain_panel = tk.Frame(parent, bg=self.COLORS["bg_medium"], padx=10, pady=8)
            chain_panel.pack(fill=tk.X, pady=(0, 8))
            ttk.Label(chain_panel, text="Chain defaults (general_settings)", style="Header.TLabel").pack(
                anchor=tk.W
            )
            self._chain_vars[chain_name] = {}
            grid = ttk.Frame(chain_panel, style="Medium.TFrame")
            grid.pack(fill=tk.X, pady=(6, 0))
            chain = self.hand_config.get("follower", {})
            chain_defaults = _follower_chain_defaults(chain)
            for col, spec in enumerate(FOLLOWER_CHAIN_PARAMS):
                self._add_float_control(
                    grid,
                    spec,
                    self._chain_vars[chain_name],
                    row=0,
                    col=col,
                    initial=chain_defaults.get(spec.key),
                    on_change=lambda c=chain_name: self._sync_chain_to_config(c),
                )

        body = ttk.Frame(parent, style="Medium.TFrame")
        body.pack(fill=tk.BOTH, expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        list_frame = ttk.Frame(body, style="Medium.TFrame")
        list_frame.grid(row=0, column=0, sticky="ns", padx=(0, 10))
        ttk.Label(list_frame, text="Motors", style="Header.TLabel").pack(anchor=tk.W)
        lb = tk.Listbox(
            list_frame,
            width=10,
            height=18,
            bg=self.COLORS["bg_light"],
            fg=self.COLORS["text_primary"],
            selectbackground=self.COLORS["accent"],
            selectforeground=self.COLORS["text_primary"],
            font=("JetBrains Mono", 11),
            relief=tk.FLAT,
            highlightthickness=0,
        )
        lb.pack(fill=tk.Y, expand=True, pady=(6, 0))
        lb.bind("<<ListboxSelect>>", lambda e, c=chain_name: self._on_motor_select(c))
        self._motor_listboxes[chain_name] = lb

        detail_outer = ttk.Frame(body, style="Medium.TFrame")
        detail_outer.grid(row=0, column=1, sticky="nsew")
        detail_canvas = tk.Canvas(
            detail_outer,
            bg=self.COLORS["bg_medium"],
            highlightthickness=0,
        )
        detail_scroll = ttk.Scrollbar(detail_outer, orient=tk.VERTICAL, command=detail_canvas.yview)
        detail_frame = ttk.Frame(detail_canvas, style="Medium.TFrame")
        detail_frame.bind(
            "<Configure>",
            lambda e, cv=detail_canvas: cv.configure(scrollregion=cv.bbox("all")),
        )
        detail_canvas.create_window((0, 0), window=detail_frame, anchor="nw")
        detail_canvas.configure(yscrollcommand=detail_scroll.set)
        detail_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        detail_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self._detail_frames[chain_name] = detail_frame

    def _add_float_control(
        self,
        parent: ttk.Frame,
        spec: FloatParam,
        storage: Dict[str, tk.Variable],
        *,
        row: int,
        col: int,
        initial: Optional[float] = None,
        on_change: Optional[Callable[[], None]] = None,
    ) -> None:
        cell = ttk.Frame(parent, style="Medium.TFrame", padding=(0, 0, 12, 6))
        cell.grid(row=row, column=col, sticky=tk.NW)
        ttk.Label(cell, text=spec.label, style="Secondary.TLabel").pack(anchor=tk.W)
        if initial is None:
            initial = spec.minimum
        var = tk.DoubleVar(value=float(initial))
        storage[spec.key] = var

        def _changed(*_args):
            if self._suppress_config_sync:
                return
            if on_change:
                on_change()

        scale = tk.Scale(
            cell,
            variable=var,
            from_=spec.minimum,
            to=spec.maximum,
            resolution=spec.resolution,
            orient=tk.HORIZONTAL,
            length=200,
            bg=self.COLORS["bg_medium"],
            fg=self.COLORS["text_primary"],
            troughcolor=self.COLORS["bg_light"],
            highlightthickness=0,
            command=lambda _v: _changed(),
        )
        scale.pack(anchor=tk.W)
        entry = tk.Entry(
            cell,
            textvariable=var,
            width=8,
            bg=self.COLORS["bg_light"],
            fg=self.COLORS["text_primary"],
            insertbackground=self.COLORS["text_primary"],
            relief=tk.FLAT,
            font=("JetBrains Mono", 9),
        )
        entry.pack(anchor=tk.W, pady=(2, 0))
        var.trace_add("write", lambda *_a: _changed())

    def _add_bool_control(
        self,
        parent: ttk.Frame,
        key: str,
        label: str,
        storage: Dict[str, tk.Variable],
        on_change: Optional[Callable[[], None]] = None,
    ) -> None:
        var = tk.BooleanVar(value=False)
        storage[key] = var

        def _changed():
            if self._suppress_config_sync:
                return
            if on_change:
                on_change()

        tk.Checkbutton(
            parent,
            text=label,
            variable=var,
            command=_changed,
            bg=self.COLORS["bg_medium"],
            fg=self.COLORS["text_primary"],
            selectcolor=self.COLORS["bg_light"],
            activebackground=self.COLORS["bg_medium"],
            font=("JetBrains Mono", 10),
        ).pack(anchor=tk.W, pady=4)

    def _build_motor_detail(self, chain_name: str, motor_id: int) -> None:
        self._suppress_config_sync = True
        frame = self._detail_frames[chain_name]
        for w in frame.winfo_children():
            w.destroy()

        chain = self.hand_config.get(chain_name, {})
        ttk.Label(frame, text=f"Motor {motor_id}", style="Header.TLabel").pack(anchor=tk.W)

        key = (chain_name, motor_id)
        self._motor_vars[key] = {}

        if chain_name == "leader":
            joint = _find_joint(chain, motor_id)
            cc = dict(joint.get("current_control") or {})
            defaults = _leader_current_defaults()
            for k, v in defaults.items():
                cc.setdefault(k, v)
            storage = self._motor_vars[key]

            self._add_bool_control(
                frame,
                "enable_force_rendering",
                "enable_force_rendering",
                storage,
                on_change=lambda: self._sync_motor_to_config(chain_name, motor_id),
            )
            self._add_bool_control(
                frame,
                "gradual_rendering",
                "gradual_rendering",
                storage,
                on_change=lambda: self._sync_motor_to_config(chain_name, motor_id),
            )
            grid = ttk.Frame(frame, style="Medium.TFrame")
            grid.pack(fill=tk.X, pady=(4, 0))
            for col, spec in enumerate(LEADER_FORCE_PARAMS):
                initial = float(cc.get(spec.key, defaults.get(spec.key, spec.minimum)))
                self._add_float_control(
                    grid,
                    spec,
                    storage,
                    row=0,
                    col=col,
                    initial=initial,
                    on_change=lambda: self._sync_motor_to_config(chain_name, motor_id),
                )
            for k, var in storage.items():
                if k.startswith("__"):
                    continue
                if k in cc:
                    if isinstance(var, tk.BooleanVar):
                        var.set(bool(cc[k]))
                    else:
                        var.set(float(cc[k]))
                elif k in ("enable_force_rendering", "gradual_rendering"):
                    var.set(bool(defaults.get(k, False)))
            self._suppress_config_sync = False
            return

        # Follower
        effective, has_override = _effective_follower_pid(chain, motor_id)
        storage = self._motor_vars[key]
        override_var = tk.BooleanVar(value=has_override)
        storage["__override__"] = override_var

        def _toggle_override():
            joint = _find_joint(chain, motor_id)
            if override_var.get():
                joint["motor_settings"] = {
                    k: int(storage[k].get()) for k in ("p_gain", "i_gain", "d_gain")
                }
            elif "motor_settings" in joint:
                del joint["motor_settings"]
            self._mark_dirty()

        tk.Checkbutton(
            frame,
            text="Per-motor PID override",
            variable=override_var,
            command=_toggle_override,
            bg=self.COLORS["bg_medium"],
            fg=self.COLORS["text_primary"],
            selectcolor=self.COLORS["bg_light"],
            activebackground=self.COLORS["bg_medium"],
            font=("JetBrains Mono", 10),
        ).pack(anchor=tk.W, pady=(6, 4))

        ttk.Label(
            frame,
            text=f"Effective: vel={effective['velocity_limit']:.0f}  "
            f"P={effective['p_gain']:.0f}  I={effective['i_gain']:.0f}  D={effective['d_gain']:.0f}",
            style="Secondary.TLabel",
        ).pack(anchor=tk.W, pady=(0, 6))

        grid = ttk.Frame(frame, style="Medium.TFrame")
        grid.pack(fill=tk.X)
        for col, spec in enumerate(FOLLOWER_MOTOR_PARAMS):
            initial = float(effective.get(spec.key, spec.minimum))
            self._add_float_control(
                grid,
                spec,
                storage,
                row=0,
                col=col,
                initial=initial,
                on_change=lambda: self._sync_follower_motor_override(chain_name, motor_id),
            )
        self._suppress_config_sync = False

    # ------------------------------------------------------------------ Config sync
    def _sync_chain_to_config(self, chain_name: str) -> None:
        if self._suppress_config_sync:
            return
        chain = self.hand_config.setdefault(chain_name, {})
        general = _chain_general(chain)
        for spec in FOLLOWER_CHAIN_PARAMS:
            val = self._chain_vars[chain_name][spec.key].get()
            general[spec.key] = int(val)
        self._mark_dirty()
        self._refresh_follower_motor_labels()

    def _sync_motor_to_config(self, chain_name: str, motor_id: int) -> None:
        if self._suppress_config_sync:
            return
        chain = self.hand_config.setdefault(chain_name, {})
        joint = _find_joint(chain, motor_id)
        cc = joint.setdefault("current_control", {})
        storage = self._motor_vars[(chain_name, motor_id)]
        cc["enable_force_rendering"] = bool(storage["enable_force_rendering"].get())
        if "gradual_rendering" in storage:
            cc["gradual_rendering"] = bool(storage["gradual_rendering"].get())
        for spec in LEADER_FORCE_PARAMS:
            val = float(storage[spec.key].get())
            cc[spec.key] = val
        self._mark_dirty()

    def _sync_follower_motor_override(self, chain_name: str, motor_id: int) -> None:
        if self._suppress_config_sync:
            return
        storage = self._motor_vars.get((chain_name, motor_id), {})
        override_var = storage.get("__override__")
        if override_var is None or not override_var.get():
            return
        chain = self.hand_config.setdefault(chain_name, {})
        joint = _find_joint(chain, motor_id)
        ms = joint.setdefault("motor_settings", {})
        for spec in FOLLOWER_MOTOR_PARAMS:
            ms[spec.key] = int(storage[spec.key].get())
        self._mark_dirty()

    def _refresh_follower_motor_labels(self) -> None:
        if self.notebook.index(self.notebook.select()) != 1:
            return
        sel = self._motor_listboxes["follower"].curselection()
        if sel:
            motor_id = int(self._motor_listboxes["follower"].get(sel[0]))
            self._build_motor_detail("follower", motor_id)

    def _mark_dirty(self) -> None:
        self.dirty = True
        base = os.path.basename(self.source_config_path) if self.source_config_path else "unsaved"
        self.status_var.set(f"{base} *")

    # ------------------------------------------------------------------ Load / save
    def _load_config_file(self, path: Path) -> None:
        try:
            hand_cfg, header = _load_yaml(path)
        except Exception as exc:
            messagebox.showerror("Load error", str(exc))
            return
        self.hand_config = hand_cfg
        self.source_config_path = path.resolve()
        self.run_config_path = self.source_config_path
        if header:
            self.package_header = header
        self.dirty = False
        self._update_path_labels()
        self._maybe_suggest_dual_u2d2()
        self._log(f"Loaded {self.source_config_path} (original is not modified)", "success")
        self._log(f"Tuned copy will save to: {_tuned_output_path(self.source_config_path)}", "info")
        self._rebuild_all_tabs()

    def _update_path_labels(self) -> None:
        if not self.source_config_path:
            self.path_label.configure(text="—")
            self.save_path_label.configure(text="—")
            return
        self.path_label.configure(text=self.source_config_path.name)
        self.save_path_label.configure(text=_tuned_output_path(self.source_config_path).name)

    def _log_chain_pid_summary(self) -> None:
        follower = self.hand_config.get("follower", {})
        defaults = _follower_chain_defaults(follower)
        self._log(
            f"Follower chain defaults: vel={defaults['velocity_limit']:.0f} "
            f"P={defaults['p_gain']:.0f} I={defaults['i_gain']:.0f} D={defaults['d_gain']:.0f}",
            "info",
        )
        tuned = (
            self.source_config_path
            and _tuned_output_path(self.source_config_path).is_file()
        )
        if tuned:
            tpath = _tuned_output_path(self.source_config_path)
            tgeneral = (yaml.safe_load(tpath.read_text()) or {}).get("follower", {}).get(
                "general_settings", {}
            )
            if int(tgeneral.get("p_gain", 0) or 0) == 0:
                self._log(
                    f"{tpath.name} has p_gain=0 — press u to rewrite from current sliders.",
                    "warning",
                )

    def _maybe_suggest_dual_u2d2(self) -> None:
        """7-DoF configs need separate leader/follower U2D2 adapters."""
        if not self.source_config_path:
            return
        leader_n = len(_motor_ids(self.hand_config.get("leader", {})))
        follower_n = len(_motor_ids(self.hand_config.get("follower", {})))
        if leader_n + follower_n < 5:
            return
        if self.u2d2_var.get() != "dual":
            self._log(
                "7-DoF hands usually need u2d2=dual (separate USB for leader vs follower). "
                "With a single adapter only motors on that bus will respond.",
                "warning",
            )

    def _rebuild_all_tabs(self) -> None:
        self._suppress_config_sync = True
        if hasattr(self, "_disable_force_var"):
            leader = self.hand_config.get("leader", {})
            self._disable_force_var.set(bool(leader.get("disable_force_rendering_global", False)))

        for chain_name in ("leader", "follower"):
            lb = self._motor_listboxes[chain_name]
            lb.delete(0, tk.END)
            chain = self.hand_config.get(chain_name, {})
            for mid in _motor_ids(chain):
                lb.insert(tk.END, str(mid))
            if chain_name == "follower":
                chain_defaults = _follower_chain_defaults(chain)
                for spec in FOLLOWER_CHAIN_PARAMS:
                    if spec.key in self._chain_vars.get(chain_name, {}):
                        self._chain_vars[chain_name][spec.key].set(
                            float(chain_defaults[spec.key])
                        )
            for mid in _motor_ids(chain):
                self._build_motor_detail(chain_name, mid)
            if lb.size():
                lb.selection_set(0)
                self._on_motor_select(chain_name)
        self._suppress_config_sync = False
        self._log_chain_pid_summary()

    def _on_motor_select(self, chain_name: str) -> None:
        lb = self._motor_listboxes[chain_name]
        sel = lb.curselection()
        if not sel:
            return
        motor_id = int(lb.get(sel[0]))
        self._selected_motor[chain_name] = motor_id
        self._build_motor_detail(chain_name, motor_id)

    def _collect_config_from_ui(self) -> None:
        """Flush all slider values into hand_config before save."""
        for chain_name in ("leader", "follower"):
            if chain_name == "follower":
                self._sync_chain_to_config(chain_name)
            for mid in _motor_ids(self.hand_config.get(chain_name, {})):
                key = (chain_name, mid)
                if key not in self._motor_vars:
                    continue
                if chain_name == "leader":
                    self._sync_motor_to_config(chain_name, mid)
                else:
                    storage = self._motor_vars.get(key, {})
                    ov = storage.get("__override__")
                    if ov is not None and ov.get():
                        self._sync_follower_motor_override(chain_name, mid)
                    elif _get_joint(self.hand_config.get("follower", {}), mid):
                        joint = _get_joint(self.hand_config.get("follower", {}), mid)
                        if joint and "motor_settings" in joint:
                            del joint["motor_settings"]

    def _bind_keyboard_shortcuts(self) -> None:
        self.root.bind("<KeyPress-u>", lambda _e: self._on_key_update_settings())
        self.root.bind("<KeyPress-f>", lambda _e: self._on_key_toggle_teleop())
        self.root.bind("<KeyPress-q>", lambda _e: self._on_close())

    def _on_key_update_settings(self) -> None:
        """Save *_tuned.yaml; next f-start loads this file (stop teleop first if running)."""
        if self.teleop_running:
            self._log("Stop teleop (f) before updating settings (u).", "warning")
            return
        if not self.source_config_path:
            self._log("Load a hand config first.", "warning")
            return
        follower = self.hand_config.get("follower", {})
        defaults = _follower_chain_defaults(follower)
        if defaults["p_gain"] <= 0:
            messagebox.showerror(
                "Invalid PID",
                "Follower P gain is 0 in the UI. Reload the source YAML or fix sliders "
                "before saving (u).",
            )
            return
        out_path = _tuned_output_path(self.source_config_path)
        self._write_config(out_path)
        self.run_config_path = out_path.resolve()
        self._log(f"Run config for teleop: {self.run_config_path.name}", "success")
        self._log("Press f to start teleop with these settings.", "info")

    def _on_key_toggle_teleop(self) -> None:
        if self.teleop_running:
            self._stop_teleop()
        else:
            self._start_teleop()

    def _write_config(self, path: Path) -> None:
        self._collect_config_from_ui()
        try:
            body = yaml.safe_dump(
                self.hand_config,
                default_flow_style=False,
                sort_keys=False,
                allow_unicode=True,
            )
            header = (self.package_header or "# @package hand_config") + "\n\n"
            path.write_text(header + body, encoding="utf-8")
        except Exception as exc:
            messagebox.showerror("Save error", str(exc))
            return
        self.dirty = False
        self.save_path_label.configure(text=path.name)
        self.status_var.set(f"saved {path.name}")
        self._log(f"Saved tuned copy: {path}", "success")
        if self.source_config_path and path.resolve() != self.source_config_path.resolve():
            self._log(f"Original unchanged: {self.source_config_path}", "info")

    def _load_dialog(self) -> None:
        path = filedialog.askopenfilename(
            title="Open hand config",
            filetypes=[("YAML", "*.yaml *.yml"), ("All files", "*.*")],
            initialdir=str(_HAND_CONFIG_DIR),
        )
        if path:
            self._load_config_file(Path(path))

    # ------------------------------------------------------------------ Teleop (keyboard f)
    def _compose_hydra_config(self) -> DictConfig:
        path = self.run_config_path or self.source_config_path
        if not path:
            raise RuntimeError("Load a hand config file first.")
        override = _hand_config_hydra_override(path)
        overrides = [
            f"hand_config={override}",
            f"u2d2={self.u2d2_var.get()}",
        ]
        if self.fake_u2d2_var.get():
            overrides.append("u2d2.fake_u2d2=true")
        with initialize_config_dir(version_base=None, config_dir=str(_CONF_DIR)):
            cfg = compose(config_name="config", overrides=overrides)
        hand_cfg, _ = _load_yaml(path)
        follower_general = (hand_cfg.get("follower") or {}).get("general_settings") or {}
        if int(follower_general.get("p_gain", 0) or 0) <= 0:
            raise ValueError(
                f"{path.name} has follower.general_settings.p_gain=0. "
                "Press u to save a valid tuned copy from the GUI."
            )
        cfg.hand_config = OmegaConf.create(hand_cfg)
        return cfg

    def _start_teleop(self) -> None:
        path = self.run_config_path or self.source_config_path
        if not path:
            self._log("Load a hand config first.", "warning")
            return
        if not path.is_file():
            self._log("Press u to save tuned settings before starting teleop.", "warning")
            return
        if self.dirty:
            self._log("Press u to save slider changes before starting teleop (f).", "warning")
            return
        try:
            cfg = self._compose_hydra_config()
        except Exception as exc:
            messagebox.showerror("Config error", str(exc))
            self._log(f"Config error: {exc}", "error")
            return
        if TeleopHand.is_idle_hand_config(cfg):
            messagebox.showerror("Config error", "Hand config has no motors.")
            return
        self._log(f"Starting teleop with {path.name} …", "info")
        self.status_var.set("teleop: connecting…")
        self.root.update()

        def _worker():
            err = None
            teleop = None
            try:
                teleop = TeleopHand(cfg)
            except Exception as exc:
                err = exc
                if teleop is not None:
                    try:
                        teleop.disconnect()
                    except Exception:
                        pass
            self.root.after(0, lambda: self._on_teleop_started(teleop, err, path.name))

        threading.Thread(target=_worker, daemon=True).start()

    def _on_teleop_started(
        self, teleop: Optional[TeleopHand], err: Optional[Exception], config_name: str
    ) -> None:
        if err:
            messagebox.showerror("Teleop error", str(err))
            self._log(f"Teleop failed: {err}", "error")
            self.status_var.set("teleop: error")
            return
        self.teleop_hand = teleop
        self.teleop_hand.running = True
        self.teleop_running = True
        self.teleop_thread = threading.Thread(
            target=self.teleop_hand.start_control_loop, daemon=True
        )
        self.teleop_thread.start()
        self._log(f"Teleop running ({config_name}). Press f to stop.", "success")
        self.status_var.set(f"teleop: running ({config_name})")

    def _stop_teleop(self) -> None:
        self.teleop_running = False
        if self.teleop_hand:
            self.teleop_hand.running = False
            time.sleep(0.15)
            try:
                self.teleop_hand.disconnect()
            except Exception as exc:
                self._log(f"Disconnect: {exc}", "warning")
            self.teleop_hand = None
        self._log("Teleop stopped. Tune, press u, then f to run again.", "info")
        if self.source_config_path:
            self.status_var.set(os.path.basename(self.source_config_path))

    # ------------------------------------------------------------------ Logging / lifecycle
    def _log(self, message: str, tag: str = "info") -> None:
        self._log_queue.append((message, tag))

    def _poll_log_queue(self) -> None:
        while self._log_queue:
            message, tag = self._log_queue.pop(0)
            self.log_text.configure(state=tk.NORMAL)
            self.log_text.insert(tk.END, message + "\n", tag)
            self.log_text.see(tk.END)
            self.log_text.configure(state=tk.DISABLED)
        self.root.after(120, self._poll_log_queue)

    def _on_close(self) -> None:
        if self.teleop_running:
            if not messagebox.askokcancel("Exit", "Stop teleop and exit?"):
                return
            self._stop_teleop()
        self.root.destroy()

    def run(self) -> None:
        self._log("Hand Config Tuner ready", "success")
        self._log("u = save tuned YAML   f = start/stop teleop   q = quit", "info")
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Hand config parameter tuning GUI")
    parser.add_argument(
        "--config",
        "-c",
        type=str,
        default=None,
        help="Path to hand_config YAML (e.g. hand_interfaces/conf/hand_config/7dof/7dof.yaml)",
    )
    parser.add_argument(
        "--u2d2",
        type=str,
        default="dual",
        help="Hydra u2d2 profile (default: dual)",
    )
    parser.add_argument(
        "--fake-u2d2",
        action="store_true",
        help="Pass u2d2.fake_u2d2=true when connecting",
    )
    args = parser.parse_args()
    gui = HandConfigTuneGUI(
        initial_path=args.config,
        u2d2=args.u2d2,
        fake_u2d2=args.fake_u2d2,
    )
    gui.run()


if __name__ == "__main__":
    main()
