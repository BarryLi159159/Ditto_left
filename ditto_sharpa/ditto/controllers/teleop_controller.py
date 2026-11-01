#!/usr/bin/env python3
"""
TeleopHand Class and Controller

A flexible teleop control system for Dynamixel motors with Hydra-based configuration management.

Usage:
    python teleop_controller.py
    python teleop_controller.py hand_config=2dof/2dof_parallel_xc330_to_xm430_torque_off
"""

import os
import time
import hydra
from typing import List, Optional
from pathlib import Path
from omegaconf import DictConfig
import queue
import threading
import numpy as np

# Add the hand_interfaces module to the path
import sys
sys.path.append(str(Path(__file__).parent.parent))

from hand_interfaces.src.hand_interface import create_hand_interface
from hand_interfaces.src.current_control import CurrentController, ControlParams, create_control_params_from_config
from utils.utils import precise_sleep, interpolate_array


def print_disable_force_rendering_banner(
    *,
    disable_force_rendering_global: bool,
    side: str | None = None,
) -> None:
    """One-line status for teleop_node stdout (launcher process panel)."""
    state = "OFF" if disable_force_rendering_global else "ON"
    tag = f"teleop{f' {side}' if side else ''}"
    print(f"\n>>> FORCE RENDERING: {state}  ({tag}) <<<\n", flush=True)


class TeleopHand:
    """
    A flexible teleop control system for Dynamixel motors.
    
    This class handles the setup and control of leader-follower motor chains
    with support for different control modes.
    """

    @staticmethod
    def is_idle_hand_config(config) -> bool:
        """True when using hand_config/none.yaml (no leader/follower hardware)."""
        return (
            config.hand_config.leader.mode == "none"
            and config.hand_config.follower.mode == "none"
        )
    
    def __init__(self, config, state_queue: Optional[queue.Queue] = None):
        """
        Initialize the TeleopHand with a Hydra configuration.
        
        Args:
            config: Hydra DictConfig object containing motor chain configurations
            state_queue: Optional thread-safe queue for state data. If provided, motor states
                        will be queued for external processing (e.g., ROS2 publishing).
                        If None, states are not queued (standalone mode).
        """
        self.config = config
        self.verbose = config.hand_config.get('verbose', config.get('verbose', True))
        self.benchmark = config.hand_config.get('benchmark', config.get('benchmark', True))

        # Initialize hand interfaces
        self.leader_hand = None
        self.follower_hand = None
        self.use_dual_u2d2 = config.u2d2.get('use_dual_u2d2', False)

        # Hand modes:
        self.leader_mode = config.hand_config.leader.mode
        self.follower_mode = config.hand_config.follower.mode

        # hand_config=none: skip everything (no motors / no USB)
        self.is_hand_disabled = TeleopHand.is_idle_hand_config(config)

        # Agent mode:
        self.is_agent = self.leader_mode == "agent"

        # Leader-only: follower disconnected; leader collects encoders only
        self.is_leader_only = (
            not self.is_hand_disabled
            and self.follower_mode == "none"
            and not self.is_agent
        )

        # Motor chains (empty idle / agent-leader skipped / leader-only-follower skipped)
        self.leader_chain = (
            []
            if (self.is_agent or self.leader_mode == "none")
            else list(config.hand_config.leader.motor_ids)
        )
        self.follower_chain = (
            []
            if (self.is_leader_only or self.is_hand_disabled)
            else list(config.hand_config.follower.motor_ids)
        )
        self.follower_offsets_counts = self._parse_follower_offsets_counts()
        
        # Validate configuration
        self._validate_config()
        
        # Control settings
        self.control_frequency = config.hand_config.get('control_frequency', config.get('control_frequency', 50))
        self.dt = 1.0 / self.control_frequency  # Target period for control loop
        self.running = False
        self.show_current_breakdown = config.hand_config.get('show_current_breakdown', config.get('show_current_breakdown', False))

        # State queue for external state handling (e.g., ROS2 publishing)
        # Thread-safe queue.Queue is used for thread-safe communication
        self.state_queue = state_queue
        
        # External command queue for agent mode (thread-safe)
        self.command_queue = queue.Queue(maxsize=10)  # Store a few pending commands
        self.agent_trajectory = None
        self.agent_traj_index = 0
        self.agent_last_command = None
        
        # Initialize current controller (only needed if leader mode is 'current')
        if self.leader_mode == 'current':
            self.current_controller = CurrentController(self.control_frequency)
            self.leader_control_params = self._create_leader_control_params()
            print_disable_force_rendering_banner(
                disable_force_rendering_global=bool(
                    config.hand_config.leader.get("disable_force_rendering_global", False)
                ),
                side=os.environ.get("DITTO_SIDE") or None,
            )
        else: 
            self.current_controller = None
            self.leader_control_params = []

        # Start interface: 
        self.connect()
        self.setup_motors()
    

    # ===============================
    # Hand interface management 
    # ===============================   

    def _get_port_and_baudrate(self, chain_type: str, u2d2_config) -> tuple:
        """Get port and baudrate for a chain type (leader or follower)."""
        if self.use_dual_u2d2:
            chain_config = u2d2_config[chain_type]
            port = chain_config.get('usb_port', '/dev/ttyUSB0' if chain_type == 'follower' else '/dev/ttyUSB1')
            baudrate = chain_config.get('baudrate', 4000000)
        else:
            port = u2d2_config.get('usb_port', '/dev/ttyUSB0')
            baudrate = u2d2_config.get('baudrate', 4000000)
        return port, baudrate
    
    def connect(self):
        """Connect to the hand interfaces."""
        if self.is_hand_disabled:
            return

        # Check if already connected
        if self.is_agent and self.follower_hand is not None:
            return
        if self.is_leader_only and self.leader_hand is not None:
            return
        if not self.is_agent and not self.is_leader_only and self.leader_hand is not None and self.follower_hand is not None:
            return
        
        # Get U2D2 configuration
        u2d2_config = self.config.u2d2
        use_fake_u2d2 = u2d2_config.get('fake_u2d2', False)
        
        # Connect follower hand (skip in leader-only mode)
        if not self.is_leader_only:
            follower_port, follower_baudrate = self._get_port_and_baudrate('follower', u2d2_config)
            
            if self.verbose:
                print(f"\n🔌 Connecting to follower U2D2 on {follower_port}...")
            
            self.follower_hand = create_hand_interface(
                self.config, "follower", "real",
                port=follower_port, baudrate=follower_baudrate, use_fake=use_fake_u2d2
            )
            self.follower_hand.connect()
        
        # Connect leader hand (only in normal teleop mode, not agent mode)
        if not self.is_agent:
            leader_port, leader_baudrate = self._get_port_and_baudrate('leader', u2d2_config)
            
            if self.verbose:
                if self.use_dual_u2d2:
                    print(f"🔌 Connecting to leader U2D2 on {leader_port}...")
                else:
                    print(f"🔌 Connecting to {'leader-only' if self.is_leader_only else 'single'} U2D2 on {leader_port}...")
            
            self.leader_hand = create_hand_interface(
                self.config, "leader", "real",
                port=leader_port, baudrate=leader_baudrate, use_fake=use_fake_u2d2
            )
            self.leader_hand.connect()
        
        # Print success message
        if self.verbose:
            if use_fake_u2d2:
                if not self.is_agent and not self.is_leader_only and self.use_dual_u2d2:
                    print("✅ Dual Fake U2D2 Interfaces Initialized! (Testing Mode) \n")
                else:
                    print("✅ Fake U2D2 Interface Initialized! (Testing Mode) \n")
            elif not self.is_agent and not self.is_leader_only and self.use_dual_u2d2:
                print("✅ Dual U2D2 Interfaces Initialized! \n")
            else:
                print("✅ U2D2 Interface Initialized! \n")

    def disconnect(self):
        """Disconnect from the hand interfaces."""

        if self.is_hand_disabled:
            return
        
        if self.verbose:
            if self.is_agent:
                print("\n🧹 Disconnecting from U2D2 interface...")
            elif self.use_dual_u2d2:
                print("\n🧹 Disconnecting from dual U2D2 interfaces...")
            else:
                print("\n🧹 Disconnecting from single U2D2...")
        
        # Disconnect hand interfaces
        if self.leader_hand is not None:
            self.leader_hand.disconnect()
            self.leader_hand = None
        
        if self.follower_hand is not None:
            self.follower_hand.disconnect()
            self.follower_hand = None
    

    # ===============================
    # Initial Setup Functions
    # ===============================

    def _create_leader_control_params(self) -> List[ControlParams]:
        """Create control parameters for leader motors."""
        control_params = []
        leader_config = self.config.hand_config.leader
        disable_force_rendering_global = leader_config.get('disable_force_rendering_global', False)
        
        for i, motor_id in enumerate(self.leader_chain):
            # Get control parameters from joint_settings
            motor_config = {}
            
            if hasattr(leader_config, 'joint_settings') and leader_config.joint_settings:
                for joint_config in leader_config.joint_settings:
                    if joint_config['motor_id'] == motor_id and 'current_control' in joint_config:
                        motor_config = joint_config['current_control']
                        break
            
            if motor_config:
                params = create_control_params_from_config(motor_id, motor_config)
            else:
                # Default parameters (zero current)
                params = ControlParams(motor_id=motor_id)

            # Allow the hand config to disable force rendering for every joint at once.
            if disable_force_rendering_global:
                params.enable_force_rendering = False
            
            control_params.append(params)
            
            if self.verbose:
                gradual_str = ",gradual" if params.gradual_rendering else ""
                force_rendering_str = f"force_rendering={params.force_rendering_gain:.3f}(thres={params.force_rendering_threshold:.1f}{gradual_str})"
                print(f"    Leader motor {motor_id} control params: "
                      f"coulomb=({params.mu_coulomb_cw:.1f}, {params.mu_coulomb_ccw:.1f}), "
                      f"damping={params.mu_damping:.3f}, "
                      f"static={params.mu_static_amplitude:.1f}@{params.static_frequency:.1f}Hz, "
                      f"{force_rendering_str}")
        
        return control_params

    def setup_motors(self) -> None:
        """Set up all motors according to their configurations."""
        if self.is_hand_disabled:
            return
        
        # Validate connections
        if not self.is_leader_only and self.follower_hand is None:
            raise RuntimeError("Follower hand interface not connected. Call connect() first.")
        
        if not self.is_agent and self.leader_hand is None:
            raise RuntimeError("Leader hand interface not connected. Call connect() first.")
        
        if self.verbose:
            print("🔧 Setting up motors...")
        
        # Setup leader motors (in normal teleop and leader-only modes, not agent mode)
        if not self.is_agent:
            self.leader_hand.setup_motors()
        
        # Setup follower motors (skip in leader-only mode)
        if not self.is_leader_only:
            self.follower_hand.setup_motors()
            self._log_follower_offsets_if_any()
        
        if self.verbose:
            print("✅ Motors ready!")


    # ===============================
    # Validation Functions
    # ===============================
    
    def _validate_config(self) -> None:
        """Validate the configuration."""
        
        # Validate control modes
        valid_leader_modes = ['torque_off', 'current', 'agent', 'none']
        valid_follower_modes = ['position', 'current', 'none']
        
        if self.leader_mode not in valid_leader_modes:
            raise ValueError(f"Invalid leader mode: {self.leader_mode}. Must be one of {valid_leader_modes}")
        
        if self.follower_mode not in valid_follower_modes:
            raise ValueError(f"Invalid follower mode: {self.follower_mode}. Must be one of {valid_follower_modes}")

        if self.leader_mode == "none" and self.follower_mode != "none":
            raise ValueError("leader mode 'none' requires follower mode 'none' (use hand_config/none.yaml pattern)")

        if self.is_hand_disabled:
            lm = list(self.config.hand_config.leader.motor_ids or [])
            fm = list(self.config.hand_config.follower.motor_ids or [])
            if lm or fm:
                raise ValueError("hand_config with both modes 'none' must have empty motor_ids")
            return

        # Leader-only mode: follower is 'none', leader must be 'torque_off'
        if self.is_leader_only:
            assert self.leader_mode == 'torque_off', (
                f"When follower mode is 'none' (leader-only), leader mode must be 'torque_off', "
                f"got '{self.leader_mode}'"
            )
            return  # Skip chain length validation for leader-only mode

        # For agent mode, leader chain length doesn't matter (leader is not used)
        if self.leader_mode != 'agent':
            if len(self.leader_chain) != len(self.follower_chain):
                raise ValueError(
                    f"Leader and follower chains must have the same length. "
                    f"Leader: {len(self.leader_chain)}, Follower: {len(self.follower_chain)}"
                )

    def _get_motor_model(self, motor_id: int, chain_type: str) -> str:
        """Get the motor model for a specific motor, checking joint settings first."""
        # Start with default from Hydra config
        motor_model = self.config.joint_configs.default.motor_model
        
        # Check for motor-specific override in joint settings
        chain_config = self.config.hand_config[chain_type]
        if hasattr(chain_config, 'joint_settings') and chain_config.joint_settings:
            for joint_config in chain_config.joint_settings:
                if joint_config['motor_id'] == motor_id and 'joint_settings' in joint_config:
                    motor_model = joint_config['joint_settings'].get('motor_model', motor_model)
                    break
        
        return motor_model


    # ===============================
    # Data management functions
    # ===============================   

    def _extract_states_from_motor_data(self, motor_data):
        """Extract state arrays from motor data list."""
        if motor_data is None:
            return None
        return {
            'positions': [data.position for data in motor_data],
            'velocities': [data.velocity for data in motor_data],
            'currents': [data.current for data in motor_data],
            'joint_angles': [data.joint_angle for data in motor_data],
            'joint_velocities': [data.joint_velocity for data in motor_data]
        }

    def _extract_follower_states_from_motor_data(self, motor_data):
        """
        Follower feedback for ROS / logging.

        Commands apply ``follower.offsets`` (leader-space target + offset → motor).
        Published joint angles undo that offset so feedback matches the commanded
        leader-equivalent pose (same frame as ``leader_joint_states`` / replay zarr).
        """
        states = self._extract_states_from_motor_data(motor_data)
        if states is None:
            return None
        if self.follower_offsets_counts:
            states = dict(states)
            states['joint_angles'] = [
                float(states['joint_angles'][i])
                - self._follower_offset_counts_to_joint_radians(i)
                for i in range(len(states['joint_angles']))
            ]
        return states
    
    def _build_states_data(self, leader_motor_data, follower_motor_data) -> dict:
        """Build states data dictionary from motor data."""
        # Build leader states (zeros in agent mode, actual data otherwise)
        if self.is_agent:
            # Use follower_chain length for leader states in agent mode:
            num_leader_motors = len(self.follower_chain)
            leader_states = {
                'positions': [0.0] * num_leader_motors,
                'velocities': [0.0] * num_leader_motors,
                'currents': [0.0] * num_leader_motors,
                'joint_angles': [0.0] * num_leader_motors,
                'joint_velocities': [0.0] * num_leader_motors
            }
        else:
            leader_states = self._extract_states_from_motor_data(leader_motor_data)
        
        # Build follower states
        if self.is_leader_only:
            # Leader-only mode: no follower, provide zeros matching leader chain length
            num_follower_motors = len(self.leader_chain)
            follower_states = {
                'positions': [0.0] * num_follower_motors,
                'velocities': [0.0] * num_follower_motors,
                'currents': [0.0] * num_follower_motors,
                'joint_angles': [0.0] * num_follower_motors,
                'joint_velocities': [0.0] * num_follower_motors
            }
        else:
            follower_states = self._extract_follower_states_from_motor_data(follower_motor_data)
        
        return {
            'leader': leader_states,
            'follower': follower_states
        }
    

    # ===============================
    # Benchmark functions
    # ===============================

    def _update_running_stats(self, stats: dict, value: float) -> None: # benchmark
        """Update running statistics efficiently without storing all values.""" # benchmark
        stats['sum'] += value # benchmark
        stats['count'] += 1 # benchmark
        stats['min'] = min(stats['min'], value) # benchmark
        stats['max'] = max(stats['max'], value) # benchmark

    def _print_benchmark_report(self, cycle: int, benchmark_stats: dict) -> None: # benchmark
        """Print detailed benchmark report for control loop performance.""" # benchmark
        if self.verbose:
            print(f"\n{'='*60}") # benchmark
            print(f"📊 BENCHMARK REPORT - Cycle {cycle}") # benchmark
            print(f"{'='*60}") # benchmark
        
        # Print motor configuration for reference # benchmark
        leader_motor_models = [] # benchmark
        follower_motor_models = [] # benchmark
        for motor_id in self.leader_chain: # benchmark
            motor_model = self._get_motor_model(motor_id, 'leader') # benchmark
            leader_motor_models.append(f"{motor_id}({motor_model})") # benchmark
        for motor_id in self.follower_chain: # benchmark
            motor_model = self._get_motor_model(motor_id, 'follower') # benchmark
            follower_motor_models.append(f"{motor_id}({motor_model})") # benchmark
        
        if self.verbose:
            print(f"Leader Motors:  {', '.join(leader_motor_models) if leader_motor_models else 'None'}") # benchmark
            print(f"Follower Motors: {', '.join(follower_motor_models) if follower_motor_models else 'None'}") # benchmark
            print(f"Control Modes:  Leader={self.config.hand_config.leader.mode}, Follower={self.config.hand_config.follower.mode}") # benchmark
            print(f"U2D2 Setup:     {'Dual U2D2' if self.use_dual_u2d2 else 'Single U2D2'}") # benchmark
            print(f"{'='*60}") # benchmark
        
        def _print_stat_line(section: str, stats: dict, indent: int = 0):
            """Print a single statistic line."""
            if stats['count'] == 0:
                return
            avg_time = stats['sum'] / stats['count']
            min_time = stats['min']
            max_time = stats['max']
            avg_ms = avg_time * 1000
            min_ms = min_time * 1000
            max_ms = max_time * 1000
            indent_str = '\t' * indent
            print(f"{indent_str}{section:15} | Avg: {avg_ms:6.2f}ms | Min: {min_ms:6.2f}ms | Max: {max_ms:6.2f}ms") # benchmark
        
        if self.verbose:
            # Print execution first
            if benchmark_stats['execution']['count'] > 0:
                _print_stat_line('execution', benchmark_stats['execution'])
                print(f"{'-'*60}") # benchmark
                # Print execution components (indented)
                _print_stat_line('|_read_states', benchmark_stats['read_states'])
                _print_stat_line('|_send_commands', benchmark_stats['send_commands'])
                print(f"{'-'*60}") # benchmark
            
            # Print sleep_time
            if benchmark_stats['sleep_time']['count'] > 0:
                _print_stat_line('sleep_time', benchmark_stats['sleep_time'])
                print(f"{'-'*60}") # benchmark
            
            # Print total
            if benchmark_stats['total']['count'] > 0:
                _print_stat_line('total', benchmark_stats['total'])
        
        # Calculate total loop statistics # benchmark
        if benchmark_stats['total']['count'] > 0: # benchmark
            total_avg = benchmark_stats['total']['sum'] / benchmark_stats['total']['count'] # benchmark
            total_min = benchmark_stats['total']['min'] # benchmark
            total_max = benchmark_stats['total']['max'] # benchmark
            
            # Calculate actual vs target frequency # benchmark
            actual_freq = 1.0 / total_avg # benchmark
            efficiency = (self.dt / total_avg) * 100 # benchmark
            
            print(f"{'='*60}") # benchmark
            print(f"🎯 PERFORMANCE SUMMARY") # benchmark
            print(f"Target Frequency: {self.control_frequency:6.1f} Hz ({self.dt*1000:6.2f}ms)") # benchmark
            print(f"Actual Frequency: {actual_freq:6.1f} Hz ({total_avg*1000:6.2f}ms)") # benchmark
            print(f"Efficiency:       {efficiency:6.1f}%") # benchmark
            print(f"Min Loop Time:    {total_min*1000:6.2f}ms") # benchmark
            print(f"Max Loop Time:    {total_max*1000:6.2f}ms") # benchmark
            print(f"{'='*60}\n") # benchmark


    # ===============================
    # Print functions
    # ===============================

    def _print_performance_warning(self, elapsed: float, target_period: float, cycle: int) -> None:
        """
        Print a performance warning when control loop runs slower than target frequency.
        
        Args:
            elapsed: Actual elapsed time for the control loop cycle
            target_period: Target period for the control loop (1.0 / control_frequency)
            cycle: Current cycle number for throttling warnings
        """
        # CRITICAL: Always print performance warnings regardless of verbose flag
        # Users must know when control frequency is not achieved
        # But reduce frequency for ultra-high frequencies to avoid spam
        warning_interval = 100 if self.control_frequency <= 200 else 1000
        if cycle % warning_interval == 0:
            performance_percent = (target_period / elapsed) * 100
            actual_freq = 1.0 / elapsed
            # Red color codes for terminal output
            RED = "\033[91m"
            RESET = "\033[0m"
            print()
            print(f"{RED}{'='*50}{RESET}")  # Red line
            print(f"{RED}⚠️  CONTROL LOOP PERFORMANCE WARNING ⚠️{RESET}")
            print(f"{RED}   Running slower than target frequency!{RESET}")
            print(f"{RED}   Elapsed: {elapsed:.3f}s (target: {target_period:.3f}s){RESET}")
            print(f"{RED}   Actual Freq: {actual_freq:.1f}Hz (target: {1.0/target_period:.1f}Hz){RESET}")
            print(f"{RED}   Performance: {performance_percent:.1f}%{RESET}")
            print(f"{RED}{'='*50}{RESET}")  # Red line
            print()

    def _print_config_summary(self):
        """Print a single, consolidated summary of the teleop configuration."""
        print("\n" + "="*60)
        print("🤖 TELEOP CONFIGURATION SUMMARY")
        print("="*60)
        
        # Basic configuration
        print(f"Config: {self.config.config_name}")
        print(f"Control Frequency: {self.control_frequency} Hz")

        if self.is_hand_disabled:
            print("Hand: none (no motors, no USB)")
            print("=" * 60)
            return
        
        # U2D2 configuration
        if self.use_dual_u2d2:
            print(f"U2D2: Dual U2D2")
            print(f"  Leader: {self.config.u2d2.leader.usb_port} @ {self.config.u2d2.leader.baudrate}")
            print(f"  Follower: {self.config.u2d2.follower.usb_port} @ {self.config.u2d2.follower.baudrate}")
        else:
            print(f"U2D2: Single U2D2")
            print(f"  Port: {self.config.u2d2.usb_port} @ {self.config.u2d2.baudrate}")
        
        # Motor chains
        if self.is_agent:
            print(f"🎯 Follower Motors: {self.follower_chain} ({self.follower_mode} mode - external commands via schedule_command() API)")
        elif self.is_leader_only:
            print(f"\n📡 Leader Motors: {self.leader_chain} ({self.leader_mode} mode)")
            print(f"🎯 Follower: None (leader-only mode, collecting encoder values only)")
        else:
            print(f"\n📡 Leader Motors: {self.leader_chain} ({self.leader_mode} mode)")
            print(f"🎯 Follower Motors: {self.follower_chain} ({self.follower_mode} mode)")
            offsets_by_motor = {
                follower_id: int(self.follower_offsets_counts[i]) for i, follower_id in enumerate(self.follower_chain)
            }
            if any(v != 0 for v in offsets_by_motor.values()):
                print(f"  Follower offsets (counts): {offsets_by_motor}")
            
            # Control laws for current mode leaders
            if self.leader_mode == 'current':
                print("  Control Laws:")
                for i, (motor_id, params) in enumerate(zip(self.leader_chain, self.leader_control_params)):
                    laws = []
                    if params.enable_coulomb_comp:
                        laws.append(f"Coulomb({params.mu_coulomb_cw:.1f},{params.mu_coulomb_ccw:.1f})")
                    if params.enable_damping_comp:
                        laws.append(f"Damping({params.mu_damping:.3f})")
                    if params.enable_static_comp:
                        laws.append(f"Static({params.mu_static_amplitude:.1f}@{params.static_frequency:.1f}Hz)")
                    if params.enable_force_rendering:
                        gradual_str = ",gradual" if params.gradual_rendering else ""
                        force_str = f"Force({params.force_rendering_gain:.3f},thres={params.force_rendering_threshold:.0f}{gradual_str})"
                        laws.append(force_str)
                    if not laws:
                        laws.append("Zero")
                    
                    print(f"    Motor {motor_id}: {' + '.join(laws)}")
            
            # Motor pairing
            print(f"\n🔗 Motor Pairing: {len(self.leader_chain)} pairs")

            # Motor model names from the *actual models list* used to build converters
            leader_models = self.leader_hand.motor_model_by_id if self.leader_hand is not None else {}
            follower_models = self.follower_hand.motor_model_by_id if self.follower_hand is not None else {}
            
            for i, (leader_id, follower_id) in enumerate(zip(self.leader_chain, self.follower_chain)):
                lm = str(leader_models.get(int(leader_id), "UNKNOWN"))
                fm = str(follower_models.get(int(follower_id), "UNKNOWN"))
                print(f"  [{i}] {leader_id} ({lm}) -> {follower_id} ({fm})")
        
        print("="*60)

    # ===============================
    # External command scheduling & 
    # interpolation for agent mode
    # ===============================

    def _parse_follower_offsets_counts(self) -> List[int]:
        """
        Parse per-follower encoder-count offsets from config.

        Supported format under hand_config.follower.offsets:
          Mapping by follower motor_id, e.g. {21: 0, 22: 12, 23: -4}
        """
        if self.is_hand_disabled or self.is_leader_only:
            return []

        raw_offsets = self.config.hand_config.follower.get('offsets', None)
        if raw_offsets is None:
            return [0 for _ in self.follower_chain]

        if hasattr(raw_offsets, "items"):
            parsed = []
            for motor_id in self.follower_chain:
                key_str = str(motor_id)
                if motor_id in raw_offsets:
                    parsed.append(int(raw_offsets[motor_id]))
                elif key_str in raw_offsets:
                    parsed.append(int(raw_offsets[key_str]))
                else:
                    parsed.append(0)
            return parsed

        raise ValueError("hand_config.follower.offsets must be a mapping keyed by follower motor_id.")

    def _follower_offset_counts_to_joint_radians(self, pair_index: int) -> float:
        """
        Convert configured follower offset in counts to joint-space radians.
        Same model as calibration GUI: zero_position shift by `offset`.
        """
        converter = self.follower_hand.joint_converters[pair_index]
        counts = float(self.follower_offsets_counts[pair_index])
        return counts * converter.radians_per_count / (converter.transmission_ratio * converter.direction)

    def _log_follower_offsets_if_any(self) -> None:
        if self.is_leader_only or not self.follower_offsets_counts:
            return
        offsets_by_motor_id = {
            motor_id: int(self.follower_offsets_counts[i]) for i, motor_id in enumerate(self.follower_chain)
        }
        if any(v != 0 for v in offsets_by_motor_id.values()):
            print(f"Follower offsets from config (motor_id -> counts): {offsets_by_motor_id}")
    
    def schedule_command(self, joint_angles: List[float], duration: float) -> None:
        """
        Schedule a command for agent mode with interpolation.
        
        This method is thread-safe and can be called from external threads
        (e.g., GUI control loop) to set joint angle targets.
        
        Uses a queue to avoid blocking and port conflicts from locks.
        
        Args:
            joint_angles: List of joint angles in radians, one per motor in follower chain
            duration: Desired time (seconds) to reach the goal
        """
        if self.leader_mode != 'agent':
            raise RuntimeError("schedule_command() can only be used when leader mode is 'agent'")
        
        if len(joint_angles) != len(self.follower_chain):
            raise ValueError(
                f"Expected {len(self.follower_chain)} joint angles, got {len(joint_angles)}"
            )
        
        # Put command in queue (non-blocking, replaces old command if queue is full)
        duration = float(duration)
        scheduled_cmd = {
            'cmd': joint_angles.copy(),
            'duration': duration
        }

        try:
            self.command_queue.put_nowait(scheduled_cmd)
        except queue.Full:
            # Queue is full, replace with new command
            try:
                self.command_queue.get_nowait()  # Remove old command
                self.command_queue.put_nowait(scheduled_cmd)  # Add new command
            except queue.Empty:
                # Shouldn't happen, but handle gracefully
                pass

    def _drain_latest_agent_command(self) -> Optional[dict]:
        """Return the most recent scheduled command from the queue."""
        latest = None
        while True:
            try:
                latest = self.command_queue.get_nowait()
            except queue.Empty:
                break
        return latest

    def _prepare_agent_trajectory(self) -> None:
        """Create or update the interpolation trajectory for agent mode."""
        latest_cmd = self._drain_latest_agent_command()
        if latest_cmd is None:
            return

        target = np.array(latest_cmd['cmd'], dtype=float)
        duration = float(latest_cmd['duration'])
        assert duration > 0.0, f"Duration must be positive, got {duration}"
        
        # Generate interpolated trajectory
        current = self.agent_last_command
        self.agent_trajectory = interpolate_array(current, target, duration, self.dt)
        self.agent_traj_index = 0

        if self.verbose:
            num_steps = len(self.agent_trajectory)
            print(f"[INTERP] New interpolation triggered: current={current.tolist()}, target={target.tolist()}, duration={duration:.4f}s, steps={num_steps}")
        
    def _next_agent_command(self) -> Optional[np.ndarray]:
        """Return the next interpolated agent command as numpy array."""
        if self.agent_trajectory is not None:
            if self.agent_traj_index < len(self.agent_trajectory):
                command = self.agent_trajectory[self.agent_traj_index]
                self.agent_last_command = command
                self.agent_traj_index += 1
                if self.agent_traj_index >= len(self.agent_trajectory):
                    self.agent_trajectory = None
                    self.agent_traj_index = 0
                return command

            # Trajectory consumed
            self.agent_trajectory = None
            self.agent_traj_index = 0

        # No active trajectory, return last command if available
        if self.agent_last_command is not None:
            return self.agent_last_command

        # No command ever received, return None to skip sending
        return None
    

    # ===============================
    # Main control loop functions
    # ===============================
    
    def _read_and_queue_states(self):
        """Read motor states and queue them if state_queue is provided."""
        if self.is_hand_disabled:
            return None, None
        try:
            # Leader-only mode: only read leader encoder values, no follower
            if self.is_leader_only:
                leader_motor_data = self.leader_hand.read_states()
                follower_motor_data = None
            elif self.is_agent:
                follower_motor_data = self.follower_hand.read_states()
                leader_motor_data = None
            else:
                follower_motor_data = self.follower_hand.read_states()
                leader_motor_data = self.leader_hand.read_states()
        except Exception as e:
            if self.verbose:
                error_str = str(e)
                if "Port is in use" in error_str or "-1000" in error_str:
                    print(f"⚠️ Port communication error: Port may be in use or hardware issue")
                else:
                    print(f"⚠️ Motor read error: {e}")
            return None, None
        
        # Queue state data if queue is provided (for ROS2 publishing, etc.)
        if self.state_queue is not None:
            states_data = self._build_states_data(leader_motor_data, follower_motor_data)
            try:
                # Thread-safe: queue.Queue.put_nowait() is atomic
                self.state_queue.put_nowait(states_data)
            except queue.Full:
                # Queue is full, drop message (non-blocking to avoid affecting control loop)
                if self.verbose:
                    print(f"⚠️ State queue full, dropping message")
            except Exception as e:
                if self.verbose:
                    print(f"⚠️ State queue error: {e}")
        
        return leader_motor_data, follower_motor_data

    def _send_commands(self, leader_motor_data, follower_motor_data) -> None:
        """Send commands to motors based on current mode."""
        if self.is_hand_disabled:
            return

        # Leader-only mode: no commands to send (leader is torque_off, no follower)
        if self.is_leader_only:
            return

        if self.is_agent:
            # ===============================================
            # Agent mode: interpolate queued commands
            # ===============================================   
            if follower_motor_data is None:
                return

            # Only extract current position if needed (first command initialization)
            if self.agent_last_command is None:
                self.agent_last_command = np.array([data.joint_angle for data in follower_motor_data], dtype=float)
            
            # Prepare interpolated trajectory
            self._prepare_agent_trajectory()
            command = self._next_agent_command()
            
            # Skip if no command available (no command ever received)
            if command is None:
                return

            try:
                if self.follower_mode == 'position':
                    # Mirror teleop: follower targets are leader-space joint angles plus
                    # hand_config.follower.offsets (counts → rad). Replay policies that
                    # publish leader_joint_states rely on this path; follower-space
                    # commands would need offsets disabled in YAML.
                    cmds = command.tolist()
                    if self.follower_offsets_counts:
                        cmds = [
                            float(cmds[i]) + self._follower_offset_counts_to_joint_radians(i)
                            for i in range(len(cmds))
                        ]
                    self.follower_hand.send_position_commands(cmds)
                else:
                    raise RuntimeError(f"Invalid follower mode: {self.follower_mode}")
            except Exception as e:
                if self.verbose:
                    error_str = str(e)
                    if "Port is in use" in error_str or "Port" in error_str:
                        print("⚠️ Port error: Port may be in use. Check for other processes using the port.")
                    else:
                        print(f"⚠️ Command send error: {e}")
            return

        else:
            # ===============================================
            # Normal teleop mode
            # ===============================================

            # Leader 
            if self.leader_mode == 'current':
                # Compute current commands using the current controller
                current_commands = self.current_controller.compute_bulk_current_commands(
                    leader_motor_data, 
                    self.leader_control_params, 
                    time.time(),
                    follower_motor_data,
                    self.show_current_breakdown
                )
                
                # Send current commands
                self.leader_hand.send_current_commands(current_commands)
            
            # Follower:
            if self.follower_mode == 'position':
                leader_joint_angles = [data.joint_angle for data in leader_motor_data]
                if self.follower_offsets_counts:
                    leader_joint_angles = [
                        leader_joint_angles[i] + self._follower_offset_counts_to_joint_radians(i)
                        for i in range(len(leader_joint_angles))
                    ]
                self.follower_hand.send_position_commands(leader_joint_angles)

    def start_control_loop(self):
        """Start the main control loop."""
        if self.is_hand_disabled:
            raise RuntimeError("start_control_loop() called while hand is idle (no hardware)")
        if self.is_leader_only:
            if self.leader_hand is None:
                raise RuntimeError("Leader hand interface not connected. Call connect() first.")
            print(f"\n🔄 Starting control loop at {self.control_frequency} Hz...")
            print(f"Leader-only mode: Reading encoder values from leader motors {self.leader_chain} (no follower)")
        elif self.is_agent:
            if self.follower_hand is None:
                raise RuntimeError("Follower hand interface not connected. Call connect() first.")
            print(f"\n🔄 Starting control loop at {self.control_frequency} Hz...")
            print(f"Agent mode: Follower motors {self.follower_chain} ({self.follower_mode} mode) controlled via update_command() API")
        else:
            if self.leader_hand is None or self.follower_hand is None:
                raise RuntimeError("Hand interfaces not connected. Call connect() first.")
            print(f"\n🔄 Starting control loop at {self.control_frequency} Hz...")
            print(f"Move the leader motors {self.leader_chain} to see the follower motors {self.follower_chain} follow!")
            print(f"Leader mode: {self.leader_mode}")
            print(f"Follower mode: {self.follower_mode}")
        
        self.running = True
        cycle = 0
        
        # Benchmark tracking - using running statistics for efficiency # benchmark
        benchmark_interval = 1000 # benchmark
        benchmark_stats = { # benchmark
            'total': {'sum': 0.0, 'count': 0, 'min': float('inf'), 'max': 0.0}, # benchmark - includes sleep
            'execution': {'sum': 0.0, 'count': 0, 'min': float('inf'), 'max': 0.0}, # benchmark - execution only (no sleep)
            'read_states': {'sum': 0.0, 'count': 0, 'min': float('inf'), 'max': 0.0}, # benchmark
            'send_commands': {'sum': 0.0, 'count': 0, 'min': float('inf'), 'max': 0.0}, # benchmark
            'sleep_time': {'sum': 0.0, 'count': 0, 'min': float('inf'), 'max': 0.0}, # benchmark - actual sleep time
        } # benchmark
        
        try:
            cycle_start_time = time.time()  # Initialize for first cycle
            while self.running:
                cycle += 1
                
                # ===============================
                # Read states and queue
                # ===============================
                read_start = time.time() # benchmark
                leader_motor_data, follower_motor_data = self._read_and_queue_states()
                read_time = time.time() - read_start # benchmark
                
                self._update_running_stats(benchmark_stats['read_states'], read_time) # benchmark
                
                # Skip this cycle if state reading failed
                if leader_motor_data is None and follower_motor_data is None:
                    continue
                
                # ===============================
                # Send commands
                # ===============================
                send_start = time.time() # benchmark
                self._send_commands(leader_motor_data, follower_motor_data)
                send_time = time.time() - send_start # benchmark

                self._update_running_stats(benchmark_stats['send_commands'], send_time) # benchmark
                
                # ===============================
                # Control loop timing 
                # ===============================
                before_timing = time.time() # benchmark
                execution_time = before_timing - cycle_start_time # benchmark - execution only
                
                self._update_running_stats(benchmark_stats['execution'], execution_time) # benchmark
               
                # Calculate sleep time (if running slower than target frequency)
                sleep_time = 0.0
                actual_sleep = 0.0 # benchmark
                if execution_time > self.dt:
                    self._print_performance_warning(execution_time, self.dt, cycle)
                else:
                    sleep_time = self.dt - execution_time
                    if sleep_time > 0.0001:  # Only sleep if more than 0.1ms left
                        sleep_start = time.perf_counter() # benchmark
                        precise_sleep(sleep_time, precise_duration=0.001)
                        actual_sleep = time.perf_counter() - sleep_start # benchmark
                # print(f"Performance: actual freq: {1.0 / execution_time:.1f}Hz (target: {1.0/self.dt:.1f}Hz)")

                self._update_running_stats(benchmark_stats['sleep_time'], actual_sleep) # benchmark
                
                # Measure total cycle time for benchmark
                after_sleep = time.time() # benchmark
                total_time = after_sleep - cycle_start_time # benchmark
                self._update_running_stats(benchmark_stats['total'], total_time) # benchmark
                
                # Print benchmark every 1000 cycles # benchmark
                if self.benchmark:
                    if cycle % benchmark_interval == 0: # benchmark
                        self._print_benchmark_report(cycle, benchmark_stats) # benchmark
                
                # Set cycle_start_time for next iteration
                cycle_start_time = time.time()
                    
        except KeyboardInterrupt:
            print("\n🛑 Stopping control loop...")
        finally:
            self.running = False

    def run(self):
        """Run the complete teleop control sequence."""
        try:
            # Connect and setup
            self.connect()
            self.setup_motors()
            
            # Display summary
            self._print_config_summary()

            if self.is_hand_disabled:
                print("\nhand_config=none — pass hand_config=... to run a physical hand, e.g.")
                print("  python controllers/teleop_controller.py hand_config=7dof/7dof \n")
                return
            
            # Wait for user input
            input("\n⏸️  Press Enter to start teleop control...")
            
            # Start control loop
            self.start_control_loop()
            
        except Exception as e:
            print(f"❌ Error during teleop control: {e}")
            raise
        finally:
            # Cleanup
            self.disconnect()
            print("✅ Cleanup complete!")
            print("✅ Teleop control completed successfully!")



def main():
    """Main function to run teleop control."""
    # Get the config directory
    config_path = Path(__file__).parent.parent / "hand_interfaces" / "conf"
    
    @hydra.main(version_base=None, config_path=str(config_path), config_name="config")
    def run_teleop(cfg: DictConfig) -> None:
        """Run teleop control with the specified configuration."""
        print(f"🤖 Teleop Hand Control - {cfg.config_name}")
        print(f"Description: {cfg.description}\n")
        print("Press Ctrl+C to stop")
        print("------------------------------------------------------------\n")
        
        # Create and run teleop hand
        teleop = TeleopHand(cfg)
        teleop.run()
    
    run_teleop()


if __name__ == "__main__":
    main()