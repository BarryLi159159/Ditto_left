#!/usr/bin/env python3
"""
TeleopHand Class and Controller

A flexible teleop control system for Dynamixel motors with Hydra-based configuration management.
Supports multiple control modes and joint abstraction.

Usage:
    python teleop_hand.py hand_config=2dof/2dof_parallel_xc330_to_xm430_torque_off
"""

import time
import hydra
from typing import List, Optional
from pathlib import Path
from omegaconf import DictConfig
import queue

# Add the hand_interfaces module to the path
import sys
sys.path.append(str(Path(__file__).parent.parent))

from hand_interfaces.src.hand_interface_multi import create_hand_interface_multi
from hand_interfaces.src.current_control_multi import CurrentControlLaws, ControlParams, create_control_params_from_config
from hand_interfaces.src.hand_data import HandData
from utils.utils import precise_sleep

class TeleopHand:
    """
    A flexible teleop control system for Dynamixel motors.
    
    This class handles the setup and control of leader-follower motor chains
    with support for different control modes.
    """
    
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

        # Initialize hand interfaces
        self.leader_hand = None  # Leader hand interface
        self.follower_hand = None  # Follower hand interface
        self.use_dual_u2d2 = config.u2d2.get('use_dual_u2d2', False)
        
        # Motor chains
        self.active_chains = ['leader', 'follower']
        self.leader_chain = list(config.hand_config.leader.motor_ids)
        self.follower_chain = list(config.hand_config.follower.motor_ids)
        self.all_motors = self.leader_chain + self.follower_chain
        
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
        
        # Initialize current control laws
        self.current_control_laws = CurrentControlLaws(self.control_frequency)
        self.leader_control_params = self._create_leader_control_params()

        # Start interface: 
        self.connect()
        self.setup_motors()
    
    def connect(self):
        """Connect to the hand interfaces."""
        if self.leader_hand is not None and self.follower_hand is not None:
            return
        
        # Get U2D2 configuration
        u2d2_config = self.config.u2d2
        use_fake_u2d2 = u2d2_config.get('fake_u2d2', False)
        
        if self.use_dual_u2d2:
            # Dual U2D2 configuration - separate physical devices
            # Follower always on USB0, leader on USB1
            follower_config = u2d2_config.follower
            follower_port = follower_config.get('usb_port', '/dev/ttyUSB0')
            follower_baudrate = follower_config.get('baudrate', 4000000)
            
            leader_config = u2d2_config.leader
            leader_port = leader_config.get('usb_port', '/dev/ttyUSB1')
            leader_baudrate = leader_config.get('baudrate', 4000000)
            
            if self.verbose:
                print(f"\n🔌 Connecting to follower U2D2 on {follower_port}...")
                print(f"🔌 Connecting to leader U2D2 on {leader_port}...")
        else:
            # Single U2D2 configuration - same physical device, separate interface instances
            usb_port = u2d2_config.get('usb_port', '/dev/ttyUSB0')
            baudrate = u2d2_config.get('baudrate', 4000000)
            leader_port = follower_port = usb_port
            leader_baudrate = follower_baudrate = baudrate
            
            if self.verbose:
                print(f"\n🔌 Connecting to single U2D2 on {usb_port}...")
        
        # Create hand interfaces
        self.leader_hand = create_hand_interface_multi(
            self.config, "leader", "real", 
            port=leader_port, baudrate=leader_baudrate, use_fake=use_fake_u2d2
        )
        self.follower_hand = create_hand_interface_multi(
            self.config, "follower", "real",
            port=follower_port, baudrate=follower_baudrate, use_fake=use_fake_u2d2
        )
        
        # Connect hand interfaces
        self.leader_hand.connect()
        self.follower_hand.connect()
        
        if self.verbose:
            if use_fake_u2d2:
                if self.use_dual_u2d2:
                    print("✅ Dual Fake U2D2 Interfaces Initialized! (Testing Mode) \n")
                else:
                    print("✅ Single Fake U2D2 Interface Initialized! (Testing Mode) \n")
            elif self.use_dual_u2d2:
                print("✅ Dual U2D2 Interfaces Initialized! \n")
            else:
                print("✅ Single U2D2 Interface Initialized! \n")

    def disconnect(self):
        """Disconnect from the hand interfaces."""
        if self.verbose:
            if self.use_dual_u2d2:
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

    def _create_leader_control_params(self) -> ControlParams:
        """Create control parameters for leader motors."""
        # Get joint settings for leader chain
        leader_config = self.config.hand_config.leader
        joint_settings = []
        
        if hasattr(leader_config, 'joint_settings') and leader_config.joint_settings:
            joint_settings = leader_config.joint_settings
        
        # Create ControlParams for all leader motors
        params = create_control_params_from_config(self.leader_chain, joint_settings)
        
        if self.verbose:
            for i, motor_id in enumerate(self.leader_chain):
                print(f"    Leader motor {motor_id} control params: "
                      f"coulomb=({params.mu_coulomb_cw[i]:.1f}, {params.mu_coulomb_ccw[i]:.1f}), "
                      f"damping={params.mu_damping[i]:.3f}, "
                      f"static={params.mu_static_amplitude[i]:.1f}@{params.static_frequency[i]:.1f}Hz, "
                      f"force_rendering={params.force_rendering_gain[i]:.3f}")
        
        return params

    def setup_motors(self) -> None:
        """Set up all motors according to their configurations."""
        if self.leader_hand is None or self.follower_hand is None:
            raise RuntimeError("Hand interfaces not connected. Call connect() first.")
        
        if self.verbose:
            print("🔧 Setting up motors...")
        
        # Setup motors using hand interfaces
        self.leader_hand.setup_motors()
        self.follower_hand.setup_motors()
        
        if self.verbose:
            print("✅ Motors ready!")

    # ===============================
    # Validation Functions
    # ===============================
    def _validate_config(self) -> None:
        """Validate the configuration."""
        if len(self.leader_chain) != len(self.follower_chain):
            raise ValueError(
                f"Leader and follower chains must have the same length. "
                f"Leader: {len(self.leader_chain)}, Follower: {len(self.follower_chain)}"
            )
        
        # Validate control modes
        valid_leader_modes = ['torque_off', 'current']
        valid_follower_modes = ['position', 'current']
        
        if self.config.hand_config.leader.mode not in valid_leader_modes:
            raise ValueError(f"Invalid leader mode: {self.config.hand_config.leader.mode}. Must be one of {valid_leader_modes}")
        
        if self.config.hand_config.follower.mode not in valid_follower_modes:
            raise ValueError(f"Invalid follower mode: {self.config.hand_config.follower.mode}. Must be one of {valid_follower_modes}")

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
        
        print(f"Leader Motors:  {', '.join(leader_motor_models)}") # benchmark
        print(f"Follower Motors: {', '.join(follower_motor_models)}") # benchmark
        print(f"Control Modes:  Leader={self.config.hand_config.leader.mode}, Follower={self.config.hand_config.follower.mode}") # benchmark
        print(f"U2D2 Setup:     {'Dual U2D2' if self.use_dual_u2d2 else 'Single U2D2'}") # benchmark
        print(f"{'='*60}") # benchmark
        
        # Calculate statistics for each section # benchmark
        for section, stats in benchmark_stats.items(): # benchmark
            if stats['count'] == 0: # benchmark
                continue # benchmark
                
            avg_time = stats['sum'] / stats['count'] # benchmark
            min_time = stats['min'] # benchmark
            max_time = stats['max'] # benchmark
            
            # Convert to milliseconds for readability # benchmark
            avg_ms = avg_time * 1000 # benchmark
            min_ms = min_time * 1000 # benchmark
            max_ms = max_time * 1000 # benchmark
            
            print(f"{section:15} | Avg: {avg_ms:6.2f}ms | Min: {min_ms:6.2f}ms | Max: {max_ms:6.2f}ms") # benchmark
        
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
            # Red color codes for terminal output
            RED = "\033[91m"
            RESET = "\033[0m"
            print()
            print(f"{RED}{'='*50}{RESET}")  # Red line
            print(f"{RED}⚠️  CONTROL LOOP PERFORMANCE WARNING ⚠️{RESET}")
            print(f"{RED}   Running slower than target frequency!{RESET}")
            print(f"{RED}   Elapsed: {elapsed:.3f}s (target: {target_period:.3f}s){RESET}")
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
        
        # U2D2 configuration
        if self.use_dual_u2d2:
            print(f"U2D2: Dual U2D2")
            print(f"  Leader: {self.config.u2d2.leader.usb_port} @ {self.config.u2d2.leader.baudrate}")
            print(f"  Follower: {self.config.u2d2.follower.usb_port} @ {self.config.u2d2.follower.baudrate}")
        else:
            print(f"U2D2: Single U2D2")
            print(f"  Port: {self.config.u2d2.usb_port} @ {self.config.u2d2.baudrate}")
        
        # Motor chains
        leader_mode = self.config.hand_config.leader.mode
        follower_mode = self.config.hand_config.follower.mode
        print(f"\n📡 Leader Motors: {self.leader_chain} ({leader_mode} mode)")
        print(f"🎯 Follower Motors: {self.follower_chain} ({follower_mode} mode)")
        
        # Control laws for current mode leaders
        if leader_mode == 'current':
            print("  Control Laws:")
            params = self.leader_control_params
            for i, motor_id in enumerate(self.leader_chain):
                laws = []
                if params.enable_coulomb_comp[i]:
                    laws.append(f"Coulomb({params.mu_coulomb_cw[i]:.1f},{params.mu_coulomb_ccw[i]:.1f})")
                if params.enable_damping_comp[i]:
                    laws.append(f"Damping({params.mu_damping[i]:.3f})")
                if params.enable_static_comp[i]:
                    laws.append(f"Static({params.mu_static_amplitude[i]:.1f}@{params.static_frequency[i]:.1f}Hz)")
                if params.enable_force_rendering[i]:
                    laws.append(f"Force({params.force_rendering_gain[i]:.3f})")
                if not laws:
                    laws.append("Zero")
                
                print(f"    Motor {motor_id}: {' + '.join(laws)}")
        
        # Motor pairing
        print(f"\n🔗 Motor Pairing: {len(self.leader_chain)} pairs")
        for i, (leader_id, follower_id) in enumerate(zip(self.leader_chain, self.follower_chain)):
            print(f"  [{i}] {leader_id} -> {follower_id}")
        
        print("="*60)

    # ===============================
    # Main control loop functions
    # ===============================

    def start_control_loop(self):
        """Start the main control loop."""
        if self.leader_hand is None or self.follower_hand is None:
            raise RuntimeError("Hand interfaces not connected. Call connect() first.")
        
        print(f"\n🔄 Starting control loop at {self.control_frequency} Hz...")
        print(f"Move the leader motors {self.leader_chain} to see the follower motors {self.follower_chain} follow!")
        print(f"Leader mode: {self.config.hand_config.leader.mode}")
        print(f"Follower mode: {self.config.hand_config.follower.mode}")
        
        self.running = True
        cycle = 0
        
        # Benchmark tracking - using running statistics for efficiency # benchmark
        benchmark_interval = 1000 # benchmark
        benchmark_stats = { # benchmark
            'total': {'sum': 0.0, 'count': 0, 'min': float('inf'), 'max': 0.0}, # benchmark - includes sleep
            'execution': {'sum': 0.0, 'count': 0, 'min': float('inf'), 'max': 0.0}, # benchmark - execution only (no sleep)
            'read_states': {'sum': 0.0, 'count': 0, 'min': float('inf'), 'max': 0.0}, # benchmark
            'queue_data': {'sum': 0.0, 'count': 0, 'min': float('inf'), 'max': 0.0}, # benchmark
            'send_commands': {'sum': 0.0, 'count': 0, 'min': float('inf'), 'max': 0.0}, # benchmark
            'sleep_time': {'sum': 0.0, 'count': 0, 'min': float('inf'), 'max': 0.0}, # benchmark - actual sleep time
        } # benchmark
        
        try:
            cycle_start_time = time.time()  # Initialize for first cycle
            while self.running:
                cycle += 1
                
                # ===============================
                # Read states
                # ===============================
                read_start = time.time() # benchmark
                try:
                    leader_hand_data = self.leader_hand.read_states()
                    follower_hand_data = self.follower_hand.read_states()
                except Exception as e:
                    print(f"⚠️ Motor read error: {e}")
                    continue
                
                read_time = time.time() - read_start # benchmark
                self._update_running_stats(benchmark_stats['read_states'], read_time) # benchmark
                
                # Queue state data if queue is provided (for ROS2 publishing, etc.)
                queue_data_start = time.time() # benchmark
                if self.state_queue is not None:
                    # Convert HandData numpy arrays to lists for ROS2 compatibility
                    states_data = {
                        'leader': {
                            'positions': leader_hand_data.positions,
                            'velocities': leader_hand_data.velocities,
                            'currents': leader_hand_data.currents,
                            'joint_angles': leader_hand_data.joint_angles,
                            'joint_velocities': leader_hand_data.joint_velocities
                        },
                        'follower': {
                            'positions': follower_hand_data.positions,
                            'velocities': follower_hand_data.velocities,
                            'currents': follower_hand_data.currents,
                            'joint_angles': follower_hand_data.joint_angles,
                            'joint_velocities': follower_hand_data.joint_velocities
                        }
                    }
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
                queue_data_time = time.time() - queue_data_start # benchmark
                self._update_running_stats(benchmark_stats['queue_data'], queue_data_time) # benchmark
                
                # ===============================
                # Send commands
                # ===============================
                send_start = time.time() # benchmark
                
                # Leader 
                if self.config.hand_config.leader.mode == 'current':
                    # Extract follower currents for force rendering
                    follower_currents = follower_hand_data.currents
                    
                    # Compute current commands using the current control laws
                    current_commands = self.current_control_laws.compute_current_command(
                        leader_hand_data, 
                        self.leader_control_params, 
                        time.time(),
                        follower_currents,
                        self.show_current_breakdown
                    )
                    
                    # Send current commands
                    self.leader_hand.send_current_commands(current_commands)
                
                # Follower
                if self.config.hand_config.follower.mode == 'position':
                    leader_joint_angles = leader_hand_data.joint_angles
                    self.follower_hand.send_position_commands(leader_joint_angles)
                
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
                
                self._update_running_stats(benchmark_stats['sleep_time'], actual_sleep) # benchmark
                
                # Measure total cycle time for benchmark
                after_sleep = time.time() # benchmark
                total_time = after_sleep - cycle_start_time # benchmark
                self._update_running_stats(benchmark_stats['total'], total_time) # benchmark
                
                # Print benchmark every 1000 cycles # benchmark
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