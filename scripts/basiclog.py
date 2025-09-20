import logging
import time
import signal
import argparse
import struct
import csv
import queue
import datetime
from pathlib import Path
from threading import Event, Lock, Thread, Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from cflib.crazyflie.commander import META_COMMAND_CHANNEL, SET_SETPOINT_CHANNEL, TYPE_HOVER

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri, run_name, config, timeout, period):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./build/cache')

        self._run_name = run_name
        self._config = config
        self._log_period = period
        self._log_file_path = Path('./logs').joinpath(f'{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}_{self._run_name}_{self._config}.csv')
        parent = self._log_file_path.parent
        if parent and not parent.exists():
            parent.mkdir(parents=True, exist_ok=True)
        self._log_file = self._log_file_path.open('w', newline='')
        self._csv_writer = csv.writer(self._log_file)
        self._csv_header_written = False
        self._log_fields = []
        self._log_queue = queue.Queue()
        self._log_lock = Lock()
        self._log_thread = None
        self._log_stop = Event()
        self._start_log_thread()

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self._timeout = timeout
        self.is_connected = True
        self._killed = False
        self._trajectory_tracking = False

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name=self._run_name, period_in_ms=self._log_period)
        if self._config == 'state':
            self._lg_stab.add_variable('stateEstimate.x', 'float')
            self._lg_stab.add_variable('stateEstimate.y', 'float')
            self._lg_stab.add_variable('stateEstimate.z', 'float')
            self._lg_stab.add_variable('stateEstimate.vx', 'float')
            self._lg_stab.add_variable('stateEstimate.vy', 'float')
            self._lg_stab.add_variable('stateEstimate.vz', 'float')
        else:
            self._lg_stab.add_variable('stabilizer.roll', 'float')
            self._lg_stab.add_variable('stabilizer.pitch', 'float')
            self._lg_stab.add_variable('stabilizer.yaw', 'float')
            self._lg_stab.add_variable('motor.m1', 'uint16_t')
            self._lg_stab.add_variable('motor.m2', 'uint16_t')
            self._lg_stab.add_variable('motor.m3', 'uint16_t')
            self._lg_stab.add_variable('motor.m4', 'uint16_t')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
            # Send arming request
            self._cf.platform.send_arming_request(True)
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def exit(self):
        self._killed = True

    def _close_log(self):
        if getattr(self, '_log_thread', None) and self._log_thread.is_alive():
            self._log_stop.set()
            if self._log_queue:
                self._log_queue.put(None)
            self._log_thread.join()
            self._log_thread = None

        if getattr(self, '_log_file', None):
            try:
                self._log_file.flush()
            finally:
                self._log_file.close()
                self._log_file = None
                self._csv_writer = None
        self._log_queue = None

    def _start_log_thread(self):
        if self._log_thread and self._log_thread.is_alive():
            return
        self._log_stop.clear()
        self._log_thread = Thread(target=self._log_writer_loop, name='CSVLogWriter', daemon=False)
        self._log_thread.start()

    def _log_writer_loop(self):
        while True:
            try:
                item = self._log_queue.get(timeout=0.5)
            except queue.Empty:
                if self._log_stop.is_set():
                    break
                continue

            if item is None:
                break

            if self._csv_writer:
                self._csv_writer.writerow(item)
                self._log_file.flush()

        # Drain any remaining items to unblock producers
        while self._log_queue and not self._log_queue.empty():
            try:
                self._log_queue.get_nowait()
            except queue.Empty:
                break

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        if not self._csv_writer or not self._log_queue:
            return

        header_row = None
        with self._log_lock:
            if not self._csv_header_written:
                self._log_fields = list(data.keys())
                header_row = ['timestamp (ms)'] + self._log_fields
                self._csv_header_written = True
            fields = list(self._log_fields)

        if header_row:
            self._log_queue.put(header_row)

        row = [timestamp] + [data[name] for name in fields]
        self._log_queue.put(row)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False
        self._close_log()

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))
        self._close_log()

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False
        self._close_log()

    def _set_param(self, name, target):
        time.sleep(1.0)
        while abs(float(self._cf.param.get_value(name)) - float(target)) > 1e-5:
            time.sleep(1.0)
            self._cf.param.set_value(name, target)
            time.sleep(1.0)
        time.sleep(1.0)
        print(f"Parameter {name} is {self._cf.param.get_value(name)} now")

    def set_mode_trajectory_tracking(self, args):
        self._trajectory_tracking = True
        self._set_param("rlt.trigger", 0) # setting the trigger mode to the custom command  (cf. https://github.com/arplaboratory/learning_to_fly_controller/blob/0a7680de591d85813f1cd27834b240aeac962fdd/rl_tools_controller.c#L80)
        self._set_param("rlt.motor_warmup", 0)
        self._set_param("rlt.wn", 4)
        self._set_param("rlt.fei", args.trajectory_interval)
        self._set_param("rlt.fes", args.trajectory_scale)
        self._set_param("rlt.target_z", args.height)

    def fly(self):
        if self._timeout is not None:
            print(f"Flying for {self._timeout} seconds")
            # Start a timer to disconnect in 10s
            t = Timer(self._timeout, self.exit)
            t.start()

        prev = time.time()
        acc = 0
        cnt = 0
        start_time = time.time()

        while not self._killed:
            current = time.time()
            acc += current - prev
            cnt += 1
            if cnt % 100 == 0:
                print(f"Average rate: {1/(acc / cnt):.3f}Hz")
                acc = 0
                cnt = 0

            if self._trajectory_tracking:
                now = time.time()
                if current - prev > 0.1:
                    start_time = now
                prev = current

                if now - start_time < args.transition_timeout:
                    self._send_hover_packet(args.height)
                else:
                    self._send_learned_policy_packet()
            else:
                prev = current
                self._send_learned_policy_packet()

        try:
            self._lg_stab.stop()
            self._cf.close_link()
            print("Link closed")
        except Exception as e:
            print(f"Exception when closing link: {e}")
        finally:
            self._close_log()

    def _send_learned_policy_packet(self):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.channel = META_COMMAND_CHANNEL
        pk.data = struct.pack('<B', 1)
        self._cf.send_packet(pk)

    def _send_hover_packet(self, height, vx=0, vy=0, yawrate=0):
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.channel = SET_SETPOINT_CHANNEL
        pk.data = struct.pack('<Bffff', TYPE_HOVER, vx, vy, yawrate, height)
        self._cf.send_packet(pk)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    default_uri = 'radio://0/88/2M/E7E7E7E7EF'
    parser.add_argument('--uri', default=default_uri)
    parser.add_argument('--height', default=0.3, type=float)
    parser.add_argument('--mode', default='hover_original', choices=['hover_learned', 'hover_original', 'takeoff_and_switch', 'trajectory_tracking'])
    parser.add_argument('--trajectory-scale', default=0.3, type=float, help="Scale of the trajectory")
    parser.add_argument('--trajectory-interval', default=5.5, type=float, help="Interval of the trajectory")
    parser.add_argument('--transition-timeout', default=3, type=float, help="Time after takeoff with the original controller after which the learned controller is used for trajectory tracking")
    parser.add_argument('--run-name', default='test', type=str, help="Name of the run, used for logging")
    parser.add_argument('--config', default='state', choices=['state', 'motors'], help="Logging configuration")
    parser.add_argument('--timeout', default=None, type=float, help="Time in seconds after which to stop the experiment")
    parser.add_argument('--period', default=10, type=int, help="Logging period in ms")

    args = parser.parse_args()
    uri = uri_helper.uri_from_env(default=default_uri)

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = LoggingExample(uri, args.run_name, args.config, args.timeout, args.period)

    def _stop_on_signal(signum, frame):
        print("Ctrl+c received")
        le.exit()

    signal.signal(signal.SIGINT, _stop_on_signal)
    signal.signal(signal.SIGTERM, _stop_on_signal)

    if args.mode == "trajectory_tracking":
        le.set_mode_trajectory_tracking(args)

    input("Press enter to start hovering")
    le.fly()
