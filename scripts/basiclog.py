import logging
import time
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
    def __init__(self, link_uri, run_name, config, timeout, period, mode):
        self._cf = Crazyflie(rw_cache='./build/cache')

        self._run_name = run_name
        self._config = config
        self._log_period = period
        self._mode = mode
        self._log_file_path = Path('./logs').joinpath(f'{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}_{self._run_name}_{self._config}_{self._mode}.csv')
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
        self._timeout_timer: Timer | None = None
        self._start_log_thread()

        # Connect callbacks
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.console.receivedChar.add_callback(self._console_log)

        print('Connecting to %s' % link_uri)

        self._cf.open_link(link_uri)

        self._timeout = timeout
        self.is_connected = False
        self._killed = Event()
        self._trajectory_tracking = False
        self._console_log_file = Path('./logs').joinpath(f'{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}_{self._run_name}_{self._config}_{self._mode}.log')

    def _console_log(self, text):
        with open(self._console_log_file, 'a') as f:
            f.write(text)

    def _connected(self, link_uri):
        print('Connected to %s' % link_uri)
        self.is_connected = True

        self._lg_stab = LogConfig(name=self._run_name, period_in_ms=self._log_period)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        if self._config == 'velocity':
            self._lg_stab.add_variable('stateEstimate.vx', 'float')
            self._lg_stab.add_variable('stateEstimate.vy', 'float')
            self._lg_stab.add_variable('stateEstimate.vz', 'float')
        elif self._config == 'motors':
            self._lg_stab.add_variable('motor.m1', 'uint16_t')
            self._lg_stab.add_variable('motor.m2', 'uint16_t')
            self._lg_stab.add_variable('motor.m3', 'uint16_t')
            self._lg_stab.add_variable('motor.m4', 'uint16_t')
        else:
            self._lg_stab.add_variable('stabilizer.roll', 'float')
            self._lg_stab.add_variable('stabilizer.pitch', 'float')
            self._lg_stab.add_variable('stabilizer.yaw', 'float')

        try:
            self._cf.log.add_config(self._lg_stab)
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            self._lg_stab.start()
            self._cf.platform.send_arming_request(True)
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

    def exit(self):
        # Make shutdown idempotent
        if self._killed.is_set():
            return
        self._killed.set()
        # Cancel timeout timer if any
        if self._timeout_timer is not None:
            try:
                self._timeout_timer.cancel()
            except Exception:
                pass
            self._timeout_timer = None
        # Try to stop logging/link even if fly() hasn't run yet
        try:
            if hasattr(self, '_lg_stab'):
                self._lg_stab.stop()
        except Exception:
            pass
        try:
            self._cf.close_link()
        except Exception:
            pass
        self._close_log()

    def _close_log(self):
        # Stop writer thread
        if getattr(self, '_log_thread', None) and self._log_thread.is_alive():
            self._log_stop.set()
            # unblock queue.get()
            if self._log_queue:
                self._log_queue.put(None)
            # join with timeout so Ctrl+C never hangs
            self._log_thread.join(timeout=2.0)
            self._log_thread = None

        if getattr(self, '_log_file', None):
            try:
                self._log_file.flush()
            finally:
                try:
                    self._log_file.close()
                except Exception:
                    pass
                self._log_file = None
                self._csv_writer = None
        self._log_queue = None

    def _start_log_thread(self):
        if self._log_thread and self._log_thread.is_alive():
            return
        self._log_stop.clear()
        # Daemon thread so process can exit on Ctrl+C even if we didn't manage to join
        self._log_thread = Thread(target=self._log_writer_loop, name='CSVLogWriter', daemon=True)
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
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
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
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False
        self._close_log()

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))
        self._close_log()

    def _disconnected(self, link_uri):
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
        self._set_param("rlt.trigger", 0)
        self._set_param("rlt.motor_warmup", 0)
        self._set_param("rlt.wn", 4)
        self._set_param("rlt.fei", args.trajectory_interval)
        self._set_param("rlt.fes", args.trajectory_scale)
        self._set_param("rlt.target_z", args.height)

    def set_mode_hover_learned(self, args):
        self._set_param("rlt.trigger", 0)
        self._set_param("rlt.wn", 1)
        self._set_param("rlt.motor_warmup", 1)
        self._set_param("rlt.target_z", args.height)

    def fly(self):
        seconds_remaining = int(self._timeout) if self._timeout is not None else None

        if self._timeout is not None:
            print(f"Flying for {self._timeout} seconds")
            self._timeout_timer = Timer(self._timeout, self.exit)
            self._timeout_timer.daemon = True
            self._timeout_timer.start()

        # Use monotonic for timing
        prev = time.monotonic()
        rate_acc = 0.0
        cnt = 0
        second_accum = 0.0
        start_time = prev

        while not self._killed.is_set():
            now = time.monotonic()
            dt = now - prev
            prev = now

            # rate / heartbeat
            rate_acc += dt
            cnt += 1
            time.sleep(0.01)
            if cnt % 100 == 0 and rate_acc > 0:
                print(f"Average rate: {1 / (rate_acc / cnt):.3f}Hz")
                rate_acc = 0.0
                cnt = 0

            # ----- per-second countdown (robust to jitter) -----
            if seconds_remaining is not None:
                second_accum += dt
                while second_accum >= 1.0 and seconds_remaining > 0:
                    seconds_remaining -= 1
                    print(f"Time remaining: {seconds_remaining} seconds")
                    second_accum -= 1.0
            # ---------------------------------------------------

            if self._mode == "trajectory_tracking":
                # detect long pause to reset transition timer
                if dt > 0.1:
                    start_time = now

                if (now - start_time) < args.transition_timeout:
                    self._send_hover_packet(args.height)
                else:
                    self._send_learned_policy_packet()
            else:
                self._send_learned_policy_packet()

        # Cleanup when loop exits
        try:
            if hasattr(self, '_lg_stab'):
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
    import argparse
    from cflib.utils import uri_helper
    import cflib.crtp
    import signal

    parser = argparse.ArgumentParser()
    default_uri = 'radio://0/88/2M/E7E7E7E7EF'
    parser.add_argument('--uri', default=default_uri)
    parser.add_argument('--height', default=0.5, type=float)
    parser.add_argument('--mode', default='position_control', choices=['position_control', 'trajectory_tracking'])
    parser.add_argument('--trajectory-scale', default=0.3, type=float)
    parser.add_argument('--trajectory-interval', default=5.5, type=float)
    parser.add_argument('--transition-timeout', default=3, type=float)
    parser.add_argument('--run-name', default='test', type=str)
    parser.add_argument('--config', default='velocity', choices=['velocity', 'attitude', 'motors'])
    parser.add_argument('--timeout', default=None, type=float)
    parser.add_argument('--period', default=10, type=int)
    args = parser.parse_args()

    uri = uri_helper.uri_from_env(default=default_uri)

    cflib.crtp.init_drivers()

    le = LoggingExample(uri, args.run_name, args.config, args.timeout, args.period, args.mode)

    while not le.is_connected:
        time.sleep(0.1)

    if args.mode == "trajectory_tracking":
        le.set_mode_trajectory_tracking(args)
    else:
        le.set_mode_hover_learned(args)

    try:
        input("Press enter to start hovering")
    except (EOFError, KeyboardInterrupt):
        print("Aborted before takeoff.")
        le.exit()
    else:
        try:
            le.fly()
        except KeyboardInterrupt:
            print("Interrupted, shutting down...")
            le.exit()