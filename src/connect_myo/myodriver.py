import sys
import time
from connect_myo.public.myohw import *
from connect_myo.myo import Myo
from connect_myo.bluetooth import Bluetooth
from connect_myo.data_handler import DataHandler
import struct


class MyoDriver:
    """
    Responsible for myo connections and messages.
    """
    def __init__(self, config):
        self.config = config
        self.addr_to_connect = None

        self.data_handler = DataHandler(self.config)
        self.bluetooth = Bluetooth(self.config.MESSAGE_DELAY)

        self.myos = []

        self.myo_to_connect = None
        self.scanning = False

        # store info for ROS
        self.emg_val = None
        self.imu_val = None
        self.gest_val = None
        self.myo_arm = None

        # Add handlers for expected events
        self.set_handlers()

    def run(self):
        """
        Main. Disconnects possible connections and starts as many connections as needed.
        """
        self.disconnect_all()
        print("$$$$$$$$$$$$$")
        print(self.bluetooth.serialport)
        while len(self.myos) < self.config.MYO_AMOUNT:
            print(
                "*** Connecting myo " + str(len(self.myos) + 1) + " out of " + str(self.config.MYO_AMOUNT) + " ***")
            print()
            self.add_myo_connection()
        self.receive()

    def receive(self):
        self.bluetooth.receive()


##############################################################################
#                                  CONNECT                                   #
##############################################################################

    def add_myo_connection(self):
        """
        Procedure for connection with the Myo Armband. Scans, connects, disables sleep and starts EMG stream.
        """
        # Discover
        self._print_status("Scanning")
        self.bluetooth.gap_discover()

        # Await myo detection and create Myo object.
        self.scanning = True
        while self.myo_to_connect is None:
            self.bluetooth.receive()

        # End gap
        self.bluetooth.end_gap()

        if (self.bluetooth.serialport == "/dev/ttyACM0" and self.myo_arm == "left") or (self.bluetooth.serialport == "/dev/ttyACM1" and self.myo_arm == "right"):
            # Add handlers
            self.bluetooth.add_connection_status_handler(self.create_connection_status_handle(self.myo_to_connect))
            self.bluetooth.add_disconnected_handler(self.create_disconnect_handle(self.myo_to_connect))

            # Direct connection. Reconnect implements the retry procedure.
            self.myos.append(self.myo_to_connect)
            self.connect_and_retry(self.myo_to_connect, self.config.RETRY_CONNECTION_AFTER, self.config.MAX_RETRIES)
        self.myo_to_connect = None

    def connect_and_retry(self, myo, timeout=None, max_retries=None):
        """
        Procedure for a reconnection.
        :param myo: Myo object to connect. Should have its address set
        :param timeout: Time to wait for response
        :param max_retries: Max retries before exiting the program
        :return: True if connection was successful, false otherwise.
        """
        retries = 0
        # The subroutine will await the response until timeout is met
        while not self.direct_connect(myo, timeout) and not myo.connected:
            retries += 1
            if max_retries is not None and retries > max_retries:
                print("Max retries reached. Exiting")
                sys.exit(1)
            print()
            print("Reconnection failed for connection " + str(myo.connection_id) + ". Retry " + str(retries) + "...")
        myo.set_connected(True)
        return True

    def direct_connect(self, myo_to_connect, timeout=None):
        """
        Procedure for a direct connection with the device.
        :param myo_to_connect: Myo object to connect. Should have its address set
        :param timeout: Time to wait for response
        :return: True if connection was successful, false otherwise.
        """
        t0 = time.time()
        # Direct connection
        self._print_status("Connecting to", myo_to_connect.address)
        self.bluetooth.direct_connect(myo_to_connect.address)

        # Await response
        while myo_to_connect.connection_id is None or not myo_to_connect.connected:
            if timeout is not None and timeout + t0 < time.time():
                return False
            self.receive()

        # Notify successful connection with self.print_status and vibration
        self._print_status("Connection successful. Setting up...")
        self._print_status()
        self.bluetooth.send_vibration_medium(myo_to_connect.connection_id)

        # Disable sleep
        self.bluetooth.disable_sleep(myo_to_connect.connection_id)

        # Enable data and subscribe
        self.bluetooth.enable_data(myo_to_connect.connection_id, self.config)

        print("Myo ready", myo_to_connect.connection_id, struct.unpack('6B', myo_to_connect.address))
        print()
        return True


##############################################################################
#                                  HANDLERS                                  #
##############################################################################

    def handle_discover(self, _, payload):
        """
        Handler for ble_evt_gap_scan_response event.
        """
        if self.scanning and not self.myo_to_connect:
            self._print_status("Device found", payload['sender'])
            if payload['data'].endswith(bytes(Final.myo_id)):
                # if not self._has_paired_with(payload['sender']) and self._need_to_connect(payload['sender']):
                if not self._has_paired_with(payload['sender']):
                    self.myo_to_connect = Myo(payload['sender'])
                    self._print_status("Myo found", self.myo_to_connect.address)
                    print(self.myo_to_connect.address)
                    if self.myo_to_connect.address == b'.\x90Z\xf7\r\xe5' or self.myo_to_connect.address == b'\xfc/\x84\x04\xeb\xf1':
                        self.myo_arm = 'right'  #          low                                                 upper
                        print('right')
                    elif self.myo_to_connect.address == b'\xd8hr\x86\x02\xdd' or self.myo_to_connect.address == b'$\x92\xe2\x05\x17\xc5':
                        self.myo_arm = 'left'   #            low                                                    upper
                        print('left')
                        
                    self._print_status()
                    self.scanning = False

    def _need_to_connect(self, address):
        """
        Checks if given address is the one want to connect 
        :param address: address to connect 
        :return: True if it is. False otherwise
        """
        print("Connecting myo", len(self.myos), " at ", address, " ---> ", self.addr_to_connect[len(self.myos)])
        if address == self.addr_to_connect[len(self.myos)]:
            return True
        return False


    def _has_paired_with(self, address):
        """
        Checks if given address has already been recorded in a Myo initialization.
        :param address: address to check
        :return: True if already paired, False otherwise.
        """
        for m in self.myos:
            if m.address == address:
                return True
        return False

    def handle_connect(self, _, payload):
        """
        Handler for ble_rsp_gap_connect_direct event.
        """
        if not payload['result'] == 0:
            if payload['result'] == 385:
                print("ERROR: Device in Wrong State")
            else:
                print(payload)
        else:
            self._print_status("Connection successful")

    def create_disconnect_handle(self, myo):
        def handle_disconnect(_, payload):
            """
            Handler for ble_evt_connection_status event.
            """
            if myo.connection_id == payload['connection']:
                print("Connection " + str(payload['connection']) + " lost.")
                myo.set_connected(False)
                if payload['reason'] == 574:
                    print("Disconnected. Reason: Connection Failed to be Established.")
                if payload['reason'] == 534:
                    print("Disconnected. Reason: Connection Terminated by Local Host.")
                if payload['reason'] == 520:
                    print("Disconnected. Reason: Connection Timeout.")
                # if self.bluetooth.serialport == '/dev/ttyACM1':
                #     print()
                else:
                    print("Disconnected:", payload)
                # Won't return until the connection is established successfully
                print("Reconnecting...")
                self.connect_and_retry(myo, self.config.RETRY_CONNECTION_AFTER, self.config.MAX_RETRIES)

        return handle_disconnect

    def create_connection_status_handle(self, myo):
        def handle_connection_status(_, payload):
            """
            Handler for ble_evt_connection_status event.
            """
            if payload['address'] == myo.address and payload['flags'] == 5:
                self._print_status("Connection status: ", payload)
                myo.set_connected(True)
                myo.set_id(payload['connection'])
                # myo.set_id(len(self.myos)-1)
                self._print_status("Connected with id", myo.connection_id)

        return handle_connection_status

    def handle_attribute_value(self, e, payload):
        """
        Handler for EMG events, expected as a ble_evt_attclient_attribute_value event with handle 43, 46, 49 or 52.
        """
        emg_handles = [
            ServiceHandles.EmgData0Characteristic,
            ServiceHandles.EmgData1Characteristic,
            ServiceHandles.EmgData2Characteristic,
            ServiceHandles.EmgData3Characteristic
        ]
        imu_handles = [
            ServiceHandles.IMUDataCharacteristic
        ]
        myo_info_handles = [
            ServiceHandles.DeviceName,
            ServiceHandles.FirmwareVersionCharacteristic,
            ServiceHandles.BatteryCharacteristic
        ]
        gest_handles = [
            ServiceHandles.ClassifierEventCharacteristic
        ]

        # Delegate EMG
        if payload['atthandle'] in emg_handles:
            self.emg_val = self.data_handler.handle_emg(payload)

        # Delegate IMU
        elif payload['atthandle'] in imu_handles:
            self.imu_val = self.data_handler.handle_imu(payload)

        # Delegate classifier
        elif payload['atthandle'] in gest_handles:
            self.gest_val = self.data_handler.handle_gest(payload)

        # Delegate myo info
        elif payload['atthandle'] in myo_info_handles:
            for myo in self.myos:
                myo.handle_attribute_value(payload)

        # Print otherwise
        else:
            self._print_status(e, payload)

    def set_handlers(self):
        """
        Set handlers for relevant events.
        """
        self.bluetooth.add_scan_response_handler(self.handle_discover)
        self.bluetooth.add_connect_response_handler(self.handle_connect)
        self.bluetooth.add_attribute_value_handler(self.handle_attribute_value)


##############################################################################
#                                    MYO                                     #
##############################################################################

    def get_info(self):
        """
        Send read attribute messages and await answer.
        """
        if len(self.myos):
            self._print_status("Getting myo info")
            self._print_status()
            for myo in self.myos:
                self.bluetooth.read_device_name(myo.connection_id)
                self.bluetooth.read_firmware_version(myo.connection_id)
                self.bluetooth.read_battery_level(myo.connection_id)
            while not self._myos_ready():
                self.receive()
            print("Myo list:")
            for myo in self.myos:
                print(" - " + str(myo))
            print()

    def disconnect_all(self):
        """
        Stop possible scanning and close all connections.
        """
        self.bluetooth.disconnect_all()

    def deep_sleep_all(self):
        """
        Send deep sleep (turn off) signal to every connected myo.
        """
        print("Turning off devices...")
        for m in self.myos:
            self.bluetooth.deep_sleep(m.connection_id)
        print("Disconnected.")


##############################################################################
#                                   UTILS                                    #
##############################################################################

    def _myos_ready(self):
        """
        :return: True if every myo has its data set, False otherwise.
        """
        for m in self.myos:
            if not m.ready():
                return False
        return True

    def _print_status(self, *args):
        """
        Printer function for VERBOSE support.
        """
        if self.config.VERBOSE:
            print(*args)

##############################################################################
#                                   Return Val to ROS                        #
##############################################################################
    
    def get_right_emg(self):
        """
        :return: two raw EMG lists in list. The first EMG list corresponds to 
                    myo (id=0) and the second list corresponds to myo (id=0)
        """
        if self.myo_arm == "right":
            return self.emg_val
    
    def get_right_imu(self):
        """
        :return: two raw IMU lists in list. The first IMU list corresponds to 
                    myo (id=0) and the second list corresponds to myo (id=0)
        """       
        if self.myo_arm == "right":   
            return self.imu_val
    
    def get_right_gest(self):
        """
        :return: predicted gesture integer and id of myo sending out the prediction
        """       
        if self.myo_arm == "right":
            return self.gest_val
        
        
    #for left arm:
    def get_left_emg(self):
        """
        :return: two raw EMG lists in list. The first EMG list corresponds to 
                    myo (id=0) and the second list corresponds to myo (id=0)
        """
        if self.myo_arm == "left":
            return self.emg_val
    
    def get_left_imu(self):
        """
        :return: two raw IMU lists in list. The first IMU list corresponds to 
                    myo (id=0) and the second list corresponds to myo (id=0)
        """       
        if self.myo_arm == "left":    
            return self.imu_val
    
    def get_left_gest(self):
        """
        :return: predicted gesture integer and id of myo sending out the prediction
        """       
        if self.myo_arm == "left":
            return self.gest_val
    