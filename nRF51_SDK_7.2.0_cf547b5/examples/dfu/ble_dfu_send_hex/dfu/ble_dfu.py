import time
from datetime import datetime, timedelta
from intelhex import IntelHex
from dfu.master_emulator import MasterEmulator

import System
import Nordicsemi

# DFU OpCodes
class OpCodes:
    START_DFU = 1
    INITIALIZE_DFU = 2
    RECEIVE_FIRMWARE_IMAGE = 3
    VALIDATE_FIRMWARE_IMAGE = 4
    ACTIVATE_FIRMWARE_AND_RESET = 5
    SYSTEM_RESET = 6
    REQ_PKT_RCPT_NOTIF = 8
    RESPONSE = 16
    PKT_RCPT_NOTIF = 17

class DfuMode:
    SOFTDEVICE  = 1
    BOOTLOADER  = 2
    SdAndBl     = 3
    SDANDBL     = 3
    APPLICATION = 4

# Textual description lookup table for status codes received from peer.
status_code_lookup = {
    1: "SUCCESS",
    2: "Invalid State",
    3: "Not Supported",
    4: "Data Size Exceeds Limit",
    5: "CRC Error",
    6: "Operation Failed"
}

hex_base = 16
word_size = 0x04
page_size = 0x400


# Helper functions
def create_byte_array(size, value=0x55):
    """ Create a IronPython byte array with initial value. """
    return System.Array[System.Byte]([value]*size)


def convert_uint32_to_array(value):
    """ Convert a number into an array of 4 bytes (LSB). """
    return [(value >> 0 & 0xFF), (value >> 8 & 0xFF),
            (value >> 16 & 0xFF), (value >> 24 & 0xFF)]


def convert_uint16_to_array(value):
    """ Convert a number into an array of 2 bytes (LSB). """
    return [(value >> 0 & 0xFF), (value >> 8 & 0xFF)]

def calc_crc16(binfile):
    crc = 0xFFFF

    for b in binfile:
        crc = (crc >> 8 & 0x00FF) | (crc << 8 & 0xFF00)
        crc = crc ^ b
        crc = crc ^ ( ( crc & 0x00FF) >> 4 )
        crc = crc ^ ( ( crc << 8) << 4)
        crc = crc ^ ( ( crc & 0x00FF) << 4) << 1
    return crc & 0xFFFF

# Service UUID
uuid_dfu_service = Nordicsemi.BtUuid('000015301212EFDE1523785FEABCD123')
# Characteristic UUID
uuid_dfu_control_state_characteristic = Nordicsemi.BtUuid('000015311212EFDE1523785FEABCD123')
uuid_dfu_packet_characteristic = Nordicsemi.BtUuid('000015321212EFDE1523785FEABCD123')
#Descriptor UUID
uuid_client_characteristic_configuration_descriptor = Nordicsemi.BtUuid(0x2902)

# number of retries for sending a packet
num_of_send_tries = 1

# NOTE:  If packet receipt notification is enabled, a packet receipt
#        notification will be received for each 'num_of_packets_between_notif'
#        number of packets.
#
# Configuration tip: Increase this to get lesser notifications from the DFU
# Target about packet receipts. Make it 0 to disable the packet receipt
# notification
num_of_packets_between_notif = 10

last_error = "SUCCESS"

class BleDfu(MasterEmulator):
    """ Class to handle upload of a new hex image to the peer device. """


    def __init__(self, peer_device_address, hexfile_path, baud_rate, program_mode, own_address=None, bond_info=None):
        super(BleDfu, self).__init__(peer_device_address, baud_rate, own_address, bond_info)

        self.hexfile_path = hexfile_path
        self.ready_to_send = True
        self.send_crc = False
        self.response_op_code_received = -1
        self.program_mode = program_mode

        self.updating_sd = self.program_mode == DfuMode.SOFTDEVICE or self.program_mode == DfuMode.SdAndBl
        self.updating_bl = self.program_mode == DfuMode.BOOTLOADER or self.program_mode == DfuMode.SdAndBl
        self.updating_app = self.program_mode == DfuMode.APPLICATION


    def ProgressChanged(self, percent, message, finished=False):
        pass


    def data_received_handler(self, sender, e):
        """ Handle received data from the peer device.
        Note: This function overrides the parent class.
        """
        if (e.PipeNumber == self.pipe_dfu_control_point_notify):
            op_code = int(e.PipeData[0])

            if op_code == OpCodes.PKT_RCPT_NOTIF:
                if self.ready_to_send == True:
                    self.log_handler.log("ERROR: !!!! Pkt receipt notification received when it is not expected")
                else:
                    self.log_handler.log("Pkt receipt notification received.")

                self.log_handler.log("Number of bytes LSB = {0}".format(e.PipeData[1]))
                self.log_handler.log("Number of bytes MSB = {0}".format(e.PipeData[2]))
                self.ready_to_send = True

            elif op_code == OpCodes.RESPONSE:
                request_op_code = int(e.PipeData[1])
                response_value = int(e.PipeData[2])
                self.log_handler.log("Response received for Request Op Code = {0}".format(request_op_code))

                status_text = "UNKNOWN"
                if status_code_lookup.has_key(response_value):
                    status_text = status_code_lookup[response_value]

                if status_text != "SUCCESS":
                    last_error = status_text

                if request_op_code == 1:
                    self.log_handler.log("Response for 'Start DFU' received - Status: %s" % status_text)
                elif request_op_code == 2:
                    self.log_handler.log("Response for 'Initialize DFU Params' received - Status: %s" % status_text)
                elif request_op_code == 3:
                    self.log_handler.log("Response for 'Receive FW Data' received - Status: %s" % status_text)
                elif request_op_code == 4:
                    self.log_handler.log("Response for 'Validate' received - Status: %s" % status_text)
                else:
                    self.log_handler.log("!!ERROR!! Response for Unknown command received.")

                self.response_op_code_received = request_op_code

        else:
            self.log_handler.log("Received data on unexpected pipe %r"%e.PipeNumber)


    def setup_service(self):
        """ Set up DFU service database. """
        # Add DFU Service
        self.master.SetupAddService(uuid_dfu_service, Nordicsemi.PipeStore.Remote)

        # Add DFU characteristics
        self.master.SetupAddCharacteristicDefinition(uuid_dfu_packet_characteristic, 2, create_byte_array(2))
        self.pipe_dfu_packet = self.master.SetupAssignPipe(Nordicsemi.PipeType.Transmit)

        self.master.SetupAddCharacteristicDefinition(uuid_dfu_control_state_characteristic, 2, create_byte_array(2))
        self.pipe_dfu_control_point = self.master.SetupAssignPipe(Nordicsemi.PipeType.TransmitWithAck)
        self.pipe_dfu_control_point_notify = self.master.SetupAssignPipe(Nordicsemi.PipeType.Receive)


    def get_uint32_from_intel_hex(self, intel_hex, start_address):
        value = 0

        for i in xrange(word_size):
            value += intel_hex[start_address + i] * 256**i

        return value


    def find_high_address(self, intel_hex):
        max_address = intel_hex.maxaddr() + 1
        #Rounding up to closest page start.
        #Because of 0-indexing address "page_size" should be rounded up to 2*page_size.
        max_address = ((max_address)/page_size + 1) * page_size

        if max_address < 0x100000:
            return max_address

        max_address = 0x100000

        while max_address >= 0:
            last_value = self.get_uint32_from_intel_hex(intel_hex, max_address)

            if last_value != 0xFFFFFFFF:
                max_address += page_size
                break;
            else:
                max_address -= page_size

        return max_address


    def char_string_to_hex_string(self, string):
        converted_string = ''

        for character in string[::-1]:
            hex_fragment = hex(ord(character))[2:]
            if len(hex_fragment) == 1:
                hex_fragment = '0' + hex_fragment
            converted_string += hex_fragment

        return converted_string


    def _get_softdevice_bin_array(self, intel_hex):
        sd_bin_array = None
        try:
            sd_magic_value_address = 0x3004
            expected_magic_value = 0x51B1E5DB
            sd_magic_value = self.get_uint32_from_intel_hex(intel_hex, sd_magic_value_address)
            if sd_magic_value == expected_magic_value:
                sd_end_address_address = 0x3008
                sd_end_address = self.get_uint32_from_intel_hex(intel_hex, sd_end_address_address)

                address_after_mbr_page = 0x1000
                sd_last_byte_address = sd_end_address - 1
                sd_bin_array = intel_hex.tobinarray(start = address_after_mbr_page,
                                                    end = sd_last_byte_address)

        except Exception, e:
            #Something is WRONG!
            pass

        return sd_bin_array


    def _get_bootloader_bin_array(self, intel_hex):
        bl_bin_array = None
        bl_start_address = 0x30000
        max_bl_start_address = 0x80000

        while bl_start_address < max_bl_start_address:
            init_stack_pointer = self.get_uint32_from_intel_hex(intel_hex, bl_start_address)
            stack_start_address = 0x20000000

            if init_stack_pointer != 0xFFFFFFFF:
                if init_stack_pointer > stack_start_address:
                    break

            bl_start_address = bl_start_address + page_size

        bl_end_address = self.find_high_address(intel_hex)

        bl_last_byte_address = bl_end_address - 1
        bl_bin_array = intel_hex.tobinarray(start = bl_start_address, end = bl_last_byte_address)

        return bl_bin_array


    def _get_application_bin_array(self, intel_hex):
        app_start_address = 0xFFFFFFFF
        app_start_address = intel_hex.minaddr()
        app_end_address = self.find_high_address(intel_hex)

        app_bin_array = intel_hex.tobinarray(start = app_start_address, end = (app_end_address - 1))

        return app_bin_array


    def _create_init_packet(self, init_packet):
        hardware_version = [0xFF, 0xFF]
        hardware_revision = [0xFF, 0xFF]
        application_version = [0xFF, 0xFF, 0xFF, 0xFF]
        softdevice_len = [0x01, 0x00]
        softdevice_array = [0xFE, 0xFF]

        init_packet.extend(hardware_version)
        init_packet.extend(hardware_revision)
        init_packet.extend(application_version)
        init_packet.extend(softdevice_len)
        init_packet.extend(softdevice_array)

    def _create_binary_array(self, image_size_packet, bin_array):
        padding = [0x00] * 4

        # Open the hex file to be sent
        intel_hex = IntelHex(self.hexfile_path)

        if self.updating_sd:
            sd_bin_array = self._get_softdevice_bin_array(intel_hex)
            if sd_bin_array:
                sd_hex_size_array_lsb = convert_uint32_to_array(len(sd_bin_array))
                image_size_packet.extend(sd_hex_size_array_lsb)
                bin_array.extend(sd_bin_array)


        expected_image_size_packet_size = 4

        if len(image_size_packet) < expected_image_size_packet_size:
            image_size_packet.extend(padding)

        if self.updating_bl:
            bl_bin_array = self._get_bootloader_bin_array(intel_hex)

            if bl_bin_array:
                bl_hex_size_array_lsb = convert_uint32_to_array(len(bl_bin_array))
                image_size_packet.extend(bl_hex_size_array_lsb)
                bin_array.extend(bl_bin_array)


        expected_image_size_packet_size = 8
        if len(image_size_packet) < expected_image_size_packet_size:
            image_size_packet.extend(padding)

        if self.updating_app:
            app_bin_array = self._get_application_bin_array(intel_hex)

            if app_bin_array:
                app_hex_size_array_lsb = convert_uint32_to_array(len(app_bin_array))
                image_size_packet.extend(app_hex_size_array_lsb)
                bin_array.extend(app_bin_array)

        expected_image_size_packet_size = 12

        if len(image_size_packet) < expected_image_size_packet_size:
            image_size_packet.extend(padding)


    def _setup_dfu_sending(self, image_size_packet, init_packet):
        try:
            self.master.OpenRemotePipe(self.pipe_dfu_control_point_notify)
        except Exception, e:
            self.ProgressChanged(0, "Device does not have the required DFU service.", True)
            return False

        if num_of_packets_between_notif:
            # Subscribing for packet receipt notifications
            self.send_data(self.pipe_dfu_control_point,
                           System.Array[System.Byte]([OpCodes.REQ_PKT_RCPT_NOTIF] + convert_uint16_to_array(num_of_packets_between_notif)),
                           num_of_send_tries,
                           "Enabling Packet receipt notifications from peer device")

        # Sending 'START DFU' command

        self.send_data(self.pipe_dfu_control_point,
                       System.Array[System.Byte]([OpCodes.START_DFU, self.program_mode]),
                       num_of_send_tries,
                       "Sending 'START DFU' command")

        # Sending image size
        self.send_data(self.pipe_dfu_packet,
                       System.Array[System.Byte](image_size_packet),
                       num_of_send_tries,
                       "Sending image size")

        self.wait_for_response(OpCodes.START_DFU)

        time.sleep(10)
        # Sending 'Init packet' command

        self.send_data(self.pipe_dfu_control_point,
                       System.Array[System.Byte]([OpCodes.INITIALIZE_DFU, 0x00]),
                       num_of_send_tries,
                       "Sending 'INIT DFU' command")

        # Sending init data
        self.send_data(self.pipe_dfu_packet,
                       System.Array[System.Byte](init_packet),
                       num_of_send_tries,
                       "Sending init data")
        
        self.send_data(self.pipe_dfu_control_point,
                       System.Array[System.Byte]([OpCodes.INITIALIZE_DFU, 0x01]),
                       num_of_send_tries,
                       "Sending 'Init Packet Complete' command")

        self.wait_for_response(OpCodes.INITIALIZE_DFU)

        # Send number of packets to receive before sending notification
        self.send_data(self.pipe_dfu_control_point,
                       System.Array[System.Byte]([0x08, 0x0A, 0x00]),
                       num_of_send_tries,
                       "Send number of packets before device sends notification")

        # Send 'RECEIVE FIRMWARE IMAGE' command to set DFU in firmware receive state.
        self.send_data(self.pipe_dfu_control_point,
                       System.Array[System.Byte]([OpCodes.RECEIVE_FIRMWARE_IMAGE]),
                       num_of_send_tries,
                       "Send 'RECEIVE FIRMWARE IMAGE' command")

        if last_error != "SUCCESS":
            self.ProgressChanged(0, "Error received from device while setting up DFU: %s" % last_error, True)
            return False

        return True


    def wait_for_response(self, op_code):
        timeout = 2
        start_time = datetime.now()

        while self.response_op_code_received != op_code:
            timedOut = datetime.now() - start_time > timedelta(0, timeout)
            if timedOut:
                self.last_error = "Timeout while waiting for response from device."
                return
            time.sleep(0.1)

        self.response_op_code_received = -1


    def dfu_send_image(self):
        """ Send hex to peer in chunks of 20 bytes. """
        if not self.connected:
            return

        image_size_packet = []
        init_packet = []
        bin_array = []

        self._create_binary_array(image_size_packet, bin_array)
        
        self.image_crc = calc_crc16(bin_array)

	self._create_init_packet(init_packet)
	init_packet.extend(convert_uint16_to_array(self.image_crc))
        
	# CCCD Enable notification bytes
        start_time = time.time()

        if not self._setup_dfu_sending(image_size_packet, init_packet):
            return False

        self.ready_to_send = True;
        pkts_sent = 0;

#        if self.updating_sd:
#        time.sleep(10)

        hex_size = len(bin_array)
        data_packet_size = 20

        # Send application data packets
        for i in range(0, hex_size, data_packet_size):
            last_progress = 0
            progress = float(i) / hex_size * 100

            if progress != last_progress:
                self.ProgressChanged(progress, "Uploading firmware...")
                last_progress = progress

            #if not self.master.IsConnected:
                #return False
            if num_of_packets_between_notif:
                while not self.ready_to_send:
                    self.log_handler.log("Waiting for packet receipt notification")
                    time.sleep(0.1)
                    #wait for 'self.ready_to_send' to be True

            data_to_send = bin_array[i:i + data_packet_size]
            # Send 20 bytes of hex image data
            self.send_data(self.pipe_dfu_packet,
                           System.Array[System.Byte](data_to_send),
                           num_of_send_tries,
                           "Sending Firmware bytes [%i, %i]" % (i, i + len(data_to_send)))

            pkts_sent = pkts_sent + 1

            if ((num_of_packets_between_notif != 0) and ((pkts_sent % num_of_packets_between_notif) == 0)):
                # Need to wait for a notification from peer
                self.log_handler.log("Need to wait for a notification from peer")
                self.ready_to_send = False

        self.wait_for_response(OpCodes.RECEIVE_FIRMWARE_IMAGE)

        # Send Validate
        self.send_data(self.pipe_dfu_control_point,
                       System.Array[System.Byte]([OpCodes.VALIDATE_FIRMWARE_IMAGE]),
                       num_of_send_tries,
                       "Sending 'VALIDATE FIRMWARE IMAGE' command")

        # Wait for notification
        self.wait_for_response(OpCodes.VALIDATE_FIRMWARE_IMAGE)

        # Send Activate and Reset
        self.send_data(self.pipe_dfu_control_point,
                       System.Array[System.Byte]([OpCodes.ACTIVATE_FIRMWARE_AND_RESET]),
                       num_of_send_tries,
                       "Sending 'ACTIVATE FIRMWARE AND RESET' command")

        end_time = time.time()
        self.log_handler.log("Total size of the Image = {0} bytes".format(len(bin_array)))
        self.log_handler.log("Time taken (excluding the service discovery) = {0} seconds".format(end_time - start_time))

        if last_error != "SUCCESS":
            self.ProgressChanged(0, "Error received from device during DFU: %s" % last_error, True)
            return False

        return True
