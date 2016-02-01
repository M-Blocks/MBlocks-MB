import time
import argparse
import sys
import os
import clr
sys.path.append(r'.\..')
import dfu

try:
    programfilesPath = os.environ['PROGRAMFILES']
    masterApiBasePath = os.path.join(programfilesPath, r'Nordic Semiconductor\Master Emulator')
    dirsandfiles = os.listdir(masterApiBasePath)
    dirs = []
    for element in dirsandfiles:
        if os.path.isdir(os.path.join(masterApiBasePath, element)):
            dirs.append(element)
    if len(dirs) == 0:
        raise Exception('Master Emulator directory not found.')
    dirs.sort()
    masterApiPath = os.path.join(masterApiBasePath, dirs[-1])
    print masterApiPath
    sys.path.append(masterApiPath)
    clr.AddReferenceToFile("MasterEmulator.dll")
except Exception, e:
    raise Exception("Cannot load MasterEmulator.dll")

from dfu.ble_dfu import BleDfu


def main():
    parser = argparse.ArgumentParser(description='Send hex file over-the-air via BLE')
    parser.add_argument('--file', '-f',
                        type=str,
                        required=True,
                        dest='file',
                        help='Filename of Hex file.')
    parser.add_argument('--address', '-a',
                        type=str,
                        required=True,
                        dest='address',
                        help='Advertising address of nrf51822 device.')
    parser.add_argument('--mode', '-m',
                        type=str,
                        required=True,
                        dest='mode',
                        help='Programming mode, valid modes are: Application, Softdevice, Bootloader, SdAndBl.')
    parser.add_argument('--baud-rate', '-b',
                        type=int,
                        required=False,
                        dest='baud_rate',
                        default=1000000,
                        help='Baud rate for communication with the master emulator device. Default 1000000.')
    parser.add_argument('--dongle', '-d',
                        type=str,
                        required=False,
                        dest='dongle',
                        default='',
                        help='Specify which Nordic USB dongle to use for the BLE connection, for example COM26.')

    args = parser.parse_args()
    print 'Sending file {0} to device {1}'.format(args.file, args.address.upper())
    print 'Programming the file as a {0}. Using baud rate {1}'.format(args.mode.upper(), args.baud_rate)

    dfu_mode = getattr(dfu.ble_dfu.DfuMode, args.mode.upper())
    ble_dfu = BleDfu(args.address.upper(), args.file, args.baud_rate, dfu_mode)

    # Connect to peer device.
    ble_dfu.scan_and_connect(args.dongle)

    # Transmit the hex image to peer device.
    ble_dfu.dfu_send_image()

    # wait a second to be able to recieve the disconnect event from peer device.
    time.sleep(1)

    # Disconnect from peer device if not done already and clean up.
    ble_dfu.disconnect()

if __name__ == '__main__':
    main()
