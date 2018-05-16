from __future__ import print_function

import time
import sys
from contextlib import contextmanager
from math import floor

import usb.core
import usb.util
from usb.core import USBError

"""
Adapted for lcr4500 from https://github.com/csi-dcsc/Pycrafter6500

DLPC350 is the controller chip on the LCR4500.

Docs: http://www.ti.com/lit/ug/dlpu010f/dlpu010f.pdf
Doc strings adapted from dlpc350_api.cpp source code.

To connect to LCR4500, install libusb-win32 driver. Recommended way to do is this is
with Zadig utility (http://zadig.akeo.ie/)
"""

__author__ = 'Alexander Tomlinson'
__email__ = 'mexander@gmail.com'
__version__ = '0.5'

LED_LUT = {'pass': 0b000, 'off': 0b000, 'black': 0b000,
           'red': 0b001, 'r': 0b001,
           'green': 0b010, 'g': 0b010,
           'yellow': 0b011, 'y': 0b011,
           'blue': 0b100, 'b': 0b100,
           'magenta': 0b101, 'm': 0b101,
           'cyan': 0b110, 'c': 0b110,
           'white': 0b111, 'w': 0b111}


def detach(dev, interface=0):
    """Detach the interface"""
    if dev.is_kernel_driver_active(interface):
        print("Detaching kernel driver for interface {interface}".format(interface=interface))
        dev.detach_kernel_driver(interface)


def attach(dev, interface=0):
    if not dev.is_kernel_driver_active(interface):
        print("Attaching kernel driver for interface {interface}".format(interface=interface))
        dev.attach_kernel_driver(interface)


def unclaim(dev, interface=0):
    print("Unclaiming interface {interface}".format(interface=interface))
    usb.util.release_interface(dev, interface)


def claim(dev, interface=0):
    """Claiming interface"""
    usb.util.claim_interface(dev, interface)


def conv_len(a, l):
    """
    Function that converts a number into a bit string of given length

    :param a: number to convert
    :param l: length of bit string
    :return: padded bit string
    """
    b = bin(a)[2:]
    padding = l - len(b)
    b = '0' * padding + b
    return b


def bits_to_bytes(a, reverse=True):
    """
    Function that converts bit string into a given number of bytes

    :param a: bites to convert
    :param reverse: whether or not to reverse the byte list
    :return: list of bytes
    """
    bytelist = []

    # check if needs padding
    if len(a) % 8 != 0:
        padding = 8 - len(a) % 8
        a = '0' * padding + a

    # convert to bytes
    for i in range(len(a) // 8):
        bytelist.append(int(a[8 * i:8 * (i + 1)], 2))

    if reverse:
        bytelist.reverse()
    return bytelist


def fps_to_period(fps):
    """
    Calculates desired period (us) from given fps
    :param fps: frames per second
    """
    period = int(floor(1.0 / fps * 10 ** 6))
    return period


@contextmanager
def connect_usb():
    """
    Context manager for connecting to and releasing usb device
    :yields: usb device
    """
    device = usb.core.find(idVendor=0x0451, idProduct=0x6401)
    device.reset()
    if sys.platform == 'linux':
        for interface in range(0, 2):
            detach(device, interface)

    device.set_configuration()

    lcr = Dlpc350(device)

    yield lcr

    device.reset()
    del lcr
    del device


class HidMessage(object):
    def __init__(self):
        self.flags = None
        self.sequence = None
        self.length_lsb = None
        self.length_msb = None
        self.data = None


class Dlpc350(object):
    """
    Class representing dmd controller.
    Can connect to different DLPCs by changing product ID. Check IDs in
    device manager.
    """

    def __init__(self, device):
        """
        connects device

        :param device: lcr4500 usb device
        """
        self.dlpc = device
        self.ans = None

        self.msg = HidMessage()

    def command(self,
                mode,
                sequence_byte,
                com1,
                com2,
                data=None):
        """
        Sends a command to the dlpc
        :param mode: whether reading or writing
        :param sequence_byte:
        :param com1: command 1
        :param com2: command 3
        :param data: data to pass with command
        """

        buffer = []

        if mode == 'r':
            flagstring = 0xc0  # 0b11000000
        else:
            flagstring = 0x40  # 0b01000000

        data_len = conv_len(len(data) + 2, 16)
        data_len = bits_to_bytes(data_len)

        buffer.append(flagstring)
        buffer.append(sequence_byte)
        buffer.extend(data_len)
        buffer.append(com2)
        buffer.append(com1)

        # if data fits into single buffer, write all and fill
        if len(buffer) + len(data) < 65:
            for i in range(len(data)):
                buffer.append(data[i])

            # append empty data to fill buffer
            for i in range(64 - len(buffer)):
                buffer.append(0x00)

            self.dlpc.write(1, buffer)

        # else, keep filling buffer and pushing until data all sent
        else:
            for i in range(64 - len(buffer)):
                buffer.append(data[i])

            self.dlpc.write(1, buffer)
            buffer = []

            j = 0
            while j < len(data) - 58:
                buffer.append(data[j + 58])
                j += 1

                if j % 64 == 0:
                    self.dlpc.write(1, buffer)
                    buffer = []

            if j % 64 != 0:
                while j % 64 != 0:
                    buffer.append(0x00)
                    j += 1

                self.dlpc.write(1, buffer)

        # wait a bit between commands
        # time.sleep(0.02)
        # time.sleep(0.02)

        # done writing, read feedback from dlpc
        try:
            self.ans = self.dlpc.read(0x81, 64)

            # Put answer in message struct
            self.msg.flags = self.ans[0]
            self.msg.sequence = self.ans[1]
            self.msg.length_lsb = self.ans[2]
            self.msg.length_msb = self.ans[3]
            self.msg.data = self.ans[4:-1]
        except USBError as e:
            print('USB Error:', e)
        time.sleep(0.02)

    def read_reply(self):
        """
        Reads in reply
        """
        data_length = self.msg.length_msb*256 + self.msg.length_lsb

        print('Flags:', bin(self.msg.flags))
        print('Sequence Byte:', hex(self.msg.sequence))
        print('Length of data:', int(data_length), 'Byte(s)')
        print('Data:')
        for i in self.msg.data:
            print(bin(i))

    def get_hardware_status(self):
        pass

    def set_power_mode(self, do_standby=False):
        """
        The Power Control places the DLPC350 in a low-power state and powers down the DMD interface. Standby mode should
        only be enabled after all data for the last frame to be displayed has been transferred to the DLPC350. Standby
        mode must be disabled prior to sending any new data.
        (USB: CMD2: 0x02, CMD3: 0x00)

        :param do_standby: True = Standby mode. Places DLPC350 in low power state and powers down the DMD interface
                           False = Normal operation. The selected external source will be displayed
        """
        do_standby = int(do_standby)
        self.command('w', 0x00, 0x02, 0x00, [do_standby])

    def start_pattern_lut_validate(self):
        """
        This API checks the programmed pattern display modes and indicates any invalid settings. This command needs to
        be executed after all pattern display configurations have been completed.

        (USB: CMD2: 0x1A, CMD3: 0x1A)
        """
        self.command('w', 0x00, 0x1a, 0x1a, bits_to_bytes(conv_len(0x00, 8)))

    def check_pat_lut_validate(self):
        self.command('r', 0x00, 0x1a, 0x1a, bits_to_bytes(conv_len(0x00, 8)))
        loop_count = 0
        while 1:
            self.command('r', 0x00, 0x1a, 0x1a, bits_to_bytes(conv_len(0x00, 8)))
            validation = self.msg.data[0]
            if not(validation & 0b10000000):
                break
            elif loop_count > 100:
                raise TimeoutError("Max Iterations reached while validating pattern LUT!")
            else:
                loop_count += 1

        if validation & 0b00000001:
            print('Selected exposure or frame period settings are invalid')
        if validation & 0b00000010:
            print('Selected pattern numbers in LUT are invalid')
        if validation & 0b00000100:
            print('Warning, continuous Trigger Out1 request or overlapping black sectors')
        if validation & 0b00001000:
            print('Warning, post vector was not inserted prior to external triggered vector')
        if validation & 0b00010000:
            print('Warning, frame period or exposure difference is less than 230usec')

    def set_display_mode(self, mode='pattern'):
        """
        Selects the input mode for the projector.
        (USB: CMD2: 0x1A, CMD3: 0x1B)

        :param mode: 0 = video mode
                     1 = pattern mode
        """
        modes = ['video', 'pattern']
        if mode in modes:
            mode = modes.index(mode)

        self.command('w', 0x00, 0x1a, 0x1b, [mode])

    def set_input_source(self, source, port_width=0):
        """
        The Input Source Selection command selects the input source to be displayed by the DLPC350: 30-bit
        Parallel Port, Internal Test Pattern, Flash memory, or FPD-link interface.
        (USB: CMD2: 0x1A, CMD3: 0x00)

        :param source: Select the input source and interface mode:
                       0 = Parallel interface with 8-bit, 16-bit, 20-bit, 24-bit, or 30-bit RGB or YCrCb data formats
                       1 = Internal test pattern; Use DLPC350_SetTPGSelect() API to select pattern
                       2 = Flash. Images are 24-bit single-frame, still images stored in flash that are uploaded on command.
                       3 = FPD-link interface
        :param port_width: Parallel Interface bit depth
                          0 = 30-bits
                          1 = 24-bits
                          2 = 20-bits
                          3 = 16-bits
                          4 = 10-bits
                          5 = 8-bits

        """
        sources = ['parallel', 'test', 'flash', 'fdp']
        if source in sources:
            source = sources.index(source)

        port_widths = [30, 24, 20, 16, 10, 8]
        if port_width in port_widths:
            port_width = port_widths.index(port_width)

        source = conv_len(source, 3)
        port_width = conv_len(port_width, 3)
        payload = '00' + port_width + source
        payload = bits_to_bytes(payload)

        self.command('w', 0x00, 0x1a, 0x00, payload)

    def set_pattern_input_source(self, mode='video'):
        """
        Selects the input type for pattern sequence.
        (USB: CMD2: 0x1A, CMD3: 0x22)

        :param mode: 0 = video
                     3 = flash
        """
        modes = ['video', '', '', 'flash']
        if mode in modes:
            mode = modes.index(mode)

        self.command('w', 0x00, 0x1a, 0x22, [mode])

    def set_pattern_trigger_mode(self, mode='IntExt'):
        """
        Selects the trigger type for pattern sequence.
        (USB: CMD2: 0x1A, CMD3: 0x23)

        :param mode: 0 = vsync
                     1 = Internally or Externally (through TRIG_IN1 and TRIG_IN2) generated trigger.
        """
        modes = ['vsync', 'IntExt']
        if mode in modes:
            mode = modes.index(mode)

        self.command('w', 0x00, 0x1a, 0x23, [mode])

    def pattern_display(self, action='start'):
        """
        This API starts or stops the programmed patterns sequence.
        (USB: CMD2: 0x1A, CMD3: 0x24)

        :param action: Pattern Display Start/Stop Pattern Sequence
                       0 = Stop Pattern Display Sequence. The next "Start" command will restart the pattern sequence
                           from the beginning.
                       1 = Pause Pattern Display Sequence. The next "Start" command will start the pattern sequence by
                           re-displaying the current pattern in the sequence.
                       2 = Start Pattern Display Sequence
        """
        actions = ['stop', 'pause', 'start']
        if action in actions:
            action = actions.index(action)

        self.command('w', 0x00, 0x1a, 0x24, [action])

    def set_exposure_frame_period(self,
                                  exposure_period,
                                  frame_period):
        """
        The Pattern Display Exposure and Frame Period dictates the time a pattern is exposed and the frame period.
        Either the exposure time must be equivalent to the frame period, or the exposure time must be less than the
        frame period by 230 microseconds. Before executing this command, stop the current pattern sequence. After
        executing this command, call DLPC350_ValidatePatLutData() API before starting the pattern sequence.
        (USB: CMD2: 0x1A, CMD3: 0x29)

        :param exposure_period: exposure time in microseconds (4 bytes)
        :param frame_period: frame period in microseconds (4 bytes)
        """
        exposure_period = conv_len(exposure_period, 32)
        frame_period = conv_len(frame_period, 32)

        payload = frame_period + exposure_period
        payload = bits_to_bytes(payload)

        self.command('w', 0x00, 0x1a, 0x29, payload)

    def set_flash_image_indexes(self, indexes):
        """
        Opens the Mailbox to define the flash image indexes. Then inserts the indexes of the images that should be
        displayed from flash memory and closes the mailbox again.
        For example, if image indexes 0 through 3 are desired, write 0x0 0x1 0x2 0x3 to the mailbox. Similary, if the
        desired image index sequence is 0, 1, 2, 1 then write 0x0 0x1 0x2 0x1 to the mailbox.
        Before executing this command, stop the current pattern sequence. After
        executing this command, call DLPC350_ValidatePatLutData() API before starting the pattern sequence.
        (USB: CMD2: 0x1A, CMD3: 0x33)
        (USB: CMD2: 0x1A, CMD3: 0x34)
        (USB: CMD2: 0x1A, CMD3: 0x33)

        Parameters
        ----------
        indexes : list
            Indexes of the image in flash memory that should be displayed
        """
        # Open Mailbox to define flash image indexes
        self.open_mailbox(1)
        # Set image indexes, that should be displayed
        self.send_img_lut(indexes)
        # Close Mailbox
        self.close_mailbox()

    def set_pattern_config(self,
                           num_lut_entries=1,
                           do_repeat=True,
                           num_pats_for_trig_out2=1,
                           num_images=3):
        """
        This API controls the execution of patterns stored in the lookup table. Before using this API, stop the current
        pattern sequence using DLPC350_PatternDisplay() API. After calling this API, send the Validation command using
        the API DLPC350_ValidatePatLutData() before starting the pattern sequence.
        (USB: CMD2: 0x1A, CMD3: 0x31)

        :param num_lut_entries: number of LUT entries
        :param do_repeat: True = execute the pattern sequence once; False = repeat the pattern sequence
        :param num_pats_for_trig_out2: Number of patterns to display(range 1 through 256). If in repeat mode, then this
            value dictates how often TRIG_OUT_2 is generated
        :param num_images: Number of Image Index LUT Entries(range 1 through 64). This Field is irrelevant for Pattern
            Display Data Input Source set to a value other than internal
        """
        num_lut_entries = '0' + conv_len(num_lut_entries - 1, 7)
        do_repeat = '0000000' + str(int(do_repeat))
        num_pats_for_trig_out2 = conv_len(num_pats_for_trig_out2 - 1, 8)
        num_images = '00' + conv_len(num_images, 6)

        payload = num_images + num_pats_for_trig_out2 + do_repeat + num_lut_entries
        payload = bits_to_bytes(payload)

        self.command('w', 0x00, 0x1a, 0x31, payload)

    def mailbox_set_address(self, address=0):
        """
        This API defines the offset location within the DLPC350 mailboxes to write data into or to read data from.
        (USB: CMD2: 0x1A, CMD3: 0x32)

        :param address: Defines the offset within the selected (opened) LUT to write/read data to/from (0-127)
        """
        address = bits_to_bytes(conv_len(address, 8))
        self.command('w', 0x00, 0x1a, 0x32, address)

    def open_mailbox(self, mbox_num):
        """
        This API opens the specified Mailbox within the DLPC350 controller. This API must be called before sending data
        to the mailbox/LUT using DLPC350_SendPatLut() or DLPC350_SendImageLut() APIs.
        (USB: CMD2: 0x1A, CMD3: 0x33)

        :param mbox_num: 0 = Disable (close) the mailboxes
                         1 = Open the mailbox for image index configuration
                         2 = Open the mailbox for pattern definition
                         3 = Open the mailbox for the Variable Exposure
        """
        mbox_num = bits_to_bytes(conv_len(mbox_num, 8))
        self.command('w', 0x00, 0x1a, 0x33, mbox_num)

    def close_mailbox(self):
        """
        This API closes the Mailbox within the DLPC350 controller. This API must be called after sending data
        to the mailbox/LUT using DLPC350_SendPatLut() or DLPC350_SendImageLut() APIs.
        (USB: CMD2: 0x1A, CMD3: 0x33)

        """
        self.command('w', 0x00, 0x1a, 0x33, bits_to_bytes(conv_len(0, 8)))

    def send_img_lut(self, img_lut):
        """
        Mailbox content to setup image LUT. See table 2-67 in programmer's guide for detailed description.
        (USB: CMD2: 0x1A, CMD3: 0x34)

        Mailbox has to be openend to define the flash image indexes.

        :param img_lut: List the image index numbers in the mailbox.
                        For example: image indexes 0 through 3 are desired - write: 0x0 0x1 0x2 0x3
                        Similarly, if the desired image index sequence is 0, 1, 2, 1 - write: 0x0 0x1 0x2 0x1

        """
        for elem_no, img_index in enumerate(img_lut):
            self.mailbox_set_address(elem_no)
            self.command('w', 0x00, 0x1a, 0x34, bits_to_bytes(conv_len(img_index, 8)))

    def send_pattern_lut(self,
                         trig_type,
                         pat_num,
                         bit_depth,
                         led_select,
                         do_invert_pat=False,
                         do_insert_black=True,
                         do_buf_swap=False,
                         do_trig_out_prev=False):
        """
        Mailbox content to setup pattern definition. See table 2-65 in programmer's guide for detailed description of
        pattern LUT entries.
        (USB: CMD2: 0x1A, CMD3: 0x34)

        :param trig_type: Select the trigger type for the pattern
                          0 = Internal trigger
                          1 = External positive
                          2 = External negative
                          3 = No Input Trigger (Continue from previous; Pattern still has full exposure time)
                          0x3FF = Full Red Foreground color intensity
        :param pat_num: Pattern number (0 based index). For pattern number 0x3F, there is no pattern display. The
            maximum number supported is 24 for 1 bit-depth patterns. Setting the pattern number to be 25, with a
            bit-depth of 1 will insert a white-fill pattern. Inverting this pattern will insert a black-fill pattern.
            These patterns will have the same exposure time as defined in the Pattern Display Exposure and Frame Period
            command. Table 2-66 in the programmer's guide illustrates which bit planes are illuminated by each pattern
            number.
        :param bit_depth: Select desired bit-depth
                          0 = Reserved
                          1 = 1-bit
                          2 = 2-bit
                          3 = 3-bit
                          4 = 4-bit
                          5 = 5-bit
                          6 = 6-bit
                          7 = 7-bit
                          8 = 8-bit
        :param led_select: Choose the LEDs that are on: b0 = Red, b1 = Green, b2 = Blue
                           0 = No LED (Pass Through)
                           1 = Red
                           2 = Green
                           3 = Yellow (Green + Red)
                           4 = Blue
                           5 = Magenta (Blue + Red)
                           6 = Cyan (Blue + Green)
                           7 = White (Red + Blue + Green)
        :param do_invert_pat: True = Invert pattern
                              False = do not invert pattern
        :param do_insert_black: True = Insert black-fill pattern after current pattern. This setting requires 230 us
                                       of time before the start of the next pattern
                                False = do not insert any post pattern
        :param do_buf_swap: True = perform a buffer swap
                            False = do not perform a buffer swap
        :param do_trig_out_prev: True = Trigger Out 1 will continue to be high. There will be no falling edge
                                        between the end of the previous pattern and the start of the current pattern.
                                        Exposure time is shared between all patterns defined under a common
                                        trigger out). This setting cannot be combined with the black-fill pattern
                                 False = Trigger Out 1 has a rising edge at the start of a pattern, and a falling edge
                                         at the end of the pattern

        """
        # byte 0
        trig_type = conv_len(trig_type, 2)
        pat_num = conv_len(pat_num, 6)
        byte_0 = pat_num + trig_type

        # byte 1
        bit_depth = conv_len(bit_depth, 4)
        led_select = conv_len(led_select, 4)

        byte_1 = led_select + bit_depth

        # byte 2
        do_invert_pat = str(int(do_invert_pat))
        do_insert_black = str(int(do_insert_black))
        do_buf_swap = str(int(do_buf_swap))
        do_trig_out_prev = str(int(do_trig_out_prev))

        byte_2 = '0000' + do_trig_out_prev + do_buf_swap + do_insert_black + do_invert_pat

        payload = byte_2 + byte_1 + byte_0
        payload = bits_to_bytes(payload)

        self.command('w', 0x00, 0x1a, 0x34, payload)

    def set_led_pwm_polarity(self, invert=0):
        """
        WARNING: SEEMS TO BE THE INVERSE
        The LED PWM Polarity command sets the polarity of all PWM signals. This command must be issued
        before powering up the LED drivers.
        (USB: CMD2: 0x1A, CMD3: 0x05)

        :param invert: Polarity of PWM signals
                       0 - Normal polarity. PWM 0 value corresponds to no current while
                                            PWM 255 value corresponds to maximum current.
                       1 - Inverted polarity. PWM 0 value corresponds to maximum current while
                                              PWM 255 value corresponds to no current.

        """
        invert_modes = [False, True]
        if invert in invert_modes:
            invert = invert_modes.index(invert)

        payload = conv_len(invert, 8)
        payload = bits_to_bytes(payload)
        self.command("w", 0x00, 0x1a, 0x05, payload)

    def set_led_current(self, red_c, green_c, blue_c):
        """
        This parameter controls the pulse duration of the specific LED PWM modulation output pin. The resolution
        is 8 bits and corresponds to a percentage of the LED current. The PWM value can be set from 0 to 100%
        in 256 steps. If the LED PWM polarity is set to normal polarity, a setting of 0xFF gives the maximum PWM
        current. The LED current is a function of the specific LED driver design.
        (USB: CMD2: 0x0B, CMD3: 0x01)

        :param red_c:

        :param green_c:

        :param blue_c:

        """

        red = conv_len(red_c, 8)
        green = conv_len(green_c, 8)
        blue = conv_len(blue_c, 8)
        payload = red + green + blue
        payload = bits_to_bytes(payload)
        print("Setting rgb intensity to %i, %i, %i" % (red_c, green_c, blue_c))
        self.command("w", 0x00, 0x0b, 0x01, payload)


def pattern_mode(input_mode='pattern',
                 input_type='flash',
                 num_pats=3,
                 trigger_type='IntExt',
                 period=fps_to_period(10),
                 bit_depth=8,
                 led_color=0b111,  # BGR
                 **kwargs
                 ):

    if 'fps' in kwargs:
        period = fps_to_period(kwargs['fps'])

    with connect_usb() as lcr:
        assert bit_depth in [1, 2, 4, 7, 8]

        # before proceeding to change params, need to stop pattern sequence mode
        lcr.pattern_display('stop')

        # 1: pattern display mode
        lcr.set_display_mode(input_mode)

        # 2: pattern display from external video
        lcr.set_pattern_input_source(input_type)

        # 3: setup number of luts
        lcr.set_pattern_config(num_lut_entries=num_pats,
                               num_pats_for_trig_out2=num_pats)

        # 4: Pattern trigger mode selection
        lcr.set_pattern_trigger_mode(trigger_type)

        # 5: Set exposure and frame rate
        lcr.set_exposure_frame_period(period, period)

        # 6: Skip setting up image indexes
        lcr.set_flash_image_indexes([0, 1, 2])


        # 7: Set up LUT
        lcr.open_mailbox(2)
        bit_map = {1: [7, 15, 23],
                   2: [3, 7, 11],
                   4: [1, 3, 5],
                   7: [0, 1, 2],
                   8: [0, 1, 2]}

        for i in range(3):
            #internal trigger type
            trig_type = 0

            lcr.mailbox_set_address(i)
            print(bit_map[bit_depth][i])
            lcr.send_pattern_lut(trig_type=trig_type,
                                 pat_num=bit_map[bit_depth][i],
                                 bit_depth=bit_depth,
                                 led_select=led_color)



        lcr.close_mailbox()

        # 8/9: validate
        lcr.start_pattern_lut_validate()

        lcr.command('r', 0x00, 0x1a, 0x1a, [])

        # 10: start sequence
        lcr.pattern_display('start')
        # idk why you need a second start coming out of video mode
        lcr.pattern_display('start')
        lcr.pattern_display('stop')


def set_flash_sequence(sequence=((4, 1), (4, 2), (4, 0), (5, 0), (5,1)),
                       repeat=False,
                       trigger_type='IntExt',
                       exposure=1000000,
                       frame_period=1000000,
                       bit_depth=8,
                       color='white',
                       ):
    with connect_usb() as lcr:
        assert bit_depth in [1, 2, 4, 7, 8]

        # before proceeding to change params, need to stop pattern sequence mode
        lcr.pattern_display('stop')

        # 1: pattern display mode
        lcr.set_display_mode('pattern')

        # 2: pattern display from external video
        lcr.set_input_source('flash')
        lcr.set_pattern_input_source('flash')

        # 3: setup number of luts
        img_lut = list()
        buffer_swap_list = list()
        last_img_num = -1
        for img_num, _ in sequence:
            if img_num != last_img_num:
                img_lut.append(img_num)
                buffer_swap_list.append(True)
            else:
                buffer_swap_list.append(False)
            last_img_num = img_num

        lcr.set_pattern_config(num_lut_entries=len(sequence),
                               num_pats_for_trig_out2=len(sequence),
                               do_repeat=repeat,
                               num_images=len(img_lut))

        # 4: Pattern trigger mode selection
        lcr.set_pattern_trigger_mode(trigger_type)

        # 5: Set exposure and frame rate
        lcr.set_exposure_frame_period(exposure, frame_period)

        # 6: Skip setting up image indexes
        lcr.set_flash_image_indexes(img_lut)

        # 7: Set up LUT
        lcr.open_mailbox(2)

        led_color = LED_LUT[color]

        for element, (_, plane_id) in enumerate(sequence):
            # Always use internal trigger
            trig_type = 0
            lcr.mailbox_set_address(element)
            lcr.send_pattern_lut(trig_type=trig_type,
                                 pat_num=plane_id,
                                 bit_depth=bit_depth,
                                 led_select=led_color,
                                 do_buf_swap=buffer_swap_list[element])

        lcr.close_mailbox()

        # 8/9: validate
        lcr.start_pattern_lut_validate()
        lcr.check_pat_lut_validate()

        # 10: start sequence
        lcr.pattern_display('start')
        # idk why you need a second start coming out of video mode
        lcr.pattern_display('start')



def video_mode():
    """
    Puts LCR4500 into video mode.
    """
    with connect_usb() as lcr:
        lcr.pattern_display('stop')
        lcr.set_display_mode('video')


def power_down():
    """
    Puts LCR4500 into standby mode.
    """
    with connect_usb() as lcr:
        lcr.pattern_display('stop')
        lcr.set_power_mode(do_standby=True)


def power_up():
    """
    Wakes LCR4500 up from standby mode.
    """
    with connect_usb() as lcr:

        lcr.set_power_mode(do_standby=False)


if __name__ == '__main__':
    # power_up()
    # with connect_usb() as lcr:
    #     lcr.command('r', 0x00, 0x02, 0x05, [])
    #     lcr.read_reply()
    #     lcr.start_pattern_lut_validate()
    #     lcr.set_display_mode('video')
        # lcr.set_led_pwm_polarity()
        # Set display to pattern mode
        #  lcr.command('w', 0x00, 0x1a, 0x1b, [1])
        #Set pattern display from flash memory

    #
    set_flash_sequence()
    # pattern_mode()
    # power_down()
        # lcr.set_led_current(100,100,100)


