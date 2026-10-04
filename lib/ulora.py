import time
import math
import micropython
from ucollections import namedtuple
from urandom import getrandbits
from machine import SPI
from machine import Pin

#Constants
FLAGS_ACK = 0x80
BROADCAST_ADDRESS = 255

# Hop count rides in the existing RadioHead flags byte (no added payload byte,
# so a plain reading like "68.0" still follows the 4-byte header). Low 3 bits
# are remaining hops. FLAGS_HOP_SET (bit 3) means a repeater wrote that count.
# Bit 3 clear (current sensors send flags 0) means the full hop budget, not
# "no hops left". FLAGS_ACK stays bit 7 (0x80).
FLAGS_HOP_SET = 0x08
FLAGS_HOP_MASK = 0x07

REG_00_FIFO = 0x00
REG_01_OP_MODE = 0x01
REG_06_FRF_MSB = 0x06
REG_07_FRF_MID = 0x07
REG_08_FRF_LSB = 0x08
REG_0E_FIFO_TX_BASE_ADDR = 0x0e
REG_0F_FIFO_RX_BASE_ADDR = 0x0f
REG_10_FIFO_RX_CURRENT_ADDR = 0x10
REG_12_IRQ_FLAGS = 0x12
REG_13_RX_NB_BYTES = 0x13
REG_1D_MODEM_CONFIG1 = 0x1d
REG_1E_MODEM_CONFIG2 = 0x1e
REG_19_PKT_SNR_VALUE = 0x19
REG_1A_PKT_RSSI_VALUE = 0x1a
REG_20_PREAMBLE_MSB = 0x20
REG_21_PREAMBLE_LSB = 0x21
REG_22_PAYLOAD_LENGTH = 0x22
REG_26_MODEM_CONFIG3 = 0x26

REG_4D_PA_DAC = 0x4d
REG_40_DIO_MAPPING1 = 0x40
REG_0D_FIFO_ADDR_PTR = 0x0d

PA_DAC_ENABLE = 0x07
PA_DAC_DISABLE = 0x04
PA_SELECT = 0x80

CAD_DETECTED_MASK = 0x01
RX_DONE = 0x40
TX_DONE = 0x08
CAD_DONE = 0x04
CAD_DETECTED = 0x01

LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_TX = 0x03
MODE_RXCONTINUOUS = 0x05
MODE_CAD = 0x07

REG_09_PA_CONFIG = 0x09
FXOSC = 32000000.0
FSTEP = (FXOSC / 524288)

def _scheduled_repeat(item):
    item[0]._enqueue_repeat(item[1])

class ModemConfig():
    Bw125Cr45Sf128 = (0x72, 0x74, 0x04) #< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
    Bw500Cr45Sf128 = (0x92, 0x74, 0x04) #< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
    Bw31_25Cr48Sf512 = (0x48, 0x94, 0x04) #< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
    Bw125Cr48Sf4096 = (0x78, 0xc4, 0x0c) #/< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, low data rate, CRC on. Slow+long range
    Bw125Cr45Sf2048 = (0x72, 0xb4, 0x04) #< Bw = 125 kHz, Cr = 4/5, Sf = 2048chips/symbol, CRC on. Slow+long range

class SPIConfig():
    # spi pin defs for various boards (channel, sck, mosi, miso)
    rp2_0 = (0, 18, 19, 16)
    rp2_1 = (1, 10, 11, 8)
    esp8286_1 = (1, 14, 13, 12)
    esp32_1 = (1, 14, 13, 12)
    esp32_2 = (2, 18, 23, 19)
    esp32s2 = (17, 36, 11, 13)

class LoRa(object):
    def __init__(self, spi_channel, interrupt, this_address, cs_pin, reset_pin=None, freq=868.0, tx_power=14,
                 modem_config=ModemConfig.Bw125Cr45Sf128, receive_all=False, acks=False, crypto=None,
                 repeater=False, repeat_hops=2, repeat_ack_from=None, repeat_seen_max=16, repeat_seen_ms=30000):
        """
        Lora(channel, interrupt, this_address, cs_pin, reset_pin=None, freq=868.0, tx_power=14,
                 modem_config=ModemConfig.Bw125Cr45Sf128, receive_all=False, acks=False, crypto=None)
        channel: SPI channel, check SPIConfig for preconfigured names
        interrupt: GPIO interrupt pin
        this_address: set address for this device [0-254]
        cs_pin: chip select pin from microcontroller 
        reset_pin: the GPIO used to reset the RFM9x if connected
        freq: frequency in MHz
        tx_power: transmit power in dBm
        modem_config: Check ModemConfig. Default is compatible with the Radiohead library
        receive_all: if True, don't filter packets on address
        acks: if True, request acknowledgments
        crypto: if desired, an instance of ucrypto AES (https://docs.pycom.io/firmwareapi/micropython/ucrypto/) - not tested
        repeater: if True, rebroadcast foreign packets once from service_repeater()
        """
        
        self._spi_channel = spi_channel
        self._interrupt = interrupt
        self._cs_pin = cs_pin

        self._mode = None
        self._cad = None
        self._freq = freq
        self._tx_power = tx_power
        self._modem_config = modem_config
        self._receive_all = receive_all
        self._acks = acks
        self._repeater = repeater
        if repeat_hops < 1:
            repeat_hops = 1
        if repeat_hops > FLAGS_HOP_MASK:
            repeat_hops = FLAGS_HOP_MASK
        self._repeat_hops = repeat_hops
        self._repeat_ack_from = repeat_ack_from
        self._repeat_seen_max = repeat_seen_max if repeat_seen_max > 0 else 1
        self._repeat_seen_ms = repeat_seen_ms if repeat_seen_ms > 0 else 1
        # Short memory of (header_from, header_id, ticks_ms). The payload is not kept.
        self._repeat_seen = []
        self._repeat_q = []
        self._repeat_q_max = 4

        self._this_address = this_address
        self._last_header_id = 0

        self._last_payload = None
        self.crypto = crypto

        self.cad_timeout = 0
        self.send_retries = 2
        self.wait_packet_sent_timeout = 1.0
        self.retry_timeout = 1.0
        
        # Setup the module
#        gpio_interrupt = Pin(self._interrupt, Pin.IN, Pin.PULL_DOWN)
        gpio_interrupt = Pin(self._interrupt, Pin.IN)
        gpio_interrupt.irq(trigger=Pin.IRQ_RISING, handler=self._handle_interrupt)
        
        # reset the board
        if reset_pin:
            gpio_reset = Pin(reset_pin, Pin.OUT)
            gpio_reset.value(0)
            time.sleep(0.01)
            gpio_reset.value(1)
            time.sleep(0.01)

        # baud rate to 5MHz
        self.spi = SPI(self._spi_channel[0], 5000000,
                       sck=Pin(self._spi_channel[1]), mosi=Pin(self._spi_channel[2]), miso=Pin(self._spi_channel[3]))

        # cs gpio pin
        self.cs = Pin(self._cs_pin, Pin.OUT)
        self.cs.value(1)
        
        # set mode
        self._spi_write(REG_01_OP_MODE, MODE_SLEEP | LONG_RANGE_MODE)
        time.sleep(0.1)
        
        # check if mode is set
        assert self._spi_read(REG_01_OP_MODE) == (MODE_SLEEP | LONG_RANGE_MODE), \
            "LoRa initialization failed"

        self._spi_write(REG_0E_FIFO_TX_BASE_ADDR, 0)
        self._spi_write(REG_0F_FIFO_RX_BASE_ADDR, 0)
        
        self.set_mode_idle()

        # set modem config (Bw125Cr45Sf128)
        self._spi_write(REG_1D_MODEM_CONFIG1, self._modem_config[0])
        self._spi_write(REG_1E_MODEM_CONFIG2, self._modem_config[1])
        self._spi_write(REG_26_MODEM_CONFIG3, self._modem_config[2])

        # set preamble length (8)
        self._spi_write(REG_20_PREAMBLE_MSB, 0)
        self._spi_write(REG_21_PREAMBLE_LSB, 8)

        # set frequency
        frf = int((self._freq * 1000000.0) / FSTEP)
        self._spi_write(REG_06_FRF_MSB, (frf >> 16) & 0xff)
        self._spi_write(REG_07_FRF_MID, (frf >> 8) & 0xff)
        self._spi_write(REG_08_FRF_LSB, frf & 0xff)
        
        # Set tx power
        if self._tx_power < 5:
            self._tx_power = 5
        if self._tx_power > 23:
            self._tx_power = 23

        # RadioHead/Adafruit polarity: PA_DAC_ENABLE only for >= 20 dBm
        if self._tx_power >= 20:
            self._spi_write(REG_4D_PA_DAC, PA_DAC_ENABLE)
            self._tx_power -= 3
        else:
            self._spi_write(REG_4D_PA_DAC, PA_DAC_DISABLE)

        self._spi_write(REG_09_PA_CONFIG, PA_SELECT | (self._tx_power - 5))
        
    def on_recv(self, message):
        # This should be overridden by the user
        pass

    def sleep(self):
        if self._mode != MODE_SLEEP:
            self._spi_write(REG_01_OP_MODE, MODE_SLEEP)
            self._mode = MODE_SLEEP

    def set_mode_tx(self):
        if self._mode != MODE_TX:
            self._spi_write(REG_40_DIO_MAPPING1, 0x40)  # Interrupt on TxDone
            self._spi_write(REG_12_IRQ_FLAGS, 0xff)
            self._spi_write(REG_01_OP_MODE, MODE_TX)
            self._mode = MODE_TX

    def set_mode_rx(self):
        # Always program the chip — _mode can be wrong if TxDone was polled via SPI
        self._spi_write(REG_40_DIO_MAPPING1, 0x00)  # Interrupt on RxDone
        self._spi_write(REG_12_IRQ_FLAGS, 0xff)
        self._spi_write(REG_01_OP_MODE, MODE_RXCONTINUOUS)
        self._mode = MODE_RXCONTINUOUS
            
    def set_mode_cad(self):
        if self._mode != MODE_CAD:
            self._spi_write(REG_01_OP_MODE, MODE_CAD)
            self._spi_write(REG_40_DIO_MAPPING1, 0x80)  # Interrupt on CadDone
            self._mode = MODE_CAD

    def _is_channel_active(self):
        self.set_mode_cad()

        while self._mode == MODE_CAD:
            yield

        return self._cad
    
    def wait_cad(self):
        if not self.cad_timeout:
            return True

        start = time.time()
        for status in self._is_channel_active():
            if time.time() - start < self.cad_timeout:
                return False

            if status is None:
                time.sleep(0.1)
                continue
            else:
                return status

    def wait_packet_sent(self):
        # Prefer IRQ-updated _mode; also poll the chip — DIO0/TxDone IRQ is
        # often missing on sensor boards, and a full timeout delays set_mode_rx
        # long enough that the bridge ACK is already gone.
        timeout_ms = int(self.wait_packet_sent_timeout * 1000)
        if timeout_ms < 1:
            timeout_ms = 1
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if self._mode != MODE_TX:
                return True
            try:
                irq_flags = self._spi_read(REG_12_IRQ_FLAGS)
                if irq_flags & TX_DONE:
                    self.set_mode_idle()
                    self._spi_write(REG_12_IRQ_FLAGS, 0xff)
                    return True
                # SX127x returns to standby after TX even if DIO edge was missed
                op = self._spi_read(REG_01_OP_MODE) & 0x07
                if op != MODE_TX:
                    self._mode = MODE_STDBY
                    self._spi_write(REG_12_IRQ_FLAGS, 0xff)
                    return True
            except Exception:
                pass
        return False

    def set_mode_idle(self):
        if self._mode != MODE_STDBY:
            self._spi_write(REG_01_OP_MODE, MODE_STDBY)
            self._mode = MODE_STDBY

    def send(self, data, header_to, header_id=0, header_flags=0, header_from=None):
        self.wait_packet_sent()
        self.set_mode_idle()
        self.wait_cad()

        # A repeat passes the original header_from; normal sends use this node.
        if header_from is None:
            header_from = self._this_address
        header = [header_to, header_from & 0xff, header_id & 0xff, header_flags & 0xff]
        if type(data) == int:
            data = [data]
        elif type(data) == bytes:
            data = [p for p in data]
        elif type(data) == str:
            data = [ord(s) for s in data]

        if self.crypto:
            data = [b for b in self._encrypt(bytes(data))]

        payload = header + data
        self._spi_write(REG_0D_FIFO_ADDR_PTR, 0)
        self._spi_write(REG_00_FIFO, payload)
        self._spi_write(REG_22_PAYLOAD_LENGTH, len(payload))

        self.set_mode_tx()
        return True

    def send_to_wait(self, data, header_to, header_flags=0, retries=0):
        self._last_header_id = (self._last_header_id + 1) & 0xff

        for attempt in range(retries + 1):
            # Drop stale RX so we don't match an old packet; must finish TX
            # before RX or the radio can sit in standby while we think we're listening.
            self._last_payload = None
            self.send(data, header_to, header_id=self._last_header_id, header_flags=header_flags)
            tx_ok = self.wait_packet_sent()
            self.set_mode_rx()
            if not tx_ok:
                print("TX done timeout id=%d try=%d" % (self._last_header_id, attempt))

            if header_to == BROADCAST_ADDRESS:  # Don't wait for acks from a broadcast message
                return True

            # ticks_ms ACK wait; poll chip RX_DONE — DIO0 IRQ is unreliable on these boards
            timeout_ms = int((self.retry_timeout + (self.retry_timeout * (getrandbits(16) / (2**16 - 1)))) * 1000)
            if timeout_ms < 1:
                timeout_ms = 1
            start = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
                try:
                    irq_flags = self._spi_read(REG_12_IRQ_FLAGS)
                    if irq_flags & RX_DONE:
                        self._handle_interrupt(None)
                except Exception:
                    pass
                if self._last_payload:
                    if self._last_payload.header_to == self._this_address and \
                            self._last_payload.header_flags & FLAGS_ACK and \
                            self._last_payload.header_id == self._last_header_id:

                        print("ACK RX id=%d from=%d" % (
                            self._last_payload.header_id, self._last_payload.header_from))
                        return True
            # Diagnostics when nothing matched
            try:
                irq = self._spi_read(REG_12_IRQ_FLAGS)
                op = self._spi_read(REG_01_OP_MODE)
                dio = self._spi_read(REG_40_DIO_MAPPING1)
            except Exception:
                irq, op, dio = -1, -1, -1
            print("ACK timeout id=%d try=%d flags=%s irq=0x%02x op=0x%02x dio=0x%02x" % (
                self._last_header_id, attempt,
                None if not self._last_payload else self._last_payload.header_flags,
                irq, op, dio))
        return False

    def send_ack(self, header_to, header_id):
        self.send(b'!', header_to, header_id, FLAGS_ACK)
        self.wait_packet_sent()

    def _hops_remaining(self, header_flags):
        if header_flags & FLAGS_HOP_SET:
            return header_flags & FLAGS_HOP_MASK
        return self._repeat_hops

    def _flags_after_hop(self, header_flags, remaining):
        hopped = remaining - 1
        if hopped < 0:
            hopped = 0
        cleared = header_flags & ~(FLAGS_HOP_SET | FLAGS_HOP_MASK)
        return cleared | FLAGS_HOP_SET | (hopped & FLAGS_HOP_MASK)

    def _repeat_is_seen(self, header_from, header_id):
        now = time.ticks_ms()
        fresh = []
        found = False
        for src, pid, seen_at in self._repeat_seen:
            if time.ticks_diff(now, seen_at) > self._repeat_seen_ms:
                continue
            fresh.append((src, pid, seen_at))
            if src == header_from and pid == header_id:
                found = True
        if len(fresh) > self._repeat_seen_max:
            fresh = fresh[-self._repeat_seen_max:]
        self._repeat_seen = fresh
        return found

    def _repeat_remember(self, header_from, header_id):
        self._repeat_seen.append((header_from, header_id, time.ticks_ms()))
        if len(self._repeat_seen) > self._repeat_seen_max:
            self._repeat_seen.pop(0)

    def _enqueue_repeat(self, item):
        if len(self._repeat_q) >= self._repeat_q_max:
            self._repeat_q.pop(0)
        self._repeat_q.append(item)

    def _consider_repeat(self, header_to, header_from, header_id, header_flags, message):
        if not self._repeater:
            return
        if header_from == self._this_address:
            return
        if header_flags & FLAGS_ACK:
            # Not a general ACK rebroadcast. Only the bridge ACK is forwarded
            # once, through this same seen-set, so a farther node can hear it.
            if self._repeat_ack_from is None or header_from != self._repeat_ack_from:
                return
        remaining = self._hops_remaining(header_flags)
        if remaining <= 0:
            return
        if self._repeat_is_seen(header_from, header_id):
            return
        self._repeat_remember(header_from, header_id)
        item = (message, header_to, header_from, header_id, self._flags_after_hop(header_flags, remaining))
        try:
            micropython.schedule(_scheduled_repeat, (self, item))
        except Exception:
            self._enqueue_repeat(item)

    def service_repeater(self):
        # One rebroadcast, then the payload is dropped. Call while idle, never from the RX IRQ.
        # The (header_from, header_id) key stays until REPEAT_SEEN_MS so the echo is not sent again.
        if not self._repeater or not self._repeat_q:
            return False
        message, header_to, header_from, header_id, header_flags = self._repeat_q.pop(0)
        print("REPEAT from=%d to=%d id=%d flags=0x%02x" % (header_from, header_to, header_id, header_flags))
        self.send(message, header_to, header_id=header_id, header_flags=header_flags, header_from=header_from)
        self.wait_packet_sent()
        self.set_mode_rx()
        return True

    def _spi_write(self, register, payload):
        if type(payload) == int:
            payload = [payload]
        elif type(payload) == bytes:
            payload = [p for p in payload]
        elif type(payload) == str:
            payload = [ord(s) for s in payload]
        self.cs.value(0)
        self.spi.write(bytearray([register | 0x80] + payload))
        self.cs.value(1)

    def _spi_read(self, register, length=1):
        self.cs.value(0)
        if length == 1:
            data = self.spi.read(length + 1, register)[1]
        else:
            data = self.spi.read(length + 1, register)[1:]
        self.cs.value(1)
        return data
        
    def _decrypt(self, message):
        decrypted_msg = self.crypto.decrypt(message)
        msg_length = decrypted_msg[0]
        return decrypted_msg[1:msg_length + 1]

    def _encrypt(self, message):
        msg_length = len(message)
        padding = bytes(((math.ceil((msg_length + 1) / 16) * 16) - (msg_length + 1)) * [0])
        msg_bytes = bytes([msg_length]) + message + padding
        encrypted_msg = self.crypto.encrypt(msg_bytes)
        return encrypted_msg

    def _handle_interrupt(self, channel):
        irq_flags = self._spi_read(REG_12_IRQ_FLAGS)

        if self._mode == MODE_RXCONTINUOUS and (irq_flags & RX_DONE):
            packet_len = self._spi_read(REG_13_RX_NB_BYTES)
            self._spi_write(REG_0D_FIFO_ADDR_PTR, self._spi_read(REG_10_FIFO_RX_CURRENT_ADDR))

            packet = self._spi_read(REG_00_FIFO, packet_len)
            self._spi_write(REG_12_IRQ_FLAGS, 0xff)  # Clear all IRQ flags

            snr = self._spi_read(REG_19_PKT_SNR_VALUE) / 4
            rssi = self._spi_read(REG_1A_PKT_RSSI_VALUE)

            if snr < 0:
                rssi = snr + rssi
            else:
                rssi = rssi * 16 / 15

            if self._freq >= 779:
                rssi = round(rssi - 157, 2)
            else:
                rssi = round(rssi - 164, 2)

            if packet_len >= 4:
                header_to = packet[0]
                header_from = packet[1]
                header_id = packet[2]
                header_flags = packet[3]
                message = bytes(packet[4:]) if packet_len > 4 else b''

                if (self._this_address != header_to) and ((header_to != BROADCAST_ADDRESS) or (self._receive_all is False)):
                    # Foreign packet. Repeater may queue it; do not ACK and do not deliver it locally.
                    if self._repeater:
                        self._consider_repeat(header_to, header_from, header_id, header_flags, message)
                    return

                if self.crypto and len(message) % 16 == 0:
                    message = self._decrypt(message)

                if self._acks and header_to == self._this_address and not header_flags & FLAGS_ACK:
                    self.send_ack(header_from, header_id)

                self.set_mode_rx()

                self._last_payload = namedtuple(
                    "Payload",
                    ['message', 'header_to', 'header_from', 'header_id', 'header_flags', 'rssi', 'snr']
                )(message, header_to, header_from, header_id, header_flags, rssi, snr)

                if not header_flags & FLAGS_ACK:
                    self.on_recv(self._last_payload)

        elif self._mode == MODE_TX and (irq_flags & TX_DONE):
            self.set_mode_idle()

        elif self._mode == MODE_CAD and (irq_flags & CAD_DONE):
            self._cad = irq_flags & CAD_DETECTED
            self.set_mode_idle()

        self._spi_write(REG_12_IRQ_FLAGS, 0xff)

    def close(self):
        self.spi.deinit()
