import time
from ulora import LoRa, ModemConfig, SPIConfig
import machine
from machine import Pin, I2C
from bme680 import BME680_I2C
import ujson
import config  # pins, radio, and timing live in config.py


print('Weather station startup: BME680 I2C recovery v5')
time.sleep_ms(config.STARTUP_DELAY_MS)

msg_seq = 0
def next_seq():
    global msg_seq
    msg_seq = (msg_seq + 1) & 0xFFFFFFFF
    return msg_seq

def release_i2c_pin(pin_number):
    return Pin(pin_number, Pin.IN, Pin.PULL_UP)

def pull_i2c_pin_low(pin_number):
    pin = Pin(pin_number, Pin.OUT)
    pin.value(0)
    return pin

def recover_i2c_bus():
    scl = release_i2c_pin(config.I2C_SCL_PIN)
    sda = release_i2c_pin(config.I2C_SDA_PIN)
    time.sleep_ms(1)

    print('Recovering I2C bus: SCL=%d SDA=%d' % (scl.value(), sda.value()))
    if scl.value() and sda.value():
        return

    for _ in range(config.I2C_RECOVERY_CLOCKS):
        if sda.value():
            break
        pull_i2c_pin_low(config.I2C_SCL_PIN)
        time.sleep_us(10)
        scl = release_i2c_pin(config.I2C_SCL_PIN)
        time.sleep_us(10)
        sda = release_i2c_pin(config.I2C_SDA_PIN)

    pull_i2c_pin_low(config.I2C_SDA_PIN)
    time.sleep_us(10)
    release_i2c_pin(config.I2C_SCL_PIN)
    time.sleep_us(10)
    sda = release_i2c_pin(config.I2C_SDA_PIN)
    time.sleep_ms(1)
    print('I2C bus after recovery: SCL=%d SDA=%d' % (scl.value(), sda.value()))

def create_i2c():
    recover_i2c_bus()
    return I2C(
        id=config.I2C_ID,
        scl=Pin(config.I2C_SCL_PIN),
        sda=Pin(config.I2C_SDA_PIN),
        freq=config.I2C_FREQ
    )

i2c = create_i2c()

def format_i2c_devices(devices):
    return ", ".join(["0x%02x" % device for device in devices])

def has_bme680_address(devices):
    for address in config.BME680_ADDRESSES:
        if address in devices:
            return True
    return False

def has_unexpected_i2c_devices(devices):
    for address in devices:
        if address not in config.BME680_ADDRESSES:
            return True
    return False

def reset_i2c(i2c):
    try:
        i2c.deinit()
    except AttributeError:
        pass
    except OSError:
        pass
    release_i2c_pin(config.I2C_SCL_PIN)
    release_i2c_pin(config.I2C_SDA_PIN)
    time.sleep_ms(config.I2C_SCAN_RETRY_DELAY_MS)
    return create_i2c()

def scan_i2c_bus(i2c):
    print('Scanning I2C bus...')
    devices = []
    for attempt in range(config.I2C_SCAN_RETRIES):
        devices = i2c.scan()
        if devices:
            break
        if devices and has_unexpected_i2c_devices(devices):
            print('Unexpected I2C scan result: %s' % format_i2c_devices(devices))
        elif not devices:
            print('No I2C devices found.')
        if attempt < config.I2C_SCAN_RETRIES - 1:
            print('I2C recovery attempt %d...' % (attempt + 1))
            i2c = reset_i2c(i2c)
    return i2c, devices

def find_bme680_address(i2c):
    i2c, devices = scan_i2c_bus(i2c)

    if not devices:
        raise RuntimeError('No I2C devices found. Check power, ground, SDA GP%d, and SCL GP%d.' % (config.I2C_SDA_PIN, config.I2C_SCL_PIN))

    print('I2C devices found:', format_i2c_devices(devices))
    for address in config.BME680_ADDRESSES:
        if address in devices:
            print('Using BME680 address: 0x%02x' % address)
            return i2c, address

    raise RuntimeError(
        'BME680 not found at 0x76 or 0x77. I2C devices found: %s' %
        format_i2c_devices(devices)
    )

def read_i2c_register(i2c, address, register):
    try:
        return i2c.readfrom_mem(address, register, 1)[0]
    except OSError:
        i2c.writeto(address, bytearray([register & 0xFF]), False)
        return i2c.readfrom(address, 1)[0]

def check_bme680_chip_id(i2c, address):
    try:
        chip_id = read_i2c_register(i2c, address, config.BME680_CHIP_ID_REGISTER)
    except OSError as exc:
        raise RuntimeError(
            'Device at 0x%02x was found by scan, but chip-id read failed: %s' %
            (address, exc)
        )

    print('BME680 chip id: 0x%02x' % chip_id)
    if chip_id != config.BME680_CHIP_ID:
        raise RuntimeError(
            'Device at 0x%02x has chip id 0x%02x, expected 0x%02x for BME680.' %
            (address, chip_id, config.BME680_CHIP_ID)
        )

def init_bme680(i2c):
    i2c, address = find_bme680_address(i2c)
    check_bme680_chip_id(i2c, address)
    try:
        sensor = BME680_I2C(i2c=i2c, address=address)
        time.sleep_ms(config.BME680_INIT_SETTLE_MS)
        return i2c, sensor
    except OSError as exc:
        raise RuntimeError(
            'BME680 responded at 0x%02x, but initialization failed: %s' %
            (address, exc)
        )


led = Pin(config.LED_PIN, Pin.OUT)
led2 = Pin(config.LED_TX_PIN, Pin.OUT)

trigger_pin = Pin(config.RAIN_PIN, Pin.IN, Pin.PULL_UP)
last_trigger_time = time.ticks_add(time.ticks_ms(), -config.RAIN_DEBOUNCE_DELAY_MS - 1)
rain_rearm_after = 0
rain_sensor_armed = True
rain_tip_count = 0

# initialise radio
lora = LoRa(SPIConfig.rp2_0, config.RFM95_INT, config.CLIENT_ADDRESS, config.RFM95_CS, reset_pin=config.RFM95_RST, freq=config.RF95_FREQ, tx_power=config.RF95_POW, acks=True)
lora.wait_packet_sent_timeout = config.LORA_TX_TIMEOUT

i2c, bme = init_bme680(i2c)

def reset_bme680(verbose=True):
    global i2c, bme

    if verbose:
        print('Recovering BME680...')
    i2c = reset_i2c(i2c)
    i2c, bme = init_bme680(i2c)

def read_bme680_data():
    last_error = None
    for attempt in range(config.BME680_READ_RETRIES):
        try:
            return (bme.temperature, bme.pressure, bme.humidity, bme.gas)
        except Exception as exc:
            last_error = exc
            if attempt == config.BME680_READ_RETRIES - 1:
                break

            print(
                'BME680 read failed (%s), retrying after recovery...' %
                exc
            )
            try:
                reset_bme680()
            except Exception as recover_exc:
                last_error = recover_exc
                print('BME680 recovery failed:', recover_exc)
            time.sleep_ms(config.BME680_READ_RETRY_DELAY_MS)

    raise RuntimeError(
        'BME680 read failed after %d attempts: %s' %
        (config.BME680_READ_RETRIES, last_error)
    )

def send_lora(readings):
    message = ujson.dumps(readings)
    print(message)
    try:
        if config.LORA_REQUIRE_ACK:
            # same message/seq on every retry — Node-RED dedupes on seq
            ok = lora.send_to_wait(message, config.SERVER_ADDRESS, retries=config.LORA_ACK_RETRIES)
        else:
            ok = lora.send(message, config.SERVER_ADDRESS)
            if ok:
                ok = lora.wait_packet_sent()
        try:
            lora.sleep()
        except Exception:
            pass
    except Exception as exc:
        print('LoRa send failed:', exc)
        ok = False
    print('LoRa send ok:', ok)
    led.value(1)
    time.sleep(0.1)
    led.value(0)
    return ok
    

def send_temp():
    if config.BME680_REINIT_BEFORE_EACH_READING:
        reset_bme680(verbose=False)

    # Read sensor data
    temp_c, pres, hum, gas_raw = read_bme680_data()
    temp_f = temp_c * (9/5) + 32
    temp = (round(temp_f, 2))
    pres = round(pres, 3)
    hum = round(hum, 2)
    gas = str(round(gas_raw/1000, 2))
        
    message_type = "env"
        
    readings = {"type": message_type, "seq": next_seq(), "temperature": temp, "humidity": hum, "pressure": pres, "gas": gas}
    send_lora(readings)
     
def rain_trigger(pin):
    global last_trigger_time, rain_rearm_after, rain_sensor_armed, rain_tip_count
    
    current_time = time.ticks_ms()
    if not rain_sensor_armed:
        return

    if time.ticks_diff(current_time, last_trigger_time) <= config.RAIN_DEBOUNCE_DELAY_MS:
        return

    last_trigger_time = current_time
    rain_rearm_after = time.ticks_add(current_time, config.RAIN_DEBOUNCE_DELAY_MS)
    rain_sensor_armed = False
    rain_tip_count += 1

def update_rain_sensor_arm():
    global rain_sensor_armed

    if rain_sensor_armed:
        return

    if trigger_pin.value() and time.ticks_diff(time.ticks_ms(), rain_rearm_after) >= 0:
        rain_sensor_armed = True

def get_pending_rain_tips():
    global rain_tip_count

    state = machine.disable_irq()
    tips = rain_tip_count
    rain_tip_count = 0
    machine.enable_irq(state)

    return tips


def send_rain_tips(tips):
    if not tips:
        return
    if config.RAIN_DEBUG:
        print('Rain tips pending:', tips)
    # one logical event (batch count); seq once; retries reuse this payload
    readings = {"type": "rain", "seq": next_seq(), "tips": tips}
    send_lora(readings)

def main():
    
    trigger_pin.irq(trigger=Pin.IRQ_FALLING, handler=rain_trigger)
    if config.RAIN_DEBUG:
        print('Rain trigger armed on GP%d, initial value:' % config.RAIN_PIN, trigger_pin.value())
    next_env_send = time.ticks_add(time.ticks_ms(), 1000)

    while True:
        update_rain_sensor_arm()

        tips = get_pending_rain_tips()
        if tips:
            send_rain_tips(tips)

        now = time.ticks_ms()
        if time.ticks_diff(now, next_env_send) >= 0:
            try:
                send_temp()
            except Exception as exc:
                print('Environmental send failed:', exc)
            next_env_send = time.ticks_add(now, config.ENV_INTERVAL_MS)

        time.sleep_ms(config.LOOP_SLEEP_MS)

def shutdown_hardware():
    try:
        lora.sleep()
        lora.close()
    except Exception:
        pass

    try:
        i2c.deinit()
    except AttributeError:
        pass
    except Exception:
        pass

    release_i2c_pin(config.I2C_SCL_PIN)
    release_i2c_pin(config.I2C_SDA_PIN)
        
if __name__ == "__main__":
    try:
        main()
    finally:
        shutdown_hardware()

