# Board wiring, radio, and timing. Change these to match your build.

# How often to send an environmental reading, and how often the main loop wakes.
ENV_INTERVAL_MS = 2 * 60 * 1000  # milliseconds between env packets
LOOP_SLEEP_MS = 100              # main loop delay
STARTUP_DELAY_MS = 500           # pause after boot before I2C setup

# Tip-bucket rain input.
RAIN_PIN = 7                      # reed switch, GP number, active low
RAIN_DEBOUNCE_DELAY_MS = 2500     # ignore another tip until this long after the last
RAIN_DEBUG = True                 # print rain counts on the serial console

# BME680 on I2C0.
I2C_ID = 0
I2C_SCL_PIN = 5                   # GP number
I2C_SDA_PIN = 4                   # GP number
I2C_FREQ = 100000                 # Hz
I2C_RECOVERY_CLOCKS = 18          # SCL pulses used to free a stuck bus
I2C_SCAN_RETRIES = 8
I2C_SCAN_RETRY_DELAY_MS = 250

# BME680 identity and read retries.
BME680_ADDRESSES = (0x76, 0x77)
BME680_CHIP_ID_REGISTER = 0xD0
BME680_CHIP_ID = 0x61
BME680_INIT_SETTLE_MS = 100
BME680_READ_RETRIES = 3
BME680_READ_RETRY_DELAY_MS = 250
BME680_REINIT_BEFORE_EACH_READING = False

# RFM95 / SX1276 pins on the Pico (GP numbers).
RFM95_RST = 9    # radio reset
RFM95_CS = 8     # SPI chip select
RFM95_INT = 10   # DIO0 interrupt

# LoRa channel. Must match the bridge.
RF95_FREQ = 915.0  # MHz
RF95_POW = 20      # transmit power, dBm

# This node and the bridge. Bridge is 2; each sensor needs its own client address.
CLIENT_ADDRESS = 6
SERVER_ADDRESS = 2

# Confirmed send. ACK_RETRIES is extra tries after the first (2 means three total).
LORA_REQUIRE_ACK = True
LORA_ACK_RETRIES = 2
LORA_TX_TIMEOUT = 2  # seconds to wait for the radio to finish a transmit

# Status LEDs (GP numbers). LED blinks on each send.
LED_PIN = 13
LED_TX_PIN = 12
