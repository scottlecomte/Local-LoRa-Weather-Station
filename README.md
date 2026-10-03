# Local LoRa Weather Station

MicroPython client for a Raspberry Pi Pico, an RFM95, a BME680, and a tipping-bucket rain gauge. It reads the environment sensor and counts rain tips, then sends JSON with RadioHead `send_to_wait` so the bridge can ACK it.

Companion receiver: [Lora to Ethernet Bridge](https://github.com/scottlecomte/Lora-to-Ethernet-Bridge). This node is client address 6 and the bridge is server address 2, on 915 MHz. Change `config.py` if your wiring or addresses differ.

## Hardware

| Item | This tree |
|------|-----------|
| MCU | Raspberry Pi Pico (RP2040), MicroPython |
| Radio | RFM95. CS GP8, reset GP9, DIO0 GP10, SPI0 (`SPIConfig.rp2_0`) |
| Environment | BME680 on I2C0. SDA GP4, SCL GP5. Addresses 0x76 or 0x77 |
| Rain | Tipping bucket. Reed switch on GP7, active low |
| LED | GP13 blinks on each send |

`ENV_INTERVAL_MS` is 2 minutes between environmental packets. The first one goes out one second after the loop starts. `LORA_ACK_RETRIES` is 2, so three tries total if the bridge does not ACK. Retries reuse the same JSON and `seq`.

## Environment sensor

The BME680 is read over I2C after a bus scan and a chip-id check (0x61). If the scan finds nothing, the script clocks SCL to try to free a stuck bus, then retries. Each sample is temperature, pressure, humidity, and gas. Temperature is converted from Celsius to Fahrenheit and rounded to two decimals. Pressure is rounded to three decimals, humidity to two. Gas is the driver value divided by 1000, rounded to two decimals, and sent as a string.

The packet fields are `type` (`env`), `seq`, `temperature`, `humidity`, `pressure`, and `gas`.

## Rain gauge

A falling edge on the reed switch counts one tip. The input is pull-up, active low, and it stays disarmed until the pin is high again and `RAIN_DEBOUNCE_DELAY_MS` (2500 ms) has elapsed. Tips that arrive between sends are added together. The loop ships them as one packet: `type` `rain`, one `seq`, and `tips` (how many since the last rain packet). Retries reuse that payload. An empty count is not sent.

## Rain downstream

Those rain packets can go through a Node-RED flow and into a database. Keep a timestamp on each stored row so you can total tips for an exact time window later, or point Grafana at the same table and graph it. The node only sends the tip count and a sequence number. The timestamp is added when the packet is stored, not on the Pico.

## Layout

```
main.py
config.py
lib/ulora.py
lib/bme680.py
```

Flash `main.py` and `config.py` to the Pico, and put `ulora.py` and `bme680.py` under `lib/`, with Thonny or `mpremote`.
