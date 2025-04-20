# OTA:file:main.py
# OTA:reboot:true
# MIT License (MIT)
# Copyright (c) 2023 Stephen Carey
# https://opensource.org/licenses/MIT

import json

import mpu6050
import uasyncio as asyncio
import utime
from machine import SoftI2C, Pin
from mqtt_as import MQTTClient
from mqtt_local import config

from bts7960 import Motor

import _thread

VERSION = 3

BASE_TOPIC = 'esp32/awning'
# states are open, opening, closed, closing, stopped
STATE_TOPIC = f'{BASE_TOPIC}/state'
COMMAND_TOPIC = f'{BASE_TOPIC}/set'
CONFIG_TOPIC = f'{BASE_TOPIC}/config'
READINGS_TOPIC = f'{BASE_TOPIC}/readings'
AVAILABLE_TOPIC = f'{BASE_TOPIC}/availability'
LOGS_TOPIC = f'{BASE_TOPIC}/logs'
VERSION_TOPIC = f'{BASE_TOPIC}/version'
OTA_TOPIC = f'{BASE_TOPIC}/ota'

logs = []

# motor pins
RIGHT_INHIBIT = 33
RIGHT_PWM = 26
RIGHT_CURRENT_SENSE = 34
LEFT_INHIBIT = 19
LEFT_PWM = 18
LEFT_CURRENT_SENSE = 23

# accelerometer pins
SDA = 16
SCL = 17

# open/close pins
CLOSE_PIN = 22
OPEN_PIN = 21

motor = Motor(RIGHT_INHIBIT, RIGHT_PWM, RIGHT_CURRENT_SENSE,
              LEFT_INHIBIT, LEFT_PWM, LEFT_CURRENT_SENSE)

close_switch = Pin(CLOSE_PIN, Pin.IN, pull=Pin.PULL_UP)
open_switch = Pin(OPEN_PIN, Pin.IN, pull=Pin.PULL_UP)

accelerometer_changes = None

wind_check_enabled = True
command = None
close_due_to_wind = False
last_published_state = None
wind_config = {"windThreshold": 1100, "sampleFrequency": 2}

# lower speed to help with the long cable run=-0
i2c = SoftI2C(scl=Pin(SCL), sda=Pin(SDA), freq=50000)
accelerometer = mpu6050.accel(i2c)
# add_log("Done initializing accelerometer...")
# await publish_logs()

last_readings = accelerometer.get_values()

def add_log(entry):
    logs.append(entry)


async def publish_logs():
    if logs:
        await client.publish(LOGS_TOPIC, "\n".join(logs), False, qos=0)
        logs.clear()


def check_fully_opened():
    return open_switch.value() == 0


def check_fully_closed():
    return close_switch.value() == 0


async def fully_open():
    await motor.slow_start(0)


async def fully_close():
    await motor.slow_start(1)


def stop():
    motor.stop()


def update_readings_thread():
    global accelerometer_changes, last_readings, command, close_due_to_wind
    while True:
        try:
            if wind_check_enabled:
                accelerometer_values = accelerometer.get_values()
                # {'GyZ': -235, 'GyY': 296, 'GyX': 16, 'Tmp': 26.64764, 'AcZ': -1552, 'AcY': -412, 'AcX': 16892}
                change_x = abs(last_readings['GyX'] - accelerometer_values['GyX'])
                change_y = abs(last_readings['GyY'] - accelerometer_values['GyY'])
                change_z = abs(last_readings['GyZ'] - accelerometer_values['GyZ'])
                last_readings = accelerometer_values
                print("X:{}, Y:{}, Z:{}".format(change_x, change_y, change_z))
                accelerometer_changes = change_x + change_y + change_z
                if change_x + change_y + change_z > wind_config['windThreshold']:
                    print("Closing due to wind!")
                    close_due_to_wind = True
                    command = 'close'
                    # reset last readings so we don't immediately close next time it opens
                    last_readings = {'GyX': 0, 'GyY': 0, 'GyZ': 0}
        except Exception as e:
            print("Problem checking wind: {}".format(e))
        finally:
            utime.sleep(wind_config['sampleFrequency'])


def handle_incoming_message(topic, msg, retained):
    global wind_config, command, close_due_to_wind
    msg_string = str(msg, 'UTF-8')
    topic_string = str(topic, 'UTF-8')
    if len(msg_string) < 500:
        print(f'{topic_string}: {msg}')
        add_log(f'{topic_string}: {msg_string}')
    else:
        print(f'Got a big message on {topic_string}...')

    if topic_string == CONFIG_TOPIC:
        wind_config = json.loads(msg_string)
    elif topic_string == COMMAND_TOPIC:
        if msg_string == 'close':
            print("Setting close_due_to_wind to False")
            close_due_to_wind = False
        command = msg_string
    elif topic_string == OTA_TOPIC:
        import ota
        ota.process_ota_msg(msg_string)


async def wifi_han(state):
    print('Wifi is ', 'up' if state else 'down')
    await asyncio.sleep(1)


# If you connect with clean_session True, must re-subscribe (MQTT spec 3.1.2.4)
async def conn_han(client):
    await client.subscribe(COMMAND_TOPIC, 0)
    await client.subscribe(CONFIG_TOPIC, 0)
    await client.subscribe(OTA_TOPIC, 0)
    await online()


async def online():
    await client.publish(AVAILABLE_TOPIC, 'online', retain=True, qos=0)


async def publish_state(state):
    global last_published_state
    if last_published_state != state:
        await client.publish(STATE_TOPIC, state, True, 0)
    last_published_state = state


async def process_command():
    global wind_check_enabled, command
    print(f"Processing {command} command...")
    add_log(f"Processing {command} command...")

    if command:
        try:
            # run the motor and wait for reed switch to stop
            wind_check_enabled = False
            command_start = utime.ticks_ms()
            while True:
                if command == 'stop':
                    stop()
                    await publish_state('stopped')
                    return
                opened = check_fully_opened()
                closed = check_fully_closed()
                print("Opened {}, Closed {}".format(opened, closed))
                if opened and closed:
                    print("Something is really messed up.  Stopping everything")
                    add_log("Something is really messed up.  Stopping everything")
                    stop()
                    await publish_state('stopped')
                    return
                elif not opened and not closed:
                    if command == 'open':
                        await publish_state('opening')
                        await fully_open()
                    elif command == 'close':
                        await publish_state('closing')
                        # if closing due to wind then close just a little in the hopes the smaller surface area will
                        # keep things under control.  Remember the include time for the slow start which takes about 6
                        # secs.
                        command_duration = utime.ticks_diff(utime.ticks_ms(), command_start)
                        print(f'{close_due_to_wind} - {command_duration} > {wind_config.get('windCloseMillis', 16_000)}')
                        if close_due_to_wind and command_duration > wind_config.get('windCloseMillis', 16_000):
                            add_log("Closed a little due to wind...")
                            stop()
                            await publish_state('open')
                            return
                        else:
                            await fully_close()
                elif opened:
                    print("Fully opened")
                    if command != 'close':
                        stop()
                        await publish_state('open')
                        # let things stabilize before checking the wind again
                        await asyncio.sleep(2)
                        wind_check_enabled = True
                        return
                    else:
                        await fully_close()
                elif closed:
                    print("Fully closed")
                    if command != 'open':
                        stop()
                        await publish_state('closed')
                        return
                    else:
                        await fully_open()
                await asyncio.sleep(0.1)
        except:
            print("Problem processing command")
            return
        finally:
            wind_check_enabled = True


async def main():
    await client.connect()
    await asyncio.sleep(2)  # Give broker time
    await online()
    global command, accelerometer_changes
    await client.publish(VERSION_TOPIC, str(VERSION), True, 0)

    while True:
        if command:
            await process_command()
            command = None
        if accelerometer_changes:
            await client.publish(READINGS_TOPIC, str(accelerometer_changes), True, 0)
            accelerometer_changes = None
        await publish_logs()
        await asyncio.sleep(1)


config['subs_cb'] = handle_incoming_message
config['connect_coro'] = conn_han
config['wifi_coro'] = wifi_han
config['will'] = [AVAILABLE_TOPIC, 'offline', True, 0]

MQTTClient.DEBUG = False
client = MQTTClient(config)

try:
    _thread.start_new_thread(update_readings_thread, ())

    loop = asyncio.get_event_loop()
    loop.create_task(main())
    loop.run_forever()
finally:
    client.close()
    asyncio.stop()
