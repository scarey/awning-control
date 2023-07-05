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

BASE_TOPIC = 'esp32/awning'
# states are open, opening, closed, closing, stopped
STATE_TOPIC = f'{BASE_TOPIC}/state'
COMMAND_TOPIC = f'{BASE_TOPIC}/set'
CONFIG_TOPIC = f'{BASE_TOPIC}/config'
READINGS_TOPIC = f'{BASE_TOPIC}/readings'
AVAILABLE_TOPIC = f'{BASE_TOPIC}/availability'

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

i2c = SoftI2C(scl=Pin(SCL), sda=Pin(SDA), freq=100000)
accelerometer = mpu6050.accel(i2c)

accelerometer_changes = None
last_readings = accelerometer.get_values()

wind_check_enabled = True
client = None
command = None
last_published_state = None
wind_config = {"windThreshold": 1100, "sampleFrequency": 2}


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
    global accelerometer_changes, last_readings, command
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
                    command = 'close'
                    # reset last readings so we don't immediately close next time it opens
                    last_readings = {'GyX': 0, 'GyY': 0, 'GyZ': 0}
        except Exception as e:
            print("Problem checking wind: {}".format(e))
        finally:
            utime.sleep(wind_config['sampleFrequency'])


def handle_incoming_message(topic, msg, retained):
    print(f'{topic}: {msg}')
    msg_string = str(msg, 'UTF-8')
    if topic == CONFIG_TOPIC:
        global wind_config
        wind_config = json.loads(msg_string)
    elif topic == COMMAND_TOPIC:
        global command
        command = msg_string


async def wifi_han(state):
    print('Wifi is ', 'up' if state else 'down')
    await asyncio.sleep(1)


# If you connect with clean_session True, must re-subscribe (MQTT spec 3.1.2.4)
async def conn_han(client):
    await client.subscribe(COMMAND_TOPIC, 0)
    await client.subscribe(CONFIG_TOPIC, 0)
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
    print("Processing {} command...".format(command))
    if command:
        try:
            # run the motor and wait for reed switch to stop
            wind_check_enabled = False
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
                    stop()
                    await publish_state('stopped')
                    return
                elif not opened and not closed:
                    # print("In the middle...")
                    if command == 'open':
                        await publish_state('opening')
                        await fully_open()
                    elif command == 'close':
                        await publish_state('closing')
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
    while True:
        if command:
            await process_command()
            command = None
        if accelerometer_changes:
            await client.publish(READINGS_TOPIC, accelerometer_changes, True, 0)
            accelerometer_changes = None
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
