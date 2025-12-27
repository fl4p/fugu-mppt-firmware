#!/usr/bin/python3
import sys

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion


def on_connect(mqttc, obj, flags, reason_code, properties):
    logger.info("Connected, reason_code: " + str(reason_code))


def on_message(mqttc, obj, msg):
    name = msg.topic.split('/')[-1]
    line = msg.payload.decode('utf-8').strip()
    if line:
        # print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload), flush=True)
        logger.info('%8s: %s', name, line)


def on_subscribe(mqttc, obj, mid, reason_code_list, properties):
    logger.info("Subscribed: " + str(mid) + " " + str(reason_code_list))


def on_log(mqttc, obj, level, string):
    logger.info('log:', string)


import logging
from logging.handlers import RotatingFileHandler

FORMAT = '%(asctime)s %(message)s'  # https://docs.python.org/3/library/logging.html#logging.Logger.debug
fmt = logging.Formatter(FORMAT)
# fmt.default_time_format = '%m-%d %H:%M:%S'

logger = logging.getLogger('console')
logger.setLevel(logging.INFO)
handler = RotatingFileHandler('fugu_console.log', maxBytes=10_000_000, backupCount=10)
handler.setFormatter(fmt)
logger.addHandler(handler)
std_h = logging.StreamHandler(stream=sys.stdout)
std_h.setFormatter(fmt)
logger.addHandler(std_h)

# If you want to use a specific client id, use
# mqttc = mqtt.Client("client-id")
# but note that the client id must be unique on the broker. Leaving the client
# id parameter empty will generate a random id for you.
mqttc = mqtt.Client(CallbackAPIVersion.VERSION2)
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
# Uncomment to enable debug messages
# mqttc.on_log = on_log

topic = "pv/log/+"

mqttc.username_pw_set("pv", "0ffgrid")
logger.info('connecting broker..')
mqttc.connect("havan.local", 1882, 60)
# pv/log/fugu-esp32s3-100100C40A24
logger.info('connected, subscribing %r', topic)
mqttc.subscribe(topic)

mqttc.loop_forever()
