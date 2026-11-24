#!/usr/bin/python3
"""Record every device console line published on pv/log/<host> to fugu_console.log.

The log rotates at 10 MB with 10 backups; the oldest backup is compressed into archive/ instead of
deleted. Broker credentials come from the environment (MQTT_USER, MQTT_PASS, MQTT_PORT) or from
the env file named by MQTT_ENV (default fugu-tools/mqtt.env). The recorder runs on the broker host,
so it connects to MQTT_CONSOLE_HOST (default localhost), not to MQTT_HOST.
"""
import logging
import os

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion

from archiving_handler import ArchivingRotatingFileHandler

TOPIC = "pv/log/+"


def load_env(path):
    if not os.path.exists(path):
        return
    for ln in open(path):
        ln = ln.strip()
        if ln and not ln.startswith("#") and "=" in ln:
            k, v = ln.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip())


def on_connect(mqttc, obj, flags, reason_code, properties):
    logger.info("Connected, reason_code: " + str(reason_code))
    mqttc.subscribe(TOPIC)  # resubscribe on every reconnect (clean session drops it)


def on_message(mqttc, obj, msg):
    name = msg.topic.split('/')[-1]
    line = msg.payload.decode('utf-8').strip()
    if line:
        logger.info('%8s: %s', name, line)


def on_subscribe(mqttc, obj, mid, reason_code_list, properties):
    logger.info("Subscribed: " + str(mid) + " " + str(reason_code_list))


logger = logging.getLogger('console')
logger.setLevel(logging.INFO)
handler = ArchivingRotatingFileHandler('fugu_console.log', 'archive', maxBytes=10_000_000, backupCount=10)
handler.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
logger.addHandler(handler)

load_env(os.environ.get("MQTT_ENV", "fugu-tools/mqtt.env"))
mqttc = mqtt.Client(CallbackAPIVersion.VERSION2)
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.username_pw_set(os.environ.get("MQTT_USER", "pv"), os.environ["MQTT_PASS"])
host = os.environ.get("MQTT_CONSOLE_HOST", "localhost")
logger.info('connecting broker %s..', host)
mqttc.connect(host, int(os.environ.get("MQTT_PORT", "1882")), 60)
mqttc.loop_forever()
