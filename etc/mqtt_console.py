#!/usr/bin/python3


import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion


def on_connect(mqttc, obj, flags, reason_code, properties):
    print("Connected, reason_code: " + str(reason_code))


def on_message(mqttc, obj, msg):
    name = msg.topic.split('/')[-1]
    line = msg.payload.decode('utf-8').strip()
    if line:
        print('%s: %s' % (name, line))
    #print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))


def on_subscribe(mqttc, obj, mid, reason_code_list, properties):
    print("Subscribed: " + str(mid) + " " + str(reason_code_list))


def on_log(mqttc, obj, level, string):
    print('log:', string)


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
mqttc.username_pw_set("pv", "0ffgrid")
mqttc.connect("havan.local", 1882, 60)
# pv/log/fugu-esp32s3-100100C40A24
mqttc.subscribe("pv/log/+")

mqttc.loop_forever()