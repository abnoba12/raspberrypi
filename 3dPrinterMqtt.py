from __future__ import print_function

import os
import time

import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from paho.mqtt.client import MQTT_ERR_SUCCESS, MQTT_ERR_NO_CONN


DEVICES_PATH = "/sys/bus/w1/devices"
MQTT_HOST = "openhabianpi"
MQTT_PORT = 1883
SLEEP_TIME = 5
ACTIVE_GPIO = [2, 3]
messages=[]

def connect_mqtt():
    mqttc = mqtt.Client()
    print("Connecting to {}".format(MQTT_HOST))
    mqttc.connect(MQTT_HOST, MQTT_PORT, 60)
    print("Connected")
    mqttc.loop_start()
    mqttc.on_disconnect = on_disconnect
    mqttc.on_message=on_message
    mqttc.connected_flag=True
    mqttc.subscribe("3dprinter/GPIO/control/#")
    return mqttc


def disconnect_mqtt(mqttc):
    mqttc.loop_stop()
    mqttc.disconnect()


def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected disconnection.")
        client.reconnect()

def on_message(client, userdata, message):
   msg=str(message.payload.decode("utf-8"))
   topic=message.topic
   messages.append([topic,msg])

def get_thermometer_names():
    thermometers = [x[1] for x in os.walk(DEVICES_PATH)][0]
    while "w1_bus_master1" in thermometers:
        thermometers.remove(
        "w1_bus_master1")
    return thermometers


def get_thermometer_temp(thermometer):
    with open("{}/{}/w1_slave".format(DEVICES_PATH, thermometer), "r") as tfile:
        text = tfile.read()
        secondline = text.split("\n")[1]
        temperaturedata = secondline.split(" ")[9]
        temperature = float(temperaturedata[2:])
        temperature = temperature / 1000
        return temperature


def init_gpio(gpio):
    mode = GPIO.getmode()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(gpio, GPIO.OUT)
    GPIO.output(gpio, 1)


def publish_temp(mqttc, thermometer, temp):
    topic = "3dprinter/w1/thermometer/{}".format(thermometer)
    (result, mid) = mqttc.publish(topic, temp)
    if result == MQTT_ERR_SUCCESS:
        print("Published {} to {}".format(temp, topic))
    elif result == MQTT_ERR_NO_CONN:
        print("Connection error")
    else:
        print("Unknown error")


def publish_digital(mqttc, GPIO, val):
    topic = "3dprinter/GPIO/{}".format(GPIO)
    (result, mid) = mqttc.publish(topic, val)
    if result == MQTT_ERR_SUCCESS:
        print("Published {} to {}".format(val, topic))
    elif result == MQTT_ERR_NO_CONN:
        print("Connection error")
    else:
        print("Unknown error")


def main():
    print("Starting")
    try:
        mqttc = connect_mqtt()
        thermometers = get_thermometer_names()
        print("Detected {} thermometers".format(len(thermometers)))
        for gpio in ACTIVE_GPIO:
            init_gpio(gpio)
        print("Initialized {} outputs".format(len(ACTIVE_GPIO)))
        while True:
            if len(messages)>0:
                m=messages.pop(0)
                print("received ",m)
                GPIO.output(int(m[0][-1]),int(m[1])) #set
                print("value is ",GPIO.input(int(m[0][-1])))
            for thermometer in thermometers:
                temp = get_thermometer_temp(thermometer)
                publish_temp(mqttc, thermometer, temp)
            for gpio in ACTIVE_GPIO:
                publish_digital(mqttc, gpio, GPIO.input(gpio))
            time.sleep(SLEEP_TIME)
    except KeyboardInterrupt:
        print("Exiting")
    finally:
        print("Disconnecting from {}".format(MQTT_HOST))
        disconnect_mqtt(mqttc)
        print("Done")


if __name__ == "__main__":
    main()
