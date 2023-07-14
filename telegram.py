import requests
import os
import time
import json

botkey = "5992286320:AAHJts5nZ33QhE0nYWvDBb3sZUtdvtiXgfw"


def send_message(text, keyboard=False):

    if keyboard:
        reply_markup = json.dumps({"keyboard": [["0","2","4","6","8"],["1","3","5","7","9"]],
                                   "one_time_keyboard": True,
                                   "resize_keyboard": True})
    else:
        reply_markup = json.dumps({"remove_keyboard": True})

    url = "https://api.telegram.org/bot"+botkey+"/sendMessage"
    params = {"chat_id": 20403805,
              "text": text,
              "reply_markup": reply_markup}

    r = requests.get(url, params=params)


def receive_message():

    while not os.path.exists("msg_rcv.txt"):
        time.sleep(0.1)

    with open("msg_rcv.txt", "r") as f:
        msg = f.read()
        print "Received:", msg

    os.remove("msg_rcv.txt")

    return msg.strip()
