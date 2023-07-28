import requests
import os
import time
import json

from api_key import botkey

def generate_keyboard(n):
    buttons = [[], []]
    for i in range(0, n):
        buttons[i%2].append(str(i))
    return json.dumps({"keyboard": buttons, "one_time_keyboard": True, "resize_keyboard": True})

def send_message(text, keyboard=0):

    if keyboard > 0:
        reply_markup = generate_keyboard(keyboard)
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
