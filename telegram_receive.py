# This file has to be runned separatly wit python3

import pzgram

from api_key import botkey

def process_message(chat, message):
    if chat.id != 20403805:
        return

    t = message.text

    with open("msg_rcv.txt", "w") as f:
        f.write(t.strip())


bot = pzgram.Bot(botkey)
bot.processAll = process_message
bot.run()