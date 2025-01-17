# TODO speak something out

import os
from gtts import gTTS
from playsound import playsound

def speakText(text):
    """Converts text to speech and plays it."""
    tts = gTTS(text=text, lang='en')
    tmpPath = "storage/tmp/response_audio.mp3"
    tts.save(tmpPath)
    # https://stackoverflow.com/questions/69245722/error-259-on-python-playsound-unable-to-sound
    playsound(tmpPath)
    os.remove(tmpPath)
    
