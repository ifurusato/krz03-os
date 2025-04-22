import time
import sys
from tiny_fx import TinyFX

print('starting UART...')

_sounds = [
    ('blip',              'blip.wav'),
    ('buzz',              'buzz.wav'),
    ('chatter 1',         'chatter-1.wav'),
    ('chatter 2',         'chatter-2.wav'),
    ('chatter 4',         'chatter-4.wav'),
    ('chatter 5',         'chatter-5.wav'),
    ('dwerp',             'dwerp.wav'),
    ('earpit',            'earpit.wav'),
    ('chirp 1',           'chirp-1.wav'),
    ('chirp 4',           'chirp-4.wav'),
    ('chirp 7',           'chirp-7.wav'),
    ('hzah',              'hzah.wav'),
    ('pew! pew! pew!',    'pew-pew-pew.wav'),
    ('sonic-bat',         'sonic-bat.wav'),
    ('telemetry',         'telemetry.wav'),
    ('tsk-tsk-tsk',       'tsk-tsk-tsk.wav'),
    ('tweak',             'tweak.wav'),
    ('twit',              'twit.wav'),
    ('zzt',               'zzt.wav'),
]


try: 

    tiny = TinyFX(wav_root="/sounds")
    tiny.wav.play_wav('sonic-bat.wav')
    time.sleep(2)

    while True:
        data = sys.stdin.read()
        if data:
            print("RP2040 received: {}".format(data))
            tiny.wav.play_wav('buzz.wav')
        else:
            print("no data.")
            tiny.wav.play_wav('chirp-1.wav')
        time.sleep(3)
except Exception as e:
    print('exception: {}'.format(e))
finally:
    print('finally.')

