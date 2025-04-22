
import time
from tiny_fx import TinyFX
from colorama import Fore, Style

from sound_dictionary import sound_dictionary

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

tiny = None

try:

    tiny = TinyFX(wav_root="/sounds") # Create a new TinyFX object and tell with where the wav files are located

    print(Fore.GREEN + Style.DIM + '-- press for boot button to exit play loop…' + Style.RESET_ALL)

    while True:
        for _name, _filename in sound_dictionary:
            print(Fore.CYAN + '-- playing ' + Fore.YELLOW + Style.BOLD + '{}'.format(_name) + Fore.CYAN + Style.NORMAL
                    + ' from file ' + Style.BOLD + '{}'.format(_filename) + Style.NORMAL + '…' + Style.RESET_ALL)
            tiny.wav.play_wav(_filename)
            while tiny.wav.is_playing():
                time.sleep(0.05)
            time.sleep(2.0)
            if tiny.boot_pressed():
                while tiny.boot_pressed():
                    print(Fore.GREEN + Style.DIM + '-- let go of the button!' + Style.RESET_ALL)
                    # wait until button released
                    time.sleep(0.01)
                break
        print('')
        while not tiny.boot_pressed():
            print(Fore.GREEN + Style.DIM + '-- waiting for boot button…' + Style.RESET_ALL)
            time.sleep(1.0)

# turn off all the outputs and audio
finally:
    if tiny:
        tiny.shutdown()

