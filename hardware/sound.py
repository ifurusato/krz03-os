#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-20
# modified: 2025-04-22
#
# DO NOT EDIT: This is an auto-generated file.
#

import json
from enum import Enum
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

# JSON serialisation ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
from json import JSONEncoder
def _default(self, obj):
    return getattr(obj.__class__, "to_json", _default.default)(obj)
_default.default = JSONEncoder().default
JSONEncoder.default = _default

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Sound(Enum):
    __order__ = " SILENCE ARMING_TONE BEEP BEEP_HI BLIP BOINK BUZZ CHATTER CHIRP CHIRP_4 CHIRP_7 CRICKET DIT_A DIT_B DIT_C DWERP EARPIT GLINCE GWOLP HONK HZAH IPPURT ITIZ IZIT PEW_PEW_PEW PIZZLE SKID_FZZT SONIC_BAT TELEMETRY TIKA_TIKA TSK_TSK_TSK TICK TWEAK TWIDDLE_POP TWIT WOW ZZT "

    #                idx   name            mnemonic     dur(s)  filename                description
    SILENCE       = (  1, 'silence',      'SILENCE',      0.0, 'silence.wav',      'silence.')
    ARMING_TONE   = (  2, 'arming-tone',  'ARMING_TONE',  1.0, 'arming-tone.wav',  'arming-tone - beep.')
    BEEP          = (  3, 'beep',         'BEEP',         1.0, 'beep.wav',         'beep.')
    BEEP_HI       = (  4, 'beep-hi',      'BEEP_HI',      1.0, 'beep-hi.wav',      'hi beep.')
    BLIP          = (  5, 'blip',         'BLIP',         1.0, 'blip.wav',         'blip.')
    BOINK         = (  6, 'boink',        'BOINK',        1.0, 'boink.wav',        'boink.')
    BUZZ          = (  7, 'buzz',         'BUZZ',         1.0, 'buzz.wav',         'buzz.')
    CHATTER       = (  8, 'chatter',      'CHATTER',      1.0, 'chatter.wav',      'chatter.')
    CHIRP         = (  9, 'chirp',        'CHIRP',        2.0, 'chirp.wav',        'chirp.')
    CHIRP_4       = ( 10, 'chirp-4',      'CHIRP_4',      1.0, 'chirp-4.wav',      'chirp 4.')
    CHIRP_7       = ( 11, 'chirp-7',      'CHIRP_7',      1.0, 'chirp-7.wav',      'chirp 7.')
    CRICKET       = ( 12, 'cricket',      'CRICKET',      1.0, 'cricket.wav',      'cricket.')
    DIT_A         = ( 13, 'dit-a',        'DIT_A',        0.0, 'dit_a.wav',        'dit A.')
    DIT_B         = ( 14, 'dit-b',        'DIT_B',        0.0, 'dit_b.wav',        'dit B.')
    DIT_C         = ( 15, 'dit-c',        'DIT_C',        0.0, 'dit_c.wav',        'dit C.')
    DWERP         = ( 16, 'dwerp',        'DWERP',        1.0, 'dwerp.wav',        'dwerp.')
    EARPIT        = ( 17, 'earpit',       'EARPIT',       1.0, 'earpit.wav',       'ear pit.')
    GLINCE        = ( 18, 'glince',       'GLINCE',       1.0, 'glince.wav',       'glince.')
    GWOLP         = ( 19, 'gwolp',        'GWOLP',        1.0, 'gwolp.wav',        'gwolp.')
    HONK          = ( 20, 'honk',         'HONK',         1.0, 'honk.wav',         'honk.')
    HZAH          = ( 21, 'hzah',         'HZAH',         1.0, 'hzah.wav',         'hzah.')
    IPPURT        = ( 22, 'ippurt',       'IPPURT',       1.0, 'ippurt.wav',       'ippurt.')
    ITIZ          = ( 23, 'itiz',         'ITIZ',         1.0, 'itiz.wav',         'itiz.')
    IZIT          = ( 24, 'izit',         'IZIT',         1.0, 'izit.wav',         'izit.')
    PEW_PEW_PEW   = ( 25, 'pew-pew-pew',  'PEW_PEW_PEW',  1.0, 'pew-pew-pew.wav',  'pew pew pew.')
    PIZZLE        = ( 26, 'pizzle',       'PIZZLE',       1.0, 'pizzle.wav',       'pizzle.')
    SKID_FZZT     = ( 27, 'skid-fzzt',    'SKID_FZZT',    1.0, 'skid-fzzt.wav',    'skid-fzzt.')
    SONIC_BAT     = ( 28, 'sonic-bat',    'SONIC_BAT',    1.0, 'sonic-bat.wav',    'sonic bat beep.')
    TELEMETRY     = ( 29, 'telemetry',    'TELEMETRY',    1.0, 'telemetry.wav',    'telemetry.')
    TIKA_TIKA     = ( 30, 'tika-tika',    'TIKA_TIKA',    1.0, 'chatter-1.wav',    'tika-tika.')
    TSK_TSK_TSK   = ( 31, 'tsk-tsk-tsk',  'TSK_TSK_TSK',  1.0, 'tsk-tsk-tsk.wav',  'tsk-tsk-tsk.')
    TICK          = ( 32, 'tick',         'TICK',         0.0, 'tick.wav',         'tick.')
    TWEAK         = ( 33, 'tweak',        'TWEAK',        1.0, 'tweak.wav',        'tweak.')
    TWIDDLE_POP   = ( 34, 'twiddle-pop',  'TWIDDLE_POP',  1.0, 'twiddle-pop.wav',  'twiddle-pop.')
    TWIT          = ( 35, 'twit',         'TWIT',         1.0, 'twit.wav',         'twit.')
    WOW           = ( 36, 'wow',          'WOW',          1.0, 'wow.wav',          'wow.')
    ZZT           = ( 37, 'zzt',          'ZZT',          1.0, 'zzt.wav',          'zzt.')

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, mnemonic, duration, filename, description):
        self._index = num
        self._name = name
        self._mnemonic = mnemonic
        self._duration = duration
        self._filename = filename
        self._description = description

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def index(self):
        return self._index

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def mnemonic(self):
        return self._mnemonic

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def duration(self):
        return self._duration

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def filename(self):
        return self._filename

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def description(self):
        return self._description

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_index(value):
        if isinstance(value, Sound):
            _index = value.index
        elif isinstance(value, int):
            _index = value
        else:
            raise Exception('expected Sound or int argument, not: {}'.format(type(value)))
        for s in Sound:
            if _index == s.index:
                return s
        raise NotImplementedError

    # JSON serialisation ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def export():
        _records = []
        for _record in Sound:
            _records.append(_record)
        with open('sounds.json', 'w', encoding='utf-8') as f:
            json.dump(_records, f, ensure_ascii=False, indent=4)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def to_json(self):
        _dict = {}
        _dict['index']       = self._index
        _dict['name']        = self._name
        _dict['mnemonic']    = self._mnemonic
        _dict['duration']    = self._duration
        _dict['filename']    = self._filename
        _dict['description'] = self._description
        return _dict

#EOF
