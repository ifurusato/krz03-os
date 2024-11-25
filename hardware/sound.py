#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-20
# modified: 2024-11-25
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
    __order__ = " SILENCE BEEP BEEP_HI BLIP BUZZ CHATTER_1 CHATTER_2 CHATTER_3 CHATTER_4 CHATTER_5 CHIRP_1 CHIRP_4 CHIRP_7 DWERP EARPIT HONK HZAH PEW_PEW_PEW PIZZLE SONIC_BAT TELEMETRY TSK_TSK_TSK TICK TWEAK TWIT ZZT "

    #                idx   name            mnemonic     dur(s)  filename                description
    SILENCE       = (  1, 'silence',      'SILENCE',      0.0, 'silence.wav',      'silence. 1')
    BEEP          = (  2, 'beep',         'BEEP',         1.0, 'beep.wav',         'beep. 2')
    BEEP_HI       = (  3, 'beep-hi',      'BEEP_HI',      1.0, 'beep-hi.wav',      'hi beep. 3')
    BLIP          = (  4, 'blip',         'BLIP',         1.0, 'blip.wav',         'blip. 4')
    BUZZ          = (  5, 'buzz',         'BUZZ',         1.0, 'buzz.wav',         'buzz. 5')
    CHATTER_1     = (  6, 'chatter-1',    'CHATTER_1',    1.0, 'chatter-1.wav',    'chatter 1. 6')
    CHATTER_2     = (  7, 'chatter-2',    'CHATTER_2',    1.0, 'chatter-2.wav',    'chatter 2. 7')
    CHATTER_3     = (  8, 'chatter-3',    'CHATTER_3',    1.0, 'chatter-3.wav',    'chatter 3. 8')
    CHATTER_4     = (  9, 'chatter-4',    'CHATTER_4',    1.0, 'chatter-4.wav',    'chatter 4. 9')
    CHATTER_5     = ( 10, 'chatter-5',    'CHATTER_5',    1.0, 'chatter-5.wav',    'chatter 5. 10')
    CHIRP_1       = ( 11, 'chirp-1',      'CHIRP_1',      2.0, 'chirp-1.wav',      'chirp 1. 11')
    CHIRP_4       = ( 12, 'chirp-4',      'CHIRP_4',      1.0, 'chirp-4.wav',      'chirp 4. 12')
    CHIRP_7       = ( 13, 'chirp-7',      'CHIRP_7',      1.0, 'chirp-7.wav',      'chirp 7. 13')
    DWERP         = ( 14, 'dwerp',        'DWERP',        1.0, 'dwerp.wav',        'dwerp. 14')
    EARPIT        = ( 15, 'earpit',       'EARPIT',       1.0, 'earpit.wav',       'ear pit. 15')
    HONK          = ( 16, 'honk',         'HONK',         1.0, 'honk.wav',         'honk. 16')
    HZAH          = ( 17, 'hzah',         'HZAH',         1.0, 'hzah.wav',         'hzah. 17')
    PEW_PEW_PEW   = ( 16, 'pew-pew-pew',  'PEW_PEW_PEW',  1.0, 'pew-pew-pew.wav',  'pew pew pew. 16')
    PIZZLE        = ( 17, 'pizzle',       'PIZZLE',       1.0, 'pizzle.wav',       'pizzle. 17')
    SONIC_BAT     = ( 18, 'sonic-bat',    'SONIC_BAT',    1.0, 'sonic-bat.wav',    'sonic bat beep. 18')
    TELEMETRY     = ( 19, 'telemetry',    'TELEMETRY',    1.0, 'telemetry.wav',    'telemetry. 19')
    TSK_TSK_TSK   = ( 20, 'tsk-tsk-tsk',  'TSK_TSK_TSK',  1.0, 'tsk-tsk-tsk.wav',  'tsk-tsk-tsk. 20')
    TICK          = ( 21, 'tick',         'TICK',         0.0, 'tick.wav',         'tick. 21')
    TWEAK         = ( 22, 'tweak',        'TWEAK',        1.0, 'tweak.wav',        'tweak. 22')
    TWIT          = ( 23, 'twit',         'TWIT',         1.0, 'twit.wav',         'twit. 23')
    ZZT           = ( 24, 'zzt',          'ZZT',          1.0, 'zzt.wav',          'zzt. 24')

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
