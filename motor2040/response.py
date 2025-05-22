#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-06
# modified: 2025-05-22
#
# I2C/application response codes.
#
# from response import (
#     RESPONSE_INIT, RESPONSE_PIR_ACTIVE, RESPONSE_PIR_IDLE, RESPONSE_OKAY,
#     RESPONSE_BAD_ADDRESS, RESPONSE_BAD_REQUEST, RESPONSE_OUT_OF_SYNC,
#     RESPONSE_INVALID_CHAR, RESPONSE_SOURCE_TOO_LARGE, RESPONSE_PAYLOAD_TOO_LARGE,
#     RESPONSE_UNVALIDATED, RESPONSE_EMPTY_PAYLOAD, RESPONSE_BUSY, RESPONSE_SKIPPED,
#     RESPONSE_CONN_ERROR, RESPONSE_RUNTIME_ERROR, RESPONSE_UNKNOWN_ERROR
# )

RESPONSE_INIT              = 0x10
RESPONSE_PIR_ACTIVE        = 0x30
RESPONSE_PIR_IDLE          = 0x31
RESPONSE_OKAY              = 0x4F  # all acceptable responses must be less than this
RESPONSE_BAD_ADDRESS       = 0x71
RESPONSE_BAD_REQUEST       = 0x72
RESPONSE_OUT_OF_SYNC       = 0x73
RESPONSE_INVALID_CHAR      = 0x74
RESPONSE_SOURCE_TOO_LARGE  = 0x75
RESPONSE_UNVALIDATED       = 0x76
RESPONSE_EMPTY_PAYLOAD     = 0x77
RESPONSE_PAYLOAD_TOO_LARGE = 0x78
RESPONSE_BUSY              = 0x79
RESPONSE_SKIPPED           = 0x80
RESPONSE_CONNECTION_ERROR  = 0x81
RESPONSE_RUNTIME_ERROR     = 0x82
RESPONSE_UNKNOWN_ERROR     = 0x83

RESP_INIT                  = "REIN"
RESP_PIR_ACTIVE            = "REPA"
RESP_PIR_IDLE              = "REPI"
RESP_OKAY                  = "REOK"
RESP_BAD_ADDRESS           = "REBA"
RESP_BAD_REQUEST           = "REBR"
RESP_OUT_OF_SYNC           = "REOS"
RESP_INVALID_CHAR          = "REIC"
RESP_SOURCE_TOO_LARGE      = "RESL"
RESP_PAYLOAD_TOO_LARGE     = "REPL"
RESP_UNVALIDATED           = "REUN"
RESP_EMPTY_PAYLOAD         = "REEP"
RESP_BUSY                  = "REBY"
RESP_SKIPPED               = "RESK"
RESP_CONN_ERROR            = "RECE"
RESP_RUNTIME_ERROR         = "RERE"
RESP_UNKNOWN_ERROR         = "REUE"

#EOF
