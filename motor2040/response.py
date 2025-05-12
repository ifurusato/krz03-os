#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-06
# modified: 2025-05-11
#
# I2C/applicatoin response codes.
#
# from response import (
#     RESPONSE_INIT, RESPONSE_PIR_ACTIVE, RESPONSE_PIR_IDLE, RESPONSE_OKAY,
#     RESPONSE_BAD_ADDRESS, RESPONSE_BAD_REQUEST, RESPONSE_OUT_OF_SYNC,
#     RESPONSE_INVALID_CHAR, RESPONSE_SOURCE_TOO_LARGE, RESPONSE_UNVALIDATED,
#     RESPONSE_EMPTY_PAYLOAD, RESPONSE_PAYLOAD_TOO_LARGE, RESPONSE_BUSY,
#     RESPONSE_CONNECTION_ERROR, RESPONSE_RUNTIME_ERROR, RESPONSE_UNKNOWN_ERROR
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
RESPONSE_CONNECTION_ERROR  = 0x80
RESPONSE_RUNTIME_ERROR     = 0x81
RESPONSE_UNKNOWN_ERROR     = 0x82

#EOF
