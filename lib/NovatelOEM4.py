#!/usr/bin/python
# -*- coding: utf-8 -*-
# The MIT License (MIT)
# Copyright (c) 2016 Bruno Tibério
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

""" Module NovatelOEM4

:Date: 28 Dec 2016

:Version: 0.3

:Author: Bruno Tibério

:Contact: bruno.tiberio@tecnico.ulisboa.pt


This module contains a few functions to interact with Novatel OEM4 GPS devices.
Currently only the most important functions and definitions are configured, but
the intention is to make it as much complete as possible.

A simple example can be run by executing the main function wich creates a Gps
class object and execute the following commands on gps receiver:

- **begin:** on default port or given port by argv[1].
- **sendUnlogall**
- **setCom(baud=115200):** changes baudrate to 115200bps
- **askLog(trigger=2,period=0.1):** ask for log *bestxyz* with trigger `ONTIME`
  and period `0.1`
- wait for 10 seconds
- **shutdown:** safely disconnects from gps receiver

Example:

.. code-block:: console

    $python NovatelOEM4.py


"""

import Queue
import binascii
import crcmod
from datetime import datetime
import logging
import os
import serial
import struct
import threading
from time import sleep


class Gps:
    """Novatel OEM4 GPS library class

    This class contents is an approach to create a library for Novatel OEM 4 GPS

    Args:
        sensorName (optional): A sensor name if used with multiple devices.


    Attributes:
            header_keys: all field keys for the headers of messages.
            MessageID: A dictionary for the types of messages sent. Not all are
                implemented yet!

    """
    #: all field keys for the headers of messages.
    header_keys = ('sync', 'headerLength', 'messageID', 'messageType',
                   'portAddress', 'messageLength', 'sequence', 'idleTime',
                   'timeStatus', 'week', 'ms', 'receiverStatus', 'reserved',
                   'swVersion')
    #: A dictionary for the types of messages sent. Not all are implemented yet!
    MessageID = {'LOG': 1,
                 'COM': 4,
                 'RESET': 18,
                 'SAVECONFIG': 19,
                 'UNLOG': 36,
                 'UNLOGALL': 38,
                 'DATUM': 160,
                 'DYNAMICS': 258,
                 'BESTXYZ': 241,
                 'SBASCONTROL': 652}

    def __init__(self, sensorName="GPS"):
        self.myPort = ""
        self.isOpen = 0  # is port open?*/
        self.baudRate = 9600  # current communication baudRate*/
        self.openError = 0  # if any error during any process occurs this will be set */
        self.dataQueue = None
        self.name = sensorName
        self.isSet = False
        self.exitFlag = threading.Event()
        self.orders = Queue.Queue()
        self.current_header = []
        self.Indice = 1

    def CRC32Value(self, i):
        '''Calculate the 32bits CRC of message.

        See OEMStar Firmware Reference Manual Rev 6 page 24 for
        more information.

        Args:
            i: message to calculate the crc-32.

        Returns:
            The CRC value calculated over the input message.

        '''
        crc = crcmod.mkCrcFun(0x104C11DB7, 0, True, 0)
        return crc(i)

    def getDebugMessage(self, message):
        '''Create a string which contains all bytes represented as hex values

        Auxiliary function for helping with debug. Receives a binary message as
        input and convert it as a string with the hexdecimal representation.

        Args:
            message: message to be represented.

        Returns:
            A string of corresponding hex representation of message.

        '''
        debugMessage = (binascii.hexlify(message)).upper()
        debugMessage = [debugMessage[i:i + 2] for i in range(0, len(debugMessage), 2)]
        debugMessage = ' '.join('0x{}'.format(item) for item in debugMessage)
        return debugMessage

    def parseResponces(self):
        '''
        A thread  to parse responses from device
        '''
        # used definitions for keys
        bestxyz_keys = ('pSolStatus', 'posType', 'position', 'positionStd',
                        'velSolStatus', 'velType', 'velocity', 'velocityStd',
                        'stnID', 'vLatency', 'diffAge', 'solAge', 'numStasVs',
                        'numSolSatVs', 'numGGL1', 'reserved', 'extSolStat',
                        'reserved2', 'sigMask', 'crc32')

        self.log.info("Entering Thread logger")
        if(not self.isOpen):
            self.log.warning('Port is not open: {0}'.format(self.myPort))
            self.log.info("Exiting Thread logger")
            return
        MYPORT = self.myPort
        # dataFile = self.dataFile
        while(self.exitFlag.isSet() == False):
            header = [0] * 14
            newByte = ord(MYPORT.read(1))
            if newByte == 0xAA:
                header[0] = [0, 0, 0]
                header[0][0] = newByte
                header[0][1] = ord(MYPORT.read(1))
                if header[0][1] == 0x44:
                    header[0][2] = ord(MYPORT.read(1))
                    if header[0][2] == 0x12:
                        # got a valid header sync vector
                        serialBuffer = MYPORT.read(25)
                        header[1:] = struct.unpack('<BHBBHHBBHlLHH',
                                                   serialBuffer)
                        self.current_header = dict(zip(self.header_keys, header))
                        header = self.current_header
                        if header['messageID'] == self.MessageID['LOG']:
                            message = [0] * 3
                            message_keys = ('responseID', 'ascii', 'crc32')
                            serialBuffer = MYPORT.read(4)
                            message[0] = struct.unpack('<I', serialBuffer)
                            message[1] = MYPORT.read(self.current_header['messageLength'] - 4)
                            message[2] = MYPORT.read(4)
                            message[2] = struct.unpack('<L', message[2])
                            message = dict(zip(message_keys, message))
                            self.log.info("LOG response received : {0}".format(message['ascii']))
                            self.orders.put({'order': 'LOG', 'data': message})
                        elif header['messageID'] == self.MessageID['UNLOGALL']:
                            message = [0] * 3
                            message_keys = ('responseID', 'ascii', 'crc32')
                            serialBuffer = MYPORT.read(4)
                            message[0] = struct.unpack('<I', serialBuffer)
                            message[1] = MYPORT.read(self.current_header['messageLength'] - 4)
                            message[2] = MYPORT.read(4)
                            message[2] = struct.unpack('<L', message[2])
                            message = dict(zip(message_keys, message))
                            self.log.info("UNLOGALL response received : {0}".format(message['ascii']))
                            self.orders.put({'order': 'UNLOGALL', 'data': message})
                        elif header['messageID'] == self.MessageID['COM']:
                            message = [0] * 3
                            message_keys = ('responseID', 'ascii', 'crc32')
                            serialBuffer = MYPORT.read(4)
                            message[0] = struct.unpack('<I', serialBuffer)
                            message[1] = MYPORT.read(self.current_header['messageLength'] - 4)
                            message[2] = MYPORT.read(4)
                            message[2] = struct.unpack('<L', message[2])
                            message = dict(zip(message_keys, message))
                            self.log.info("COM response received : {0}".format(message['ascii']))
                            self.orders.put({'order': 'COM', 'data': message})
                        elif header['messageID'] == self.MessageID['DYNAMICS']:
                            # is message responce to command setDynamics?
                            if header['messageType'] == 130:
                                message = [0] * 3
                                message_keys = ('responseID', 'ascii', 'crc32')
                                serialBuffer = MYPORT.read(4)
                                message[0] = struct.unpack('<I', serialBuffer)
                                message[1] = MYPORT.read(self.current_header['messageLength'] - 4)
                                message[2] = MYPORT.read(4)
                                message[2] = struct.unpack('<L', message[2])
                                message = dict(zip(message_keys, message))
                                self.log.info("DYNAMICS response received : {0}".format(message['ascii']))
                                self.orders.put({'order': 'DYNAMICS', 'data': message})
                            else:
                                # is a log dynamic response
                                message = [0] * 2
                                message_keys = ('dynamicID', 'crc32')
                                serialBuffer = MYPORT.read(4)
                                message[0] = struct.unpack('<L', serialBuffer)
                                message[1] = MYPORT.read(4)
                                message[1] = struct.unpack('<L', message[1])
                                message = dict(zip(message_keys, message))
                                self.log.info("DYNAMICS response received : dynamicID={0}".format(message['dynamicID'][0]))
                        elif header['messageID'] == self.MessageID['RESET']:
                            message = [0] * 3
                            message_keys = ('responseID', 'ascii', 'crc32')
                            serialBuffer = MYPORT.read(4)
                            message[0] = struct.unpack('<I', serialBuffer)
                            message[1] = MYPORT.read(self.current_header['messageLength'] - 4)
                            message[2] = MYPORT.read(4)
                            message[2] = struct.unpack('<L', message[2])
                            message = dict(zip(message_keys, message))
                            self.log.info("RESET response received : {0}".format(message['ascii']))
                            self.orders.put({'order': 'RESET', 'data': message})
                        elif header['messageID'] == self.MessageID['SAVECONFIG']:
                            message = [0] * 3
                            message_keys = ('responseID', 'ascii', 'crc32')
                            serialBuffer = MYPORT.read(4)
                            message[0] = struct.unpack('<I', serialBuffer)
                            message[1] = MYPORT.read(self.current_header['messageLength'] - 4)
                            message[2] = MYPORT.read(4)
                            message[2] = struct.unpack('<L', message[2])
                            message = dict(zip(message_keys, message))
                            self.log.info("SAVECONFIG response received : {0}".format(message['ascii']))
                            self.orders.put({'order': 'SAVECONFIG', 'data': message})
                        elif header['messageID'] == self.MessageID['SBASCONTROL']:
                            message = [0] * 3
                            message_keys = ('responseID', 'ascii', 'crc32')
                            serialBuffer = MYPORT.read(4)
                            message[0] = struct.unpack('<I', serialBuffer)
                            message[1] = MYPORT.read(self.current_header['messageLength'] - 4)
                            message[2] = MYPORT.read(4)
                            message[2] = struct.unpack('<L', message[2])
                            message = dict(zip(message_keys, message))
                            self.log.info("SBASCONTROL response received : {0}".format(message['ascii']))
                            self.orders.put({'order': 'SBASCONTROL', 'data': message})
                        elif header['messageID'] == self.MessageID['BESTXYZ']:
                            message = [0] * 20
                            serialBuffer = MYPORT.read(self.current_header['messageLength'])
                            message[0:2] = struct.unpack('<II', serialBuffer[0:8])
                            message[2] = [0, 0, 0]
                            message[2][0:] = struct.unpack('<ddd', serialBuffer[8:32])
                            message[3] = [0, 0, 0]
                            message[3][0:] = struct.unpack('<fff', serialBuffer[32:44])
                            message[4:6] = struct.unpack('<II', serialBuffer[44:52])
                            message[6] = [0, 0, 0]
                            message[6][0:] = struct.unpack('<ddd', serialBuffer[52:76])
                            message[7] = [0, 0, 0]
                            message[7][0:] = struct.unpack('<fff', serialBuffer[76:88])
                            message[8] = serialBuffer[88:92]
                            message[9:12] = struct.unpack('<fff', serialBuffer[92:104])
                            message[12:15] = struct.unpack('<3B', serialBuffer[104:107])
                            message[15] = [0, 0]
                            message[15][0:] = struct.unpack('<2B', serialBuffer[107:109])
                            message[16:19] = struct.unpack('<3B', serialBuffer[109:112])
                            serialBuffer = MYPORT.read(4)  # crc32
                            message[19] = struct.unpack('<I', serialBuffer)
                            message = dict(zip(bestxyz_keys, message))
                            currentTime = datetime.now()
                            myTime = '{0:%Y-%m-%d %H:%M:%S}'.format(currentTime) + '.{0:02.0f}'.format(round(currentTime.microsecond / 10000.0))
                            outMessage = dict(Indice=self.Indice, Time=myTime)
                            outMessage.update(message)
                            self.dataQueue.put_nowait(outMessage)

                            self.Indice = self.Indice + 1
                        else:
                            # .. todo:: error processing.
                            pass
            else:
                self.log.debug("New Byte unexpected: 0x{0:X}".format(newByte))
        self.log.info("Exiting Thread logger")
        return

    def begin(self, dataQueue,
              comPort="/dev/ttyUSB0",
              baudRate=9600):
        ''' Initializes the gps receiver.

        This function resets the current port to factory default and setup the
        gps receiver to be able to acept new commands. If connection to gps
        is made, it launchs a thread used to parse messages comming from gps.

        Args:
            comPort: system port where receiver is connected.
            dataQueue: a Queue object to store incoming bestxyz messages.
            baudRate: baudrate to configure port. (should always be equal to
                factory default of receiver).

        Returns:
            True or False if the setup has gone as expected or not.

        :Example:
          .. code-block:: python

              Gps.begin(comPort="<port>",
                  dataQueue=<your Queue obj>,
                  baudRate=9600)

        **Default values**

        :comPort:  "/dev/ttyUSB0"
        :baudRate:  9600

        .. warning::
            This class uses module ``logging`` wich must be configured in your
            main program using the ``basicConfig`` method. Check documentation
            of `module logging`_ for more info.

        **HW info:**

        :Receptor: Novatel Flexpak G2L-3151W.
        :Antenna: Novatel Pinwheel.

        .. _module logging: https://docs.python.org/2/library/logging.html

        '''

        self.log = logging.getLogger(self.name)

        # checking if port exists on system
        if not os.path.exists(comPort):
            self.log.warning('Port is not available: {0}'.format(comPort))
            return False
        else:
            # port exists, open it
            self.myPort = serial.Serial(comPort, baudrate=baudRate)
            if not self.myPort.is_open:
                self.log.warning("Error opening port: {0}".format(comPort))
                self.isOpen = False
                return False
            # reset port settings to default
            self.myPort.break_condition = True
            self.myPort.send_break()
            sleep(1.5)
            self.myPort.send_break()
            sleep(0.25)
            self.baudRate = baudRate
            self.isOpen = True
            # open dataFile to save GPS data
            self.dataQueue = dataQueue
            # start thread to handle GPS responces
            self.threadID = threading.Thread(name="Logger", target=self.parseResponces)
            self.threadID.start()
            self.log.info("Started Logger Thread")
            sleep(0.1)
            return True

    def create_header(self, messageID, messageLength, portAddress=192):
        '''Creates a header object to be passed to receiver.

        Args:
            messageID: the corresponding value of identifying the message body.
            messageLength: size of message in bytes excluding CRC-32bit code.
            portAddress: port from where message request is sent.

        Returns:
            The header of message.

        The header is defined as:

        +-----+--------------+-----------+-----------------------------+
        |Field| Value        | N Bytes   | Description                 |
        +=====+==============+===========+=============================+
        |1    | sync[0]      | UChar = 1 | Hexadecimal 0xAA.           |
        +-----+--------------+-----------+-----------------------------+
        |2    | sync[1]      | UChar = 1 | Hexadecimal 0x44.           |
        +-----+--------------+-----------+-----------------------------+
        |3    | sync[2]      | UChar = 1 | Hexadecimal 0x12.           |
        +-----+--------------+-----------+-----------------------------+
        |4    | headerLength | UChar = 1 |Length of the header (should |
        |     |              |           |always be 28 unless some     |
        |     |              |           |firmware update)             |
        +-----+--------------+-----------+-----------------------------+
        |5    | messageID    | UShort = 2| This is the Message ID code |
        +-----+--------------+-----------+-----------------------------+
        |6    | messageType  | UChar = 1 |message type mask (binary and|
        |     |              |           |original message)            |
        +-----+--------------+-----------+-----------------------------+
        |7    | portAddress  | Uchar = 1 |Corresponding value of port  |
        +-----+--------------+-----------+-----------------------------+
        |8    | messageLength| UShort = 2|Length of message body       |
        +-----+--------------+-----------+-----------------------------+
        |9    | sequence     | UShort = 2|This is used for multiple    |
        |     |              |           |related logs.                |
        +-----+--------------+-----------+-----------------------------+
        |10   | idleTime     | UChar = 1 |The time that the processor  |
        |     |              |           |is idle in the last second   |
        |     |              |           |between successive logs with |
        |     |              |           |the same Message ID          |
        +-----+--------------+-----------+-----------------------------+
        |11   | timeStatus   | Enum = 1  |Indicates the quality of the |
        |     |              |           |GPS time                     |
        +-----+--------------+-----------+-----------------------------+
        |12   | week         | UShort = 2| GPS week number.            |
        +-----+--------------+-----------+-----------------------------+
        |13   | ms           | int = 4   |Milliseconds from the        |
        |     |              |           |beginning of the GPS week.   |
        +-----+--------------+-----------+-----------------------------+
        |14   |receiverStatus| Ulong = 4 |32 bits representing the     |
        |     |              |           |status of various hardware   |
        |     |              |           |and software components of   |
        |     |              |           |the receiver.                |
        +-----+--------------+-----------+-----------------------------+
        |15   | reserved     | UShort = 2|                             |
        +-----+--------------+-----------+-----------------------------+
        |16   | swVersion    | UShort = 2|receiver software build      |
        |     |              |           |number.                      |
        +-----+--------------+-----------+-----------------------------+


        .. note::  portAddress=192 (equal to thisport)

        '''
        header = [0] * 16
        # First 3 start bytes are always.
        header[0] = 0xAA
        header[1] = 0x44
        header[2] = 0x12
        # header Length 1C
        header[3] = 0x1C
        # messageID
        header[4] = messageID
        # messageType binary and original responce = b000xxxxx = 0x02?
        header[5] = 0x02
        # port address
        header[6] = portAddress
        # message length in bytes
        header[7] = messageLength
        return header

    def sendUnlogall(self, port=8, held=1):
        '''Send command unlogall to gps device.

        On sucess clears all logs on all ports even held logs.

        Returns:
          True or False if the request has gone as expected or not.

        unlogall message is defined as:

        +------+------------+----------+-------------------------------+
        | Field| value      | N  Bytes | Description                   |
        +======+============+==========+===============================+
        |1     | header     | H = 28   | Header of message             |
        +------+------------+----------+-------------------------------+
        |2     | port       | ENUM = 4 | identification of port        |
        +------+------------+----------+-------------------------------+
        |3     | Held       | ENUM = 4 | can only be 0 or 1. Clear logs|
        |      |            |          | with hold flag or not?        |
        +------+------------+----------+-------------------------------+
        | CRC32|            | UL = 4   |                               |
        +------+------------+----------+-------------------------------+

        .. note:: See: OEMStar Firmware Reference Manual Rev 6 page 161

        '''
        if self.isOpen:
            MYPORT = self.myPort

            messageSize = 8  # 2 * ENUM
            header = self.create_header(messageID=38,
                                        messageLength=messageSize)
            myMessage = struct.pack('<BBBBHBBHHBBHlLHH', *header)
            myMessage = myMessage + struct.pack('<LL', port, held)
            crc_value = self.CRC32Value(myMessage)
            finalMessage = myMessage + struct.pack('<L', crc_value)

            # print messages to logFile
            self.log.info("Requested unlogall")
            self.log.debug('Message sent to GPS: {0}'.format(self.getDebugMessage(finalMessage)))
            MYPORT.write(finalMessage)
            MYPORT.flush()

            # wait for data on queue with response
            message = self.orders.get()
            if message['order'] == 'UNLOGALL':
                message = message['data']
                self.log.info("Unlogall response received : {0}".format(message['ascii']))
                if message['responseID'][0] == 1:
                    return True
                else:
                    return False
            else:
                self.log.warning("Unexpected responce type: {0}".format(message['order']))
        else:
            self.log.info("Port not open. Couldn't request unlogall command")
            return False

    def setCom(self, baud, port=6, parity=0, databits=8, stopbits=1,
               handshake=0, echo=0, breakCond=1):
        '''Set com configuration.

        Args:
            baud: communication baudrate.
            port: Novatel serial ports identifier (default 6 = "thisport").
            parity: byte parity check (default 0).
            databits: Number of data bits (default 8).
            stopbits: Number of stop bits (default 1).
            handshake: Handshaking (default No handshaking).
            echo: echo input back to user (default false)
            breakCond: Enable break detection (default true)

        Return:
            True or false if command was sucessfull or not.

        The com request command is defined as:

        +-----+------------+----------+----------------------------------+
        |Field| ID         | N  Bytes | Description                      |
        +=====+============+==========+==================================+
        |1    | Com header | H = 28   | Header of message                |
        +-----+------------+----------+----------------------------------+
        |2    | port       | ENUM = 4 | identification of port           |
        +-----+------------+----------+----------------------------------+
        |3    | baud       | Ulong = 4| Communication baud rate (bps)    |
        +-----+------------+----------+----------------------------------+
        |4    | parity     | ENUM = 4 | Parity                           |
        +-----+------------+----------+----------------------------------+
        |5    | databits   | Ulong = 4| Number of data bits (default = 8)|
        +-----+------------+----------+----------------------------------+
        |6    | stopbits   | Ulong = 4| Number of stop bits (default = 1)|
        +-----+------------+----------+----------------------------------+
        |7    | handshake  | ENUM = 4 | Handshaking                      |
        +-----+------------+----------+----------------------------------+
        |8    | echo       | ENUM = 4 | No echo (default)(must be 0 or 1)|
        +-----+------------+----------+----------------------------------+
        |9    | break      | ENUM = 4 |Enable break detection (default 0)|
        |     |            |          |,(must be 0 or 1)                 |
        +-----+------------+----------+----------------------------------+

        .. note:: Total byte size = header + 32 = 60 bytes

        COM Serial Port Identifiers (field 2):

        +-------+--------------+--------------------+
        |Binary |ASCII         |Description         |
        +=======+==============+====================+
        |1      | COM1         |COM port 1          |
        +-------+--------------+--------------------+
        |2      | COM2         |COM port 2          |
        +-------+--------------+--------------------+
        |6      | THISPORT     |The current COM port|
        +-------+--------------+--------------------+
        |8      | ALL          |All COM ports       |
        +-------+--------------+--------------------+
        |9      | XCOM1        |Virtual COM1 port   |
        +-------+--------------+--------------------+
        |10     | XCOM2        |Virtual COM2 port   |
        +-------+--------------+--------------------+
        |13     | USB1         |USB port 1          |
        +-------+--------------+--------------------+
        |14     | USB2         |USB port 2          |
        +-------+--------------+--------------------+
        |15     | USB3         |USB port 3          |
        +-------+--------------+--------------------+
        |17     | XCOM3        |Virtual COM3 port   |
        +-------+--------------+--------------------+

        Parity(field 4):

        +-------+-----------+-----------------------+
        |Binary |ASCII      |Description            |
        +=======+===========+=======================+
        |0      | N         | No parity (default)   |
        +-------+-----------+-----------------------+
        |1      | E         | Even parity           |
        +-------+-----------+-----------------------+
        |2      | O         | Odd parity            |
        +-------+-----------+-----------------------+

        Handshaking (field 7):

        +-------+---------+-------------------------------+
        |Binary |ASCII    |Description                    |
        +=======+=========+===============================+
        |0      |N        |No handshaking (default)       |
        +-------+---------+-------------------------------+
        |1      |XON      |XON/XOFF software handshaking  |
        +-------+---------+-------------------------------+
        |2      |CTS      |CTS/RTS hardware handshaking   |
        +-------+---------+-------------------------------+

        .. note:: See: OEMStar Firmware Reference Manual Rev 6 page 56

        '''
        if self.isOpen:
            MYPORT = self.myPort
            messageSize = 32  # 32bytes length
            header = self.create_header(messageID=4, messageLength=messageSize)
            myMessage = struct.pack('<BBBBHBBHHBBHlLHH', *header)
            myMessage = myMessage + struct.pack('<8L', port, baud, parity, databits,
                                                stopbits, handshake, echo, breakCond)
            crc_value = self.CRC32Value(myMessage)
            finalMessage = myMessage + struct.pack('<L', crc_value)
            self.log.info("Requested Com command")
            self.log.debug('Message sent to GPS: {0}'.format(self.getDebugMessage(finalMessage)))
            MYPORT.write(finalMessage)
            MYPORT.flush()
            self.log.info("waiting for port settings to change")
            sleep(1)
            portOptions = MYPORT.get_settings()
            # change port settings
            portOptions['baudrate'] = baud
            # auxiliar vector
            parity_vect = [serial.PARITY_NONE, serial.PARITY_EVEN, serial.PARITY_ODD]
            portOptions['parity'] = parity_vect[parity]
            portOptions['bytesize'] = databits
            portOptions['stopbits0'] = stopbits
            if handshake == 0:
                portOptions['xonxoff'] = False
                portOptions['rtscts'] = False
            elif handshake == 1:
                portOptions['xonxoff'] = True
                portOptions['rtscts'] = False
            elif handshake == 2:
                portOptions['xonxoff'] = False
                portOptions['rtscts'] = True
            MYPORT.apply_settings(portOptions)
            MYPORT.reset_input_buffer()
            MYPORT.reset_output_buffer()
            if MYPORT.is_open:
                self.log.info("changed port settings: {0}".format(MYPORT.portstr))
                return True
            else:
                self.log.warning("Not able to change port settings: {0}".format(MYPORT.portstr))
                return False
        else:
            self.log.info("Port not open. Couldn't request COM command")
            return False

    def askLog(self, logID='BESTXYZ', port=192, trigger=4, period=0, offset=0, hold=0):
        '''Request a log from receiver.

        Args:
            logID: log type to request.
            port: port to report log.
            trigger: trigger identifier.
            period: the period of log.
            offset: offset in seconds after period.
            hold: mark log with hold flag or not.

        Returns:
            True or false if command was sucessfull or not.

        The log request command is defined as:

        +-----+------------+--------------+-----------------------------------+
        |Field| ID         | N  Bytes     | Description                       |
        +=====+============+==============+===================================+
        |1    | Com header | H = 28       | Header of message                 |
        +-----+------------+--------------+-----------------------------------+
        |2    | port       | ENUM = 4     | identification of port            |
        +-----+------------+--------------+-----------------------------------+
        |3    | message    | Ushort = 2   | Message ID of log to output       |
        +-----+------------+--------------+-----------------------------------+
        |4    | messageType| char = 1     | Message type (Binary)             |
        +-----+------------+--------------+-----------------------------------+
        |5    | RESERVED   | char = 1     |                                   |
        +-----+------------+--------------+-----------------------------------+
        |6    | trigger    | ENUM = 4     | message trigger                   |
        +-----+------------+--------------+-----------------------------------+
        |7    | period     | double = 8   | Log period (for ONTIME in secs)   |
        +-----+------------+--------------+-----------------------------------+
        |8    | offset     | double = 8   | Offset for period (ONTIME in secs |
        +-----+------------+--------------+-----------------------------------+
        |9    | hold       | ENUM = 4     | Hold log                          |
        +-----+------------+--------------+-----------------------------------+
        |10   | crc32      | Ulong = 4    | crc32 value                       |
        +-----+------------+--------------+-----------------------------------+

        .. note:: Total byte size = header + 32 = 60 bytes

        Log trigger Identifiers (field 6):

        +-------+-----------+-------------------------------------------------+
        |Binary |ASCII      |Description                                      |
        +=======+===========+=================================================+
        |0      | ONNEW     | | when the message is updated (not necessarily  |
        |       |           | | changed)                                      |
        +-------+-----------+-------------------------------------------------+
        |1      | ONCHANGED | | Current message and then continue to output   |
        |       |           | | when the message is changed                   |
        +-------+-----------+-------------------------------------------------+
        |2      | ONTIME    |Output on a time interval                        |
        +-------+-----------+-------------------------------------------------+
        |3      | ONNEXT    |Output only the next message                     |
        +-------+-----------+-------------------------------------------------+
        |4      | ONCE      |Output only the current message                  |
        +-------+-----------+-------------------------------------------------+
        |5      | ONMARK    | | Output when a pulse is detected on the mark 1 |
        |       |           | | input                                         |
        +-------+-----------+-------------------------------------------------+

        '''
        if self.isOpen:
            MYPORT = self.myPort
            messageSize = 32
            header = self.create_header(messageID=1, messageLength=messageSize)
            myMessage = struct.pack('<BBBBHBBHHBBHlLHH', *header)
            myMessage = myMessage + struct.pack('<LHBBLddL', port, self.MessageID[logID],
                                                0, 0, trigger, period, offset, hold)
            crc_value = self.CRC32Value(myMessage)
            finalMessage = myMessage + struct.pack('<L', crc_value)
            self.log.info("Requested Log command")
            self.log.debug('Message sent to GPS: {0}'.format(self.getDebugMessage(finalMessage)))
            MYPORT.write(finalMessage)
            MYPORT.flush()
            # wait for responce
            message = self.orders.get()
            if message['order'] == 'LOG':
                message = message['data']
                self.log.info("LOG response received : {0}".format(message['ascii']))
                if message['responseID'][0] == 1:
                    return True
                else:
                    return False
            else:
                self.log.warning("Unexpected responce type: {0}".format(message['order']))
        else:
            self.log.info("Port not open. Couldn't request LOG command")
            return False

    def setDynamics(self, dynamicID):
        '''Set Dynamics of receiver.

        Args:
            dynamicID: identifier of the type of dynamic.

        Returns:
          True or False if the request has gone as expected or not.

        dynamics message is defined as:

        +------+------------+----------+-------------------------------+
        | Field| value      | N  Bytes | Description                   |
        +======+============+==========+===============================+
        |1     | header     | H = 28   | Header of message             |
        +------+------------+----------+-------------------------------+
        |2     | dynamics   | ENUM = 4 | identification of dynamics    |
        +------+------------+----------+-------------------------------+
        | CRC32|            | UL = 4   |                               |
        +------+------------+----------+-------------------------------+

        The dynamics identifiers (field 2) are defined as:

        +-------+--------+------------------------------------------------+
        |Binary |ASCII   |Description                                     |
        +=======+========+================================================+
        |0      | AIR    | | Receiver is in an aircraft or a land vehicle,|
        |       |        | | for example a high speed train, with velocity|
        |       |        | | greater than 110 km/h (30 m/s). This is also |
        |       |        | | the most suitable dynamic for a jittery      |
        |       |        | | vehicle at any speed.                        |
        +-------+--------+------------------------------------------------+
        |1      | LAND   | | Receiver is in a stable land vehicle with    |
        |       |        | | velocity less than 110 km/h (30 m/s).        |
        +-------+--------+------------------------------------------------+
        |2      | FOOT   | | Receiver is being carried by a person with   |
        |       |        | | velocity less than 11 km/h (3 m/s).          |
        +-------+--------+------------------------------------------------+

        This command adjusts the receiver dynamics to that of your environment.
        It is used to optimally tune receiver parameters.
        The DYNAMICS command adjusts the Tracking State transition time-out
        value of the receiver.
        When the receiver loses the position solution, it attempts to steer the
        tracking loops for fast reacquisition (5 s time-out by default).
        The DYNAMICS command allows you to adjust this time-out value,
        effectively increasing the steering time. The three states 0, 1, and 2
        set the time-out to 5, 10, or 20 seconds respectively.

        .. note::
            * The DYNAMICS command should only be used by advanced users of GPS.
              The default of AIR should not be changed except under very
              specific conditions.
            * The DYNAMICS command affects satellite reacquisition. The
              constraint of the DYNAMICS filter with FOOT is very tight and is
              appropriate for a user on foot. A sudden tilted or up and down
              movement, for example while a tractor is moving slowly along a
              track, may trip the RTK filter to reset and cause the position to
              jump. AIR should be used in this case.

        '''
        if self.isOpen:
            if dynamicID in [0, 1, 2]:
                MYPORT = self.myPort
                messageSize = 4  # ENUM
                header = self.create_header(messageID=self.MessageID['DYNAMICS'],
                                            messageLength=messageSize)
                myMessage = struct.pack('<BBBBHBBHHBBHlLHH', *header)
                myMessage = myMessage + struct.pack('<L', dynamicID)
                crc_value = self.CRC32Value(myMessage)
                finalMessage = myMessage + struct.pack('<L', crc_value)

                # print messages to logFile
                self.log.info("Requested dynamics")
                self.log.debug('Message sent to GPS: {0}'.format(self.getDebugMessage(finalMessage)))
                MYPORT.write(finalMessage)
                MYPORT.flush()

                # wait for data on queue with response
                message = self.orders.get()
                if message['order'] == 'DYNAMICS':
                    message = message['data']
                    self.log.info("DYNAMICS response received : {0}".format(message['ascii']))
                    if message['responseID'][0] == 1:
                        return True
                    else:
                        return False
                else:
                    self.log.warning("Unexpected responce type: {0}".format(message['order']))
            else:
                self.log.info("dynamicID not valid")
                return False
        else:
            self.log.info("Port not open. Couldn't request DYNAMICS command")
            return False

#     def setPDPFilter(self, switchID):
#         ''' set PDPFilter type
#
#         Args:
#             switchID: Enable, disable or reset.
#
#         Returns:
#             A boolean if request was sucessful or not
#
#         PDPFilter message is defined as:
#
#         +------+------------+----------+-------------------------------+
#         | Field| value      | N  Bytes | Description                   |
#         +======+============+==========+===============================+
#         |1     | header     | H = 28   | Header of message             |
#         +------+------------+----------+-------------------------------+
#         |2     | switchID   | ENUM = 4 | Enable, disable or reset.     |
#         +------+------------+----------+-------------------------------+
#         | CRC32|            | UL = 4   |                               |
#         +------+------------+----------+-------------------------------+
#
#         the switchID is defined as:
#
#         +-------+--------+------------------------------------------------+
#         |Binary |ASCII   |Description                                     |
#         +=======+========+================================================+
#         |0      | DISABLE| | Enable/disable/reset the PDP filter. A reset |
#         +-------+--------+ | clears the filter memory so that the pdp     |
#         |1      | ENABLE | | filter can start over.                       |
#         +-------+--------+                                                |
#         |2      | Reset  |                                                |
#         +-------+--------+------------------------------------------------+
#
#         This command enables, disables or resets the Pseudorange/Delta-Phase
#         (PDP) filter. The main advantages of the Pseudorange/Delta-Phase
#         (PDP) implementation are:
#
#         * Smooths a jumpy position
#         * Bridges outages in satellite coverage (the solution is degraded from
#           normal but there is at least a reasonable solution without gaps)
#
#         .. note::
#             For channel configurations that include GPS, PDP is enabled by
#             default on the OEMStar.
#             With PDP enabled (default), the BESTPOS log is not updated until the
#             receiver has achieved FINESTEERING.
#             PDP and GLIDE are disabled for GLONASS-only applications.
#             Enable the PDP filter to output the PDP solution in BESTPOS, BESTVEL
#             and NMEA logs.
#
#         '''
#         if self.isOpen:
#             if switchID in [0, 1, 2]:
#                 MYPORT = self.myPort
#                 messageSize = 4  # ENUM
#                 header = self.create_header(messageID=self.MessageID('PDPFILTER'),
#                                             messageLength=messageSize)
#                 myMessage = struct.pack('<BBBBHBBHHBBHlLHH', *header)
#                 myMessage = myMessage + struct.pack('<L', switchID)
#                 crc_value = self.CRC32Value(myMessage)
#                 finalMessage = myMessage + struct.pack('<L', crc_value)
#
#                 # print messages to logFile
#                 self.log.info("Requested PDPFilter")
#                 self.log.debug('Message sent to GPS: {0}'.format(self.getDebugMessage(finalMessage)))
#                 MYPORT.write(finalMessage)
#                 MYPORT.flush()
#
#                 # wait for data on queue with response
#                 message = self.orders.get()
#                 if message['order'] == 'PDPFILTER':
#                     message = message['data']
#                     self.log.info("PDPFilter response received : {0}".format(message['ascii']))
#                     if message['responseID'][0] == 1:
#                         return True
#                     else:
#                         return False
#                 else:
#                     self.log.warning("Unexpected responce type: {0}".format(message['order']))
#             else:
#                 self.log.info("switchID not valid")
#                 return False
#         else:
#             self.log.info("Port not open. Couldn't request PDPFILTER command")
#             return False
#
#     def setPDPMode(self, mode, dynamics):
#         pass

    def reset(self, delay=0):
        ''' Performs a hardware reset

        Args:
            delay: seconds to wait before resetting. Default to zero.

        Returns:
            A boolean if request was sucessful or not

        The reset message is defined as:

        +------+------------+----------+-------------------------------+
        | Field| value      | N  Bytes | Description                   |
        +======+============+==========+===============================+
        |1     | header     | H = 28   | Header of message             |
        +------+------------+----------+-------------------------------+
        |2     | delay      | UL = 4   | Seconds to wait before reset  |
        +------+------------+----------+-------------------------------+
        | CRC32|            | UL = 4   |                               |
        +------+------------+----------+-------------------------------+

        Following a RESET command, the receiver initiates a coldstart boot up.
        Therefore, the receiver configuration reverts either to the factory
        default, if no user configuration was saved, or the last SAVECONFIG
        settings.
        The optional delay field is used to set the number of seconds the
        receiver is to wait before resetting.

        '''
        if self.isOpen:
            MYPORT = self.myPort
            messageSize = 4  # Ulong
            header = self.create_header(messageID=self.MessageID('RESET'),
                                        messageLength=messageSize)
            myMessage = struct.pack('<BBBBHBBHHBBHlLHH', *header)
            myMessage = myMessage + struct.pack('<L', delay)
            crc_value = self.CRC32Value(myMessage)
            finalMessage = myMessage + struct.pack('<L', crc_value)

            # print messages to logFile
            self.log.info("Requested Reset")
            self.log.debug('Message sent to GPS: {0}'.format(self.getDebugMessage(finalMessage)))
            MYPORT.write(finalMessage)
            MYPORT.flush()

            # wait for data on queue with response
            message = self.orders.get()
            if message['order'] == 'RESET':
                message = message['data']
                self.log.info("RESET response received : {0}".format(message['ascii']))
                if message['responseID'][0] == 1:
                    return True
                else:
                    return False
            else:
                self.log.warning("Unexpected responce type: {0}".format(message['order']))
        else:
            self.log.info("Port not open. Couldn't request RESET command")
            return False

    def saveconfig(self):
            ''' Save user current configuration

            Returns:
                A boolean if request was sucessful or not

            Saveconfig message is defined as:

            +------+------------+----------+-------------------------------+
            | Field| value      | N  Bytes | Description                   |
            +======+============+==========+===============================+
            |1     | header     | H = 28   | Header of message             |
            +------+------------+----------+-------------------------------+
            | CRC32|            | UL = 4   |                               |
            +------+------------+----------+-------------------------------+

            This command saves the user’s present configuration in non-volatile
            memory. The configuration includes the current log settings, FIX
            settings, port configurations, and so on. Its output is in the
            RXCONFIG log.
            '''
            if self.isOpen:
                MYPORT = self.myPort
                messageSize = 0  # Ulong
                header = self.create_header(messageID=self.MessageID('RESET'),
                                            messageLength=messageSize)
                myMessage = struct.pack('<BBBBHBBHHBBHlLHH', *header)
                crc_value = self.CRC32Value(myMessage)
                finalMessage = myMessage + struct.pack('<L', crc_value)

                # print messages to logFile
                self.log.info("Requested SAVECONFIG")
                self.log.debug('Message sent to GPS: {0}'.format(self.getDebugMessage(finalMessage)))
                MYPORT.write(finalMessage)
                MYPORT.flush()

                # wait for data on queue with response
                message = self.orders.get()
                if message['order'] == 'SAVECONFIG':
                    message = message['data']
                    self.log.info("SAVECONFIG response received : {0}".format(message['ascii']))
                    if message['responseID'][0] == 1:
                        return True
                    else:
                        return False
                else:
                    self.log.warning("Unexpected responce type: {0}".format(message['order']))
            else:
                self.log.info("Port not open. Couldn't request SAVCONFIG command")
                return False

    def sbascontrol(self, keywordID=1, systemID=1, prn=0, testmode=0):
            ''' Set SBAS test mode and PRN SBAS

            Args:
                keywordID: True or false. Control the reception of SBAS
                    corrections Enable = 1, Disable = 0.
                systemID: SBAS system to be used.
                prn: PRN corrections to be used.
                testmode: Interpretation of type 0 messages.

            Returns:
                A boolean if request was sucessful or not

            sbascontrol message is defined as:

            +------+------------+----------+-------------------------------+
            | Field| value      | N  Bytes | Description                   |
            +======+============+==========+===============================+
            |1     | header     | H = 28   | Header of message             |
            +------+------------+----------+-------------------------------+
            |2     | keyword    | Enum = 4 |Enable = 1 or Disable = 0      |
            +------+------------+----------+-------------------------------+
            |3     | system     | Enum = 4 |Choose the SBAS the receiver   |
            |      |            |          |will use                       |
            +------+------------+----------+-------------------------------+
            |4     | prn        | UL = 4   | 0 - Receiver will use any PRN |
            |      |            |          +-------------------------------+
            |      |            |          | 120~138 - Receiver will use   |
            |      |            |          | SBAS only from this PRN       |
            +------+------------+----------+-------------------------------+
            |5     | testmode   | Enum = 4 | Interpretation of type 0      |
            |      |            |          | messages                      |
            +------+------------+----------+-------------------------------+
            | CRC32|            | UL = 4   |                               |
            +------+------------+----------+-------------------------------+

            System (Field 2) is defined as:

            +-------+--------+------------------------------------------------+
            |Binary |ASCII   |Description                                     |
            +=======+========+================================================+
            |0      | NONE   | Don't use any SBAS satellites.                 |
            +-------+--------+------------------------------------------------+
            |1      | AUTO   | Automatically determinate satellite system to  |
            |       |        | use (default).                                 |
            +-------+--------+------------------------------------------------+
            |2      | ANY    | Use any and all SBAS satellites found          |
            +-------+--------+------------------------------------------------+
            |3      | WAAS   | Use only WAAS satellites                       |
            +-------+--------+------------------------------------------------+
            |4      | EGNOS  | Use only EGNOS satellites                      |
            +-------+--------+------------------------------------------------+
            |5      | MSAS   | Use only MSAS satellites                       |
            +-------+--------+------------------------------------------------+

            Testmode (field 5) is defined as:

            +-------+-----------+----------------------------------------------+
            |Binary |ASCII      |Description                                   |
            +=======+===========+==============================================+
            |0      | NONE      | Interpret Type 0 messages as they are        |
            |       |           | intended (as do not use).(default)           |
            +-------+-----------+----------------------------------------------+
            |1      | ZEROTOTWO | Interpret Type 0 messages as type 2 messages |
            +-------+-----------+----------------------------------------------+
            |2      | IGNOREZERO| Ignore the usual interpretation of Type 0    |
            |       |           | messages (as do not use) and continue        |
            +-------+-----------+----------------------------------------------+

            This command allows you to dictate how the receiver handles
            Satellite Based Augmentation System (SBAS) corrections and replaces
            the now obsolete WAASCORRECTION command. The receiver automatically
            switches to Pseudorange Differential (RTCM or RTCA) or RTK if the
            appropriate corrections are received, regardless of the current
            setting.
            '''
            if self.isOpen:
                MYPORT = self.myPort
                messageSize = 0  # Ulong
                header = self.create_header(messageID=self.MessageID('SBASCONTROL'),
                                            messageLength=messageSize)
                myMessage = struct.pack('<BBBBHBBHHBBHlLHH', *header)
                myMessage = myMessage + struct.pack('<LLLL', keywordID, systemID,
                                                    prn, testmode)
                crc_value = self.CRC32Value(myMessage)
                finalMessage = myMessage + struct.pack('<L', crc_value)

                # print messages to logFile
                self.log.info("Requested SBASCONTROL")
                self.log.debug('Message sent to GPS: {0}'.format(self.getDebugMessage(finalMessage)))
                MYPORT.write(finalMessage)
                MYPORT.flush()

                # wait for data on queue with response
                message = self.orders.get()
                if message['order'] == 'SBASCONTROL':
                    message = message['data']
                    self.log.info("SBASCONTROL response received : {0}".format(message['ascii']))
                    if message['responseID'][0] == 1:
                        return True
                    else:
                        return False
                else:
                    self.log.warning("Unexpected responce type: {0}".format(message['order']))
            else:
                self.log.info("Port not open. Couldn't request SBASCONTROL command")
                return False

    def shutdown(self):
        '''Prepare for exiting program

        Returns:
            always returns true after all tasks are done.

        Prepare for turn off the program by executing the following tasks:

        * unlogall
        * reset port settings
        * close port
        
        '''
        self.sendUnlogall()
        self.exitFlag.set()
        self.sendUnlogall()
        self.threadID.join()
        # reset port settings to default
        self.myPort.break_condition = True
        self.myPort.send_break()
        sleep(1.5)
        self.myPort.send_break()
        sleep(0.25)
        self.myPort.close()
        self.isOpen = False
        self.log.info("Shuting down")
        return True


def main():
    '''Set of test to run to see if class behaves as expected.

    Creates a Gps class object and execute the following commands on gps receiver:

    - begin: on default port or given port by argv[1].
    - sendUnlogall
    - setCom(baud=115200): changes baudrate to 115200bps
    - askLog(trigger=2, period=0.1): ask for log *bestxyz* with trigger `ONTIME` and period `0.1`
    - wait for 10 seconds
    - shutdown: safely disconnects from gps receiver

    '''
    import optparse

    def printData(dataQueue, exitFlag):
        ''' prints data to console

        Thread used to print data from request log (bestxyz) to the console.

        Args:
            dataQueue: queue class object where data is stored
            exitFlag: a flag to control the exit of program gracefully

        '''
        print("Indice,Time,PSolStatus,X,Y,Z,stdX,stdY,stdZ,VSolStatus,VX,VY,VZ,stdVX,stdVY,stdVZ,VLatency,SolAge,SolSatNumber\n")
        while(exitFlag.isSet() == False):
            if(dataQueue.empty() == False):
                newData = dataQueue.get()
                print('{0:5d},{1},{2},{3},{4},{5},'
                      '{6},{7},{8},{9},{10},{11},'
                      '{12},{13},{14},{15},{16},'
                      '{17},{18}\n'.format(newData['Indice'],
                                           newData['Time'],
                                           newData['pSolStatus'],
                                           newData['position'][0],
                                           newData['position'][1],
                                           newData['position'][2],
                                           newData['positionStd'][0],
                                           newData['positionStd'][1],
                                           newData['positionStd'][2],
                                           newData['velSolStatus'],
                                           newData['velocity'][0],
                                           newData['velocity'][1],
                                           newData['velocity'][2],
                                           newData['velocityStd'][0],
                                           newData['velocityStd'][1],
                                           newData['velocityStd'][2],
                                           newData['vLatency'],
                                           newData['solAge'],
                                           newData['numSolSatVs']
                                           ))
            else:
                sleep(0.1)
        return
    ############################################################################
    #
    # Start of main part
    #
    ############################################################################
    parser = optparse.OptionParser(usage="usage: %prog [options] args")
    parser.add_option("-p", "--port", action="store", type="string",
                      dest="port", default="/dev/ttyUSB0")
    parser.add_option("-n", "--name", action="store", type="string",
                      dest="name", default="GPS1")
    parser.add_option("--log", action="store", type="string",
                      dest="log", default="output.log")
    parser.add_option("--log-level", action="store", type="int",
                      dest="logLevel", default=20)
    (opts, args) = parser.parse_args()
    if len(args) > 4:
        parser.error("incorrect number of arguments")
        return

    logging.basicConfig(filename=opts.log,
                        level=opts.logLevel,
                        format='[%(asctime)s] [%(threadName)-10s] %(levelname)-8s %(message)s',
                        filemode="w")
    # event flag to exit
    exitFlag = threading.Event()
    # create a queue to receive comands
    dataFIFO = Queue.Queue()
    # create a thread to parse responses
    thread1 = threading.Thread(name="printData", target=printData,
                               args=(dataFIFO, exitFlag))
    thread1.start()
    # instanciate a class object
    gps = Gps(opts.name)
    # begin
    if(gps.begin(dataFIFO, comPort=opts.port) != 1):
        print("Not able to begin device properly... check logfile")
        return

    # send unlogall
    if(gps.sendUnlogall() != 1):
        print("Unlogall command failed... check logfile")
        gps.myPort.close()
        return
    # reconfigure port
    gps.setCom(baud=115200)
    # ask for bestxyz log
    gps.askLog(trigger=2, period=0.1)
    # wait 10 seconds
    sleep(10)
    # stop logs and shutdown gps
    gps.shutdown()
    # stop print thread
    exitFlag.set()
    thread1.join()
    logging.shutdown()
    # exit
    print('Exiting now')
    return


if __name__ == '__main__':
    main()
