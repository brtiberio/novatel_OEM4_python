Welcome to NovatelOEM4 GPS Library's documentation!
===================================================

:Date: 24 Jul 2018

:Version: 0.4

:Author: Bruno Tibério

:Contact: bruno.tiberio@tecnico.ulisboa.pt

This is an attempt to create a library to communicate with Novatel Flexpak G2 GPS device as part of a project.
Is still a lot incomplete.

Read the documentation at: `NovatelOEM4 Python <http://novatel-oem4-python.readthedocs.io/en/latest/>`

=========
Changelog
=========

:version 0.4:
   Moved from optoparse to argparse module.
   Changed Queue to make it compatible with python3 queue. Backwards compatibility is maintained.
   Restructured default location. Moved from Lib folder to base path.
   Moved examples to proper folder. This cause backwards compatibility problems. On import, replace
   ``import Lib.NovatelOEM4`` with simply ``import NovatelOEM4``

:version 0.3:
   logging configuration as moved outside module to enable user to use already
   configured logging handler. Check `multimodule logging docs`_

.. _multimodule logging docs: https://docs.python.org/2/howto/logging-cookbook.html#using-logging-in-multiple-modules`

:version 0.2:
    data from bestxyz message is now placed into a Queue.Queue() FIFO

:version 0.1: initial release

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

**Example:**

.. code-block:: console

    $python NovatelOEM4.py