.. NovatleOEM4 GPS Library documentation master file, created by
   sphinx-quickstart on Fri Sep 30 16:28:14 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to NovatelOEM4 GPS Library's documentation!
===================================================

:Date: 07 May 2019

:Version: 0.4.2

:Author: Bruno Tibério

:Contact: bruno.tiberio@tecnico.ulisboa.pt

.. include:: changelog.rst

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


Contents:

.. toctree::
   :maxdepth: 4
   
   gps
   
