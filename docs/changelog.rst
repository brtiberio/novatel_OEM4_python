=========
Changelog
=========
:version 0.4:
   Moved from optoparse to argparse module.
   changed Queue to make it compatible with python3 queue. Backwards compatibility is maintained.
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
