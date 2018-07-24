import signal
import threading
import logging
from time import sleep
import sys
# make it compatible with python 2.7
try:
    import queue
except ImportError:
    import Queue as queue

sys.path.append('..')
import NovatelOEM4 as Novatel


def main():
    import argparse

    def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        gps.shutdown()
        return

    def saveData(dataQueue, fileFP, exitFlag):

        fileFP.write("Index,Time,PSolStatus,X,Y,Z,stdX,stdY,stdZ,VSolStatus,VX,VY,VZ,stdVX,stdVY,stdVZ,VLatency,SolAge,SolSatNumber\n")
        fileFP.flush()
        while(exitFlag.isSet() == False):
            if(dataQueue.empty() == False):
                newData = dataQueue.get()
                fileFP.write('{0:5d},{1},{2},{3},{4},{5},'
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
                fileFP.flush()
            else:
                sleep(0.1)
        return

    # -------------------------------------------------------------------------
    # Start of the main program
    # -------------------------------------------------------------------------
    # add arguments options
    parser = argparse.ArgumentParser(add_help=True,
                                     description='Simple tester for Novatel library')
    parser.add_argument('-f', '--file', action='store', type=str, dest="filename",
                        default='./output.csv', help='file to save data')
    parser.add_argument('--log', action='store', type=str, dest='log', default='output.log',
                        help='log file to be used')

    parser.add_argument("--log-level", action="store", type=str,
                        dest="logLevel", default='info',
                        help='Log level to be used. See logging module for more info',
                        choices=['critical', 'error', 'warning', 'info', 'debug'])

    args = parser.parse_args()

    log_Level = {'error': logging.ERROR,
                'debug': logging.DEBUG,
                'info': logging.INFO,
                'warning': logging.WARNING,
                'critical': logging.CRITICAL
                }

    logging.basicConfig(filename=args.log,
                        level=log_Level[args.logLevel],
                        format='[%(asctime)s] [%(threadName)-10s] %(levelname)-8s %(message)s',
                        filemode="w")
    # ---------------------------------------------------------------------------
    # define a Handler which writes INFO messages or higher in console
    # ---------------------------------------------------------------------------
    console = logging.StreamHandler()
    console.setLevel(log_Level[args.logLevel])
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-20s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)
    # ---------------------------------------------------------------------------

    dataFile = args.filename
    # create a queue for data input
    dataQueue = queue.Queue()
    # open file for saving data input
    myFile = open(dataFile, 'w')
    # create a flag to signal we want to exit program
    exitFlag = threading.Event()
    # now define the thread
    threadID = threading.Thread(name="saveData", target=saveData,
                                args=(dataQueue, myFile, exitFlag))
    threadID.start()
    # prepare signal and handlers
    signal.signal(signal.SIGINT, signal_handler)
    gps = Novatel.Gps()
    if gps.begin(dataQueue) != 1:
        print("Not able to begin device properly... check logfile")
        return
    gps.setCom(baud=115200)
    # 0 Air 1 Land 2 foot
    # gps.setDynamics(2)
    gps.askLog(trigger=2, period=0.05)
    print('Press Ctrl+C to Exit')
    signal.pause()
    exitFlag.set()
    threadID.join()
    myFile.flush()
    myFile.close()
    logging.shutdown()
    print('Exiting now')
    return


if __name__ == '__main__':
    main()
