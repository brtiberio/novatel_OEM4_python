import signal
import Queue
import threading
import optparse
from time import sleep
import lib.NovatelOEM4 as Novatel


def main():

    def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        gps.shutdown()
        return

    def saveData(dataQueue, fileFP, exitFlag):

        fileFP.write("Indice,Time,PSolStatus,X,Y,Z,stdX,stdY,stdZ,VSolStatus,VX,VY,VZ,stdVX,stdVY,stdVZ,VLatency,SolAge,SolSatNumber\n")
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

    '''#########################################################################

    Start of the main program

    #########################################################################'''
    # add arguments options
    parser = optparse.OptionParser(usage="usage: %prog [options] arg1")
    parser.add_option("-f", "--file", action="store", type="string",
                      dest="filename", default="./output.csv")

    (opts, args) = parser.parse_args()
    if len(args) > 1:
        parser.error("incorrect number of arguments")
        return

    dataFile = opts.filename
    # create a queue for data input
    dataQueue = Queue.Queue()
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
    if (gps.begin(dataQueue) != 1):
        print("Not able to begin device properly... check logfile")
        return
    gps.setCom(baud=115200)
    # 0 Air 1 Land 2 foot
    # gps.setDynamics(2)
    gps.askLog(trigger=2, period=0.1)
    print ('Press Ctrl+C to Exit')
    signal.pause()
    exitFlag.set()
    threadID.join()
    myFile.flush()
    myFile.close()
    print('Exiting now')
    return


if __name__ == '__main__':
    main()
