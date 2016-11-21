import signal
import lib.NovatelOEM4 as Novatel


def main():
    def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        gps.shutdown()
        return

    signal.signal(signal.SIGINT, signal_handler)
    gps = Novatel.Gps()
    if (gps.begin() != 1):
        print("Not able to begin device properly... check logfile")
        return
    gps.setCom(baud=115200)
    # 0 Air 1 Land 2 foot
    # gps.setDynamics(2)
    gps.askLog(trigger=2, period=0.1)
    print ('Press Ctrl+C to Exit')
    signal.pause()
    return


if __name__ == '__main__':
    main()
