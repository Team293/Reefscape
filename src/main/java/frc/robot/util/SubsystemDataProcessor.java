package frc.robot.util;

import java.util.List;

public class SubsystemDataProcessor implements Runnable {
    public interface DataReaderAndLogger {
        void readAndLogDataFromIO();
    }

    public interface IODataRefresher {
        void refreshData();
    }

    public static final int LOOP_TIME = 20;

    public static void createAndStartSubsystemDataProcessor(
            DataReaderAndLogger dataReaderAndLogger, IODataRefresher IODataRefresher) {
        new Thread(new SubsystemDataProcessor(dataReaderAndLogger, IODataRefresher)).start();
    }

    public static void createAndStartSubsystemDataProcessor(
            DataReaderAndLogger dataReaderAndLogger,
            IODataRefresher IODataRefresher1,
            IODataRefresher IODataRefresher2,
            IODataRefresher IODataRefresher3) {
        new Thread(new SubsystemDataProcessor(
                        dataReaderAndLogger, IODataRefresher1, IODataRefresher2, IODataRefresher3))
                .start();
    }

    private double timestamp = 0.0;
    private DataReaderAndLogger dataReaderAndLogger;
    private List<IODataRefresher> IODataRefreshers;

    public SubsystemDataProcessor(DataReaderAndLogger dataReaderAndLogger, IODataRefresher IODataRefresher) {
        this.dataReaderAndLogger = dataReaderAndLogger;
        IODataRefreshers = List.of(IODataRefresher);
    }

    public SubsystemDataProcessor(
            DataReaderAndLogger dataReaderAndLogger,
            IODataRefresher IODataRefresher1,
            IODataRefresher IODataRefresher2,
            IODataRefresher IODataRefresher3) {
        this.dataReaderAndLogger = dataReaderAndLogger;
        IODataRefreshers = List.of(IODataRefresher1, IODataRefresher2, IODataRefresher3);
    }

    public void run() {
        while (true) {
            timestamp = System.currentTimeMillis();
            for (IODataRefresher IODataRefresher : IODataRefreshers) {
                IODataRefresher.refreshData();
            }

            dataReaderAndLogger.readAndLogDataFromIO();
            try {
                var difference = System.currentTimeMillis() - timestamp;
                if (difference < LOOP_TIME) {
                    Thread.sleep((long) (LOOP_TIME - difference));
                }
            } catch (InterruptedException e) {
            }
        }
    }
}