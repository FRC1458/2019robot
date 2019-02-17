package frc.team1458.lib.util.logging

import java.io.File

class ThreadLogger(private val delayTimeMS: Int = 15) {
    private var runnable: LoggerRunnable? = null
    private var loggingThread: Thread? = null

    @Synchronized fun setup(logDirectory: String, keys: Array<String>) {
        val logNumber = File(logDirectory).list().size
        val logPath = logDirectory + "log$logNumber.csv"

        runnable = LoggerRunnable(logPath, keys, intervalMS = delayTimeMS)
        loggingThread = Thread(runnable)
    }

    @Synchronized fun start() {
        runnable!!.setup()
        loggingThread!!.start()
    }

    @Synchronized fun stop() {
        runnable!!.stop()
    }

    @Synchronized fun update(key: String, newValue: Any) {
        runnable!!.updateKey(key, newValue)
    }
}