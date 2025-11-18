package frc.demacia.utils.Log;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import frc.demacia.utils.Data;

/**
 * Fluent builder for creating log entries with various options.
 */
public class LogEntryBuilder<T> {

    /**
     * Enum representing LogLevel.
     * 
     * <p><b>Log Levels:</b></p>
     * <ul>
     *   <li><b>LOG_ONLY_NOT_IN_COMP:</b> File only, disabled in competition (default)</li>
     *   <li><b>LOG_ONLY:</b> File only, always active</li>
     *   <li><b>LOG_AND_NT_NOT_IN_COMP:</b> File + NetworkTables, disabled NT in comp</li>
     *   <li><b>LOG_AND_NT:</b> File + NetworkTables, always active</li>
     * </ul>
     */
    public static enum LogLevel { LOG_ONLY_NOT_IN_COMP, LOG_ONLY, LOG_AND_NT_NOT_IN_COMP, LOG_AND_NT} 

    private String name;
    private LogLevel logLevel = LogLevel.LOG_ONLY_NOT_IN_COMP;
    private String metadata = "";
    private BiConsumer<T[], Long> consumer = null;

    private Data<T> data;
    
    @SuppressWarnings("unchecked")
    LogEntryBuilder(String name, StatusSignal<T> ... statusSignals) {
        this.name = name;
        data = new Data<>(statusSignals);
    }
    
    @SuppressWarnings("unchecked")
    LogEntryBuilder(String name, Supplier<T> ... suppliers) {
        this.name = name;
        data = new Data<>(suppliers);
    }
    
    /**
     * Sets the logging level for this entry.
     * 
     * <p>Choose based on performance needs:</p>
     * <ul>
     *   <li>LOG_ONLY_NOT_IN_COMP - Most data (development/tuning)</li>
     *   <li>LOG_AND_NT - Critical data that needs dashboard visibility</li>
     * </ul>
     * 
     * @param level The log level to use
     * @return this builder for chaining
     */
    public LogEntryBuilder<T> withLogLevel(LogLevel level) {
        this.logLevel = level;
        return this;
    }
    
    /**
     * Sets metadata for this log entry.
     * 
     * <p>Metadata is stored in the log file and can be used for filtering
     * or categorization in analysis tools.</p>
     * 
     * @param metaData Metadata string
     * @return this builder for chaining
     */
    public LogEntryBuilder<T> withMetaData(String metaData) {
        this.metadata = metaData;
        return this;
    }
    
    /**
     * Marks this entry as motor-related.
     * 
     * <p>Shortcut for withMetaData("motor"). Used for automatic grouping
     * and optimized logging of motor telemetry.</p>
     * 
     * @return this builder for chaining
     */
    public LogEntryBuilder<T> WithIsMotor() {
        this.metadata = "motor";
        return this;
    }
    
    /**
     * Adds a consumer to process logged data.
     * 
     * <p>The consumer receives the data values and timestamp each time
     * data is logged. Useful for custom processing or analysis.</p>
     * 
     * @param consumer Function that accepts (values array, timestamp)
     * @return this builder for chaining
     */
    public LogEntryBuilder<T> WithConsumer(BiConsumer<T[], Long> consumer) {
        this.consumer = consumer;
        return this;
    }
    
    /**
     * Creates and registers the log entry.
     * 
     * <p>Must be called to finalize the entry. The entry will automatically
     * log data each robot cycle.</p>
     * 
     * @return The created log entry
     * @throws IllegalStateException if LogManager not initialized
     * @throws IllegalArgumentException if name is empty or level is null
     */
    public LogEntry<T> build() {
        if (LogManager.logManager == null) {
            throw new IllegalStateException("LogManager2 not initialized. Create LogManager2 instance before building log entries.");
        }
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("Log entry name cannot be null or empty");
        }
        if (logLevel == null) {
            throw new IllegalArgumentException("Log level cant be null: ");
        }
        LogEntry<T> entry = LogManager.logManager.add(name, data, logLevel, metadata);
        if (consumer != null) {
            entry.setConsumer(consumer);
        }
        return entry;
    }
}