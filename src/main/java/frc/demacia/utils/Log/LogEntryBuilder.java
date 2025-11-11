package frc.demacia.utils.Log;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import frc.demacia.utils.Data;

public class LogEntryBuilder<T> implements AutoCloseable {
    private String name;
    private int logLevel = 3;
    private String metadata = "";
    private BiConsumer<T[], Long> consumer = null;

    private Data<T> data;

    private boolean built = false;
    
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
    
    public LogEntryBuilder<T> withLogLevel(int level) {
        this.logLevel = level;
        return this;
    }
    
    public LogEntryBuilder<T> withMetaData(String metaData) {
        this.metadata = metaData;
        return this;
    }
    
    public LogEntryBuilder<T> WithIsMotor() {
        this.metadata = "motor";
        return this;
    }
    
    public LogEntryBuilder<T> WithConsumer(BiConsumer<T[], Long> consumer) {
        this.consumer = consumer;
        return this;
    }
    
    public LogEntry2<T> build() {
        if (LogManager2.logManager == null) {
            throw new IllegalStateException("LogManager2 not initialized. Create LogManager2 instance before building log entries.");
        }
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("Log entry name cannot be null or empty");
        }
        if (logLevel < 1 || logLevel > 4) {
            throw new IllegalArgumentException("Log level must be between 1 and 4, got: " + logLevel);
        }
        built = true;
        LogEntry2<T> entry = LogManager2.logManager.add(name, data, logLevel, metadata);
        if (consumer != null) {
            entry.setConsumer(consumer);
        }
        return entry;
    }
    
    @Override
    public void close() {
        if (!built) {
            build();
        }
    }
}