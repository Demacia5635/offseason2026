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

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }
    
}