// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.Log;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import frc.robot.RobotContainer;

public class LogEntry<T> {

    private final LogManager logManager;

    DataLogEntry entry;
    StatusSignal<T>[] phoenix6Status;
    Supplier<T> getter;
    BiConsumer<T, Long> consumer = null;
    String name;
    String metaData;
    Publisher ntPublisher;
    T lastValue;
    private double precision = 0;
    private int skipedCycles2 = 0;
    private int SkipCycle = 1;

    private boolean isFloat;
    private boolean isBoolean;
    private boolean isArray;

    /*
        * the log levels are this:
        * 1 -> log if it is not in a compition
        * 2 -> only log
        * 3 -> log and add to network tables if not in a compition
        * 4 -> log and add to network tables
    */
    public int logLevel;

    /*
        * Constructor with the suppliers and boolean if add to network table
    */
    LogEntry(String name, StatusSignal<T>[] phoenix6Status, Supplier<T> getter, int logLevel, String metaData, boolean isFloat, boolean isBoolean, boolean isArray) {

        logManager = LogManager.logManager;

        this.name = name;
        this.logLevel = logLevel;
        this.phoenix6Status = phoenix6Status;
        this.getter = getter;
        this.metaData = metaData;

        this.isFloat = isFloat;
        this.isBoolean = isBoolean;
        this.isArray = isArray;

        this.entry = createLogEntry(logManager.log, name, metaData);

        if (logLevel == 4 || (logLevel == 3 && !RobotContainer.isComp())) {
            this.ntPublisher = createPublisher(logManager.table, name);
        } else {
            this.ntPublisher = null;
        }
    }

    void log() {
        skipedCycles2++;
        if (skipedCycles2 < SkipCycle) {
            return;
        }
        skipedCycles2 = 0;

        T newValue = null;
        long time = 0;
        boolean hasChanged = false;

        if (phoenix6Status != null) {
            StatusCode st = StatusSignal.refreshAll(phoenix6Status);
            if (st == StatusCode.OK) {
                newValue = extractPhoenixValue();
                if (newValue != null) {
                    hasChanged = hasValueChanged(newValue);
                    time = (long) (phoenix6Status[0].getTimestamp().getTime() * 1000);
                }
            } else {
                newValue = lastValue;
                hasChanged = false;
            }
        } else if (getter != null) {
            T rawValue = getter.get();
            if (rawValue != null) {
                newValue = rawValue;
                hasChanged = hasValueChanged(newValue);
                time = 0;
            }
        }
        
        if (hasChanged && newValue != null) {
            log(newValue, time);
            lastValue = copyValue(newValue);
        }
    }

    public void log(T value) {
        log(value, 0);
    }

    public void log(T value, long time) {
        if (value == null) return;
        
        appendEntry(value, time);

        if (ntPublisher != null) {
            publishToNetworkTable(value);
        }
        
        if (consumer != null) {
            consumer.accept(value, time);
        }
    }

    

    public void setPrecision(double precision) {
        this.precision = Math.max(0, precision);
    }

    public double getPrecision() {
        return precision;
    }

    public void setSkipCycles(int interval) {
        SkipCycle = Math.max(1, interval);
    }

    public int getSkipCycles() {
        return SkipCycle;
    }

    public void setConsumer(BiConsumer<T, Long> consumer) {
        this.consumer = consumer;
    }

    public void removeInComp() {
        if (logLevel == 3 && ntPublisher != null) {
            ntPublisher.close();
        }
    }

    @SuppressWarnings("unchecked")
    private T extractPhoenixValue() {
        if (isArray) {
            if (isFloat) {
                float[] floatArray = new float[phoenix6Status.length];
                for (int i = 0; i < phoenix6Status.length; i++) {
                    floatArray[i] = (float)phoenix6Status[i].getValueAsDouble();
                }
                return (T) floatArray;
            } else if (isBoolean) {
                boolean[] booleanArray = new boolean[phoenix6Status.length];
                for (int i = 0; i < phoenix6Status.length; i++) {
                    booleanArray[i] = (Boolean)phoenix6Status[i].getValue();
                }
                return (T) booleanArray;
            } else {
                String[] stringArray = new String[phoenix6Status.length];
                for (int i = 0; i < phoenix6Status.length; i++) {
                    stringArray[i] = phoenix6Status[i].getValue().toString();
                }
                return (T) stringArray;
            }
        } else {
            if (isFloat) {
                return (T) Float.valueOf((float)phoenix6Status[0].getValueAsDouble());
            } else if (isBoolean) {
                return (T) phoenix6Status[0].getValue();
            } else{
                return (T) phoenix6Status[0].getValue().toString();
            }
        }
    }

    private boolean hasValueChanged(T newValue) {
        if (lastValue == null) {
            return true;
        }

        if (isArray){
            if (isFloat){
                float[] newArr = toFloatArray(newValue);
                float[] lastArr = toFloatArray(lastValue);
                if (newArr.length != lastArr.length) {
                    return true;
                }
                for (int i = 0; i < newArr.length; i++) {
                    if (Math.abs(newArr[i] - lastArr[i]) >= precision) {
                        return true;
                    }
                }
                return false;
            } else if (isBoolean){
                boolean[] newArr = (boolean[]) newValue;
                boolean[] lastArr = (boolean[]) lastValue;
                if (newArr.length != lastArr.length) {
                    return true;
                }
                for (int i = 0; i < newArr.length; i++) {
                    if (newArr[i] != lastArr[i]) {
                        return true;
                    }
                }
                return false;
            } else{
                String[] newArr = toStringArray(newValue);
                String[] lastArr = toStringArray(lastValue);
                if (newArr.length != lastArr.length) {
                    return true;
                }
                for (int i = 0; i < newArr.length; i++) {
                    if (!(newArr[i].toString()).equals((lastArr[i].toString()))) {
                        return true;
                    }
                }
                return false;
            }
        } else{
            if (isFloat){
                return Math.abs(((Number) newValue).floatValue() - ((Number) lastValue).floatValue()) >= precision;
            } else if (isBoolean){
                return !newValue.equals(lastValue);
            } else{
                return !(newValue.toString()).equals((lastValue.toString()));
            }
        }
    }

    @SuppressWarnings("unchecked")
    private T copyValue(T value) {
        if (isArray){
            if (isFloat) {
                return (T) toFloatArray(value);
            } else if (isBoolean) {
                return (T) ((boolean[]) value).clone();
            } else {
                return (T) toStringArray(value);
            }
        }
        else{
            if (!(isFloat || isBoolean)) {
                return (T) value.toString();
            } else{
                return value;
            }
        }
    }

    private float[] toFloatArray(T value){
        int length = java.lang.reflect.Array.getLength(value);
        float[] floatArr = new float[length];
            for (int i = 0; i < length; i++) {
                Object elem = java.lang.reflect.Array.get(value, i);
                floatArr[i] = (elem != null) ? ((Number) elem).floatValue() : 0f;
            }
            return floatArr;
    }

    private String[] toStringArray(T value){
        int length = java.lang.reflect.Array.getLength(value);
            String[] stringArr = new String[length];
            for (int i = 0; i < length; i++) {
                Object elem = java.lang.reflect.Array.get(value, i);
                stringArr[i] = (elem != null) ? elem.toString() : null;
            }
            return stringArr;
    }

    private DataLogEntry createLogEntry(DataLog log, String name, String metaData) {
        if (isArray) {
            if (isFloat){
                return new FloatArrayLogEntry(log, name, metaData);
            } else if (isBoolean){
                return new BooleanArrayLogEntry(log, name, metaData);
            } else{
                return new StringArrayLogEntry(log, name, metaData);
            }
        } else {
            if (isFloat){
                return new FloatLogEntry(log, name, metaData);
            } else if (isBoolean){
                return new BooleanLogEntry(log, name, metaData);
            } else{
                return new StringLogEntry(log, name, metaData);
            }
        }
    }

    private Publisher createPublisher(NetworkTable table, String name) {
        if (isArray) {
            if (isFloat){
                return table.getFloatArrayTopic(name).publish();
            } else if (isBoolean){
                return table.getBooleanArrayTopic(name).publish();
            } else{
                return table.getStringArrayTopic(name).publish();
            }
        } else {
            if (isFloat){
                return table.getFloatTopic(name).publish();
            } else if (isBoolean){
                return table.getBooleanTopic(name).publish();
            } else{
                return table.getStringTopic(name).publish();
            }
        }
    }

    private void appendEntry(T value, long time) {
        if (isArray) {
            if (isFloat){
                ((FloatArrayLogEntry) entry).append(toFloatArray(value), time);
            } else if (isBoolean){
                ((BooleanArrayLogEntry) entry).append((boolean[]) value, time);
            } else{
                ((StringArrayLogEntry) entry).append(toStringArray(value), time);
            }
        } else {
            if (isFloat){
                ((FloatLogEntry) entry).append(((Number) value).floatValue()
                , time);
            } else if (isBoolean){
                ((BooleanLogEntry) entry).append((Boolean) value, time);
            } else{
                ((StringLogEntry) entry).append(value.toString(), time);
            }
        }
    }

    private void publishToNetworkTable(T value) {
        if (isArray) {
            if (isFloat){
                ((FloatArrayPublisher) ntPublisher).set(toFloatArray(value));
            } else if (isBoolean){
                ((BooleanArrayPublisher) ntPublisher).set((boolean[]) value);
            } else{
                ((StringArrayPublisher) ntPublisher).set(toStringArray(value));
            }
        } else {
            if (isFloat){
                ((FloatPublisher) ntPublisher).set(((Number) value).floatValue());
            } else if (isBoolean){
                ((BooleanPublisher) ntPublisher).set((Boolean) value);
            } else{
                ((StringPublisher) ntPublisher).set(value.toString());
            }
        }
    }
}