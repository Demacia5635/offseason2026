// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.Log;

import java.util.function.BiConsumer;

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
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.demacia.utils.Data;
import frc.robot.RobotContainer;

public class LogEntry2<T> {

    private final LogManager2 logManager;

    DataLogEntry entry;
    Data<T> data;
    BiConsumer<T[], Long> consumer = null;
    String name;
    String metaData;
    Publisher ntPublisher;

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
    LogEntry2(String name, Data<T> data, int logLevel, String metaData) {

        logManager = LogManager2.logManager;

        this.name = name;
        this.logLevel = logLevel;
        this.data = data;
        this.metaData = metaData;

        this.isFloat = data.isDouble();
        this.isBoolean = data.isBoolean();
        this.isArray = data.isArray();

        this.entry = createLogEntry(logManager.log, name, metaData);

        if (logLevel == 4 || (logLevel == 3 && !RobotContainer.isComp())) {
            this.ntPublisher = createPublisher(logManager.table, name);
        } else {
            this.ntPublisher = null;
        }
    }

    public void addData(String name, Data<T> data){
        this.name = this.name + " | " + name;
        if (this.data.getSignals() != null){
            this.data.expandWithSignals(data.getSignals());
        } else {
            this.data.expandWithSuppliers(data.getSuppliers());
        }
        this.isArray = this.data.isArray();
        if (ntPublisher != null) ntPublisher.close();
        
        entry = createLogEntry(logManager.log, this.name, metaData);

        if (logLevel == 4 || (logLevel == 3 && !RobotContainer.isComp())) {
            ntPublisher = createPublisher(logManager.table, this.name);
        } else {
            ntPublisher = null;
        }
    }

    public void removeData(int nameIndex, int dataIndex, int count) {
        int actualIndex = nameIndex - 1;

        String[] parts = name.split(" \\| ");

        if (actualIndex >= parts.length || actualIndex < 0) {
            LogManager2.log("removeData: nameIndex out of range: " + nameIndex + " (parts length: " + parts.length + ")", AlertType.kWarning);
            return;
        }
        StringBuilder newName = new StringBuilder();
        for (int i = 0; i < parts.length; i++) {
            if (i != actualIndex) {
                if (newName.length() > 0) newName.append(" | ");
                newName.append(parts[i]);
            }
        }

        name = newName.toString();

        if (data != null) {
            try {
                if (data.getSignals() != null) {
                    data.removeSignalRange(dataIndex, count);
                } else if (data.getSuppliers() != null) {
                    data.removeSupplierRange(dataIndex, count);
                }
            } catch (Exception e) {
                LogManager2.log("removeData: failed to remove from data: " + e.getMessage(), AlertType.kError);
            }
        }

        if (name.isEmpty()) {
            if (ntPublisher != null) {
                ntPublisher.close();
                ntPublisher = null;
            }
            entry = null;
            data = null;
            return;
        }

        if (ntPublisher != null) ntPublisher.close();

        entry = createLogEntry(logManager.log, name, metaData);
        if (logLevel == 4 || (logLevel == 3 && !RobotContainer.isComp())) {
            ntPublisher = createPublisher(logManager.table, name);
        } else {
            ntPublisher = null;
        }
    }

    void log() {
        if (!data.hasChanged()) {
            return;
        }

        long time = data.getTime();

        appendEntry(time);

        if (ntPublisher != null) {
            publishToNetworkTable();
        }
        
        if (consumer != null) {
            consumer.accept(data.getValueArray(), time);
        } 
    }

    public void setPrecision(double precision) {
        data.setPrecision(Math.max(0, precision));
    }

    public double getPrecision() {
        return data.getPrecision();
    }

    public void setConsumer(BiConsumer<T[], Long> consumer) {
        this.consumer = consumer;
    }

    public void removeInComp() {
        if (logLevel == 3 && ntPublisher != null) {
            ntPublisher.close();
        }
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

    private void appendEntry(long time) {
        if (isArray) {
            if (isFloat){
                ((FloatArrayLogEntry) entry).append(data.getFloatArray(), time);
            } else if (isBoolean){
                ((BooleanArrayLogEntry) entry).append(data.getBooleanArray(), time);
            } else{
                ((StringArrayLogEntry) entry).append(data.getStringArray(), time);
            }
        } else {
            if (isFloat){
                ((FloatLogEntry) entry).append(data.getFloat(), time);
            } else if (isBoolean){
                ((BooleanLogEntry) entry).append(data.getBoolean(), time);
            } else{
                ((StringLogEntry) entry).append(data.getString(), time);
            }
        }
    }

    private void publishToNetworkTable() {
        if (isArray) {
            if (isFloat){
                ((FloatArrayPublisher) ntPublisher).set(data.getFloatArray());
            } else if (isBoolean){
                ((BooleanArrayPublisher) ntPublisher).set(data.getBooleanArray());
            } else{
                ((StringArrayPublisher) ntPublisher).set(data.getStringArray());
            }
        } else {
            if (isFloat){
                ((FloatPublisher) ntPublisher).set(data.getFloat());
            } else if (isBoolean){
                ((BooleanPublisher) ntPublisher).set(data.getBoolean());
            } else{
                ((StringPublisher) ntPublisher).set(data.getString());
            }
        }
    }
}