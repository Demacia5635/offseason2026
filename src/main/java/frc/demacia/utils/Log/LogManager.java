// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.Log;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LogManager extends SubsystemBase {

  public static LogManager logManager;

  DataLog log;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Log");

  private static ArrayList<ConsoleAlert> activeConsole;
  
  private static int SkipedCycles1 = 0;
  private static int SKIP_CYCLES = 1;
  private static boolean isLoggingEnabled = true;

  ArrayList<LogEntry<?>> logEntries = new ArrayList<>();

  public LogManager() {
    logManager = this;

    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    log = DataLogManager.getLog();
    DriverStation.startDataLog(log);
    
    activeConsole = new ArrayList<>();
    log("log manager is ready");
  }

  @SuppressWarnings("unchecked")
  public static <T> LogEntry<T> addEntry(String name, StatusSignal<T> phoenixStatus, int logLevel, String metaData) {
    boolean isFloat = false;
    boolean isBoolean = false;
    boolean isArray = false;
    try{
      phoenixStatus.getValueAsDouble();
      isFloat = true;
    } catch(Exception e){
      if (phoenixStatus.getValue() instanceof Boolean){
        isBoolean = true;
      }
    }
    return logManager.add(name, new StatusSignal[] {phoenixStatus}, null, logLevel, metaData, isFloat, isBoolean, isArray);
  }

  public static <T> LogEntry<T> addEntry(String name, StatusSignal<T> phoenix6Status, int logLevel) {
    return addEntry(name, phoenix6Status, logLevel, "");
  }

  public static <T> LogEntry<T> addEntry(String name, StatusSignal<T> phoenix6Status, String metaData) {
    return addEntry(name, phoenix6Status, 4, metaData);
  }

  public static <T> LogEntry<T> addEntry(String name, StatusSignal<T> phoenix6Status) {
    return addEntry(name, phoenix6Status, 4, "");
  }

  public static <T> LogEntry<T> addEntry(String name, StatusSignal<T>[] phoenixStatus, int logLevel, String metaData) {
    boolean isFloat = false;
    boolean isBoolean = false;
    boolean isArray = true;
    try {
      phoenixStatus[0].getValueAsDouble();
      isFloat = true;
    } catch (Exception e) {
      if (phoenixStatus[0].getValue() instanceof Boolean){
        isBoolean = true;
      }
    }
    return logManager.add(name,  phoenixStatus, null, logLevel, metaData, isFloat, isBoolean, isArray);
  }

  public static <T> LogEntry<T> addEntry(String name, StatusSignal<T>[] phoenixStatus, int logLevel) {
    return addEntry(name, phoenixStatus, logLevel, "");
  }

  public static <T> LogEntry<T> addEntry(String name, StatusSignal<T>[] phoenixStatus, String metaData) {
    return addEntry(name, phoenixStatus, 4, metaData);
  }

  public static <T> LogEntry<T> addEntry(String name, StatusSignal<T>[] phoenixStatus) {
    return addEntry(name, phoenixStatus, 4, "");
  }

  public static <T> LogEntry<T> addEntry(String name, Supplier<T> getter, int logLevel, String metaData) {
    boolean isFloat = false;
    boolean isBoolean = false;
    boolean isArray = false;
    T value = getter.get();
    
    if (value.getClass().isArray()) {
      isArray = true;
      try {
        Object first = java.lang.reflect.Array.get(value, 0);
        ((Number) first).floatValue();
        isFloat = true;
      } catch (Exception e) {
        if (value instanceof boolean[] || value instanceof Boolean[]){
          isBoolean = true;
        }
      }
    } else {
        try {
            ((Number) value).floatValue();
            isFloat = true;
        } catch (Exception e) {
          if (value instanceof  Boolean){
            isBoolean = true;
          }
        }
    }
    
    return logManager.add(name, null, getter, logLevel, metaData, isFloat, isBoolean, isArray);
  }

  public static <T> LogEntry<T> addEntry(String name, Supplier<T> getter, int logLevel) {
    return addEntry(name, getter, logLevel, "");
  }

  public static <T> LogEntry<T> addEntry(String name, Supplier<T> getter, String metaData) {
    return addEntry(name, getter, 4, metaData);
  }

  public static <T> LogEntry<T> addEntry(String name, Supplier<T> getter) {
    return addEntry(name, getter, 4, "");
  }

  public static LogEntry<?> getEntry(String name) {
    return logManager.get(name);
  }

  public static void removeInComp() {
    for (int i = 0; i < logManager.logEntries.size(); i++) {
      logManager.logEntries.get(i).removeInComp();
      if (logManager.logEntries.get(i).logLevel == 1) {
        logManager.logEntries.remove(logManager.logEntries.get(i));
        i--;
      }
    }
  }
  
  public static void clearEntries() {
    if (logManager != null) {
      logManager.logEntries.clear();
    }
  }
  
  public static int getEntryCount() {
    return logManager != null ? logManager.logEntries.size() : 0;
  }

  public static ConsoleAlert log(Object message, AlertType alertType) {
    DataLogManager.log(String.valueOf(message));
    
    ConsoleAlert alert = new ConsoleAlert(String.valueOf(message.toString()), alertType);
    alert.set(true);
    if (activeConsole.size() > ConsoleConstants.CONSOLE_LIMIT) {
      activeConsole.get(0).close();
      activeConsole.remove(0);
    }
    activeConsole.add(alert);
    return alert;
  }

  public static ConsoleAlert log(Object meesage) {
    return log(meesage, AlertType.kInfo);
  }

  public static void setStaticSkipCycles(int cycles) {
    SKIP_CYCLES = Math.max(1, cycles);
  }

  public static int getSStaticSkipCycles() {
    return SKIP_CYCLES;
  }
  
  public static void setLoggingEnabled(boolean isenabled) {
    isLoggingEnabled = isenabled;
  }

  public static boolean getLoggingEnabled() {
    return isLoggingEnabled;
  }

  @Override
  public void periodic() {
    SkipedCycles1++;
    if (SkipedCycles1 < SKIP_CYCLES || !isLoggingEnabled) {
      return;
    }
    SkipedCycles1 = 0;

    for (LogEntry<?> e : logEntries) {
      e.log();
    }
  }

  private <T> LogEntry<T> add(String name, StatusSignal<T>[] phoenix6Status, Supplier<T> getter, int logLevel, String metaData, boolean isFloat, boolean isBoolean, boolean isArray) {
    LogEntry<T> entry = new LogEntry<T>(name, phoenix6Status, getter,  logLevel, metaData, isFloat, isBoolean, isArray);
    logEntries.add(entry);
    return entry;
  }

  @SuppressWarnings({ "unchecked", "rawtypes" })
  private LogEntry<?> get(String name) {
    LogEntry<?> e = find(name);
    return e != null 
    ?e 
    :new LogEntry(name, null, null, 1, "", true, false, false);
  }

  private LogEntry<?> find(String name) {
    for (LogEntry<?> entry : logEntries) {
      if (entry.name.equals(name)) {
        return entry;
      }
    }
    return null;
  }
}