// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Data;

public class LogManager2 extends SubsystemBase {

  public static LogManager2 logManager;

  DataLog log;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Log");

  private static ArrayList<ConsoleAlert> activeConsole;

  ArrayList<LogEntry2<?>> individualLogEntries = new ArrayList<>();
  
  LogEntry2<?>[] categoryLogEntries = new LogEntry2<?>[16];

  private Map<String, Pair<Pair<Integer, Integer>, Pair<Integer, Integer>>> entryLocationMap = new HashMap<>();

  public LogManager2() {
    logManager = this;

    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    log = DataLogManager.getLog();
    DriverStation.startDataLog(log);
    
    activeConsole = new ArrayList<>();
    log("log manager is ready");
  }

  @SuppressWarnings("unchecked")
  public static <T> LogEntryBuilder<T> addEntry(String name, StatusSignal<T> ... statusSignals) {
      return new LogEntryBuilder<T>(name, statusSignals);
  }

  @SuppressWarnings("unchecked")
  public static <T> LogEntryBuilder<T> addEntry(String name, Supplier<T> ... suppliers) {
    return new LogEntryBuilder<T>(name, suppliers);
  }

  public static void removeInComp() {
    for (int i = 0; i < logManager.individualLogEntries.size(); i++) {
      logManager.individualLogEntries.get(i).removeInComp();
      if (logManager.individualLogEntries.get(i).logLevel == 1) {
        logManager.individualLogEntries.remove(logManager.individualLogEntries.get(i));
        i--;
      }
    }

    for (int i = 0; i < 4; i++) {
      if (logManager.categoryLogEntries[i] != null && logManager.categoryLogEntries[i] != null) {
        logManager.categoryLogEntries[i].removeInComp();
        if (logManager.categoryLogEntries[i].logLevel == 1) {
          logManager.categoryLogEntries[i] = null;
          int index = i;
          logManager.entryLocationMap.entrySet().removeIf(entry -> entry.getValue().getFirst().getFirst() == index);
        }
      }
    }
  }
  
  public static void clearEntries() {
    if (logManager != null) {
      logManager.individualLogEntries.clear();
      for (int i = 0; i < logManager.categoryLogEntries.length; i++) {
        if (logManager.categoryLogEntries[i] != null) {
          logManager.categoryLogEntries[i] = null;
        }
      }
      logManager.entryLocationMap.clear();
    }
  }
  
  public static int getEntryCount() {
    if (logManager == null) return 0;

    int count = logManager.individualLogEntries.size();
    for (LogEntry2<?> entry2 : logManager.categoryLogEntries) {
      if (entry2 != null) {
        count++;
      }
    }
    return count;
  }

  public static LogEntry2<?> findEntry(String name) {
    if (logManager == null) return null;
    
    Pair< Pair<Integer, Integer>, Pair<Integer, Integer>> location = logManager.entryLocationMap.get(name);
    if (location == null) return null;
    
    int categoryIndex = location.getFirst().getFirst();
    
    if (categoryIndex == -1) {
      int index = location.getFirst().getSecond();
      if (index >= 0 && index < logManager.individualLogEntries.size()) {
        return logManager.individualLogEntries.get(index);
      }
    } else {
      return logManager.categoryLogEntries[categoryIndex];
    }
    
    return null;
  }

  public static boolean removeEntry(String name) {
      if (logManager == null) return false;
      
      Pair<Pair<Integer, Integer>, Pair<Integer, Integer>> location = logManager.entryLocationMap.get(name);
      if (location == null) return false;
      
      int categoryIndex = location.getFirst().getFirst();
      
      if (categoryIndex == -1) {
          int index = location.getFirst().getSecond();
          if (index >= 0 && index < logManager.individualLogEntries.size()) {
              logManager.individualLogEntries.remove(index);
              logManager.entryLocationMap.remove(name);
              
              logManager.updateIndicesAfterRemoval(index);
              return true;
          }
      } else {
          int subIndex = location.getFirst().getSecond();
          int dataIndex = location.getSecond().getFirst();
          int dataCount = location.getSecond().getSecond();
          
          logManager.entryLocationMap.remove(name);
          
          logManager.categoryLogEntries[categoryIndex].removeData(subIndex, dataIndex, dataCount);
          
          logManager.updateSubIndicesAfterRemoval(categoryIndex, subIndex);
          logManager.updateDataIndicesAfterRemoval(categoryIndex, dataIndex, dataCount);
          
          if (logManager.categoryLogEntries[categoryIndex].data == null || 
              logManager.categoryLogEntries[categoryIndex].name == null || 
              logManager.categoryLogEntries[categoryIndex].name.trim().isEmpty()) {
              logManager.categoryLogEntries[categoryIndex] = null;
              final int catIndex = categoryIndex;
              logManager.entryLocationMap.entrySet().removeIf(
                  entry -> entry.getValue().getFirst().getFirst() == catIndex
              );
          }
          
          return true;
      }
      
      return false;
  }

  public static ConsoleAlert log(Object message, AlertType alertType) {
    DataLogManager.log(String.valueOf(message));
    
    ConsoleAlert alert = new ConsoleAlert(String.valueOf(message), alertType);
    alert.set(true);
    if (activeConsole.size() > ConsoleConstants.CONSOLE_LIMIT) {
      activeConsole.get(0).close();
      activeConsole.remove(0);
    }
    activeConsole.add(alert);
    return alert;
  }

  public static ConsoleAlert log(Object message) {
    return log(message, AlertType.kInfo);
  }

  @Override
  public void periodic() {
    for (LogEntry2<?> e : individualLogEntries) {
      e.log();
    }
    for (LogEntry2<?> e : categoryLogEntries) {
      if (e != null){
        e.log();
      }
    }
  }

  public <T> LogEntry2<T> add(String name, Data<T> data, int logLevel, String metaData) {
    LogEntry2<T> entry = null;

    int categoryIndex = getCategoryIndex(data, logLevel, metaData);

    if (categoryIndex == -1){
      entry = new LogEntry2<T>(name, data, logLevel, metaData);
      individualLogEntries.add(entry);
      int index = individualLogEntries.size() - 1;
      entryLocationMap.put(name, new Pair<>(new Pair<>(-1, index), new Pair<>(null, null)));
    } else{
      entry = addToEntryArray(categoryIndex, name, data);
    }

    return entry;
  }

  @SuppressWarnings("unchecked")
  private <T> LogEntry2<T> addToEntryArray(int i, String name, Data<T> data) {
    int subIndex;
    int dataIndex = 0;
    
    if (categoryLogEntries[i] == null || categoryLogEntries[i].data == null) {
        categoryLogEntries[i] = new LogEntry2<>(name, data, i/4 + 1, "");
        subIndex = 1;
        dataIndex = 0;
    } else {
        String[] parts = categoryLogEntries[i].name.split(" \\| ");
        subIndex = parts.length + 1;

        if (categoryLogEntries[i].data.getSignals() != null) {
          dataIndex = categoryLogEntries[i].data.getSignals().length;
        } else if (categoryLogEntries[i].data.getSuppliers() != null) {
          dataIndex = categoryLogEntries[i].data.getSuppliers().length;
        }

        try {
            ((LogEntry2<T>) categoryLogEntries[i]).addData(name, data);
        } catch (Exception e) {
            LogManager2.log("Error combining log entries: " + e.getMessage(), AlertType.kError);
        }
    }

    int dataLength = 0;
    if (data.getSignals() != null) {
      dataLength = data.getSignals().length;
    } else if (data.getSuppliers() != null) {
      dataLength = data.getSuppliers().length;
    }


    entryLocationMap.put(name, new Pair<>(new Pair<>(i, subIndex), new Pair<>(dataIndex, dataLength)));
    log("added:" + name + 
    " with: " + i + 
    " and: " + subIndex + 
    " and: " + dataIndex + 
    " and: " + dataLength);
    
    return (LogEntry2<T>) categoryLogEntries[i];
}

  private int getCategoryIndex(Data<?> data, int logLevel, String metaData) {
    boolean isSignal = data.getSignals() != null;
    boolean isSupplier = data.getSuppliers() != null;
    boolean isDouble = data.isDouble();
    boolean isBoolean = data.isBoolean();
    
    if (!(isDouble || isBoolean) || !(isSignal || isSupplier) || metaData != "") {
      return -1;
    }
    
    int baseIndex = (isSignal ? 0 : 2) + (isDouble ? 0 : 1);
    int levelOffset = (logLevel - 1) * 4;
    
    return baseIndex + levelOffset;
  }

  private void updateIndicesAfterRemoval(int removedIndex) {
      for (Map.Entry<String, Pair<Pair<Integer, Integer>, Pair<Integer, Integer>>> entry : entryLocationMap.entrySet()) {
          Pair<Pair<Integer, Integer>, Pair<Integer, Integer>> location = entry.getValue();
          if (location.getFirst().getFirst() == -1 && location.getFirst().getSecond() > removedIndex) {
              entry.setValue(
                  new Pair<>(
                      new Pair<>(-1, location.getFirst().getSecond() - 1), 
                      new Pair<>(location.getSecond().getFirst(), location.getSecond().getSecond())
                  )
              );
          }
      }
  }

  private void updateSubIndicesAfterRemoval(int categoryIndex, int removedSubIndex) {
      for (Map.Entry<String, Pair<Pair<Integer, Integer>, Pair<Integer, Integer>>> entry : entryLocationMap.entrySet()) {
          Pair<Pair<Integer, Integer>, Pair<Integer, Integer>> location = entry.getValue();
          if (location.getFirst().getFirst() == categoryIndex && location.getFirst().getSecond() > removedSubIndex) {
              entry.setValue(
                  new Pair<>(
                      new Pair<>(categoryIndex, location.getFirst().getSecond() - 1),
                      new Pair<>(location.getSecond().getFirst(), location.getSecond().getSecond())
                  )
              );
          }
      }
  }

  private void updateDataIndicesAfterRemoval(int categoryIndex, int removedDataIndex, int removedCount) {
      for (Map.Entry<String, Pair<Pair<Integer, Integer>, Pair<Integer, Integer>>> entry : entryLocationMap.entrySet()) {
          Pair<Pair<Integer, Integer>, Pair<Integer, Integer>> location = entry.getValue();
          if (location.getFirst().getFirst() == categoryIndex && 
              location.getSecond().getFirst() != null &&
              location.getSecond().getFirst() >= removedDataIndex + removedCount) {
              entry.setValue(
                  new Pair<>(
                      new Pair<>(location.getFirst().getFirst(), location.getFirst().getSecond()),
                      new Pair<>(location.getSecond().getFirst() - removedCount, location.getSecond().getSecond())
                  )
              );
          }
      }
  }


  @SuppressWarnings("unused")
  private LogEntry2<?> get(String name) {
    LogEntry2<?> e = find(name);
    return e != null 
    ?e 
    :new LogEntry2<>(name, null, 1, "");
  }

  private LogEntry2<?> find(String name) {
    for (LogEntry2<?> entry : individualLogEntries) {
      if (entry.name.equals(name)) {
        return entry;
      }
    }
    return null;
  }
}