package frc.Demacia.utils.Log;


import java.util.ArrayList;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import frc.Demacia.utils.Log.LogManager.LOG_TARGET;

/*
   * class for a single data entry
   */
  public class LogEntry {
    /**
     *
     */
    private final LogManager logManager;

    DataLogEntry entry; // wpilib log entry
    LogSupplier[] suppliers;
    String name;
    String meta;
    Publisher ntPublisher[] = null; 
    float[] floatData;
    boolean[] booleanData;
    boolean isDouble = true;
    boolean isArray = false;
    boolean updateNetworkTable;

    /*
     * the log levels are this:
     * 1 -> log if it is not in a compition
     * 2 -> only log
     * 3 -> log and add to network tables if not in a compition
     * 4 -> log and add to network tables
     */
    public LOG_TARGET logTarget;

    private static ArrayList<LogEntry> logEntries = new ArrayList<>();

    public static void periodic() {
        for(LogEntry entry : logEntries) {
            entry.log();
        }
    }

    /*
     * Constructor with the suppliers and boolean if add to network table
     */
    public LogEntry(String name, LogSupplier[] suplliers, LOG_TARGET logTarget, String type, String subType, String meta) {

      this.logManager = LogManager.logManager;
      this.name = name;
      this.logTarget = logTarget;
      this.suppliers = suplliers;
      this.meta = "Type:" + type + ";Subtype:" + subType + ";" + meta;
      updateNetworkTable = (logTarget == LOG_TARGET.LOG_AND_NT || logTarget == LOG_TARGET.LOG_NT_NOT_COMPETIION);
      if(suplliers == null) {
        isDouble = true;
        isArray = false;

      } else {
        isDouble = suplliers[0].isFloat();
        isArray = suplliers.length > 1;
      }
      if(updateNetworkTable) {
        ntPublisher = new Publisher[suplliers.length];
      }
      if(!isArray) {
        if(isDouble) { // Single Double
            entry = new FloatLogEntry(logManager.log, name, this.meta);
            floatData = new float[] {0};
            if(updateNetworkTable) {
                DoubleTopic dt = this.logManager.table.getDoubleTopic(name);
                ntPublisher[0] = dt.publish();
            }
        } else { // Single Boolean
            entry = new BooleanLogEntry(logManager.log, name, this.meta);
            booleanData = new boolean[] {true};
            if(updateNetworkTable) {
                BooleanTopic bt = this.logManager.table.getBooleanTopic(name);
                ntPublisher[0] = bt.publish();
            }
        }
      } else if(isDouble) { // double array
        entry = new FloatArrayLogEntry(logManager.log, name, this.meta);
        floatData = new float[suplliers.length];
        for(int i = 0; i < suplliers.length; i++) {
            floatData[i] = suplliers[i].getFloat();
        }
        if(updateNetworkTable) {
            for(int i = 0; i < ntPublisher.length; i++) {
                ntPublisher[i] = this.logManager.table.getFloatTopic(name + "/" + (i+1)).publish();
            }
        }
      } else { // Boolean array
        entry = new BooleanArrayLogEntry(logManager.log, name, this.meta);
        isDouble = false;
        booleanData = new boolean[suplliers.length];
        for(int i = 0; i < suplliers.length; i++) {
            booleanData[i] = suplliers[i].getBoolean();
        }
        if(updateNetworkTable) {
            for(int i = 0; i < ntPublisher.length; i++) {
                ntPublisher[i] = this.logManager.table.getBooleanTopic(name + "/" + (i+1)).publish();
            }
        }
      }
      logEntries.add(this);
    }

    public LogEntry(String name, LOG_TARGET logTarget) {
        this(name, null, logTarget, "data", "", "");
    }


    /*
     * perform a periodic log
     * get the data from the getters and call the actual log
     */
    void log() {
        boolean changed = false;
        for(int i = 0; i < suppliers.length; i++) {
            if(isDouble) {
                floatData[i] = suppliers[i].getFloat();
            } else {
                booleanData[i] = suppliers[i].getBoolean();
            }
            changed = changed || suppliers[i].changed();
        }
        if(changed) {
            if(isDouble) {
                if(isArray) {
                    ((FloatArrayLogEntry)entry).append(floatData);
                    if(updateNetworkTable) {
                        for(int i = 0; i < ntPublisher.length; i++)
                            ((FloatPublisher)ntPublisher[i]).accept(floatData[i]);
                    }
                } else {
                    ((FloatLogEntry)entry).append(floatData[0]);
                    if(updateNetworkTable) {
                        for(int i = 0; i < ntPublisher.length; i++)
                            ((FloatPublisher)ntPublisher[i]).accept(floatData[i]);
                    }
                }
            } else if(isArray) {
                ((BooleanArrayLogEntry)entry).append(booleanData);
                if(updateNetworkTable) {
                    for(int i = 0; i < ntPublisher.length; i++)
                        ((BooleanPublisher)ntPublisher[i]).accept(booleanData[i]);
        }
            } else {
                ((BooleanLogEntry)entry).append(booleanData[0]);
                if(updateNetworkTable) {
                    for(int i = 0; i < ntPublisher.length; i++)
                        ((BooleanPublisher)ntPublisher[i]).accept(booleanData[i]);
                }
            }
        }
    }
    
    public void removeInComp() {
      if (logTarget == LOG_TARGET.LOG_NT_NOT_COMPETIION) {
        updateNetworkTable = false;
      }
    }
  }