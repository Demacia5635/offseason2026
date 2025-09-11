package frc.Demacia.utils.Log;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.constants.UtilsContants.ConsoleConstants;

public class LogManager extends SubsystemBase {

  public static enum LOG_TARGET {LOG_AND_NT, LOG_ONLY, LOG_NOT_COMPETIION, LOG_NT_NOT_COMPETIION}
  
  public static LogManager logManager = new LogManager(); // singelton reference

  DataLog log;
  private NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  NetworkTable table = ntInst.getTable("Log");

  private static ArrayList<ConsoleAlert> activeConsole;

  // array of log entries
  ArrayList<LogEntry> logEntries = new ArrayList<>();

  // Log managerconstructor
  public LogManager() {

    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    log = DataLogManager.getLog();
    DriverStation.startDataLog(log);
    
    activeConsole = new ArrayList<>();
    log("log manager is ready");
  }


  public static void removeInComp() {
    for (int i = 0; i < LogManager.logManager.logEntries.size(); i++) {
      LogManager.logManager.logEntries.get(i).removeInComp();
      if (LogManager.logManager.logEntries.get(i).logTarget == LOG_TARGET.LOG_NOT_COMPETIION) {
        LogManager.logManager.logEntries.remove(LogManager.logManager.logEntries.get(i));
        i--;
      }
    }
  }

  
  /*
   * Log text message - also will be sent System.out
   */
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

  @Override
  public void periodic() {
    LogEntry.periodic();
  }
}