package frc.Demacia.SysID;

import java.util.ArrayList;
import java.util.Iterator;

import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.datalog.DataLogRecord.StartRecordData;

/**
 * Class to store records of an entry
 * include the start record - with the name, type and meta
 * and all data records
 */
public class LogDataEntry {
    StartRecordData startRecord; // the start record - name, type, meta
    ArrayList<DataLogRecord> records = new ArrayList<>(); // array of data records - time/data
    String name = null;
    boolean isMotor = false;
    boolean isSwerve = false;
    MotorData motorData = null;
    private Iterator<DataLogRecord> iterator; // an iterator to iterate over the data

    /**
     * Constructor 
     * @param start record
     * @param logReader
     */
    LogDataEntry(StartRecordData start) {
        startRecord = start;
        name = start.name;
        isMotor = start.metadata != null && start.metadata.length() > 10 && start.metadata.startsWith("Type:Motor;");
        if(isMotor) {
            MotorData.motors.add(this);
        } else {
            isSwerve = start.metadata != null && start.metadata.length() > 10 && start.metadata.startsWith("Type:Swerve;");
        }
    }

    /**
     * add a data record
     * @param record
     */
    void add(DataLogRecord record) {
        records.add(record);
    }

    public MotorData getMotorData() {
        if(isMotor && motorData == null) {
            motorData = new MotorData(this);
        }
        return motorData;
    }

    /**
     * reset the iterator
     */
    void resetIterator() {
        iterator = records.iterator();
    }
    /**
     * get the next data from the iterator
     * @return
     */
    DataLogRecord next() {
        if(iterator.hasNext()) 
            return iterator.next();
        return null;
    }

    @Override
    public String toString() {
        return name + " have " + records.size() + " " + startRecord.type;
    }
}