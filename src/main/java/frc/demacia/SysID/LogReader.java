package frc.Demacia.Sysid;
import java.io.IOException;
import java.util.Arrays;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;

/**
 * Class to read the log
 * Create the hirerchy of entries
 * Crerate the list of motors
 */
public class LogReader {

    LogDataEntry[] entries = new LogDataEntry[1000]; // array of entries - assume that no more than 1000
    boolean debug = false;

    /**
     * Create the Log Reader
     * Can throw IO exception
     * @param file
     * @throws IOException
     */
    public LogReader(String file) throws IOException {
        this(file, false);
    }
    public LogReader(String file, boolean debug) throws IOException {
        this.debug = debug;
        Arrays.fill(entries, null); // set all entries to null
        try {
            DataLogReader reader = new DataLogReader(file);
            reader.forEach(this::process); // process all records
        } catch (IOException e) {
            System.err.println(" Can not open Log File " + file + " error:" + e.getMessage());
            throw e;
        }
    }

    /**
     * process the record from the file
     * @param record
     */
    private void process(DataLogRecord record) {
        if(record.isStart()) { // definition of a new data field
            var startRecord = record.getStartData();
            entries[startRecord.entry] =  new LogDataEntry(startRecord); // add the data to the entries
            if(debug) {
                System.out.println(" added " + startRecord.entry + " " + startRecord.name + " type=" + startRecord.type + " meta=" + startRecord.metadata);
            }
        } else if(!record.isControl() && !record.isSetMetadata()) { // ignore end/meta record
            entries[record.getEntry()].add(record);
        }
    }

    

    public static void main(String[] args) {
        String fileName = "D:\\Projects\\2026Training\\Demacia\\Training2026\\logs\\FRC_20250721_102912.wpilog";
        try {
            new LogReader(fileName, true);
        } catch(Exception e) {
            e.printStackTrace();
        }
    }

}
