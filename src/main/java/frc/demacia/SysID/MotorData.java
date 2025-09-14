package frc.Demacia.Sysid;
import java.util.ArrayList;
import java.util.Vector;

import edu.wpi.first.util.datalog.DataLogRecord;

public class MotorData  {

    protected static Vector<LogDataEntry> motors = new Vector<>();

    LogDataEntry entry;
    ArrayList<MotorTimeData> data = new ArrayList<>();
    double maxVelocity = 0;
    double minPowerToMove = 12;


    public MotorData(LogDataEntry entry) {
        this.entry = entry;
        createData();
    }
    private void createData() {
        entry.resetIterator();
        double maxVolt = 0;
        DataLogRecord record = entry.next();
        while(record != null) {
            try {
                float[] d = record.getFloatArray();
                data.add(new MotorTimeData(d[2], d[1],d[3],d[0],record.getTimestamp()));
                if(Math.abs(d[2]) > maxVelocity) {
                    maxVelocity = Math.abs(d[2]);
                }
                if(Math.abs(d[0]) > maxVolt) {
                    maxVolt = Math.abs(d[0]);
                }
            } catch (Exception e) {
                System.err.println("Error getting double array for " + entry.name  +  " after " + data.size() + "  : " + e);
            }
            record = entry.next();
        }
        updateAcceleration();
    }

    private void updateAcceleration() {
        MotorTimeData prev = null;
        for(MotorTimeData m : data) {
            m.rawAcceleration = m.acceleration;
            if(prev != null) {
                double deltaTime = (m.time - prev.time)/1000.0;
                double acc = (m.velocity - prev.velocity) / deltaTime;
                m.acceleration = (m.acceleration * deltaTime + acc * 0.02) / (deltaTime + 0.02);
                double absVolt = Math.abs(m.voltage);
                if(prev.velocity == 0 && m.velocity != 0 && absVolt > 0.01 && (m.velocity*m.voltage) > 0  && absVolt < minPowerToMove) {
                    minPowerToMove = absVolt;   
                }
                m.prev = prev;
            }
            prev = m;
        }
    }

    public double maxVelocity() {
        return maxVelocity;
    }
    public ArrayList<MotorTimeData> data() {
        return data;
    }


    public class MotorTimeData {

        public double velocity;
        public double position;
        public double acceleration;
        public double rawAcceleration;
        public double voltage;
        public long time;
        public MotorTimeData prev = null;

        public MotorTimeData(double velocity, double position, double acceleration, double voltage, long time) {
            this.velocity = velocity;
            this.position = position;
            this.acceleration = acceleration;
            this.voltage = voltage;
            this.time = time;
            maxVelocity = Math.max(maxVelocity, Math.abs(velocity));
        }

        @Override
        public String toString() {
            if(prev == null) {
                return String.format("volt=%4.2f  vel=%5.2f  acc=%5.2f  pos=%6.2f", voltage, velocity, rawAcceleration, position);
            } else {
                int deltaTime  = (int)((time - prev.time)/1000.0);
                return String.format("volt=%4.2f-%4.2f  vel=%5.2f-%5.2f  acc=%5.2f-%5.2f  pos=%6.2f-%6.2f  timeDiff=%d", 
                    prev.voltage, voltage, prev.velocity, velocity, prev.rawAcceleration, rawAcceleration, prev.position, position, deltaTime);
            }
        }
    }
 
}