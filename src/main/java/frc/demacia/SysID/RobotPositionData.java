package frc.Demacia.Sysid;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.util.datalog.DataLogRecord;
import frc.Demacia.Geometry.Pose2d;
import frc.Demacia.Geometry.Rotation2d;

/**
 * Reads robot position data from WPILib logs for field visualization
 */
public class RobotPositionData {

    public static RobotPositionData robotPositionData = null;
    
    public static class PositionData {
        public final double timestamp;
        public final Pose2d pose;
        
        public PositionData(double timestamp, Pose2d pose) {
            this.timestamp = timestamp;
            this.pose = pose;
        }
    }
    
    private List<PositionData> positions = new ArrayList<>();
    private LogDataEntry swerve;
    private boolean debug = false;
    
    public RobotPositionData(LogDataEntry swerve) {
        this.swerve = swerve;
        robotPositionData = this;
    }
        
    public void extractPositions() {
        if(swerve != null && !swerve.records.isEmpty()) {
            for(DataLogRecord r : swerve.records) {
                double timestamp = r.getTimestamp() / 1_000_000.0; // Convert to seconds
                double data[] = r.getDoubleArray();
                double x = data[16];
                double y = data[17];
                Rotation2d heading = Rotation2d.fromDegrees(data[18]);
                
                Pose2d pose = new Pose2d(x, y, heading);
                positions.add(new PositionData(timestamp, pose));
                
            }
        }
        
        if(debug) {
            System.out.println("Extracted " + positions.size() + " position samples");
        }
    }
    
    public List<PositionData> getPositions() {
        return positions;
    }
    
    public void printPositions() {
        System.out.println("Robot Positions:");
        for(PositionData pos : positions) {
            System.out.printf("Time: %.3fs, X: %.2fm, Y: %.2fm, Heading: %.1f°%n", 
                pos.timestamp, 
                pos.pose.getX(), 
                pos.pose.getY(), 
                Math.toDegrees(pos.pose.getRotation().getRadians()));
        }
    }
}