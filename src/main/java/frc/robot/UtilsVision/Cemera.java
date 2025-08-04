package frc.robot.UtilsVision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Cemera {
    private String name;
    private Translation2d robotToCamPos;
    private double CamHight;
    private double pitch;
    private double yaw;

    public Cemera(String name,Translation2d robotToCamPos,double CamHight,double pitch,double yaw){
        this.name = name;
        this.robotToCamPos = robotToCamPos;
        this.CamHight = CamHight;
        this.pitch = pitch;
        this.yaw = yaw;
    }
        public Translation2d getRobotToCamPosition() {
        return robotToCamPos;
    }

    public double getCamHeight() {
        return CamHight;
    }

    public double getPitch() {
        return this.pitch;
    }

    public double getYaw() {
        return this.yaw;
    }

    public String getName() {
        return this.name;
    }
}
