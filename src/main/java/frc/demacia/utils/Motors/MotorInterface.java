package frc.demacia.utils.Motors;

public interface MotorInterface {

    String name();

    void changeSlot(int slot);    // change the slot used in the motor for PID/FF calculation
    void setNeutralMode(boolean isBrake);
    void setDuty(double power); // power -1 to 1
    void setVoltage(double voltage);
    void setVelocity(double velocity, double feedForward); // velocity with additional calculated volts
    void setVelocity(double velocity);
    void setMotion(double position, double feedForward); // position profiled motion in motor with feedForward value calculated in roborio
    void setMotion(double position);
    void setAngle(double angle, double feedForward); // position profiled motion in motor with feedForward value calculated in roborio
    void setAngle(double angle);
    void setPositionVoltage(double position, double feedForward);
    void setPositionVoltage(double position);
    void setVelocityWithFeedForward(double velocity);
    void setMotionWithFeedForward(double velocity);

    String getCurrentControlMode();
    double getCurrentClosedLoopSP();
    double getCurrentClosedLoopError();
    double getCurrentPosition();
    double getCurrentAngle(); // for radians and degrees motor
    double getCurrentVelocity();
    double getCurrentAcceleration();
    double getCurrentVoltage();
    double getCurrentCurrent();

    public void checkElectronics();

    void setEncoderPosition(double position);

    void showConfigPIDFSlotCommand(int slot);
    void showConfigMotionVelocitiesCommand();

}
