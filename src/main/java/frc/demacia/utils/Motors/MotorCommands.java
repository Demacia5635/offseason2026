package frc.Demacia.utils.Motors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MotorCommands {

    
    public static Command getPositionCommand(String name, Subsystem subsystem, MotorInterface...motors) {
        String fldName = name + ":";
        SmartDashboard.putNumber(fldName, 0);
        return new RunCommand(()->{
            double p = SmartDashboard.getNumber(fldName, 0);
            for(MotorInterface motor : motors) {
                motor.setPositionVoltage(p);
            }
        }, subsystem)
            .finallyDo((boolean b)-> {
                for(MotorInterface motor : motors) {
                    motor.setDuty(0);
                }
    
            });
    }
    public static Command getAngleCommand(String name, Subsystem subsystem, MotorInterface...motors) {
        String fldName = name + ":";
        SmartDashboard.putNumber(fldName, 0);
        return new RunCommand(()->{
            double p = SmartDashboard.getNumber(fldName, 0);
            for(MotorInterface motor : motors) {
                motor.setAngle(p);
            }
        }, subsystem)
            .finallyDo((boolean b)-> {
                for(MotorInterface motor : motors) {
                    motor.setDuty(0);
                }
    
            });
    }

    public static Command getMotionCommand(String name, Subsystem subsystem, MotorInterface...motors) {
        String fldName = name + ":";
        SmartDashboard.putNumber(fldName, 0);
        return new RunCommand(()->{
            double p = SmartDashboard.getNumber(fldName, 0);
            for(MotorInterface motor : motors) {
                motor.setMotion(p);
            }
        }, subsystem)
            .finallyDo((boolean b)-> {
                for(MotorInterface motor : motors) {
                    motor.setDuty(0);
                }
    
            });
    }

    public static Command getVelocityCommand(String name, Subsystem subsystem, MotorInterface...motors) {
        String fldName = name + ":";
        SmartDashboard.putNumber(fldName, 0);
        return new RunCommand(()->{
            double p = SmartDashboard.getNumber(fldName, 0);
            for(MotorInterface motor : motors) {
                motor.setVelocity(p);
            }
        }, subsystem)
            .finallyDo((boolean b)-> {
                for(MotorInterface motor : motors) {
                    motor.setDuty(0);
                }
    
            });
    }

    public static Command getPowerCommand(String name, Subsystem subsystem, MotorInterface...motors) {
        String fldName = name + ":";
        SmartDashboard.putNumber(fldName, 0);
        return new RunCommand(()->{
            double p = SmartDashboard.getNumber(fldName, 0);
            for(MotorInterface motor : motors) {
                motor.setDuty(p);
            }
        }, subsystem)
            .finallyDo((boolean b)-> {
                for(MotorInterface motor : motors) {
                    motor.setDuty(0);
                }
    
            });
    }
    public static Command getRandomPowerCommand(String name, double minPower, double maxPower, double rampTime, Subsystem subsystem, MotorInterface...motors) {
        RandomPowerGenerator generator = new RandomPowerGenerator(minPower,maxPower,rampTime);
        return new RunCommand(()->{
            double p = generator.next();
            System.out.println(" random power - " + p);
            for(MotorInterface motor : motors) {
                motor.setDuty(p/12.0);
            }
        }, subsystem)
            .beforeStarting(()->generator.reset(), subsystem)
            .finallyDo((boolean b)-> {
                for(MotorInterface motor : motors) {
                    motor.setDuty(0);
                }
    
            });
    }
    public static Command getSlowPowerCommand(String name, double minPower, double deltaPower, double waitTime, Subsystem subsystem, MotorInterface...motors) {
        SlowPowerGenerator generator = new SlowPowerGenerator(minPower, deltaPower, waitTime);
        return new RunCommand(()->{
            double p = generator.next();
            for(MotorInterface motor : motors) {
                motor.setDuty(p/12.0);
            }
        }, subsystem)
            .beforeStarting(()->generator.reset(), subsystem)
            .finallyDo((boolean b)-> {
                for(MotorInterface motor : motors) {
                    motor.setDuty(0);
                }
    
            });
    }


    public static void showRandomPowerCommand(String name, double minPower, double maxPower, double rampTime, Subsystem subsystem, MotorInterface...motors) {
        SmartDashboard.putData(name, getRandomPowerCommand(name, minPower, maxPower, rampTime, subsystem, motors));
    }
    public static void showPositionCommand(String name, Subsystem subsystem, MotorInterface...motors) {
        SmartDashboard.putData(name, getPositionCommand(name, subsystem, motors));
    }

    public static void showAngleCommand(String name, Subsystem subsystem, MotorInterface...motors) {
        SmartDashboard.putData(name, getAngleCommand(name, subsystem, motors));
    }

    public static void showMotionCommand(String name, Subsystem subsystem, MotorInterface...motors) {
        SmartDashboard.putData(name, getMotionCommand(name, subsystem, motors));
    }

    public static void showVelocityCommand(String name, Subsystem subsystem, MotorInterface...motors) {
        SmartDashboard.putData(name, getVelocityCommand(name, subsystem, motors));
    }

    public static void showPowerCommand(String name, Subsystem subsystem, MotorInterface...motors) {
        SmartDashboard.putData(name, getPowerCommand(name, subsystem, motors));
    }
    public static void showSlowPowerCommand(String name, double minPower, double stepPower, double waitTime, Subsystem subsystem, MotorInterface...motors) {
        SmartDashboard.putData(name, getSlowPowerCommand(name, minPower, stepPower, waitTime, subsystem, motors));
    }

}
