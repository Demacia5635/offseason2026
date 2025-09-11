package frc.Demacia.utils.Elastic;

import java.util.function.Consumer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UpdateArray {
/**
     * creates a widget in elastic of the pid and ff for hot reload
     * 
     * @param slot the slot of the close loop perams (from 0 to 2)
     */
    public static void show(String name, String[] names, double[] data, Consumer<double[]> update) {
        SmartDashboard.putData(name, new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("ArrayUpdate");
                for(int i = 0; i < names.length; i++) {
                    DisplayDoubleFromArray d = new DisplayDoubleFromArray(data, i, names[i]);
                    builder.addDoubleProperty(d.name, d::get, d::update);
                }
                builder.addBooleanProperty("Update", () -> false,
                        (value) -> {System.out.println("update"); if(value) {update.accept(data);}});
            }
        });
    }

    public static class DisplayDoubleFromArray {
        int i;
        double[] array;
        String name;
        protected DisplayDoubleFromArray(double[] array, int index, String name) {
            this.i = index;
            this.array = array;
            this.name = name;
        }
        protected void update(double value) {
            array[i] = value;
        }
        protected double get() {
            return array[i];
        }
    }
}
