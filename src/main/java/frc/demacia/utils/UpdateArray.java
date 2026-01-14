package frc.demacia.utils;

import java.util.function.Consumer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Dashboard widget for hot-reloading array values.
 * 
 * <p>Creates interactive dashboard elements for tuning without code redeployment.
 * Used internally by motor classes for PID tuning and configuration.</p>
 * 
 * <p><b>Example Usage:</b></p>
 * <pre>
 * double[] pidValues = {0.1, 0.0, 0.01};  // kP, kI, kD
 * String[] names = {"kP", "kI", "kD"};
 * 
 * UpdateArray.show("Shooter PID", names, pidValues, (updated) -> {
 *     motor.setPID(updated[0], updated[1], updated[2]);
 *     System.out.println("PID updated!");
 * });
 * 
 * // Dashboard will show editable fields for each value
 * // Press "Update" button to apply changes
 * </pre>
 * 
 * <p><b>Use Cases:</b></p>
 * <ul>
 *   <li>PID tuning during testing</li>
 *   <li>Feed-forward calibration</li>
 *   <li>Motion profile adjustment</li>
 *   <li>Mechanism position presets</li>
 * </ul>
 */
public class UpdateArray {

    /**
     * Creates a dashboard widget for array editing.
     * 
     * @param name Widget name (displayed on dashboard)
     * @param names Labels for each array element
     * @param data Initial data values (will be modified in place)
     * @param update Callback function when "Update" button is pressed
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
