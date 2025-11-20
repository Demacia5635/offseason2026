package frc.demacia.utils.Sensors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

public class StatusSignalHelper {
    
    /**
     * Generic status signal getter
     */
    @SuppressWarnings("rawtypes")
    public static double getStatusSignal(StatusSignal statusSignal, double lastValue, double multiplier) {
        statusSignal.refresh();
        if (statusSignal.getStatus() == StatusCode.OK) {
            return statusSignal.getValueAsDouble() * multiplier;
        }
        return lastValue;
    }
    
    /**
     * Status signal getter with conversion to radians
     */
    @SuppressWarnings("rawtypes")
    public static double getStatusSignalInRad(StatusSignal statusSignal, double lastValue) {
        return getStatusSignal(statusSignal, lastValue, Math.PI / 180);
    }
    
    /**
     * Status signal getter with 2π multiplier for full rotations
     */
    @SuppressWarnings("rawtypes")
    public static double getStatusSignalWith2Pi(StatusSignal statusSignal, double lastValue) {
        return getStatusSignal(statusSignal, lastValue, 2 * Math.PI);
    }
    
    /**
     * Basic status signal getter without multiplication
     */
    @SuppressWarnings("rawtypes")
    public static double getStatusSignalBasic(StatusSignal statusSignal, double lastValue) {
        return getStatusSignal(statusSignal, lastValue, 1.0);
    }
}