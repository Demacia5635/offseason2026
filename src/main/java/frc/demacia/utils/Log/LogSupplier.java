package frc.Demacia.utils.Log;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.function.BooleanConsumer;
import frc.Demacia.utils.StatusSignalData;

@SuppressWarnings("rawtypes")
public class LogSupplier {
    private DoubleSupplier doubleSupplier;
    private BooleanSupplier booleanSupplier;
    private StatusSignalData statusSignal;
    private boolean isFloat;
    private String name;
    private float lastValue = 0;
    private boolean lastBool = false;
    private float newValue = 0;
    private boolean newBool = false;
    private DoubleConsumer doubleConsumer = null;
    private BooleanConsumer booleanConsumer = null;


    public LogSupplier(DoubleSupplier supplier, String name, DoubleConsumer consumer) {
      statusSignal = null;
      doubleSupplier = supplier;
      booleanSupplier = null;
      doubleConsumer = consumer;
      isFloat = true;
      this.name = name;
    }
    public LogSupplier(StatusSignalData supplier, String name, DoubleConsumer consumer) {
      statusSignal = supplier;
      doubleSupplier = null;
      booleanSupplier = null;
      doubleConsumer = consumer;
      isFloat = true;
      this.name = name;
    }
    public LogSupplier(BooleanSupplier supplier, String name, BooleanConsumer consumer) {
      statusSignal = null;
      doubleSupplier = null;
      booleanSupplier = supplier;
      booleanConsumer = consumer;
      isFloat = false;
      this.name = name;
    }

    public LogSupplier(StatusSignalData<Boolean> supplier, String name, BooleanConsumer consumer) {
      statusSignal = supplier;
      doubleSupplier = null;
      booleanSupplier = null;
      booleanConsumer = consumer;
      isFloat = false;
      this.name = name;
    }
    
    float getFloat() {
      lastValue = newValue;
      if(statusSignal != null) {
          newValue =  (float)statusSignal.get();
      } else {
        newValue =  (float)doubleSupplier.getAsDouble();
      }
      if(newValue != lastValue && doubleConsumer != null) {
          doubleConsumer.accept(newValue);
      }
      return newValue;
    }

    boolean getBoolean() {
      lastBool = newBool;
      if(statusSignal != null) {
        newBool =  (Boolean)statusSignal.getValue();
      } else {
        newBool = booleanSupplier.getAsBoolean();
      }
      if(newBool != lastBool && booleanConsumer != null) {
          booleanConsumer.accept(newBool);
      }
      return newBool;
    }

    boolean isFloat() {
      return isFloat;
    }

    String name() {
      return name;
    }

    boolean changed() {
      return isFloat? lastValue != newValue : lastBool != newBool;
    }

  }