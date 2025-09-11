package frc.Demacia.utils;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;

public class StatusSignalData<T> {

    @SuppressWarnings("rawtypes")
    private static ArrayList<StatusSignalData> signals = new ArrayList<>();


    StatusSignal<T> signal;
    double lastValue;
    double multiplier = 1;

    public StatusSignalData(StatusSignal<T> signal) {
        this.signal = signal;
        signal.refresh();
        lastValue = signal.getValueAsDouble();
        signals.add(this);
    }
    public StatusSignalData(StatusSignal<T> signal, double multiplier) {
        this(signal);
        setMultiplier(multiplier);
    }

    public void setMultiplier(double multiplier) {
        this.multiplier = multiplier;
    }

    public double get() {
        signal.refresh();
        if(signal.getStatus().isOK()) {
            lastValue = signal.getValueAsDouble();
        }
        return lastValue * multiplier;
    }

    public T getValue() {
        return signal.getValue();
    }

    public String getString() {
        signal.refresh();
        return signal.getValue().toString();
    }

    public StatusSignal<T> signal() {
        return signal;
    }

    @SuppressWarnings({ "rawtypes", "unchecked" })
    public void refreshAll() {
        for(StatusSignalData s : signals) {
            s.signal.refresh();
            if(s.signal.getStatus().isOK()) {
                s.lastValue = s.signal.getValueAsDouble();
            }
        }
    }

}
