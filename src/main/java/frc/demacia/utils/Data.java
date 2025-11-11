package frc.demacia.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

public class Data<T> {
    
    private static ArrayList<Data<?>> signals = new ArrayList<>();

    private StatusSignal<T>[] signal;
    private Supplier<T>[] supplier;
    private Supplier<T>[] oldSupplier;
    private T[] currentValues;
    private T[] previousValues;
    private double precision = 0;
    private int length;

    private boolean isDouble = false;
    private boolean isBoolean = false;
    private boolean isArray = false;

    @SuppressWarnings("unchecked")
    public Data(StatusSignal<T> ... signal){
        this.signal = signal;

        signals.add(this);
        
        length = signal.length;

        currentValues = (T[]) new Object[length];
        previousValues = (T[]) new Object[length];
        
        refresh();

        if (length > 1){
            isArray = true;
        }

        try {
            signal[0].getValueAsDouble();
            isDouble = true;
        } catch (Exception e) {
            if (signal[0].getValue() instanceof Boolean){
              isBoolean = true;
            }
        }
    }
    
    @SuppressWarnings("unchecked")
    public Data(Supplier<T> ... supplier){
        this.supplier = supplier;
        this.oldSupplier = supplier;

        signals.add(this);

        T value = supplier[0].get();

        if (value == null){
            length = 0;
            currentValues = (T[]) new Object[0];
            previousValues = (T[]) new Object[length];
            return;
        }

        length = supplier.length;
        currentValues = (T[]) new Object[length];
        
        refresh();

        if (value.getClass().isArray()) {
            isArray = true;
            int arrayLength = java.lang.reflect.Array.getLength(value);
            oldSupplier = new Supplier[java.lang.reflect.Array.getLength(value)];
            final T finalValue = value;
            for (int i = 0; i < arrayLength; i++){
                final int index = i;
                oldSupplier[i] = () -> (T) java.lang.reflect.Array.get(finalValue, index);
            }
            try {
                Object first = java.lang.reflect.Array.get(value, 0);
                ((Number) first).doubleValue();
                isDouble = true;
            } catch (Exception e) {
                if (value instanceof boolean[] || value instanceof Boolean[]){
                    isBoolean = true;
                }
            }
        } else {
            try {
                ((Number) value).doubleValue();
                isDouble = true;
            } catch (Exception e) {
                if (value instanceof  Boolean){
                    isBoolean = true;
                }
            }
            if (length > 1){
                isArray = true;
                this.oldSupplier = Arrays.copyOf(supplier, supplier.length);
                T[] valueArray = (T[]) new Object[length];
                for (int i = 0; i < length; i++){
                    valueArray[i] = supplier[i].get();
                }
                this.supplier = new Supplier[] {() -> valueArray};
                length = 1;
            }
        }
    }

    @SuppressWarnings("unchecked")
    public void expandWithSignals(StatusSignal<T>... newSignals) {
        if (signal == null || newSignals == null || newSignals.length == 0) {
            return;
        }
        
        int oldLength = signal.length;
        int newLength = oldLength + newSignals.length;
        
        StatusSignal<T>[] expandedSignals = new StatusSignal[newLength];
        System.arraycopy(signal, 0, expandedSignals, 0, oldLength);
        System.arraycopy(newSignals, 0, expandedSignals, oldLength, newSignals.length);
        signal = expandedSignals;
        
        T[] expandedCurrent = (T[]) new Object[newLength];
        T[] expandedPrevious = (T[]) new Object[newLength];
        
        if (currentValues != null) {
            System.arraycopy(currentValues, 0, expandedCurrent, 0, Math.min(oldLength, currentValues.length));
        }
        if (previousValues != null) {
            System.arraycopy(previousValues, 0, expandedPrevious, 0, Math.min(oldLength, previousValues.length));
        }
        
        currentValues = expandedCurrent;
        previousValues = expandedPrevious;
        length = newLength;
        
        if (length > 1) {
            isArray = true;
        }
        
        refresh();
    }
    
    @SuppressWarnings("unchecked")
    public void expandWithSuppliers(Supplier<T>... newSuppliers) {
        if (supplier == null || newSuppliers == null || newSuppliers.length == 0) {
            return;
        }
    
        Supplier<?>[] existingSuppliers = getSuppliers();
        
        Supplier<T>[] combined = new Supplier[existingSuppliers.length + newSuppliers.length];
        System.arraycopy(existingSuppliers, 0, combined, 0, existingSuppliers.length);
        for (int j = 0; j < newSuppliers.length; j++) {
            combined[existingSuppliers.length + j] = (Supplier<T>) newSuppliers[j];
        }
        
        T value = combined[0].get();
        
        if (value == null) {
            supplier = combined;
            oldSupplier = combined;
            length = 0;
            currentValues = (T[]) new Object[0];
            previousValues = (T[]) new Object[0];
            return;
        }
        
        if (value.getClass().isArray()) {
            isArray = true;
            int arrayLength = java.lang.reflect.Array.getLength(value);
            oldSupplier = new Supplier[arrayLength];
            final T finalValue = value;
            for (int i = 0; i < arrayLength; i++) {
                final int index = i;
                oldSupplier[i] = () -> (T) java.lang.reflect.Array.get(finalValue, index);
            }
            supplier = new Supplier[] {combined[0]};
            length = 1;
        } else {
            if (combined.length > 1) {
                isArray = true;
                oldSupplier = Arrays.copyOf(combined, combined.length);
                T[] valueArray = (T[]) new Object[combined.length];
                for (int i = 0; i < combined.length; i++) {
                    valueArray[i] = combined[i].get();
                }
                supplier = new Supplier[] {() -> valueArray};
                length = 1;
            } else {
                oldSupplier = combined;
                supplier = combined;
                length = combined.length;
            }
        }
        
        currentValues = (T[]) new Object[length];
        previousValues = (T[]) new Object[length];
        
        refresh();
    }

    @SuppressWarnings("unchecked")
    public void removeSignalRange(int startIndex, int count) {
        if (signal == null || signal.length == 0 || count <= 0) return;

        StatusSignal<T>[] newSignals = new StatusSignal[length - count];
        int c = 0;
        for (int i = 0; i < length; i++) {
            if (i < startIndex || i >= startIndex + count) {
                newSignals[i - c] = signal[i];
            } else {
                c += 1;
            }
        }

        signal = newSignals;
        length = newSignals.length;
        
        currentValues = (T[]) new Object[length];
        previousValues = (T[]) new Object[length];
        
        refresh();
        
        if (length > 1) {
            isArray = true;
        } else {
            isArray = false;
        }
    }
    
    public void removeSignal(int startIndex){
        removeSignalRange(startIndex, 1);
    }


    @SuppressWarnings("unchecked")
    public void removeSupplierRange(int startIndex, int count) {
        if (oldSupplier == null || oldSupplier.length == 0 || count <= 0) return;
        if (startIndex < 0 || startIndex >= oldSupplier.length) return;
        
        count = Math.min(count, oldSupplier.length - startIndex);
        
        int newLength = oldSupplier.length - count;
        Supplier<T>[] newOldSuppliers = new Supplier[newLength];
        
        System.arraycopy(oldSupplier, 0, newOldSuppliers, 0, startIndex);
        
        if (startIndex + count < oldSupplier.length) {
            System.arraycopy(oldSupplier, startIndex + count, newOldSuppliers, 
                            startIndex, oldSupplier.length - startIndex - count);
        }
        
        oldSupplier = newOldSuppliers;
        
        T value = newOldSuppliers.length > 0 ? newOldSuppliers[0].get() : null;
        
        if (value == null) {
            supplier = newOldSuppliers;
            length = 0;
            currentValues = (T[]) new Object[0];
            previousValues = (T[]) new Object[0];
            isArray = false;
            return;
        }
        
        if (value.getClass().isArray()) {
            isArray = true;
            int arrayLength = java.lang.reflect.Array.getLength(value);
            Supplier<T>[] arraySuppliers = new Supplier[arrayLength];
            final T finalValue = value;
            for (int i = 0; i < arrayLength; i++) {
                final int index = i;
                arraySuppliers[i] = () -> (T) java.lang.reflect.Array.get(finalValue, index);
            }
            oldSupplier = arraySuppliers;
            supplier = new Supplier[] {newOldSuppliers[0]};
            length = 1;
        } else {
            if (newOldSuppliers.length > 1) {
                isArray = true;
                T[] valueArray = (T[]) new Object[newOldSuppliers.length];
                for (int i = 0; i < newOldSuppliers.length; i++) {
                    valueArray[i] = newOldSuppliers[i].get();
                }
                supplier = new Supplier[] {() -> valueArray};
                length = 1;
            } else {
                supplier = newOldSuppliers;
                length = newOldSuppliers.length;
                isArray = false;
            }
        }
        
        currentValues = (T[]) new Object[length];
        previousValues = (T[]) new Object[length];
        
        refresh();
    }

    public void removeSupplier(int startIndex){
        removeSupplierRange(startIndex, 1);
    }

    public Double getDouble() {
        if (!isDouble || length == 0) {return null;}
        refresh();
        if (signal != null){
            return (double)signal[0].getValueAsDouble();
        }
        else {
            return toDouble(currentValues[0]);
        }
    }

    public double[] getDoubleArray() {
        if (!isDouble || length == 0) {return null;}
        refresh();
        if (signal != null){
            double[] doubleArray = new double[signal.length];
                for (int i = 0; i < signal.length; i++) {
                    doubleArray[i] = (double)signal[i].getValueAsDouble();
                }
                return doubleArray;
        }
        else {
            return toDoubleArray(currentValues);
        }
    }

    public Float getFloat() {
        if (!isDouble || length == 0) {return null;}
        refresh();
        if (signal != null){
            return (float) signal[0].getValueAsDouble();
        }
        else {
            Double d = toDouble(currentValues[0]);
            return d != null ? d.floatValue() : null;
        }
    }

    public float[] getFloatArray() {
        if (!isDouble || length == 0) {return null;}
        refresh();
        if (signal != null){
            float[] floatArray = new float[signal.length];
                for (int i = 0; i < signal.length; i++) {
                    floatArray[i] = (float)signal[i].getValueAsDouble();
                }
                return floatArray;
        }
        else {
            return toFloatArray(currentValues);
        }
    }

    public Boolean getBoolean() {
        if (!isBoolean || length == 0) {return null;}
        refresh();
        if (signal != null){
            return (Boolean) signal[0].getValue();
        }
        else {
            return (Boolean) currentValues[0];
        }
    }

    public boolean[] getBooleanArray() {
        if (!isBoolean || length == 0) {return null;}
        refresh();
        if (signal != null){
            boolean[] booleanArray = new boolean[signal.length];
                for (int i = 0; i < signal.length; i++) {
                    booleanArray[i] = (Boolean)signal[i].getValue();
                }
                return booleanArray;
        }
        else {
            return toBooleanArray(currentValues);
        }
    }

    public String getString() {
        if (isDouble || isBoolean || length == 0) {return "";}
        refresh();
        if (signal != null){
            return signal[0].getValue().toString();
        }
        else {
            return currentValues[0] != null ? currentValues[0].toString() : "";
        }
    }

    public String[] getStringArray() {
        if (isDouble || isBoolean || length == 0) {return null;}
        refresh();
        if (signal != null){
            String[] stringArray = new String[signal.length];
                for (int i = 0; i < signal.length; i++) {
                    stringArray[i] = signal[i].getValue().toString();
                }
                return stringArray;
        }
        else {
            return toStringArray(currentValues);
        }
    }

    public T getValue() {
        if (length == 0) {return null;}
        refresh();
        return currentValues[0];
    }

    public T[] getValueArray() {
        if (length == 0) {return null;}
        refresh();
        return currentValues;
    }

    public StatusSignal<T> getSignal() {
        return (signal != null && signal.length > 0) ? signal[0] : null;
    }

    public StatusSignal<T>[] getSignals() {
        return signal;
    }

    public Supplier<T> getSupplier() {
        return (oldSupplier != null && oldSupplier.length > 0) ? oldSupplier[0] : null;
    }

    public Supplier<T>[] getSuppliers() {
        return oldSupplier;
    }

    public void refresh() {
        if (currentValues != null) {
            previousValues = Arrays.copyOf(currentValues, currentValues.length);
            for (int i = 0; i < previousValues.length; i++) {
                if (previousValues[i] != null && previousValues[i].getClass().isArray()) {
                    previousValues[i] = copyArray(previousValues[i]);
                }
            }
        }
        if (signal != null) {
            StatusCode st = StatusSignal.refreshAll(signal);
            if(st == StatusCode.OK) {
                for (int i = 0; i < length; i++){
                    currentValues[i] = signal[i].getValue();
                }
            }
        } else {
            for (int i = 0; i < length; i++){
                currentValues[i] = supplier[i].get();
            }
        }
    }

    @SuppressWarnings("rawtypes")
    public static void refreshAll() {
        for(Data s : signals) {
            s.refresh();
        }
    }

    public boolean hasChanged() {
        if (previousValues == null) {
            return true;
        }

        if (length == 0) {
            return false;
        }

        if (length > 1) {
            for (int i = 0; i < length; i++) {
                if (hasValueChanged(currentValues[i], previousValues[i])) {
                    return true;
                }
            }
            return false;
        } else {
            return hasValueChanged(currentValues[0], previousValues[0]);
        }
    }

    private boolean hasValueChanged(T newValue, T oldValue) {
        if (oldValue == null) {
            return newValue != null;
        }
        
        if (newValue == null) {
            return true;
        }

        if (newValue.getClass().isArray()) {
            return hasArrayChanged(newValue, oldValue);
        } else {
            if (isDouble) {
                try {
                    double newVal = ((Number) newValue).doubleValue();
                    double oldVal = ((Number) oldValue).doubleValue();
                    return Math.abs(newVal - oldVal) >= precision;
                } catch (ClassCastException e) {
                    return !newValue.toString().equals(oldValue.toString());
                }
            } else if (isBoolean) {
                return !newValue.equals(oldValue);
            } else {
                return !newValue.toString().equals(oldValue.toString());
            }
        }
    }

    public void setPrecision(double precision) {
        this.precision = precision;
    }

    public double getPrecision() {
        return precision;
    }

    public long getTime() {
        if (signal != null) {
            return (long) (signal[0].getTimestamp().getTime() * 1000);
        }
        return 0;
    }

    public boolean isDouble(){
        return isDouble;
    }

    public boolean isBoolean(){
        return isBoolean;
    }

    public boolean isArray(){
        return isArray;
    }

    private boolean hasArrayChanged(T newArray, T oldArray) {
        if (oldArray == null) {
            return newArray != null;
        }
        
        if (newArray == null) {
            return true;
        }

        int newLength = java.lang.reflect.Array.getLength(newArray);
        int oldLength = java.lang.reflect.Array.getLength(oldArray);
        
        if (newLength != oldLength) {
            return true;
        }

        if (isDouble) {
            for (int i = 0; i < newLength; i++) {
                Object newElem = java.lang.reflect.Array.get(newArray, i);
                Object oldElem = java.lang.reflect.Array.get(oldArray, i);
                
                if (newElem == null && oldElem == null) continue;
                if (newElem == null || oldElem == null) return true;
                
                try {
                    double newVal = ((Number) newElem).doubleValue();
                    double oldVal = ((Number) oldElem).doubleValue();
                    
                    if (Math.abs(newVal - oldVal) >= precision) {
                        return true;
                    }
                } catch (ClassCastException e) {
                    if (!newElem.toString().equals(oldElem.toString())) {
                        return true;
                    }
                }
            }
            return false;
        } else if (isBoolean) {
            for (int i = 0; i < newLength; i++) {
                Object newElem = java.lang.reflect.Array.get(newArray, i);
                Object oldElem = java.lang.reflect.Array.get(oldArray, i);
                
                if (!Objects.equals(newElem, oldElem)) {
                    return true;
                }
            }
            return false;
        } else {
            for (int i = 0; i < newLength; i++) {
                Object newElem = java.lang.reflect.Array.get(newArray, i);
                Object oldElem = java.lang.reflect.Array.get(oldArray, i);
                
                String newStr = (newElem != null) ? newElem.toString() : null;
                String oldStr = (oldElem != null) ? oldElem.toString() : null;
                
                if (!Objects.equals(newStr, oldStr)) {
                    return true;
                }
            }
            return false;
        }
    }

    @SuppressWarnings("unchecked")
    private T copyArray(T array) {
        if (array == null) return null;
        
        Class<?> componentType = array.getClass().getComponentType();
        int length = java.lang.reflect.Array.getLength(array);
        Object newArray = java.lang.reflect.Array.newInstance(componentType, length);
        
        System.arraycopy(array, 0, newArray, 0, length);
        return (T) newArray;
    }

    private double[] toDoubleArray(T[] value){
        if (value == null) return null;

        if (isArray){
            int arrayLength  = java.lang.reflect.Array.getLength(value[0]);
            double[] doubleArr = new double[arrayLength];
            for (int i = 0; i < arrayLength; i++) {
                Object elem = java.lang.reflect.Array.get(value[0], i);
                doubleArr[i] = (elem != null) ? ((Number) elem).doubleValue() : 0.0;
            }
            return doubleArr;
        } else{
            double[] doubleArr = new double[length];
            for (int i = 0; i < length; i++) {
                Object elem = java.lang.reflect.Array.get(value, i);
                doubleArr[i] = (elem != null) ? ((Number) elem).doubleValue() : 0.0;
            }
            return doubleArr;
        }
    }

    private Double toDouble(T value){
        if (value == null) return null;

        if (isArray) {
            Object first = java.lang.reflect.Array.get(value, 0);
            return (first != null) ? ((Number) first).doubleValue() : null;
        }
        return ((Number) value).doubleValue();
    }

    private float[] toFloatArray(T[] value){
        if (value == null) return null;

        if (isArray){
            int arrayLength  = java.lang.reflect.Array.getLength(value[0]);
            float[] floatArr = new float[arrayLength];
            for (int i = 0; i < arrayLength; i++) {
                Object elem = java.lang.reflect.Array.get(value[0], i);
                floatArr[i] = (elem != null) ? ((Number) elem).floatValue() : 0f;
            }
            return floatArr;
        } else {
            float[] floatArr = new float[length];
            for (int i = 0; i < length; i++) {
                Object elem = java.lang.reflect.Array.get(value, i);
                floatArr[i] = (elem != null) ? ((Number) elem).floatValue() : 0f;
            }
            return floatArr;
        }
    }

    private boolean[] toBooleanArray(T[] value){
        if (value == null) return null;

        if (isArray){
            int arrayLength  = java.lang.reflect.Array.getLength(value[0]);
            boolean[] booleanArr = new boolean[arrayLength];
            for (int i = 0; i < arrayLength; i++) {
                Object elem = java.lang.reflect.Array.get(value[0], i);
                booleanArr[i] = (elem != null) ? (Boolean) elem : false;
            }
            return booleanArr;
        } else {
            boolean[] booleanArr = new boolean[length];
            for (int i = 0; i < length; i++) {
                Object elem = java.lang.reflect.Array.get(value, i);
                booleanArr[i] = (elem != null) ? (Boolean) elem : false;
            }
            return booleanArr;
        }
    }
    
    private String[] toStringArray(T[] value){
        if (value == null) return null;

        if (isArray){
            int arrayLength  = java.lang.reflect.Array.getLength(value[0]);
            String[] stringArr = new String[arrayLength];
            for (int i = 0; i < arrayLength; i++) {
                Object elem = java.lang.reflect.Array.get(value[0], i);
                stringArr[i] = (elem != null) ? elem.toString() : null;
            }
            return stringArr;
        } else {
            String[] stringArr = new String[length];
            for (int i = 0; i < length; i++) {
                Object elem = java.lang.reflect.Array.get(value, i);
                stringArr[i] = (elem != null) ? elem.toString() : null;
            }
            return stringArr;
        }
    }

    public void cleanup() {
        signals.remove(this);
    }
    
    public static void clearAllSignals() {
        signals.clear();
    }
}