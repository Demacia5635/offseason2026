package frc.demacia.utils.Sensors;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import frc.demacia.utils.Log.LogManager;

public class ColorSensor extends ColorSensorV3 implements SensorInterface {

    private final ColorSensorConfig config;
    private final String name;
    private final ColorMatch matcher;
    private String colors;
    private Color lastColor;
    private ColorMatchResult lastMatch;
    private int lastProximity;

    public ColorSensor(ColorSensorConfig config) {
        super(I2C.Port.kOnboard); 
        this.config = config;
        this.name = config.name;
        this.matcher = new ColorMatch();

        addDefaultColors();
        updateReadings();
        addLog();

        LogManager.log(name + " color sensor initialized");
     
    }

    private void addDefaultColors() {
        matcher.addColorMatch(Color.kBlue);
        matcher.addColorMatch(Color.kRed);
        matcher.addColorMatch(Color.kGreen);
        matcher.addColorMatch(Color.kYellow);
    }

    private void updateReadings() {
        lastColor = getColor(); 
        lastMatch = matcher.matchClosestColor(lastColor);
        lastProximity = getProximity(); 
    }

    private void addLog() {
        LogManager.addEntry(name + " Color and Proximity", () -> new double[] {
            lastColor.red, lastColor.green, lastColor.blue, lastProximity
        }, 3);
        LogManager.addEntry(colors + "Colors values:", this::getRGBString, 3);
    }   
    public String getName() {
        return config.name;
        }

        public String getRedValue() {
        updateReadings();
        return Integer.toString((int)(lastColor.red * 255));
        }

        public String getGreenValue() {
        updateReadings();
        return Integer.toString((int)(lastColor.green * 255));
        }

        public String getBlueValue() {
         updateReadings();
        return Integer.toString((int)(lastColor.blue * 255));
            }

        public int getProximityValue() {
        updateReadings();
        return lastProximity;
    }

    public String getRGBString() {
        updateReadings();
        int r = (Integer.parseInt (getRedValue()));
        int g = (Integer.parseInt(getGreenValue()));
        int b = (Integer .parseInt(getBlueValue()));
        return r +  ", " + g + ", " + b;
    }
    public String getMatchedColor() {
        updateReadings();
        if (lastMatch.color == Color.kBlue) return "Blue";
        if (lastMatch.color == Color.kRed) return "Red";
        if (lastMatch.color == Color.kGreen) return "Green";
        if (lastMatch.color == Color.kYellow) return "Yellow";
        return "Unknown";
    }
    
}