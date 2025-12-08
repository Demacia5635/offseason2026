package frc.demacia.SysID;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;
import java.util.List;
import java.util.function.Consumer;

class SysidApp {
    public static void main(String[] args) {
        Sysid app = new Sysid();
        app.show();
    }
}

public class Sysid implements Consumer<File> {
    JFrame frame = new JFrame("Sysid");
    FileChooserPanel fileChooser = new FileChooserPanel(this);
    DefaultListModel<MotorData> listModel = new DefaultListModel<>();
    JList<MotorData> motorList = new JList<>(listModel);
    SysidResultPanel result = new SysidResultPanel(this);
    JTextArea msgArea = new JTextArea();
    JScrollPane msgPane = new JScrollPane(msgArea, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
    Map<String, LogReader.SysIDResults> analysisResults;

    private static Sysid sysid = null;

    public Sysid() {
        sysid = this;
        frame.setSize(1024,800);
        frame.setMinimumSize(new Dimension(1024,800));
        frame.setLocation(300,200);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        var pane = frame.getContentPane();
        pane.setLayout(new GridBagLayout());
        motorList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
        motorList.setMinimumSize(new Dimension(300,400));
        motorList.setBorder(BorderFactory.createEtchedBorder());
        
        motorList.addListSelectionListener(e -> {
            if (!e.getValueIsAdjusting()) {
                MotorData selected = motorList.getSelectedValue();
                if (selected != null) {
                    msg("Selected motor: " + selected.name);
                    result.updateDisplay(selected);
                }
            }
        });
        
        pane.add(fileChooser, new GridBagConstraints(0, 0, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.HORIZONTAL, new Insets(5, 5, 5, 0), 5, 5));
        pane.add(motorList, new GridBagConstraints(0, 1, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(5, 5, 5, 0), 5, 5));
        pane.add(msgPane, new GridBagConstraints(1, 0, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(5, 5, 5, 0), 5, 5));
        pane.add(result, new GridBagConstraints(1, 1, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(5, 5, 5, 0), 5, 5));
        frame.pack();
    }

    public void show() {
        frame.setVisible(true);
    }

    public static void msg(String msg) {
        if(sysid != null) {
            sysid.msgArea.append(msg + "\n");
            sysid.msgArea.setCaretPosition(sysid.msgArea.getDocument().getLength());
        }
    }

    @Override
    public void accept(File file) {
        System.out.println("File set to " + file);
        try {
            MotorData.motors.clear();
            listModel.clear();
            
            analysisResults = LogReader.getResult(file.getAbsolutePath());
            
            msg("Found " + analysisResults.size() + " motor groups in analysis");
            
            for(String groupName : analysisResults.keySet()) {
                MotorData motorData = new MotorData();
                motorData.name = groupName;
                motorData.sysidResult = analysisResults.get(groupName);
                MotorData.motors.add(motorData);
                listModel.addElement(motorData);
                msg("Added motor: " + groupName);
            }
            
            msg("File " + file.getName() + " loaded with " + MotorData.motors.size() + " motors");
            
            if (!listModel.isEmpty()) {
                motorList.setSelectedIndex(0);
            }
            
        } catch (Exception e) {
            msg("IO error - for file " + file + " error=" + e);
            e.printStackTrace();
            fileChooser.field.setText("");
        }
    }

    public MotorData getMotor() {
        return motorList.getSelectedValue();
    }
}

class FileChooserPanel extends JPanel implements ActionListener {
    JButton button;
    JTextField field;
    JFileChooser chooser;
    Consumer<File> consumer;

    public FileChooserPanel(Consumer<File> consumer) {
        super(new FlowLayout(FlowLayout.LEFT, 10, 5));
        this.consumer = consumer;
        button = new JButton("File:");
        field = new JTextField(50);
        field.setEditable(false);
        button.addActionListener(this);
        chooser = new JFileChooser(System.getProperty("user.dir"));
        chooser.setFileFilter(new FileNameExtensionFilter("wpi log", "wpilog"));
        add(button);
        add(field);
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        int res = chooser.showOpenDialog(null);
        if(res == JFileChooser.APPROVE_OPTION) {
            File file = chooser.getSelectedFile();
            field.setText(file.getAbsolutePath());
            if(consumer != null) {
                consumer.accept(file);
            }
        }
    }
}

class SysidResultPanel extends JPanel {
    public static int nK = KTypes.values().length;
    JCheckBox[] checkBoxes;
    JLabel[][] k;
    JButton applyButton;
    JLabel[] velLabels = {new JLabel("Velocity Range"), new JLabel("0%-30%"), new JLabel("30%-70%"), new JLabel("70%-100%")};
    JLabel[] countLabels = {new JLabel("Number of records"), new JLabel("0"), new JLabel("0"), new JLabel("0")};
    JLabel[] avgErrorLabels = {new JLabel("Average Error %"), new JLabel("0"), new JLabel("0"), new JLabel("0")};
    JLabel[] maxErrorLabels = {new JLabel("Max Error %"), new JLabel("0"), new JLabel("0"), new JLabel("0")};
    JLabel[] kpLabels = {new JLabel("KP"), new JLabel("0"), new JLabel("0"), new JLabel("0")};
    Sysid app;

    public SysidResultPanel(Sysid app) {
        super(new GridLayout(nK + 7, 4, 5, 5));
        this.app = app;

        for(JLabel l : velLabels) add(l);
        for(JLabel l : countLabels) add(l);
        for(JLabel l : avgErrorLabels) add(l);
        for(JLabel l : maxErrorLabels) add(l);

        checkBoxes = new JCheckBox[nK];
        k = new JLabel[nK][3];
        for(int i = 0; i < nK; i++) {
            checkBoxes[i] = new JCheckBox(KTypes.values()[i].name());
            if(i < 3) { checkBoxes[i].setEnabled(false); checkBoxes[i].setSelected(true); } else { checkBoxes[i].setSelected(false);}
            add(checkBoxes[i]);
            for(int j=0;j<3;j++){
                k[i][j] = new JLabel("0");
                add(k[i][j]);
            }
        }

        for(JLabel l : kpLabels) add(l);

        applyButton = new JButton("Calculate");
        add(applyButton);

        applyButton.addActionListener(e -> {
            MotorData motorData = app.getMotor();
            if(motorData != null && motorData.sysidResult != null) {
                updateDisplay(motorData);
            } else {
                Sysid.msg("No motor selected or no analysis data available");
            }
        });
    }
    
    public void updateDisplay(MotorData motorData) {
        if (motorData == null || motorData.sysidResult == null) {
            clearDisplay();
            return;
        }
        
        LogReader.SysIDResults result = motorData.sysidResult;
        LogReader.BucketResult[] buckets = {result.slow, result.mid, result.high};
        
        for(int rangeIdx = 0; rangeIdx < 3; rangeIdx++) {
            LogReader.BucketResult bucket = buckets[rangeIdx];
            
            if(bucket != null) {
                k[KTypes.KS.ordinal()][rangeIdx].setText(String.format("%7.5f", bucket.ks));
                k[KTypes.KV.ordinal()][rangeIdx].setText(String.format("%7.5f", bucket.kv));
                k[KTypes.KA.ordinal()][rangeIdx].setText(String.format("%7.5f", bucket.ka));
                
                countLabels[rangeIdx+1].setText(Integer.toString(bucket.points));
                avgErrorLabels[rangeIdx+1].setText(String.format("%4.2f%%", bucket.avgError*100));
                maxErrorLabels[rangeIdx+1].setText(String.format("%4.2f%%", bucket.avgError*120));
                
                double kp = LogReader.CalculateFeedbackGains.calculateFeedbackGains(bucket.kv, bucket.ka);
                kpLabels[rangeIdx+1].setText(String.format("%.5f", kp));
            } else {
                k[KTypes.KS.ordinal()][rangeIdx].setText("N/A");
                k[KTypes.KV.ordinal()][rangeIdx].setText("N/A");
                k[KTypes.KA.ordinal()][rangeIdx].setText("N/A");
                countLabels[rangeIdx+1].setText("0");
                avgErrorLabels[rangeIdx+1].setText("N/A");
                maxErrorLabels[rangeIdx+1].setText("N/A");
                kpLabels[rangeIdx+1].setText("N/A");
            }
        }
        
        Sysid.msg("Analysis complete for " + motorData.name);
    }
    
    private void clearDisplay() {
        for(int i = 0; i < nK; i++) {
            for(int j = 0; j < 3; j++) {
                k[i][j].setText("0");
            }
        }
        for(int i = 1; i < 4; i++) {
            countLabels[i].setText("0");
            avgErrorLabels[i].setText("0");
            maxErrorLabels[i].setText("0");
            kpLabels[i].setText("0");
        }
    }
}

class MotorData {
    static List<MotorData> motors = new ArrayList<>();
    String name = "Motor";
    LogReader.SysIDResults sysidResult;

    @Override
    public String toString() {
        return name;
    }
}

enum VelocityRange {SLOW,MID,HIGH}

enum KTypes {KS,KV,KA}

class SysidCalculate {
    MotorData motor;
    EnumSet<KTypes> types;

    Map<VelocityRange, LogReader.BucketResult> results = new HashMap<>();

    public SysidCalculate(MotorData motor, EnumSet<KTypes> types) {
        this.motor = motor;
        this.types = types;
        analyze();
    }

    private void analyze() {
        if(motor.sysidResult != null) {
            LogReader.SysIDResults result = motor.sysidResult;
            results.put(VelocityRange.SLOW, result.slow);
            results.put(VelocityRange.MID, result.mid);
            results.put(VelocityRange.HIGH, result.high);
        }
    }

    public double getK(KTypes type, VelocityRange range) {
        LogReader.BucketResult b = results.get(range);
        if(b==null) return 0;
        switch(type) {
            case KS: return b.ks;
            case KV: return b.kv;
            case KA: return b.ka;
        }
        return 0;
    }

    public int getCount(VelocityRange range) {
        LogReader.BucketResult b = results.get(range);
        return b != null ? b.points : 0;
    }
    
    public double getAverageError(VelocityRange range) {
        LogReader.BucketResult b = results.get(range);
        return b != null ? b.avgError*100 : 0;
    }
    
    public double getMaxError(VelocityRange range) {
        LogReader.BucketResult b = results.get(range);
        return b != null ? b.avgError*120 : 0;
    }
    
    public double getKP(VelocityRange range) {
        LogReader.BucketResult b = results.get(range);
        if(b != null) {
            double kp = LogReader.CalculateFeedbackGains.calculateFeedbackGains(b.kv, b.ka);
            return kp;
        }
        return 0;
    }
}