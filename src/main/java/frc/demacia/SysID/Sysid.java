package frc.Demacia.Sysid;

import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JList;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.ListSelectionModel;

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.io.File;
import java.io.IOException;
import java.util.function.Consumer;



public class Sysid implements Consumer<File> {
    JFrame frame = new JFrame("Sysid");
    FileChooserPanel fileChooser = new FileChooserPanel(this);
    JList<LogDataEntry> motorList = new JList<>();
    SysidResultPanel result = new SysidResultPanel(this);
    JTextArea msgArea = new JTextArea();
    JScrollPane msgPane = new JScrollPane(msgArea, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
    LogReader log;

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
        pane.add(fileChooser, new GridBagConstraints(0, 0, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.HORIZONTAL, new Insets(5, 5, 5, 0), 5, 5));
        pane.add(motorList, new GridBagConstraints(0, 1, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(5, 5, 5, 0), 5, 5));
        pane.add(msgPane, new GridBagConstraints(1, 0, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(5, 5, 5, 0), 5, 5));
        pane.add(result, new GridBagConstraints(1, 1, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(5, 5, 5, 0), 5, 5));
        frame.pack();
    }

    private void show() {
        frame.setVisible(true);
    }

    public static void msg(String msg) {
        if(sysid != null) {
            sysid.msgArea.append(msg + "\n");
        }
    }

    @Override
    public void accept(File file) {
        System.out.println(" file set to " + file);
        try {
            MotorData.motors.clear();
            log = new LogReader(file.getAbsolutePath());
            for(LogDataEntry m : MotorData.motors) {
                m.getMotorData();
            }
            motorList.setListData(MotorData.motors);
            msg("file " + file.getName() + " loaded");
        } catch (IOException e) {
            msg(" IO error - for file " + file + " error=" + e);
            fileChooser.field.setText("");
        }

    }

    public MotorData getMotor() {
        var s = motorList.getSelectedValue();
        if(s != null) {
            return s.motorData;
        }
        return null;
    }

    public static void main(String[] args) {
        Sysid app = new Sysid();
        app.show();        
    }
}
