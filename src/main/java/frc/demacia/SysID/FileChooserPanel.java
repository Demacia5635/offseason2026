package frc.Demacia.Sysid;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.util.function.Consumer;

import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.filechooser.FileNameExtensionFilter;

public class FileChooserPanel extends JPanel implements ActionListener {
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
