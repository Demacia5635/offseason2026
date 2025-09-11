package frc.Demacia.SysID;

import javax.swing.*;

import edu.wpi.first.math.geometry.Pose2d;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.List;
import javax.imageio.ImageIO;

/**
 * Visualizes robot positions on a field image
 */
public class FieldVisualizer extends JFrame {
    
    private static final int WINDOW_WIDTH = 1200;
    private static final int WINDOW_HEIGHT = 800;
    
    // FRC field dimensions in meters (2024 field)
    private static final double FIELD_WIDTH = 16.54; // meters
    private static final double FIELD_HEIGHT = 8.21; // meters
    
    private BufferedImage fieldImage;
    private List<RobotPositionData.PositionData> positions;
    private FieldPanel fieldPanel;
    
    public FieldVisualizer() {
        setTitle("Robot Position Visualizer");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(WINDOW_WIDTH, WINDOW_HEIGHT);
        setLocationRelativeTo(null);
        
        setupUI();
    }
    
    private void setupUI() {
        setLayout(new BorderLayout());
        
        // Control panel
        JPanel controlPanel = new JPanel();
        JButton loadLogButton = new JButton("Load Log File");
        JButton loadImageButton = new JButton("Load Field Image");
        
        loadLogButton.addActionListener(e -> loadLogFile());
        loadImageButton.addActionListener(e -> loadFieldImage());
        
        controlPanel.add(loadLogButton);
        controlPanel.add(loadImageButton);
        
        add(controlPanel, BorderLayout.NORTH);
        
        // Field visualization panel
        fieldPanel = new FieldPanel();
        add(fieldPanel, BorderLayout.CENTER);
        
        // Status panel
        JLabel statusLabel = new JLabel("Load a log file and field image to visualize robot positions");
        add(statusLabel, BorderLayout.SOUTH);
    }
    
    private void loadLogFile() {
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setFileFilter(new javax.swing.filechooser.FileNameExtensionFilter("WPI Log files", "wpilog"));
        
        int result = fileChooser.showOpenDialog(this);
        if (result == JFileChooser.APPROVE_OPTION) {
            File selectedFile = fileChooser.getSelectedFile();
            try {
                RobotPositionData.robotPositionData = null;
                new LogReader(selectedFile.getAbsolutePath(), true);
                if(RobotPositionData.robotPositionData != null) {
                    RobotPositionData.robotPositionData.extractPositions();
                    fieldPanel.setPositions(RobotPositionData.robotPositionData.getPositions());
                    fieldPanel.repaint();
                
                    JOptionPane.showMessageDialog(this, 
                        "Loaded " + positions.size() + " position samples from log file", 
                        "Log Loaded", 
                        JOptionPane.INFORMATION_MESSAGE);
                }
            } catch (IOException ex) {
                JOptionPane.showMessageDialog(this, 
                    "Error loading log file: " + ex.getMessage(), 
                    "Error", 
                    JOptionPane.ERROR_MESSAGE);
            }
        }
    }
    private void loadFieldImage() {
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setFileFilter(new javax.swing.filechooser.FileNameExtensionFilter("Image files", "jpg", "jpeg", "png", "gif"));
        
        int result = fileChooser.showOpenDialog(this);
        if (result == JFileChooser.APPROVE_OPTION) {
            File selectedFile = fileChooser.getSelectedFile();
            try {
                fieldImage = ImageIO.read(selectedFile);
                fieldPanel.setFieldImage(fieldImage);
                fieldPanel.repaint();
                
                JOptionPane.showMessageDialog(this, 
                    "Field image loaded successfully", 
                    "Image Loaded", 
                    JOptionPane.INFORMATION_MESSAGE);
            } catch (IOException ex) {
                JOptionPane.showMessageDialog(this, 
                    "Error loading image: " + ex.getMessage(), 
                    "Error", 
                    JOptionPane.ERROR_MESSAGE);
            }
        }
    }
    
    private class FieldPanel extends JPanel {
        private List<RobotPositionData.PositionData> positions;
        private BufferedImage fieldImage;
        
        public void setPositions(List<RobotPositionData.PositionData> positions) {
            this.positions = positions;
        }
        
        public void setFieldImage(BufferedImage fieldImage) {
            this.fieldImage = fieldImage;
        }
        
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2d = (Graphics2D) g;
            g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            int panelWidth = getWidth();
            int panelHeight = getHeight();
            
            // Draw field image if available
            if (fieldImage != null) {
                g2d.drawImage(fieldImage, 0, 0, panelWidth, panelHeight, null);
            } else {
                // Draw simple field outline
                g2d.setColor(Color.GREEN);
                g2d.fillRect(0, 0, panelWidth, panelHeight);
                g2d.setColor(Color.WHITE);
                g2d.drawRect(0, 0, panelWidth - 1, panelHeight - 1);
            }
            
            // Draw robot positions if available
            if (positions != null && !positions.isEmpty()) {
                drawRobotPath(g2d, panelWidth, panelHeight);
            }
        }
        
        private void drawRobotPath(Graphics2D g2d, int panelWidth, int panelHeight) {
            if (positions.size() < 2) return;
            
            // Convert field coordinates to screen coordinates
            double scaleX = panelWidth / FIELD_WIDTH;
            double scaleY = panelHeight / FIELD_HEIGHT;
            
            // Draw path
            g2d.setColor(Color.RED);
            g2d.setStroke(new BasicStroke(2));
            
            for (int i = 1; i < positions.size(); i++) {
                Pose2d prevPose = positions.get(i - 1).pose;
                Pose2d currPose = positions.get(i).pose;
                
                int x1 = (int) (prevPose.getX() * scaleX);
                int y1 = panelHeight - (int) (prevPose.getY() * scaleY); // Flip Y coordinate
                int x2 = (int) (currPose.getX() * scaleX);
                int y2 = panelHeight - (int) (currPose.getY() * scaleY); // Flip Y coordinate
                
                g2d.drawLine(x1, y1, x2, y2);
            }
            
            // Draw robot positions as dots
            g2d.setColor(Color.BLUE);
            for (RobotPositionData.PositionData pos : positions) {
                int x = (int) (pos.pose.getX() * scaleX);
                int y = panelHeight - (int) (pos.pose.getY() * scaleY); // Flip Y coordinate
                g2d.fillOval(x - 2, y - 2, 4, 4);
            }
            
            // Draw start position (green)
            if (!positions.isEmpty()) {
                Pose2d startPose = positions.get(0).pose;
                int startX = (int) (startPose.getX() * scaleX);
                int startY = panelHeight - (int) (startPose.getY() * scaleY);
                g2d.setColor(Color.GREEN);
                g2d.fillOval(startX - 5, startY - 5, 10, 10);
                g2d.setColor(Color.BLACK);
                g2d.drawString("START", startX + 8, startY + 5);
            }
            
            // Draw end position (red)
            if (positions.size() > 1) {
                Pose2d endPose = positions.get(positions.size() - 1).pose;
                int endX = (int) (endPose.getX() * scaleX);
                int endY = panelHeight - (int) (endPose.getY() * scaleY);
                g2d.setColor(Color.RED);
                g2d.fillOval(endX - 5, endY - 5, 10, 10);
                g2d.setColor(Color.BLACK);
                g2d.drawString("END", endX + 8, endY + 5);
            }
            
            // Draw robot orientation arrows at key points
            g2d.setColor(Color.ORANGE);
            g2d.setStroke(new BasicStroke(1));
            int step = Math.max(1, positions.size() / 20); // Show about 20 orientation arrows
            
            for (int i = 0; i < positions.size(); i += step) {
                Pose2d pose = positions.get(i).pose;
                int x = (int) (pose.getX() * scaleX);
                int y = panelHeight - (int) (pose.getY() * scaleY);
                
                double heading = pose.getRotation().getRadians();
                int arrowLength = 15;
                int arrowX = (int) (x + arrowLength * Math.cos(heading));
                int arrowY = (int) (y - arrowLength * Math.sin(heading)); // Flip Y for screen coordinates
                
                g2d.drawLine(x, y, arrowX, arrowY);
                
                // Draw arrowhead
                double arrowHeadAngle = Math.PI / 6;
                int arrowHeadLength = 5;
                int arrowHead1X = (int) (arrowX - arrowHeadLength * Math.cos(heading - arrowHeadAngle));
                int arrowHead1Y = (int) (arrowY + arrowHeadLength * Math.sin(heading - arrowHeadAngle));
                int arrowHead2X = (int) (arrowX - arrowHeadLength * Math.cos(heading + arrowHeadAngle));
                int arrowHead2Y = (int) (arrowY + arrowHeadLength * Math.sin(heading + arrowHeadAngle));
                
                g2d.drawLine(arrowX, arrowY, arrowHead1X, arrowHead1Y);
                g2d.drawLine(arrowX, arrowY, arrowHead2X, arrowHead2Y);
            }
        }
    }
    
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            new FieldVisualizer().setVisible(true);
        });
    }
}