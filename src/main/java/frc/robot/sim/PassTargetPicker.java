package frc.robot.sim;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import javax.imageio.ImageIO;
import javax.swing.JCheckBox;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PassTargetConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.FieldConstants;

public class PassTargetPicker {
    private static final int REFRESH_MS = 200;

    private final DoubleArrayPublisher fieldClickPub;
    private final BufferedImage fieldImage;
    private final FieldPanel fieldPanel;
    private final JCheckBox enableBox;
    private final JLabel coordsLabel;
    private JFrame frame;

    public PassTargetPicker() {
        NetworkTable smart = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        fieldClickPub = smart.getDoubleArrayTopic(PassTargetConstants.fieldClickKey).publish();
        fieldImage = loadFieldImage();
        fieldPanel = new FieldPanel();
        enableBox = new JCheckBox("Pass target enabled");
        coordsLabel = new JLabel();
    }

    public void start() {
        SwingUtilities.invokeLater(() -> {
            if (frame != null) {
                frame.setVisible(true);
                return;
            }

            frame = new JFrame("Pass Target Picker");
            frame.setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);

            enableBox.setSelected(SmartDashboard.getBoolean(
                PassTargetConstants.enableKey,
                PassTargetConstants.defaultEnable
            ));
            enableBox.addActionListener(event -> SmartDashboard.putBoolean(
                PassTargetConstants.enableKey,
                enableBox.isSelected()
            ));

            JPanel controls = new JPanel(new FlowLayout(FlowLayout.LEFT));
            controls.add(enableBox);
            controls.add(coordsLabel);

            frame.getContentPane().setLayout(new BorderLayout());
            frame.getContentPane().add(fieldPanel, BorderLayout.CENTER);
            frame.getContentPane().add(controls, BorderLayout.SOUTH);
            frame.setMinimumSize(new Dimension(700, 400));
            frame.setLocationByPlatform(true);
            frame.setVisible(true);

            Timer refresh = new Timer(REFRESH_MS, event -> refreshUi());
            refresh.start();

            refreshUi();
        });
    }

    private void refreshUi() {
        enableBox.setSelected(SmartDashboard.getBoolean(
            PassTargetConstants.enableKey,
            PassTargetConstants.defaultEnable
        ));
        double targetX = SmartDashboard.getNumber(
            PassTargetConstants.targetXKey,
            PassTargetConstants.defaultTargetX
        );
        double targetY = SmartDashboard.getNumber(
            PassTargetConstants.targetYKey,
            PassTargetConstants.defaultTargetY
        );
        coordsLabel.setText(String.format("Target: X %.2f, Y %.2f", targetX, targetY));
        fieldPanel.repaint();
    }

    private BufferedImage loadFieldImage() {
        String localAppData = System.getenv("LOCALAPPDATA");
        if (localAppData == null) {
            return null;
        }
        Path imagePath = Paths.get(
            localAppData,
            "Programs",
            "FRC Elastic",
            "data",
            "flutter_assets",
            "assets",
            "fields",
            "2026-field.png"
        );
        File file = imagePath.toFile();
        if (!file.exists()) {
            return null;
        }
        try {
            return ImageIO.read(file);
        } catch (IOException ex) {
            return null;
        }
    }

    private final class FieldPanel extends JComponent {
        private final Rectangle imageRect = new Rectangle();

        private FieldPanel() {
            addMouseListener(new MouseAdapter() {
                @Override
                public void mousePressed(MouseEvent event) {
                    handleClick(event.getX(), event.getY());
                }
            });
        }

        @Override
        protected void paintComponent(Graphics graphics) {
            super.paintComponent(graphics);
            Graphics2D g2 = (Graphics2D) graphics.create();
            g2.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            computeImageRect();
            if (fieldImage != null) {
                g2.drawImage(fieldImage, imageRect.x, imageRect.y, imageRect.width, imageRect.height, null);
            } else {
                g2.setColor(new Color(40, 40, 40));
                g2.fillRect(imageRect.x, imageRect.y, imageRect.width, imageRect.height);
            }

            drawTargetOverlay(g2);

            g2.dispose();
        }

        private void computeImageRect() {
            int panelWidth = getWidth();
            int panelHeight = getHeight();
            if (panelWidth <= 0 || panelHeight <= 0) {
                imageRect.setBounds(0, 0, 0, 0);
                return;
            }

            double fieldAspect = FieldConstants.fieldLength / FieldConstants.fieldWidth;
            int drawWidth = panelWidth;
            int drawHeight = (int) Math.round(drawWidth / fieldAspect);
            if (drawHeight > panelHeight) {
                drawHeight = panelHeight;
                drawWidth = (int) Math.round(drawHeight * fieldAspect);
            }

            int x = (panelWidth - drawWidth) / 2;
            int y = (panelHeight - drawHeight) / 2;
            imageRect.setBounds(x, y, drawWidth, drawHeight);
        }

        private void drawTargetOverlay(Graphics2D g2) {
            boolean enabled = SmartDashboard.getBoolean(
                PassTargetConstants.enableKey,
                PassTargetConstants.defaultEnable
            );
            if (!enabled) {
                return;
            }
            double targetX = SmartDashboard.getNumber(
                PassTargetConstants.targetXKey,
                PassTargetConstants.defaultTargetX
            );
            double targetY = SmartDashboard.getNumber(
                PassTargetConstants.targetYKey,
                PassTargetConstants.defaultTargetY
            );
            int px = fieldToPixelX(targetX);
            int py = fieldToPixelY(targetY);

            double scaleX = imageRect.getWidth() / FieldConstants.fieldLength;
            double scaleY = imageRect.getHeight() / FieldConstants.fieldWidth;
            double radiusPixels = TurretConstants.passTargetRadiusMeters * Math.min(scaleX, scaleY);

            g2.setColor(new Color(180, 180, 180, 200));
            g2.setStroke(new BasicStroke(2.0f));
            g2.drawOval(
                (int) Math.round(px - radiusPixels),
                (int) Math.round(py - radiusPixels),
                (int) Math.round(radiusPixels * 2.0),
                (int) Math.round(radiusPixels * 2.0)
            );

            g2.setColor(new Color(240, 240, 240));
            g2.fillOval(px - 4, py - 4, 8, 8);
            g2.setFont(g2.getFont().deriveFont(Font.BOLD, 12f));
            g2.drawString("Pass", px + 6, py - 6);
        }

        private void handleClick(int x, int y) {
            if (!imageRect.contains(x, y)) {
                return;
            }
            double fieldX = ((x - imageRect.x) / (double) imageRect.width) * FieldConstants.fieldLength;
            double fieldY = FieldConstants.fieldWidth
                - ((y - imageRect.y) / (double) imageRect.height) * FieldConstants.fieldWidth;

            fieldX = clamp(fieldX, 0.0, FieldConstants.fieldLength);
            fieldY = clamp(fieldY, 0.0, FieldConstants.fieldWidth);

            fieldClickPub.set(new double[] { fieldX, fieldY });
        }

        private int fieldToPixelX(double fieldX) {
            double x = imageRect.x + (fieldX / FieldConstants.fieldLength) * imageRect.width;
            return (int) Math.round(x);
        }

        private int fieldToPixelY(double fieldY) {
            double y = imageRect.y + ((FieldConstants.fieldWidth - fieldY) / FieldConstants.fieldWidth) * imageRect.height;
            return (int) Math.round(y);
        }

        private double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }
}
