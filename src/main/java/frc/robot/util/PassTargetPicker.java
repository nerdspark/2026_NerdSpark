package frc.robot.util;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JCheckBox;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PassTargetPicker {
    private static final int REFRESH_MS = 200;

    // 2026 field dimensions in metres
    private static final double FIELD_LENGTH = 16.541;
    private static final double FIELD_WIDTH  = 8.069;

    // Pass target circle radius in metres
    private static final double PASS_RADIUS_METERS = 0.075;

    // NT / SmartDashboard keys (must match PassTargetConstants on the robot)
    private static final String ENABLE_KEY      = "PassTarget/Enable";
    private static final String TARGET_X_KEY    = "PassTarget/X";
    private static final String TARGET_Y_KEY    = "PassTarget/Y";
    private static final String FIELD_CLICK_KEY = "Field/PassTargetClick";

    // Defaults — centre of field
    private static final boolean DEFAULT_ENABLE   = false;
    private static final double  DEFAULT_TARGET_X = FIELD_LENGTH / 2.0;
    private static final double  DEFAULT_TARGET_Y = FIELD_WIDTH  / 2.0;

    private final DoubleArrayPublisher fieldClickPub;
    private final BufferedImage fieldImage;
    private final FieldPanel fieldPanel;
    private final JCheckBox enableBox;
    private final JLabel coordsLabel;
    private JFrame frame;

    public PassTargetPicker() {
        fieldClickPub = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleArrayTopic(FIELD_CLICK_KEY)
            .publish();
        fieldImage = loadFieldImage();
        fieldPanel  = new FieldPanel();
        enableBox   = new JCheckBox("Pass target enabled");
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

            enableBox.setSelected(SmartDashboard.getBoolean(ENABLE_KEY, DEFAULT_ENABLE));
            enableBox.addActionListener(event ->
                SmartDashboard.putBoolean(ENABLE_KEY, enableBox.isSelected()));

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
        enableBox.setSelected(SmartDashboard.getBoolean(ENABLE_KEY, DEFAULT_ENABLE));
        double targetX = SmartDashboard.getNumber(TARGET_X_KEY, DEFAULT_TARGET_X);
        double targetY = SmartDashboard.getNumber(TARGET_Y_KEY, DEFAULT_TARGET_Y);
        coordsLabel.setText(String.format("Target: X %.2f, Y %.2f", targetX, targetY));
        fieldPanel.repaint();
    }

    private BufferedImage loadFieldImage() {
        try {
            return ImageIO.read(getClass().getResource("/frc/robot/resources/2026_field.png"));
        } catch (IOException | IllegalArgumentException e) {
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
            int panelWidth  = getWidth();
            int panelHeight = getHeight();
            if (panelWidth <= 0 || panelHeight <= 0) {
                imageRect.setBounds(0, 0, 0, 0);
                return;
            }

            double fieldAspect = FIELD_LENGTH / FIELD_WIDTH;
            int drawWidth  = panelWidth;
            int drawHeight = (int) Math.round(drawWidth / fieldAspect);
            if (drawHeight > panelHeight) {
                drawHeight = panelHeight;
                drawWidth  = (int) Math.round(drawHeight * fieldAspect);
            }

            imageRect.setBounds(
                (panelWidth  - drawWidth)  / 2,
                (panelHeight - drawHeight) / 2,
                drawWidth, drawHeight
            );
        }

        private void drawTargetOverlay(Graphics2D g2) {
            if (!SmartDashboard.getBoolean(ENABLE_KEY, DEFAULT_ENABLE)) return;

            double targetX = SmartDashboard.getNumber(TARGET_X_KEY, DEFAULT_TARGET_X);
            double targetY = SmartDashboard.getNumber(TARGET_Y_KEY, DEFAULT_TARGET_Y);
            int px = fieldToPixelX(targetX);
            int py = fieldToPixelY(targetY);

            double scaleX = imageRect.getWidth()  / FIELD_LENGTH;
            double scaleY = imageRect.getHeight() / FIELD_WIDTH;
            double radiusPixels = PASS_RADIUS_METERS * Math.min(scaleX, scaleY);

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
            if (!imageRect.contains(x, y)) return;

            double fieldX = clamp(((x - imageRect.x) / (double) imageRect.width)  * FIELD_LENGTH, 0.0, FIELD_LENGTH);
            double fieldY = clamp(FIELD_WIDTH - ((y - imageRect.y) / (double) imageRect.height) * FIELD_WIDTH, 0.0, FIELD_WIDTH);

            fieldClickPub.set(new double[] { fieldX, fieldY });
            SmartDashboard.putNumber(TARGET_X_KEY, fieldX);
            SmartDashboard.putNumber(TARGET_Y_KEY, fieldY);
            repaint();
        }

        private int fieldToPixelX(double fieldX) {
            return (int) Math.round(imageRect.x + (fieldX / FIELD_LENGTH) * imageRect.width);
        }

        private int fieldToPixelY(double fieldY) {
            return (int) Math.round(imageRect.y + ((FIELD_WIDTH - fieldY) / FIELD_WIDTH) * imageRect.height);
        }

        private double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }
}
