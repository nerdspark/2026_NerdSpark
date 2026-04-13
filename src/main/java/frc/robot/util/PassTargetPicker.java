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

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;

public class PassTargetPicker {
    private static final int REFRESH_MS = 50; // ~20 Hz for smooth joystick movement

    private static final double FIELD_LENGTH = 16.541;
    private static final double FIELD_WIDTH  = 8.069;

    private static final double PASS_RADIUS_METERS = 0.075;

    private static final double JOYSTICK_DEADBAND = 0.1;
    private static final double SLOW_SPEED_MPS    = 1.0;
    private static final double FAST_SPEED_MPS    = 5.0;

    private static final String ENABLE_KEY      = "PassTarget/Enable";
    private static final String TARGET_X_KEY    = "PassTarget/X";
    private static final String TARGET_Y_KEY    = "PassTarget/Y";
    private static final String FIELD_CLICK_KEY = "Field/PassTargetClick";

    // Defaults — centre of field
    private static final boolean DEFAULT_ENABLE   = false;
    private static final double  DEFAULT_TARGET_X = FIELD_LENGTH / 2.0;
    private static final double  DEFAULT_TARGET_Y = FIELD_WIDTH  / 2.0;

    // Cursor position (moves via joystick, committed on A press)
    private double cursorX = DEFAULT_TARGET_X;
    private double cursorY = DEFAULT_TARGET_Y;

    private boolean fastMode    = false;
    private boolean prevAButton = false;

    // NT publishers/subscribers — no SmartDashboard so standalone NT client works
    private final DoubleArrayPublisher fieldClickPub;
    private final DoublePublisher      targetXPub;
    private final DoublePublisher      targetYPub;
    private final BooleanPublisher     enablePub;
    private final BooleanSubscriber    enableSub;
    private final DoubleSubscriber     targetXSub;
    private final DoubleSubscriber     targetYSub;

    private final XboxController controller;
    private final BufferedImage  fieldImage;
    private final FieldPanel     fieldPanel;
    private final JCheckBox      enableBox;
    private final JLabel         coordsLabel;
    private final JLabel         speedLabel;
    private JFrame frame;

    public PassTargetPicker() {
        // XboxController works after HAL.initialize(); no command scheduler needed
        this.controller = new XboxController(1);

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        var table = nt.getTable("SmartDashboard");

        fieldClickPub = table.getDoubleArrayTopic(FIELD_CLICK_KEY).publish();
        targetXPub    = table.getDoubleTopic(TARGET_X_KEY).publish();
        targetYPub    = table.getDoubleTopic(TARGET_Y_KEY).publish();
        enablePub     = table.getBooleanTopic(ENABLE_KEY).publish();

        enableSub  = table.getBooleanTopic(ENABLE_KEY).subscribe(DEFAULT_ENABLE);
        targetXSub = table.getDoubleTopic(TARGET_X_KEY).subscribe(DEFAULT_TARGET_X);
        targetYSub = table.getDoubleTopic(TARGET_Y_KEY).subscribe(DEFAULT_TARGET_Y);

        fieldImage  = loadFieldImage();
        fieldPanel  = new FieldPanel();
        enableBox   = new JCheckBox("Pass target enabled");
        coordsLabel = new JLabel();
        speedLabel  = new JLabel("SLOW");
    }

    public void start() {
        SwingUtilities.invokeLater(() -> {
            if (frame != null) {
                frame.setVisible(true);
                return;
            }

            frame = new JFrame("Pass Target Picker");
            frame.setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);

            enableBox.setSelected(enableSub.get());
            enableBox.addActionListener(event -> enablePub.set(enableBox.isSelected()));

            speedLabel.setOpaque(true);
            speedLabel.setForeground(Color.WHITE);
            speedLabel.setHorizontalAlignment(JLabel.CENTER);

            JPanel controls = new JPanel(new FlowLayout(FlowLayout.LEFT));
            controls.add(enableBox);
            controls.add(coordsLabel);
            controls.add(speedLabel);

            frame.getContentPane().setLayout(new BorderLayout());
            frame.getContentPane().add(fieldPanel, BorderLayout.CENTER);
            frame.getContentPane().add(controls, BorderLayout.SOUTH);
            frame.setMinimumSize(new Dimension(700, 400));
            frame.setLocationByPlatform(true);
            frame.setVisible(true);

            Timer refresh = new Timer(REFRESH_MS, event -> tick());
            refresh.start();

            refreshUi();
        });
    }

    private void tick() {
        DriverStation.refreshData(); // required to update joystick data outside of robot program
        pollController();
        refreshUi();
    }

    private void pollController() {
        double rawX = controller.getRightX();
        double rawY = controller.getRightY();
        boolean aPressed = controller.getAButton();

        System.out.println("rawX=" + rawX + " rawY=" + rawY + " a=" + aPressed + " rb=" + controller.getRightBumperButton());


        double jx = applyDeadband(rawX, JOYSTICK_DEADBAND);
        double jy = applyDeadband(rawY, JOYSTICK_DEADBAND);

        double speed = fastMode ? FAST_SPEED_MPS : SLOW_SPEED_MPS;
        double dt    = REFRESH_MS / 1000.0;

        cursorX = clamp(cursorX + jx * speed * dt, 0.0, FIELD_LENGTH);
        cursorY = clamp(cursorY - jy * speed * dt, 0.0, FIELD_WIDTH);

        // Right bumper held — fast mode (getRightBumper() deprecated since 2025)
        fastMode = controller.getRightBumperButton();

        // A button rising edge — commit target
        if (aPressed && !prevAButton) {
            commitTarget();
        }
        prevAButton = aPressed;
    }

    private void commitTarget() {
        fieldClickPub.set(new double[]{ cursorX, cursorY });
        targetXPub.set(cursorX);
        targetYPub.set(cursorY);
    }

    private void refreshUi() {
        enableBox.setSelected(enableSub.get());
        double lockedX = targetXSub.get();
        double lockedY = targetYSub.get();
        coordsLabel.setText(String.format(
            "  Cursor: (%.2f, %.2f)  |  Locked: (%.2f, %.2f)",
            cursorX, cursorY, lockedX, lockedY
        ));
        speedLabel.setText(fastMode ? "FAST" : "SLOW");
        speedLabel.setBackground(fastMode
            ? new Color(200, 80, 60)
            : new Color(60, 120, 200));
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

            drawLockedTargetOverlay(g2);
            drawCursorOverlay(g2);
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

        private void drawLockedTargetOverlay(Graphics2D g2) {
            if (!enableSub.get()) return;

            double targetX = targetXSub.get();
            double targetY = targetYSub.get();
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
            g2.drawString("Locked", px + 6, py - 6);
        }

        private void drawCursorOverlay(Graphics2D g2) {
            int px = fieldToPixelX(cursorX);
            int py = fieldToPixelY(cursorY);

            Color cursorColor = fastMode
                ? new Color(255, 100, 60, 220)
                : new Color(60, 200, 255, 220);

            g2.setColor(cursorColor);
            g2.setStroke(new BasicStroke(1.5f));

            int crossSize = 10;
            g2.drawLine(px - crossSize, py, px + crossSize, py);
            g2.drawLine(px, py - crossSize, px, py + crossSize);

            int circleR = 8;
            g2.drawOval(px - circleR, py - circleR, circleR * 2, circleR * 2);

            g2.setFont(g2.getFont().deriveFont(Font.BOLD, 11f));
            g2.drawString(fastMode ? "FAST" : "SLOW", px + 10, py + 4);
        }

        private void handleClick(int x, int y) {
            if (!imageRect.contains(x, y)) return;

            cursorX = clamp(((x - imageRect.x) / (double) imageRect.width)  * FIELD_LENGTH, 0.0, FIELD_LENGTH);
            cursorY = clamp(FIELD_WIDTH - ((y - imageRect.y) / (double) imageRect.height) * FIELD_WIDTH, 0.0, FIELD_WIDTH);

            commitTarget();
            repaint();
        }

        private int fieldToPixelX(double fieldX) {
            return (int) Math.round(imageRect.x + (fieldX / FIELD_LENGTH) * imageRect.width);
        }

        private int fieldToPixelY(double fieldY) {
            return (int) Math.round(imageRect.y + ((FIELD_WIDTH - fieldY) / FIELD_WIDTH) * imageRect.height);
        }
    }

    private static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) return 0.0;
        return (value - Math.signum(value) * deadband) / (1.0 - deadband);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

// MOUSE BASED TARGET PICKER
// package frc.robot.util;

// import java.awt.BasicStroke;
// import java.awt.BorderLayout;
// import java.awt.Color;
// import java.awt.Dimension;
// import java.awt.FlowLayout;
// import java.awt.Font;
// import java.awt.Graphics;
// import java.awt.Graphics2D;
// import java.awt.Rectangle;
// import java.awt.RenderingHints;
// import java.awt.event.MouseAdapter;
// import java.awt.event.MouseEvent;
// import java.awt.image.BufferedImage;
// import java.io.IOException;

// import javax.imageio.ImageIO;
// import javax.swing.JCheckBox;
// import javax.swing.JComponent;
// import javax.swing.JFrame;
// import javax.swing.JLabel;
// import javax.swing.JPanel;
// import javax.swing.SwingUtilities;
// import javax.swing.Timer;

// import edu.wpi.first.networktables.DoubleArrayPublisher;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class PassTargetPicker {
//     private static final int REFRESH_MS = 200;

//     // 2026 field dimensions in metres
//     private static final double FIELD_LENGTH = 16.541;
//     private static final double FIELD_WIDTH  = 8.069;

//     // Pass target circle radius in metres
//     private static final double PASS_RADIUS_METERS = 0.075;

//     // NT / SmartDashboard keys (must match PassTargetConstants on the robot)
//     private static final String ENABLE_KEY      = "PassTarget/Enable";
//     private static final String TARGET_X_KEY    = "PassTarget/X";
//     private static final String TARGET_Y_KEY    = "PassTarget/Y";
//     private static final String FIELD_CLICK_KEY = "Field/PassTargetClick";

//     // Defaults — centre of field
//     private static final boolean DEFAULT_ENABLE   = false;
//     private static final double  DEFAULT_TARGET_X = FIELD_LENGTH / 2.0;
//     private static final double  DEFAULT_TARGET_Y = FIELD_WIDTH  / 2.0;

//     private final DoubleArrayPublisher fieldClickPub;
//     private final BufferedImage fieldImage;
//     private final FieldPanel fieldPanel;
//     private final JCheckBox enableBox;
//     private final JLabel coordsLabel;
//     private JFrame frame;

//     public PassTargetPicker() {
//         fieldClickPub = NetworkTableInstance.getDefault()
//             .getTable("SmartDashboard")
//             .getDoubleArrayTopic(FIELD_CLICK_KEY)
//             .publish();
//         fieldImage = loadFieldImage();
//         fieldPanel  = new FieldPanel();
//         enableBox   = new JCheckBox("Pass target enabled");
//         coordsLabel = new JLabel();
//     }

//     public void start() {
//         SwingUtilities.invokeLater(() -> {
//             if (frame != null) {
//                 frame.setVisible(true);
//                 return;
//             }

//             frame = new JFrame("Pass Target Picker");
//             frame.setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);

//             enableBox.setSelected(SmartDashboard.getBoolean(ENABLE_KEY, DEFAULT_ENABLE));
//             enableBox.addActionListener(event ->
//                 SmartDashboard.putBoolean(ENABLE_KEY, enableBox.isSelected()));

//             JPanel controls = new JPanel(new FlowLayout(FlowLayout.LEFT));
//             controls.add(enableBox);
//             controls.add(coordsLabel);

//             frame.getContentPane().setLayout(new BorderLayout());
//             frame.getContentPane().add(fieldPanel, BorderLayout.CENTER);
//             frame.getContentPane().add(controls, BorderLayout.SOUTH);
//             frame.setMinimumSize(new Dimension(700, 400));
//             frame.setLocationByPlatform(true);
//             frame.setVisible(true);

//             Timer refresh = new Timer(REFRESH_MS, event -> refreshUi());
//             refresh.start();

//             refreshUi();
//         });
//     }

//     private void refreshUi() {
//         enableBox.setSelected(SmartDashboard.getBoolean(ENABLE_KEY, DEFAULT_ENABLE));
//         double targetX = SmartDashboard.getNumber(TARGET_X_KEY, DEFAULT_TARGET_X);
//         double targetY = SmartDashboard.getNumber(TARGET_Y_KEY, DEFAULT_TARGET_Y);
//         coordsLabel.setText(String.format("Target: X %.2f, Y %.2f", targetX, targetY));
//         fieldPanel.repaint();
//     }

//     private BufferedImage loadFieldImage() {
//         try {
//             return ImageIO.read(getClass().getResource("/frc/robot/resources/2026_field.png"));
//         } catch (IOException | IllegalArgumentException e) {
//             return null;
//         }
//     }

//     private final class FieldPanel extends JComponent {
//         private final Rectangle imageRect = new Rectangle();

//         private FieldPanel() {
//             addMouseListener(new MouseAdapter() {
//                 @Override
//                 public void mousePressed(MouseEvent event) {
//                     handleClick(event.getX(), event.getY());
//                 }
//             });
//         }

//         @Override
//         protected void paintComponent(Graphics graphics) {
//             super.paintComponent(graphics);
//             Graphics2D g2 = (Graphics2D) graphics.create();
//             g2.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
//             g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

//             computeImageRect();
//             if (fieldImage != null) {
//                 g2.drawImage(fieldImage, imageRect.x, imageRect.y, imageRect.width, imageRect.height, null);
//             } else {
//                 g2.setColor(new Color(40, 40, 40));
//                 g2.fillRect(imageRect.x, imageRect.y, imageRect.width, imageRect.height);
//             }

//             drawTargetOverlay(g2);
//             g2.dispose();
//         }

//         private void computeImageRect() {
//             int panelWidth  = getWidth();
//             int panelHeight = getHeight();
//             if (panelWidth <= 0 || panelHeight <= 0) {
//                 imageRect.setBounds(0, 0, 0, 0);
//                 return;
//             }

//             double fieldAspect = FIELD_LENGTH / FIELD_WIDTH;
//             int drawWidth  = panelWidth;
//             int drawHeight = (int) Math.round(drawWidth / fieldAspect);
//             if (drawHeight > panelHeight) {
//                 drawHeight = panelHeight;
//                 drawWidth  = (int) Math.round(drawHeight * fieldAspect);
//             }

//             imageRect.setBounds(
//                 (panelWidth  - drawWidth)  / 2,
//                 (panelHeight - drawHeight) / 2,
//                 drawWidth, drawHeight
//             );
//         }

//         private void drawTargetOverlay(Graphics2D g2) {
//             if (!SmartDashboard.getBoolean(ENABLE_KEY, DEFAULT_ENABLE)) return;

//             double targetX = SmartDashboard.getNumber(TARGET_X_KEY, DEFAULT_TARGET_X);
//             double targetY = SmartDashboard.getNumber(TARGET_Y_KEY, DEFAULT_TARGET_Y);
//             int px = fieldToPixelX(targetX);
//             int py = fieldToPixelY(targetY);

//             double scaleX = imageRect.getWidth()  / FIELD_LENGTH;
//             double scaleY = imageRect.getHeight() / FIELD_WIDTH;
//             double radiusPixels = PASS_RADIUS_METERS * Math.min(scaleX, scaleY);

//             g2.setColor(new Color(180, 180, 180, 200));
//             g2.setStroke(new BasicStroke(2.0f));
//             g2.drawOval(
//                 (int) Math.round(px - radiusPixels),
//                 (int) Math.round(py - radiusPixels),
//                 (int) Math.round(radiusPixels * 2.0),
//                 (int) Math.round(radiusPixels * 2.0)
//             );

//             g2.setColor(new Color(240, 240, 240));
//             g2.fillOval(px - 4, py - 4, 8, 8);
//             g2.setFont(g2.getFont().deriveFont(Font.BOLD, 12f));
//             g2.drawString("Pass", px + 6, py - 6);
//         }

//         private void handleClick(int x, int y) {
//             if (!imageRect.contains(x, y)) return;

//             double fieldX = clamp(((x - imageRect.x) / (double) imageRect.width)  * FIELD_LENGTH, 0.0, FIELD_LENGTH);
//             double fieldY = clamp(FIELD_WIDTH - ((y - imageRect.y) / (double) imageRect.height) * FIELD_WIDTH, 0.0, FIELD_WIDTH);

//             fieldClickPub.set(new double[] { fieldX, fieldY });
//             SmartDashboard.putNumber(TARGET_X_KEY, fieldX);
//             SmartDashboard.putNumber(TARGET_Y_KEY, fieldY);
//             repaint();
//         }

//         private int fieldToPixelX(double fieldX) {
//             return (int) Math.round(imageRect.x + (fieldX / FIELD_LENGTH) * imageRect.width);
//         }

//         private int fieldToPixelY(double fieldY) {
//             return (int) Math.round(imageRect.y + ((FIELD_WIDTH - fieldY) / FIELD_WIDTH) * imageRect.height);
//         }

//         private double clamp(double value, double min, double max) {
//             return Math.max(min, Math.min(max, value));
//         }
//     }
// }