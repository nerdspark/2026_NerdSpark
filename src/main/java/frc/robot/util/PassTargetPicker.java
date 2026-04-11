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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 
public class PassTargetPicker {
    private static final int REFRESH_MS = 50; // ~20 Hz for smooth joystick movement
 
    // 2026 field dimensions in metres
    private static final double FIELD_LENGTH = 16.541;
    private static final double FIELD_WIDTH  = 8.069;
 
    // Pass target circle radius in metres
    private static final double PASS_RADIUS_METERS = 0.075;
 
    // Joystick settings
    private static final double JOYSTICK_DEADBAND = 0.1;
    private static final double SLOW_SPEED_MPS    = 1.0; // metres per second in slow mode
    private static final double FAST_SPEED_MPS    = 5.0; // metres per second in fast mode
 
    // NT / SmartDashboard keys (must match PassTargetConstants on the robot)
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
 
    // Speed mode
    private boolean fastMode = false;
 
    // Button edge-detection state
    private boolean prevAButton = false;
    
 
    private final CommandXboxController controller;
    private final DoubleArrayPublisher fieldClickPub;
    private final BufferedImage fieldImage;
    private final FieldPanel fieldPanel;
    private final JCheckBox enableBox;
    private final JLabel coordsLabel;
    private final JLabel speedLabel;
    private JFrame frame;
 
    /**
     * No-arg constructor used by desktop launcher and tests.
     *
     * Always uses controller port 1 (device index 1).
     */
    public PassTargetPicker() {
        this.controller = new CommandXboxController(1);

        fieldClickPub = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getDoubleArrayTopic(FIELD_CLICK_KEY)
            .publish();
        fieldImage        = loadFieldImage();
        fieldPanel        = new FieldPanel();
        enableBox         = new JCheckBox("Pass target enabled");
        coordsLabel       = new JLabel();
        speedLabel = new JLabel("SLOW");
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
 
            // Swing timer drives both joystick polling and UI refresh
            Timer refresh = new Timer(REFRESH_MS, event -> tick());
            refresh.start();
 
            refreshUi();
        });
    }
 
    // Called every REFRESH_MS ms from the Swing timer
    private void tick() {
        pollController();
        refreshUi();
    }
 
    private void pollController() {
    double rawX = controller.getRightX();
    double rawY = controller.getRightY();
    boolean aPressed = controller.a().getAsBoolean();
 
        // Apply deadband
        double jx = applyDeadband(rawX, JOYSTICK_DEADBAND);
        double jy = applyDeadband(rawY, JOYSTICK_DEADBAND);
 
        // Move cursor — right stick Y is inverted (up = negative axis = increase field Y)
        double speed = fastMode ? FAST_SPEED_MPS : SLOW_SPEED_MPS;
        double dt = REFRESH_MS / 1000.0;
 
        cursorX = clamp(cursorX + jx * speed * dt, 0.0, FIELD_LENGTH);
        cursorY = clamp(cursorY - jy * speed * dt, 0.0, FIELD_WIDTH);
 
        // Right bumper held — fast mode while held
        fastMode = controller.rightBumper().getAsBoolean();
 
        // A button rising edge — commit cursor position as locked target
        if (aPressed && !prevAButton) {
            commitTarget();
        }
 
    prevAButton = aPressed;
    }
 
    // fastMode is controlled directly by the right bumper; no toggle method needed
 
    // Publishes the cursor position to NT as the locked pass target
    private void commitTarget() {
        fieldClickPub.set(new double[] { cursorX, cursorY });
        SmartDashboard.putNumber(TARGET_X_KEY, cursorX);
        SmartDashboard.putNumber(TARGET_Y_KEY, cursorY);
    }
 
    private void refreshUi() {
        enableBox.setSelected(SmartDashboard.getBoolean(ENABLE_KEY, DEFAULT_ENABLE));
        double lockedX = SmartDashboard.getNumber(TARGET_X_KEY, DEFAULT_TARGET_X);
        double lockedY = SmartDashboard.getNumber(TARGET_Y_KEY, DEFAULT_TARGET_Y);
        coordsLabel.setText(String.format(
            "  Cursor: (%.2f, %.2f)  |  Locked: (%.2f, %.2f)",
            cursorX, cursorY, lockedX, lockedY
        ));
        // Update speed label to reflect current mode
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
 
        // White dot + ring — the committed target the robot is actually using
        private void drawLockedTargetOverlay(Graphics2D g2) {
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
            g2.drawString("Locked", px + 6, py - 6);
        }
 
        // Cyan/orange crosshair — the moving joystick cursor
        private void drawCursorOverlay(Graphics2D g2) {
            int px = fieldToPixelX(cursorX);
            int py = fieldToPixelY(cursorY);
 
            Color cursorColor = fastMode
                ? new Color(255, 100, 60, 220)  // orange-red in fast mode
                : new Color(60, 200, 255, 220); // cyan in slow mode
 
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
 
        // Mouse click moves cursor and immediately commits
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