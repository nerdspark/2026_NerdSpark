package frc.robot.sim;

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

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PassTargetConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.FieldConstants;

public class PassTargetPicker {
    private static final int REFRESH_MS = 50;

    private final BufferedImage fieldImage;
    private final FieldPanel fieldPanel;
    private final JCheckBox enableBox;
    private final JLabel statusLabel;

    // Snapshot NT values once per timer tick and pass them into paint,
    // so refreshUi() and paintComponent() always see the same frame of data.
    private double cursorX = PassTargetConstants.defaultTargetX;
    private double cursorY = PassTargetConstants.defaultTargetY;
    private double lockedX = PassTargetConstants.defaultTargetX;
    private double lockedY = PassTargetConstants.defaultTargetY;
    private boolean enabled = PassTargetConstants.defaultEnable;

    // Use typed subscribers consistently instead of mixing SmartDashboard,
    // raw getEntry(), and typed Publisher APIs across the two files.
    private final DoubleSubscriber cursorXSub;
    private final DoubleSubscriber cursorYSub;
    private final DoubleSubscriber lockedXSub;
    private final DoubleSubscriber lockedYSub;
    private final BooleanSubscriber enabledSub;

    private double currentSpeed;

    private JFrame frame;

    public PassTargetPicker() {
        fieldImage  = loadFieldImage();
        fieldPanel  = new FieldPanel();
        enableBox   = new JCheckBox("Pass target enabled");
        statusLabel = new JLabel();

        // Local — only needed to create the subscribers; not stored as a field.
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");

        cursorXSub = table.getDoubleTopic(PassTargetConstants.cursorXKey)
                          .subscribe(PassTargetConstants.defaultTargetX);
        cursorYSub = table.getDoubleTopic(PassTargetConstants.cursorYKey)
                          .subscribe(PassTargetConstants.defaultTargetY);
        lockedXSub = table.getDoubleTopic(PassTargetConstants.targetXKey)
                          .subscribe(PassTargetConstants.defaultTargetX);
        lockedYSub = table.getDoubleTopic(PassTargetConstants.targetYKey)
                          .subscribe(PassTargetConstants.defaultTargetY);
        enabledSub = table.getBooleanTopic(PassTargetConstants.enableKey)
                          .subscribe(PassTargetConstants.defaultEnable);

        currentSpeed = SmartDashboard.getNumber("Turret/Pass/CursorSpeed", 0.0);
    }

    public void start() {
        SwingUtilities.invokeLater(() -> {
            if (frame != null) { frame.setVisible(true); return; }

            frame = new JFrame("Pass Target Picker");
            frame.setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);

            enableBox.setSelected(enabledSub.get());
            enableBox.addActionListener(e -> SmartDashboard.putBoolean(
                PassTargetConstants.enableKey, enableBox.isSelected()));

            JPanel controls = new JPanel(new FlowLayout(FlowLayout.LEFT));
            controls.add(enableBox);
            controls.add(statusLabel);

            frame.getContentPane().setLayout(new BorderLayout());
            frame.getContentPane().add(fieldPanel, BorderLayout.CENTER);
            frame.getContentPane().add(controls, BorderLayout.SOUTH);

            frame.setMinimumSize(new Dimension(700, 400));
            frame.setLocationByPlatform(true);
            frame.setVisible(true);

            Timer refresh = new Timer(REFRESH_MS, e -> refreshUi());
            refresh.start();
            refreshUi();
        });
    }

    // Called exclusively on the EDT via the Swing Timer. NT subscriber.get() calls
    // are thread-safe per the NT4 API, and the snapshot fields (cursorX, etc.) are
    // both written here and read in paintComponent() — both on the EDT — so no
    // additional synchronization is needed.
    private void refreshUi() {
        enabled = enabledSub.get();
        cursorX = cursorXSub.get();
        cursorY = cursorYSub.get();
        lockedX = lockedXSub.get();
        lockedY = lockedYSub.get();

        currentSpeed = SmartDashboard.getNumber("Turret/Pass/CursorSpeed", 0.0);

        enableBox.setSelected(enabled);
        statusLabel.setText(String.format(
            "Cursor: (%.2f, %.2f) Locked: (%.2f, %.2f) Current Cursor Speed: %.2f",
            cursorX, cursorY, lockedX, lockedY, currentSpeed));

        fieldPanel.repaint();
    }

    private BufferedImage loadFieldImage() {
        try {
            return ImageIO.read(getClass().getResource("/2026_field.png"));
        } catch (IOException | IllegalArgumentException e) {
            e.printStackTrace();
            return null;
        }
    }

    private final class FieldPanel extends JComponent {
        private final Rectangle imageRect = new Rectangle();

        @Override
        protected void paintComponent(Graphics graphics) {
            super.paintComponent(graphics);
            Graphics2D g2 = (Graphics2D) graphics.create();
            g2.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                                RenderingHints.VALUE_INTERPOLATION_BILINEAR);
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                                RenderingHints.VALUE_ANTIALIAS_ON);

            computeImageRect();

            if (fieldImage != null) {
                g2.drawImage(fieldImage,
                    imageRect.x, imageRect.y, imageRect.width, imageRect.height, null);
            } else {
                g2.setColor(new Color(40, 40, 40));
                g2.fillRect(imageRect.x, imageRect.y, imageRect.width, imageRect.height);
            }

            if (enabled) drawLockedMarker(g2, lockedX, lockedY);
            drawCursorCrosshair(g2, cursorX, cursorY);

            g2.dispose();
        }

        private void drawLockedMarker(Graphics2D g2, double fx, double fy) {
            int px = fieldToPixelX(fx);
            int py = fieldToPixelY(fy);

            double scaleX = imageRect.getWidth()  / FieldConstants.fieldLength;
            double scaleY = imageRect.getHeight() / FieldConstants.fieldWidth;
            double r = TurretConstants.passTargetRadiusMeters * Math.min(scaleX, scaleY);

            g2.setColor(new Color(100, 180, 255, 150));
            g2.setStroke(new BasicStroke(2.5f));
            g2.drawOval(
                (int) Math.round(px - r), (int) Math.round(py - r),
                (int) Math.round(r * 2),  (int) Math.round(r * 2));

            g2.setColor(new Color(80, 160, 255));
            g2.fillOval(px - 5, py - 5, 10, 10);

            g2.setColor(Color.WHITE);
            g2.setFont(g2.getFont().deriveFont(Font.BOLD, 12f));
            g2.drawString("Locked", px + 8, py - 6);
        }

        private void drawCursorCrosshair(Graphics2D g2, double fx, double fy) {
            int px = fieldToPixelX(fx);
            int py = fieldToPixelY(fy);
            int inner = 6, arm = 14;

            g2.setColor(new Color(255, 215, 50, 230));
            g2.setStroke(new BasicStroke(2.0f));
            g2.drawLine(px - arm, py, px - inner, py);
            g2.drawLine(px + inner, py, px + arm,  py);
            g2.drawLine(px, py - arm, px, py - inner);
            g2.drawLine(px, py + inner, px, py + arm);
            g2.drawOval(px - inner, py - inner, inner * 2, inner * 2);

            g2.setFont(g2.getFont().deriveFont(Font.PLAIN, 11f));
            g2.setColor(new Color(255, 215, 50));
            g2.drawString("A to lock", px + 10, py + 4);
        }

        private void computeImageRect() {
            int pw = getWidth(), ph = getHeight();
            if (pw <= 0 || ph <= 0) { imageRect.setBounds(0, 0, 0, 0); return; }

            double aspect = FieldConstants.fieldLength / FieldConstants.fieldWidth;
            int dw = pw, dh = (int) Math.round(pw / aspect);
            if (dh > ph) { dh = ph; dw = (int) Math.round(ph * aspect); }
            imageRect.setBounds((pw - dw) / 2, (ph - dh) / 2, dw, dh);
        }

        private int fieldToPixelX(double fx) {
            return (int) Math.round(
                imageRect.x + (fx / FieldConstants.fieldLength) * imageRect.width);
        }

        private int fieldToPixelY(double fy) {
            return (int) Math.round(
                imageRect.y + ((FieldConstants.fieldWidth - fy)
                               / FieldConstants.fieldWidth) * imageRect.height);
        }
    }
}