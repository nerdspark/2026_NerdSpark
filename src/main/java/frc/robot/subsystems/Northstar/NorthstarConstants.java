// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Northstar;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

import lombok.Builder;
import lombok.experimental.ExtensionMethod;
import frc.robot.util.Northstar.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class NorthstarConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.01;
  public static final double thetaStdDevCoefficient = 0.03;

  private static int monoExposure = 1500;
  private static double monoGain = 15.0;
  private static double monoDenoise = 1.0;
  
  public static CameraConfig[] cameras = new CameraConfig[] {
    CameraConfig.builder()
      .poseFunction(
        (Double timestamp) -> {
          return Optional.of(
            new Pose3d(
              Units.inchesToMeters(-28.0 / 2.0 + 2.5),
              Units.inchesToMeters(-28.0 / 2.0 + 2.75),
              Units.inchesToMeters(18.75),
              new Rotation3d(
                0.0,
                Units.degreesToRadians(-27.0),
                Units.degreesToRadians(-152.5)
              )
            )
          );
        }
      )
      .name("cam_name")
      .width(1600) 
      .height(1200)
      .exposure(monoExposure)
      .gain(monoGain)
      .denoise(monoDenoise)
      .stdDevFactor(1.0)
      .build()
  };

  public static final Map<String, Integer> cam_names;

  static {
    Map<String, Integer> map = new HashMap<>();
    for (int i = 0; i < cameras.length; i++) {
      map.put(cameras[i].name(), i);
    }
    cam_names = Collections.unmodifiableMap(map);
  }


  @Builder
  public record CameraConfig(
      Function<Double, Optional<Pose3d>> poseFunction,
      String name,
      int width,
      int height,
      int autoExposure,
      int exposure,
      double gain,
      double denoise,
      double stdDevFactor) {}

  private NorthstarConstants() {}
}
