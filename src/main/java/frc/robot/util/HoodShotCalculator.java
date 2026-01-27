// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TurretConstants;

/** Add your docs here. */
public class HoodShotCalculator {

  public record ShotSolution(Angle hoodAngle, LinearVelocity exitVelocity) {}

  private static LinearVelocity solveVelocityGivenAngle(
      Distance distance, Distance deltaHeight, Angle shotAngle) {

    double d = distance.in(Meters);
    double dH = deltaHeight.in(Meters);
    double angle = shotAngle.in(Radians);

    double g = 9.80665;

    double denominator = 2 * Math.cos(angle) * Math.cos(angle) * (d * Math.tan(angle) - dH);

    if (denominator <= 0) {
      return MetersPerSecond.of(Double.NaN);
    }

    double velocity = Math.sqrt((g * d * d) / denominator);
    return MetersPerSecond.of(velocity);
  }

  public static ShotSolution solveShot(
      Distance distance, Distance targetHeight, LinearVelocity preferredVelocity) {

    double d = distance.in(Meters);
    double dh = targetHeight.minus(TurretConstants.robotToTurret.getMeasureZ()).in(Meters);
    double v = preferredVelocity.in(MetersPerSecond);
    double g = 9.80665;

    double vSq = v * v;
    double discriminant = vSq * vSq - g * (g * d * d + 2 * dh * vSq);

    if (discriminant >= 0) {
      Angle lowShot = Radians.of(Math.atan((vSq - Math.sqrt(discriminant)) / (g * d)));
      Angle highShot = Radians.of(Math.atan((vSq + Math.sqrt(discriminant)) / (g * d)));

      Angle lowHood = Degrees.of(90).minus(highShot);
      Angle highHood = Degrees.of(90).minus(lowShot);

      boolean lowValid =
          lowHood.in(Degrees) >= HoodConstants.minAngle.in(Degrees)
              && lowHood.in(Degrees) <= HoodConstants.maxAngle.in(Degrees);

      if (lowValid) {
        return new ShotSolution(lowHood, preferredVelocity);
      }
    }

    // force hood to min angle and solve for velocityy
    // TO DO: potentially force it to a more optimal angle???
    Angle forcedHoodAngle = HoodConstants.minAngle;
    Angle forcedShotAngle = Degrees.of(90).minus(forcedHoodAngle);

    LinearVelocity newVelocity =
        solveVelocityGivenAngle(
            distance,
            targetHeight.minus(TurretConstants.robotToTurret.getMeasureZ()),
            forcedShotAngle);

    if (Double.isNaN(newVelocity.in(MetersPerSecond))) {
      return new ShotSolution(forcedHoodAngle, MetersPerSecond.of(0));
    }

    return new ShotSolution(forcedHoodAngle, newVelocity);
  }
}
