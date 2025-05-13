package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
  public record FlywheelGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record FlywheelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final FlywheelHardwareConfig EXAMPLE_CONFIG =
      new FlywheelHardwareConfig(new int[] {42}, new boolean[] {false}, 24.0 / 48.0, "");

  public static final FlywheelGains EXAMPLE_GAINS =
      new FlywheelGains(0.00006, 0.0, 0.0, 0.0, 0.01, 0.0);
}
