package frc.robot.subsystems.PositionalPID;

public class PositionalPIDConstants {
  public record PositionalPIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record PositionalPIDHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final PositionalPIDHardwareConfig EXAMPLE_CONFIG =
      new PositionalPIDHardwareConfig(new int[] {42}, new boolean[] {false}, (24.0 / 48.0), "");

  public static final PositionalPIDGains EXAMPLE_GAINS =
      new PositionalPIDGains(0.00006, 0.0, 0.0, 0.0, 0.01, 0.0);
}
