package frc.robot.subsystems.PositionalPID;

import frc.robot.subsystems.PositionalPID.PositionalPIDConstants.PositionalPIDGains;
import org.littletonrobotics.junction.AutoLog;

public interface PositionalPIDIO {
  @AutoLog
  public static class PositionalPIDIOInputs {
    public double velocity = 0.0;
    public double desiredPosition = 0.0;
    public double desiredVelocity = 0.0;

    public boolean[] motorsConnected = {false};

    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};
  }

  public default void updateInputs(PositionalPIDIOInputs inputs) {}

  public default void setVelocity(double velocity) {}

  public default void setPosition(double position, double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(PositionalPIDGains gains) {}

  public default String getName() {
    return "PositionalPID";
  }
}
