package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {
  }

  public static void main(String... args) {
    frc.team_8840_lib.listeners.Robot.assignListener(new MyRobot());
    RobotBase.startRobot(frc.team_8840_lib.listeners.Robot::new);
  }
}
