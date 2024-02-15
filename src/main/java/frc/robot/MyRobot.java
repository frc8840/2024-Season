package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.listeners.EventListener;
import frc.team_8840_lib.utils.async.Promise;

public class MyRobot extends EventListener {

    private RobotContainer robotContainer;

    @Override
    public void onAutonomousEnable() {
        // TODO Auto-generated method stub

    }

    @Override
    public void onAutonomousPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void onDisabled() {
        // TODO Auto-generated method stub

    }

    @Override
    public void onDisabledPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void onTeleopEnable() {
        // TODO Auto-generated method stub

    }

    @Override
    public void onTeleopPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void onTestEnable() {
        // TODO Auto-generated method stub

    }

    @Override
    public void onTestPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void robotInit() {
        Logger.Log("Hello world!");
        robotContainer = new RobotContainer();
        // frc.team_8840_lib.listeners.Robot.getRealInstance().waitForFullfillConditions(
        // 3000,
        // new Promise((res, rej) -> {
        // Promise.WaitThen(() -> {
        // return robotContainer.getSwerve().getSwerveDrive().isReady();
        // }, res, rej, 10);
        // }));

    }

    @Override
    public void robotPeriodic() {
        // TODO Auto-generated method stub
        CommandScheduler.getInstance().run();
    }

}
