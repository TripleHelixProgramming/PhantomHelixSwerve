package org.usfirst.frc.team2363.robot.commands;

import static org.usfirst.frc.team2363.robot.Robot.drivetrain;
import static org.usfirst.frc.team2363.robot.Robot.oi;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDTurnModules extends Command {

    public PIDTurnModules() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
    	drivetrain.setFrontLeftAngle((oi.getZ() + 1) / 2 * 1000);
    	drivetrain.setFrontRightAngle((oi.getZ() + 1) / 2 * 1000);
    	drivetrain.setRearLeftAngle((oi.getZ() + 1) / 2 * 1000);
    	drivetrain.setRearRightAngle((oi.getZ() + 1) / 2 * 1000);
    	
    	SmartDashboard.putNumber("Left Front Position", drivetrain.getFrontLeftAngle());
    	SmartDashboard.putNumber("Right Front Position", drivetrain.getFrontRightAngle());
    	SmartDashboard.putNumber("Left Front Rear", drivetrain.getRearLeftAngle());
    	SmartDashboard.putNumber("Right Rear Position", drivetrain.getRearRightAngle());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}
