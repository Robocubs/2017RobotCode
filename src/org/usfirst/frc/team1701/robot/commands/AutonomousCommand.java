package org.usfirst.frc.team1701.robot.commands;

import org.usfirst.frc.team1701.robot.Robot;
import org.usfirst.frc.team1701.robot.RobotMap;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutonomousCommand extends Command {
	public static final double DRIVE_FORWARD_DISTANCE = 135.0; // enough to
	// cross the
	// baseline
	// (tested
	// to be 95)
	// reversed 3/4 at Southfield after round 63
	// I don't know why we need to
	public static final double AUTO_DRIVE_SPEED = -1.0;
	public static final double AUTO_TURN_CORRECT = -.03;
	public static final double DRIVE_CORRECTION = 1.3;
	private double actualDriveSpeed = 0;
	private final double AUTO_TURN_SPEED = .3;
	private boolean isFinished = false;
	private int currentState;
	private NetworkTable visionTable;
	private boolean turnLeft = false;
	private int autonomousMode;

	public AutonomousCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
        requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrain.resetLeftEncoder();
		Robot.driveTrain.resetRightEncoder();
		// RobotMap.navx.reset();
		// actualDriveSpeed = 0;
		Robot.driveTrain.setActualDriveSpeed(0.0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.lights.getTargetingLED().set(Relay.Value.kOn);
		SmartDashboard.putNumber("Left Encoder Reading: ", Robot.driveTrain.getLeftDistance());
		SmartDashboard.putNumber("Right Encoder Reading: ", Robot.driveTrain.getRightDistance());
		// SmartDashboard.putNumber("Navx Reading: ", RobotMap.navx.getYaw());
		 if (Robot.driveTrain.getLeftDistance() > -1 * DRIVE_CORRECTION * DRIVE_FORWARD_DISTANCE || 
				 Robot.driveTrain.getRightDistance() > -1 * DRIVE_CORRECTION * DRIVE_FORWARD_DISTANCE) {
			 Robot.driveTrain.teleopControl(AUTO_DRIVE_SPEED, 0);
			 SmartDashboard.putNumber("AutoForward Speed: ", AUTO_DRIVE_SPEED);
		 } else {
			 RobotMap.driveTrainRM.arcadeDrive(0, 0);
		 }
		
		//drivetrain is already reversed
		
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		RobotMap.driveTrainRM.arcadeDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		RobotMap.driveTrainRM.arcadeDrive(0, 0);
	}
}
