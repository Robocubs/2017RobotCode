// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc.team1701.robot.subsystems;

import org.usfirst.frc.team1701.robot.RobotMap;
import org.usfirst.frc.team1701.robot.commands.StopClimb;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	CANTalon climberMotor1 = (CANTalon) RobotMap.climberMotor1;
	CANTalon climberMotor2 = (CANTalon) RobotMap.climberMotor2;
	
	public static double CLIMB_POWER = .70;
	public static int CLIMB_DIRECTION = -1;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public void resetEncoder() {
		climberMotor1.setEncPosition(0);
	}

	public int getEncVelocity() {
		return climberMotor1.getEncVelocity();
	}

	public int getEncPosition() {
		return climberMotor1.getEncPosition();
	}

	public double getCurrent() {
		return climberMotor1.getOutputCurrent();
	}

	public void turnOn() {
		//moved to command
//		if (RobotMap.lightsLED2.get() == Relay.Value.kForward) {
//			RobotMap.lightsLED2.set(Relay.Value.kOn);
//		} else {
//			RobotMap.lightsLED2.set(Relay.Value.kReverse);
//		}

		climberMotor1.set(CLIMB_DIRECTION * CLIMB_POWER);
		climberMotor2.set(CLIMB_DIRECTION * CLIMB_POWER);
	}

	public void turnOff() {
		climberMotor1.set(0);
		climberMotor2.set(0);
	}

	public void initDefaultCommand() {
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new StopClimb());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}