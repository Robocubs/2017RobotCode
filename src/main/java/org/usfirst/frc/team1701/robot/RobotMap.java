/**
 * RobotMap.java
 *
 * Created by Nicholas Hubbard on 11/07/2017.
 *
 * Copyright (c) 2017 Team 1701 (Robocubs)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 
 * Neither the name of the project's author nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.usfirst.frc.team1701.robot;
import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // Create all of our initial variables. To be populated.
    public static Encoder shooterSystemShootingEncoder;
    public static DoubleSolenoid shooterSystemShooterLoader;
    public static DigitalOutput visionDigitalOutput1;
    public static AnalogInput gearArmGearDistanceSensor;
    public static DoubleSolenoid gearArmGearPusher;
    public static Relay lightsLED1;
    public static Relay lightsLED2;
    public static Relay lightsLED3;
    public static Relay lightsTargeting;
    public static SpeedController driveTrainLeft_1;
    public static SpeedController driveTrainLeft_2;
    public static SpeedController driveTrainRight_1;
    public static SpeedController driveTrainRight_2;
    public static SpeedController climberMotor1;
    public static SpeedController climberMotor2;
    public static SpeedController pickupBottomMotor;
    public static RobotDrive driveTrainRM;
    public static SpeedController shoot1;
    public static SpeedController hopperMotor;
    public static final int VERSAPULSES = 1024;
    public static AHRS navx;
    public static void init() {
        // Populate these values.
        shooterSystemShootingEncoder = new Encoder(8, 9, false, EncodingType.k4X);
        LiveWindow.addSensor("ShooterSystem", "ShootingEncoder", shooterSystemShootingEncoder);
        shooterSystemShootingEncoder.setDistancePerPulse(1.0);
        shooterSystemShootingEncoder.setPIDSourceType(PIDSourceType.kRate);
        shooterSystemShooterLoader = new DoubleSolenoid(0, 0, 1);
        LiveWindow.addActuator("ShooterSystem", "ShooterLoader", shooterSystemShooterLoader);
        visionDigitalOutput1 = new DigitalOutput(0);
        LiveWindow.addActuator("Vision", "Digital Output 1", visionDigitalOutput1);
        gearArmGearDistanceSensor = new AnalogInput(0);
        LiveWindow.addSensor("GearArm", "GearDistanceSensor", gearArmGearDistanceSensor);
        gearArmGearPusher = new DoubleSolenoid(0, 2, 3);
        LiveWindow.addActuator("GearArm", "GearPusher", gearArmGearPusher);
        lightsLED1 = new Relay(1);
        LiveWindow.addActuator("Lights", "LED1", lightsLED1);
        lightsLED2 = new Relay(2);
        LiveWindow.addActuator("Lights", "LED2", lightsLED2);
        lightsLED3 = new Relay(3);
        LiveWindow.addActuator("Lights", "LED3", lightsLED3);
        lightsTargeting = new Relay(0);
        LiveWindow.addActuator("Lights", "Targeting", lightsTargeting);  
        driveTrainLeft_1 = new CANTalon(2);
        LiveWindow.addActuator("DriveTrain", "Left_1", (CANTalon) driveTrainLeft_1);
        driveTrainLeft_2 = new CANTalon(3);
        LiveWindow.addActuator("DriveTrain", "Left_2", (CANTalon) driveTrainLeft_2);
        driveTrainRight_1 = new CANTalon(12);
        LiveWindow.addActuator("DriveTrain", "Right_1", (CANTalon) driveTrainRight_1);
        driveTrainRight_2 = new CANTalon(13);
        LiveWindow.addActuator("DriveTrain", "Right_2", (CANTalon) driveTrainRight_2);
        driveTrainRM = new RobotDrive(driveTrainLeft_1, driveTrainLeft_2, driveTrainRight_1, driveTrainRight_2);
        driveTrainRM.setSafetyEnabled(false);
        driveTrainRM.setExpiration(0.1);
        driveTrainRM.setSensitivity(0.5);
        driveTrainRM.setInvertedMotor(MotorType.kFrontRight, true);
        driveTrainRM.setInvertedMotor(MotorType.kRearRight, true);
        driveTrainRM.setInvertedMotor(MotorType.kFrontLeft, true);
        driveTrainRM.setInvertedMotor(MotorType.kRearLeft, true);
        driveTrainRM.setMaxOutput(1.0);
        shoot1 = new CANTalon(1);
        climberMotor1 = new CANTalon(15);// top climber motor
        ((CANTalon) climberMotor1).configEncoderCodesPerRev(VERSAPULSES);
        climberMotor2 = new CANTalon(14); // bottom climber motor
        pickupBottomMotor = new CANTalon(11);
        ((CANTalon) pickupBottomMotor).configEncoderCodesPerRev(VERSAPULSES);
        hopperMotor = new CANTalon(0);
        // We initialize the NavX in the main file.
    }
}