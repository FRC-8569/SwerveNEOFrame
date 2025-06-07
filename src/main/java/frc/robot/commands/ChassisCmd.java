package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveChassis;

public class ChassisCmd extends Command {
    public SwerveChassis chassis;
    public Supplier<Double> xSpeed, ySpeed, zRotation;

    public ChassisCmd(SwerveChassis chassis, @SuppressWarnings("unchecked") Supplier<Double>... suppliers){
        this.chassis = chassis;
        this.xSpeed = suppliers[0];
        this.ySpeed = suppliers[1];
        this.zRotation = suppliers[2];
    }

    @Override
    public void execute(){
        chassis.drive(new ChassisSpeeds(xSpeed.get(), ySpeed.get(), zRotation.get()));
    }
}
