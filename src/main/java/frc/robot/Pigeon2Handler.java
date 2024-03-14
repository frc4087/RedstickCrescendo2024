// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.MathUtil;
import com.ctre.phoenix6.hardware.Pigeon2;


/** Add your docs here. */
public class Pigeon2Handler {
    private Pigeon2 pigeon = new Pigeon2(10);
   
    //private double initialAngle;

    //public double getInitialAngle() {
        //return initialAngle;
    //}

    //public void setInitialAngle() {
        //initialAngle = getAngleRad();
    //}

    public Pigeon2 getAhrs() {
        return pigeon;
    }

   

    public Pigeon2Handler() {

        
      
    }

  

    public void printEverything() {
        // SmartDashboard.putNumber("getAngle()", ahrs.getAngle());

        // SmartDashboard.putNumber("getDisplacementX()", ahrs.getDisplacementX());
        // SmartDashboard.putNumber("getDisplacementY()", ahrs.getDisplacementY());
        // SmartDashboard.putNumber("getDisplacementZ()", ahrs.getDisplacementZ());

        // SmartDashboard.putNumber("getVelocityX()", ahrs.getVelocityX());
        // SmartDashboard.putNumber("getVelocityY()", ahrs.getVelocityY());
        // SmartDashboard.putNumber("getVelocityZ()", ahrs.getVelocityZ());

        // SmartDashboard.putNumber("Pitch", pigeon.getPitch());
        // SmartDashboard.putNumber("Roll", pigeon.getRoll());
        // SmartDashboard.putNumber("Yaw", pigeon.getYaw());
        //SmartDashboard.putNumber("pigeon2 Angle", MathUtil.mod(getAngleRad(), 2 * Math.PI));
    }

    public Rotation2d getAngleRad() {
        return Rotation2d.fromRadians(MathUtil.mod(pigeon.getYaw().getValue() * 2 * Math.PI / 360, Math.PI * 2));
    }

    public Rotation2d getAngleDeg() {
         return Rotation2d.fromDegrees(pigeon.getYaw().getValue());
    }

    /*public double getAngle() {
        return ahrs.getAngleAdjustment();
    }*/

    public void zeroYaw() {
        pigeon.setYaw(-90);

    }

    /*public double getVelocity() {
        return Math.sqrt(Math.pow(ahrs.getVelocityX(), 2) + Math.pow(ahrs.getVelocityY(), 2));
    }*/
    public double AccumZ(){
       return 0;
    }
}
