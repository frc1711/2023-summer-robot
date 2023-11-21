// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class Kinematics {

    AHRS gyro;

    Timer xAccelTimer, yAccelTimer;

    public Kinematics (AHRS gyro) {
        this.gyro = gyro;
        xAccelTimer = new Timer();
        yAccelTimer = new Timer();
        xAccelTimer.start();
        yAccelTimer.start();
    }

    public boolean directionHasChanged (Vector vectorOne, Vector vectorTwo) {
        return getDirectionChange(vectorOne, vectorTwo) >= 1;
    }

    public double getRobotAccelerationX () {
        return gyro.getWorldLinearAccelX();
    }

    public double getRobotAccelerationY () {
        return gyro.getWorldLinearAccelY();
    }

    private double totalDisplacementY;
    public double getRobotDisplacementY () {
        return 0;
    }

    private double totalDisplacementX;
    public double getRobotDisplacementX () {
        return 0;
    }

    private void resetTimerX () {
        xAccelTimer.reset();
    }
    
    public Vector getAccelerationVector() {
        return new Vector<>(
            Nat.N1(),
            getRobotAccelerationX(),
            getRobotAccelerationY());
    }

    public double getDirectionChange (Vector vectorOne, Vector vectorTwo) {
        return Vector.getAngle(vectorTwo) - Vector.getAngle(vectorOne);
    }

    public Transform2d getRobotRelativePosition () {
        return null;
    }
}
