// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Kinematics extends SubsystemBase {

        AHRS gyro;

        Timer xAccelTimer, yAccelTimer, totalTimer;

        StartPosition startPosition;

        public Kinematics (AHRS gyro, StartPosition startPosition) {
            this.gyro = gyro;
            this.startPosition = startPosition;
            xAccelTimer = new Timer();
            yAccelTimer = new Timer();
            totalTimer = new Timer();
            xAccelTimer.start();
            yAccelTimer.start();
        }

        //TODO: Find coordinates of start positions
        public enum StartPosition {
            WIREGUARD (0, 0),
            BALANCE (0, 0),
            INSIDE (0, 0);
            double xVariable, yVariable;
            StartPosition (double xVariable, double yVariable) {
                this.xVariable = xVariable;
                this.yVariable = yVariable;
            }
        }

        public boolean directionHasChanged (Vector<N2> vectorOne, Vector<N2> vectorTwo) {
            return getDirectionChange(vectorOne, vectorTwo) >= 1;
        }

        public double getRobotAccelerationX () {
            return gyro.getWorldLinearAccelX();
        }

        public double getRobotAccelerationY () {
            return gyro.getWorldLinearAccelY();
        }

        public double getRobotDisplacementY () {
            return .5 * getRobotAccelerationY() * Math.pow(yAccelTimer.get(), 2) + startVelocityY * yAccelTimer.get() + startPosition.yVariable; //TODO: Find unknown constant either through testing or math
        }

        public double getRobotDisplacementX () {
            return .5 * getRobotAccelerationX() * Math.pow(xAccelTimer.get(), 2) + startVelocityX * xAccelTimer.get() + startPosition.xVariable; //TODO: Find unknown constant either through testing or math
        }

        private double startVelocityX;
        public double getVelocityX (double startVelocityX) {
            this.startVelocityX = startVelocityX;
            return getRobotAccelerationX() * xAccelTimer.get() + startVelocityX; //TODO: find constant variable through testing(?)
        }

        private double startVelocityY;
        public double getVelocityY (double startVelocityY) {
            this.startVelocityY = startVelocityY;
            return getRobotAccelerationY() * yAccelTimer.get() + startVelocityY; //TODO: find unknown constant variable
        }

        public Vector<N2> getDisplacementVector () {
            return new Vector<>(Nat.N2(), getRobotAccelerationX(), getRobotDisplacementY());
        }
        
        public Vector<N2> getAccelerationVector() {
            return new Vector<>(
                Nat.N2(),
                getRobotAccelerationX(),
                getRobotAccelerationY());
        }

        public double getDirectionChange (Vector<N2> vectorOne, Vector<N2> vectorTwo) {
            return Vector.getAngle(vectorTwo) - Vector.getAngle(vectorOne);
        }

        Vector<N2> position;
        public void updateRobotPosition (Vector<N2> vectorOne, Vector<N2> vectorTwo) {
            if (vectorOne == null) vectorOne = new Vector<>(Nat.N2(), 0, 0);
            if (vectorTwo == null) vectorTwo = new Vector<>(Nat.N2(), 0, 0);
            position = vectorOne.add(vectorTwo);
        }

        public Translation2d getRobotRelativePosition () {
            return new Translation2d(position.getX(), position.getY());
        }

        Vector<N2> vectorOne = null;
        Vector<N2> vectorTwo = null;   
        boolean oneHasBeenSet = false;
        boolean twoHasBeenSet = false;
        @Override
        public void periodic() {
            if (totalTimer.hasElapsed(.1) && !oneHasBeenSet) {
                vectorOne = getDisplacementVector();

                oneHasBeenSet = true;
            }
            if (totalTimer.hasElapsed(.5) && twoHasBeenSet) {
                vectorTwo = getDisplacementVector();

                twoHasBeenSet = true;
            }        
            
            if (oneHasBeenSet && twoHasBeenSet) {
                updateRobotPosition(vectorOne, vectorTwo);

                oneHasBeenSet = false;
                twoHasBeenSet = false;
                totalTimer.reset();
            }

            if (directionHasChanged(vectorOne, vectorTwo)) {
                xAccelTimer.reset();
                yAccelTimer.reset();
            }
        }
    }
