package org.firstinspires.ftc.teamcode.PathFollower;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class ZachPathFollower {
    SampleMecanumDrive drive;

    //0 to 1
    double maxVelocity;

    //distance to start slowing down
    double accelerationDistance;

    //current path
    ZachPathGenerator currentPath;

    State currentState = State.Idle;
    enum State {
        Following,
        Idle,
    }

    public ZachPathFollower(SampleMecanumDrive drive) {
        this.drive = drive;
        this.maxVelocity = 0.5;
        this.accelerationDistance = 10;
    }

    public ZachPathFollower(SampleMecanumDrive drive, double maxVelocity,double accelerationDistance) {
        this.drive = drive;
        this.maxVelocity = maxVelocity;
        this.accelerationDistance = accelerationDistance;
    }

    //follows path
    public void FollowPathAsync(ZachPathGenerator path){
        currentState = State.Following;
    }

    //follows path
    public void FollowPath(ZachPathGenerator path){
        FollowPathAsync(path);
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    //return if currently following a path
    public boolean isBusy(){
        return currentState == State.Following;
    }

    //cancel the following
    public void cancleFollowing(){
        currentState = State.Idle;
    }

    //update wheel speeds must be called in a loop while following a path from zachPathGenerator
    public void update(){
        if(currentState == State.Following){
            double power = 1;

            double DistanceToTarget = Math.abs(currentPath.getEndX() - drive.getPoseEstimate().getX());

            //if we are close to target start reducing power
            if(DistanceToTarget < accelerationDistance){
                //start slowing down always keep power over 0.1
                //graph I made: motorPower = y, DistanceToTarget = x
                //https://www.desmos.com/calculator/9qcrvbhcsn
                power = ((0.9 / accelerationDistance) * DistanceToTarget) + 0.1;

            }
            power *= maxVelocity;


            //towards chalkboard
            if(currentPath.getEndX() > drive.getPoseEstimate().getX()){
                drive.setDrivePower(new Pose2d(Math.abs(power),0,0));
            }
            else{
                drive.setDrivePower(new Pose2d(-Math.abs(power),0,0));
            }

            if(DistanceToTarget < 0.5){
                drive.setDrivePower(new Pose2d(0,0,0));
                currentState = State.Idle;
            }
        }
    }
}
