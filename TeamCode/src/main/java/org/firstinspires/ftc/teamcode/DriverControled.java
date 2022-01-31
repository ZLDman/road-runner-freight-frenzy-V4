package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class DriverControled extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);

        double bucketPos = 0.08;
        double liftPos = 0.93;
        double intakePos = 0.45;

        boolean erik = false;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        //put encoder servo down
        robot.encoderservo.setPosition(0);

        while (!isStopRequested()) {

            //Erik
            if (gamepad1.x)
            {
                erik = true;
            }

            if (gamepad1.y)
            {
                erik = false;
            }

            if(erik) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_x / 1.5,
                                (-gamepad1.left_stick_y / 1.5) + 0.1,
                                -gamepad1.right_stick_y / 2
                        )
                );
            }
            //Renee
            else{
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y / 1.5,
                                (-gamepad1.left_stick_x / 1.5) + 0.1,
                                -gamepad1.right_stick_x / 2
                        )
                );
            }


            if(gamepad2.x){
                robot.intakeState = Robot.IntakeState.INTAKE;
            }

            if(gamepad2.y){
                robot.extendState = Robot.ExtendState.EXTEND;
            }


            /* INTAKE BUCKET */
            if(gamepad2.dpad_up){
                robot.setIntakeBucketState(Robot.IntakeBucket.UP);
            }
            else if(gamepad2.dpad_right){
                robot.setIntakeBucketState(Robot.IntakeBucket.RIGHT);
            }
            else if(gamepad2.dpad_left){
                robot.setIntakeBucketState(Robot.IntakeBucket.LEFT);
            }
            robot.updateIntakeBucket();

            /* INTAKE */
            if(robot.intakeState == Robot.IntakeState.MANUAL){
                if(gamepad2.right_bumper) {
                    robot.setIntake1Speed(-gamepad2.right_trigger);
                }
                else{
                    robot.setIntake1Speed(gamepad2.right_trigger);
                }

                if(gamepad2.left_bumper) {
                    robot.setIntake2Speed(-gamepad2.left_trigger);
                }
                else{
                    robot.setIntake2Speed(gamepad2.left_trigger);
                }
            }

            /* CAROUSEL */
            robot.setCarSpeed(gamepad1.right_trigger - gamepad1.left_trigger);

            //robot.updateIntake();
            robot.updateExtend();

            robot.tapelift += -gamepad2.left_stick_y * 0.01;

            robot.tapelift = Math.max(0.43, robot.tapelift);
            robot.tapelift = Math.min(0.66, robot.tapelift);

            robot.tapeextendservo.setPower(gamepad2.right_stick_y);
            robot.tapeliftservo.setPosition(robot.tapelift);

            if(gamepad2.right_stick_x > 0) {
                robot.taperotateservo.setPower(gamepad2.right_stick_x / 10);
            }
            else{
                robot.taperotateservo.setPower(gamepad2.right_stick_x / 20);
            }

            //robot.encoderservo.setPosition(0.25);

            //robot.setExtendSpeed(gamepad2.left_stick_y);

            //lift pos = 0.758

            /*

            reset
            l 0.04
            b 0

            0.3
            0.18

            0.73
            0.56

            high
            l 0.73
            b 0.91



             */

            liftPos += gamepad2.right_stick_y * 0.01;
            bucketPos += gamepad2.right_stick_x * 0.03;

            if(gamepad2.a){//home
                liftPos = 0.94;
                bucketPos = 0.08;
            }
            if(gamepad2.x){//high
                liftPos = 0.4;
                bucketPos = 0.45;
            }
            if(gamepad2.b){//low
                liftPos = 0;
                bucketPos = 0.95;
            }
            if(gamepad2.y){//place
                if(liftPos == 0.4) {
                    bucketPos = 0.18;
                }
                if(liftPos == 0) {
                    bucketPos = 0.54;
                }

            }

            liftPos = Math.max(0,liftPos);
            liftPos = Math.min(1,liftPos);

            bucketPos = Math.max(0,bucketPos);
            bucketPos = Math.min(1,bucketPos);

            //robot.bucket.setPosition(bucketPos);
            //robot.setLiftPosition(liftPos);

            robot.updateLiftServo();
/*

            robot.setIntake1Speed(gamepad2.left_trigger - gamepad2.right_trigger);

            if(gamepad2.dpad_up){
                intakePos += 0.02;
            }
            else if(gamepad2.dpad_down){
                intakePos -= 0.02;
            }

            intakePos = Math.max(0.14,intakePos);
            intakePos = Math.min(0.77,intakePos);
            if(robot.getColor(-1,1) < 1 && robot.intake1.getPower() == 0){
                robot.intake.setPosition(0.5);
            }
            else {
                robot.intake.setPosition(intakePos);
            }


            if (gamepad1.a) {
                robot.setCarSpeed(1);
            }
            else {
                robot.setCarSpeed(0);
            }*/

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            //position
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            //motor encoders
            telemetry.addData("extend: ", robot.extend.getCurrentPosition());
            telemetry.addData("intake1: ", robot.intake1.getCurrentPosition());
            telemetry.addData("intake1: ", robot.intake1.getCurrentPosition() % 45);
            telemetry.addData("intake1: ", robot.intake1.getTargetPosition());
            telemetry.addData("lift: ", robot.lift.getPosition());
            telemetry.addData("bucket: ", robot.bucket.getPosition());
            telemetry.addData("intake: ", intakePos);

            telemetry.addData("tape lift: ", robot.tapeliftservo.getPosition());
            telemetry.addData("encoderServo: ", robot.encoderservo.getPosition());

            //digital channels
            telemetry.addData("extendStop: ", robot.extendStop.getState());


            //color sensor
            //telemetry.addData("distance ", robot.getColor(-1,1));
            //telemetry.addData("alpha ", robot.getColor(0,1));
            //telemetry.addData("red ", robot.getColor(1,1));
            //telemetry.addData("green ", robot.getColor(2,1));
            //telemetry.addData("blue ", robot.getColor(3,1));

            //telemetry.addData("distance ", robot.getColor(-1,2));
            //telemetry.addData("alpha ", robot.getColor(0,2));
            //telemetry.addData("red ", robot.getColor(1,2));
            //telemetry.addData("green ", robot.getColor(2,2));
            //telemetry.addData("blue ", robot.getColor(3,2));

            telemetry.addData("time: ", System.currentTimeMillis() - robot.intakeClock);
            telemetry.addData("time: ", robot.intakeState);
            telemetry.update();
        }
    }
}