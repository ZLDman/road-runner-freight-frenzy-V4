package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class Robot {

    public DcMotor extend;

    //non linear slide side
    public DcMotor intake1;
    //linear slide side
    public DcMotor intake2;

    public DcMotor car;

    public Servo intake;
    public Servo lift;
    public Servo bucket;

    public Servo encoderservo;

    public Servo tapeliftservo;
    public Servo taperotateservo;
    public CRServo tapeextendservo;

    public double tapelift = 0.9;
    public double taperotate = 1;
    public double liftPosition = 0.4;


    public static double carMaxSpeed = 0.75;

    public DigitalChannel extendStop;

    public static double bucketLevelMultiplier = 0.6;

    int level = 3;

    public ColorSensor sensorColor1;
    public DistanceSensor sensorDistance1;

    public ColorSensor sensorColor2;
    public DistanceSensor sensorDistance2;

    enum IntakeState {
        MANUAL,     //manual control
        INTAKE,     //spin intake
        LIFT,       //lift for transfer
        RESET       //reset
    }
    enum ExtendState {
        INIT,       //initialize extend
        MANUAL,     //manual control
        RESET,      //spin intake
        EXTEND,     //lift for transfer
        DUMP,       //reset
        WAIT        //wait for dump
    }

    public IntakeState intakeState = IntakeState.MANUAL;
    public ExtendState extendState = ExtendState.INIT;

    public long intakeClock = System.currentTimeMillis();
    public long extendClock = System.currentTimeMillis();


    public Robot(HardwareMap hardwareMap) {
        extend = hardwareMap.get(DcMotor.class, "extend");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        car = hardwareMap.get(DcMotor.class, "car");

        lift = hardwareMap.get(Servo.class, "lift");
        bucket = hardwareMap.get(Servo.class, "bucket");
        intake = hardwareMap.get(Servo.class, "intake");

        encoderservo = hardwareMap.get(Servo.class, "encoderservo");

        tapeextendservo = hardwareMap.get(CRServo.class, "tapeextendservo");
        tapeliftservo = hardwareMap.get(Servo.class, "tapeliftservo");
        taperotateservo = hardwareMap.get(Servo.class, "taperotateservo");

        extendStop = hardwareMap.get(DigitalChannel.class,"extendStop");

        extend.setDirection(DcMotorSimple.Direction.REVERSE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to the color sensor.
        sensorColor1 = hardwareMap.get(ColorSensor .class, "color1");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance1 = hardwareMap.get(DistanceSensor .class, "color1");

        // get a reference to the color sensor.
        sensorColor2 = hardwareMap.get(ColorSensor .class, "color2");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance2 = hardwareMap.get(DistanceSensor .class, "color2");

    }

    public void updateIntake(){
        switch (intakeState){
            case INTAKE:{
                setIntake1Speed(1);

                //if we sense something
                if(getColor(-1,1) < 1){
                    //raise intake scoop
                    intakeState = IntakeState.LIFT;
                    intakeClock = System.currentTimeMillis();
                }
                break;
            }
            case LIFT:{
                if(System.currentTimeMillis() - intakeClock > 1000) {
                    //stop intake
                    setIntake1Speed(0);

                    //if intake has stopped
                    if (intake1.getPower() == 0) {

                        //raise intake scoop
                        intake.setPosition(0.5);

                        //reset time so we know when to go down
                        intakeClock = System.currentTimeMillis();
                        intakeState = IntakeState.RESET;
                    }
                }
                else{
                    setIntake1Speed(0);
                }
                break;
            }
            case RESET:{
                setIntake1Speed(0);

                //if time has passed lower intake scoop
                if(System.currentTimeMillis() - intakeClock > 1000){
                    intake.setPosition(0.10);
                }

                //note the extend code will switch back to intake when freight has dumped so we do not possess more than 1 freight
                break;
            }
        }
    }

    public void updateExtend() throws InterruptedException {

        if(extendState != ExtendState.DUMP && extendState != ExtendState.WAIT) {
            bucket.setPosition(liftPosition * bucketLevelMultiplier);
        }

        switch (extendState){
            //manual
            case MANUAL:{
                extend.setPower(0);
            }
            //init
            case INIT:{

                extendState = ExtendState.RESET;

            }
            //reset
            case RESET:{

                if(extend.getCurrentPosition() > 500){
                    extend.setTargetPosition(0);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(0.75);
                    extendClock = System.currentTimeMillis();
                }

                if(System.currentTimeMillis() - extendClock > 150) {
                    bucket.setPosition();
                    setLiftPosition(0);
                }

                if(!extend.isBusy()){
                    extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extend.setPower(0);
                }

                break;
            }
            //extend + lift bucket
            case EXTEND:{
                if (level == 1)
                {
                    extend.setTargetPosition(2200);
                    setLiftPosition(0.22);
                }
                else if (level == 2)
                {
                    extend.setTargetPosition(2100);
                    setLiftPosition(0.47);
                }
                else
                {
                    extend.setTargetPosition(2800);
                    setLiftPosition(0.73);
                }

                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extend.setPower(0.75);

                extendClock = System.currentTimeMillis();
                extendState = ExtendState.DUMP;

                break;
            }
            //dump
            case DUMP:{
                bucket.setPosition(liftPosition * bucketLevelMultiplier);

                if(System.currentTimeMillis() - extendClock > 500 && !extend.isBusy()) {
                    if (level == 1)
                    {
                        bucket.setPosition(0.6);
                    }
                    else if (level == 2)
                    {
                        bucket.setPosition(0.75);
                    }
                    else
                    {
                        bucket.setPosition(0.93);
                    }

                    extendClock = System.currentTimeMillis();
                    extendState = ExtendState.WAIT;
                }

                break;
            }
            case WAIT:{
                //wait 1/2 second to give the bucket time to dump then reset
                if(System.currentTimeMillis() - extendClock > 750) {
                    extendClock = System.currentTimeMillis();
                    extendState = ExtendState.RESET;
                }
            }
        }
    }

    public void setLevel(int level) {
        this.level = level;
    }

    public void setIntake1Speed(double i) {
        int intakePosition = (int) (intake1.getCurrentPosition() % 45);

        if(i == 0 && (intakePosition > 5 && intakePosition < 40)){
            if(intakePosition > 30){
                intake1.setPower(0.2);
            }
            else if(intakePosition < 15){
                intake1.setPower(-0.2);
            }
            else {
                intake1.setPower(0.3);
            }
        }
        else {
            intake1.setPower(i);
        }
    }

    public void setIntake2Speed(double i) {
        int intakePosition = (int) (intake2.getCurrentPosition() % 45);

        if(i == 0 && (intakePosition > 5 && intakePosition < 40)){
            if(intakePosition > 30){
                intake2.setPower(0.35);
            }
            else if(intakePosition < 15){
                intake2.setPower(-0.35);
            }
            else {
                intake2.setPower(0.5);
            }
        }
        else {
            intake2.setPower(i);
        }
    }

    public void setLiftPosition(double p){
        liftPosition = p;
    }

    public void updateLiftServo(){
        if(Math.abs(lift.getPosition() - liftPosition) > 0.1) {
            if (lift.getPosition() > liftPosition) {
                lift.setPosition(lift.getPosition() - 0.1);
            } else if (lift.getPosition() < liftPosition) {
                lift.setPosition(lift.getPosition() + 0.1);
            }
        }
        else{
            lift.setPosition(liftPosition);
        }
    }

    //void setExtendSpeed(double s){
        //if(!extend.isBusy()) {
        //    if (s < 0 && extendStop.getState()) s = 0;
        //    extend.setPower(s);
        //}
    //}

    void setExtendPos(int p){
        extend.setTargetPosition(p);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setPower(1);
        while(extend.isBusy()){

        }
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @param s speed min: 0 Max: 1
     */
    void setCarSpeed(double s){
        car.setPower(s * carMaxSpeed);
    }


    // get the color sensor values
    double getColor(int channel,int s) {
        if(s == 1) {
            if (channel == -1) return sensorDistance1.getDistance(DistanceUnit.INCH);
            if (channel == 1) return sensorColor1.red();
            if (channel == 2) return sensorColor1.green();
            if (channel == 3) return sensorColor1.blue();
            return sensorColor1.alpha();
        }

        if (channel == -1) return sensorDistance2.getDistance(DistanceUnit.INCH);
        if (channel == 1) return sensorColor2.red();
        if (channel == 2) return sensorColor2.green();
        if (channel == 3) return sensorColor2.blue();
        return sensorColor2.alpha();
    }
}
