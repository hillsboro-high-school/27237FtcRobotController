package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Odometry Pods", group="Linear OpMode")

public class OdometryPods extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;

    private DcMotor leftEncoderMotor = null;
    private double leftEncoderPos = 0;
    private double deltaLeftEncoder = 0;

    private DcMotor rightEncoderMotor = null;
    private double rightEncoderPos = 0;
    private double deltaRightEncoder = 0;

    private DcMotor centerEncoderMotor = null;
    private double centerEncoderPos = 0;
    private double deltaCenterEncoder = 0;

    private double theta = 0;

    private boolean rightStop = false;
    private boolean leftStop = false;

    private double tileMatLength = 12*2;  // Inches

    // Calculates the circumference for the Odometry pods
    // Divides by 25.4 to change mm to inches
    double OPcircumference = 2.0*Math.PI*(16.0/25.4);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FL = hardwareMap.get(DcMotor.class, "left_front_drive");
        BL = hardwareMap.get(DcMotor.class, "left_back_drive");
        FR = hardwareMap.get(DcMotor.class, "right_front_drive");
        BR = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftEncoderMotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightEncoderMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        centerEncoderMotor = hardwareMap.get(DcMotor.class, "left_back_drive");

        leftEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);  // Directions taken from BlackBoxBot.java
        rightEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        centerEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setNormalDrive();  // Sets all motors to correct forward/reverse

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetTicks();

        strafeRight(InchesToTicks(tileMatLength), -0.2);
        sleep(1000*10);
        driveForward(InchesToTicks(tileMatLength/2.0));
        sleep(1000*10);
        driveBackward(InchesToTicks(tileMatLength/2.0));
        sleep(1000*10);
        strafeRight(InchesToTicks(tileMatLength*2.0), -0.2);
        sleep(1000*10);
        driveBackward(InchesToTicks(tileMatLength*1.2));

        telemetry.addData("Left pos", getLeftTicks());
        telemetry.addData("Right pos", getRightTicks());
        telemetry.addData("Center pos", getCenterTicks());
        telemetry.update();

        sleep(10000);
    }

    public void driveForward(double targetTicks) {
        resetTicks();
        setAllPower(-0.5);
        while (!(rightStop && leftStop)) {
            if (getRightTicks() >= targetTicks) {
                rightStop = true;
                stopRightPower();
            }
            if (getLeftTicks() >= targetTicks) {
                leftStop = true;
                stopLeftPower();
            }
            telemetry.addData("Forward!", rightStop);
            telemetry.addData("Target pos", targetTicks);
            telemetry.addData("Left pos", getLeftTicks());
            telemetry.addData("Right pos", getRightTicks());
            telemetry.update();
        }
        rightStop = false;
        leftStop = false;

        stopAllPower();
        resetTicks();
    }

    public void driveBackward(double targetTicks) {
        resetTicks();
        setAllPower(0.5);
        while (!(rightStop && leftStop)) {
            if (Math.abs(getRightTicks()) >= targetTicks) {
                rightStop = true;
                stopRightPower();
            }
            if (Math.abs(getLeftTicks()) >= targetTicks) {
                leftStop = true;
                stopLeftPower();
            }
            telemetry.addData("Backward!", rightStop);
            telemetry.addData("Target pos", targetTicks);
            telemetry.addData("Left pos", Math.abs(getLeftTicks()));
            telemetry.addData("Right pos", Math.abs(getRightTicks()));
            telemetry.update();
        }
        rightStop = false;
        leftStop = false;
        stopAllPower();
        resetTicks();
    }

    public void strafeRight(double targetTicks, double power) {
        resetTicks();
        setStrafingDrive();
        setAllPower(power);

        telemetry.addData("StrafeRight!", rightStop);
        telemetry.addData("Center pos", getCenterTicks());
        telemetry.addData("Target pos", targetTicks);
        telemetry.update();

        while (getCenterTicks() < targetTicks){
            telemetry.addData("StrafeRight!", rightStop);
            telemetry.addData("Center pos", getCenterTicks());
            telemetry.addData("Target pos", targetTicks);
            telemetry.update();
        }
        stopAllPower();
        resetTicks();
        setNormalDrive();
    }

    public void setNormalDrive(){
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setStrafingDrive(){
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void resetTicks(){
        resetLeftTicks();
        resetRightTicks();
        resetCenterTicks();
    }

    public void setAllPower(double p){
        FL.setPower(p);
        FR.setPower(p);
        BL.setPower(p);
        BR.setPower(p);
    }

    public void stopAllPower(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void setLeftPower(double p){
        FL.setPower(p);
        BL.setPower(p);
    }

    public void setRightPower(double p){
        FR.setPower(p);
        BR.setPower(p);
    }

    public void stopLeftPower(){
        FL.setPower(0);
        BL.setPower(0);
    }

    public void stopRightPower(){
        FR.setPower(0);
        BR.setPower(0);
    }

    public void resetLeftTicks(){
        leftEncoderPos = leftEncoderMotor.getCurrentPosition();
    }

    public double getLeftTicks(){
        return (-(leftEncoderMotor.getCurrentPosition() - leftEncoderPos));
    }

    public void resetRightTicks(){
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }

    public double getRightTicks(){
        return (rightEncoderMotor.getCurrentPosition() - rightEncoderPos);
    }

    public void resetCenterTicks(){
        centerEncoderPos = centerEncoderMotor.getCurrentPosition();
        telemetry.addData("Center Pos", centerEncoderPos);
        telemetry.update();
        sleep(1000*5);
    }

    public double getCenterTicks(){
        telemetry.addData("Center Pos", (centerEncoderMotor.getCurrentPosition() - centerEncoderPos));
        telemetry.update();
        sleep(1000*5);
        return (centerEncoderMotor.getCurrentPosition() - centerEncoderPos);
    }

    public double TicksToInches(double ticks){
        double rev = (double)ticks/2000;
        double inches = OPcircumference * rev;
        return inches;
    }
    public double InchesToTicks(double inches){
        double rev = inches/OPcircumference;
        double tick = 2000*rev;
        return tick;
    }
}
