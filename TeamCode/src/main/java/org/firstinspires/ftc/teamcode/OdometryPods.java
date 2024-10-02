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
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    // Calculates the circumference for the Odometry pods
    // Divides by 25.4 to change mm to inches
    double OPcircumference = 2*Math.PI*(16/25.4);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // 3
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);  // 2
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);  // 1
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);  // 0

        // Declares what motors have encoders/ Odometry pods

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // x
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // x
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // y

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double power = -0.2;

        int target_dist_in = 12*10;
        double target_dist_pulse = InchesToPulse(target_dist_in);

        rightFrontDrive.setTargetPosition((int)target_dist_pulse);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

        while (rightFrontDrive.isBusy()){
            // wait until target pos is reached
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        telemetry.addData("Target Pulses: ", (int)target_dist_pulse);
        telemetry.addData("RFD Pulses: ", rightFrontDrive.getCurrentPosition());
        telemetry.update();

        int secs = 1000*20;
        sleep(secs);

    }

    public double PulseToInches(int pulses){
        double rev = (double)pulses/2000;
        double inches = OPcircumference * rev;
        return inches;
    }
    public double InchesToPulse(int inches){
        double rev = inches/OPcircumference;
        double pulses = 2000*rev;
        return pulses;
    }
}
