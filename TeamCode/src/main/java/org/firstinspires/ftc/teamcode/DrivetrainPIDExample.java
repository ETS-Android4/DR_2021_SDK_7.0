package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoPID.PIDController;


@Autonomous(name="DrivetrainPIDExample")
public class DrivetrainPIDExample extends LinearOpMode {
    public DcMotor motorRF;
    public DcMotor motorRB;
    public DcMotor motorLB;
    public DcMotor motorLF;

    private BNO055IMU imu;

    // target in encoder ticks for each motor
    double targetLeft = 1000;
    double targetRight = 1000;
    double referenceAngle = Math.toRadians(270);


    // initialize error
    double leftError = 1000;
    double rightError = 1000;

    // proportional gain
    double Kp = 0.1;


    double lastError = 0;
    double integral = 0;
    boolean angleWrap = false;double Ki;
    double Kd;


    // pid controllers set with arbitrary values, use at your own risk
    PIDController leftController = new PIDController(0.5,0.1,0.2);
    PIDController rightController = new PIDController(0.5,0.1,0.2);

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        // configure your motors and other hardware stuff here
        // make sure the strings match the names that you have set on your robot controller configuration
        motorRF = hardwareMap.get(DcMotor.class,"motorRF");
        motorRB = hardwareMap.get(DcMotor.class,"motorRB");
        motorLB = hardwareMap.get(DcMotor.class,"motorLB");
        motorLF = hardwareMap.get(DcMotor.class,"motorLF");


        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reverse the appropriate motor

        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // wait until the start button is pressed
        waitForStart();
        /*
         * use proportional feedback to move drive train to reference
         */
        while (opModeIsActive()) {

            motorRF.setPower(-rightController.output(targetRight,motorRF.getCurrentPosition()));
            motorRB.setPower(-rightController.output(targetRight,motorRB.getCurrentPosition()));
            motorLB.setPower(leftController.output(targetLeft,motorLB.getCurrentPosition()));
            motorLF.setPower(leftController.output(targetLeft,motorLF.getCurrentPosition()));

            sleep(1000);
            sleep(1000);
            sleep(1000);
            sleep(1000);

            double power = PIDControl(referenceAngle, imu.getAngularOrientation().firstAngle);
            motorRF.setPower(power);
            motorRB.setPower(power);
            motorLB.setPower(power);
            motorLF.setPower(power);

            sleep(1000);
            sleep(1000);
            sleep(1000);
            sleep(1000);
        }
    }
    public double PIDControl(double reference, double state) {
        double error;
        double derivative;
        // check if we need to unwrap angle
        if (angleWrap) {
            error = angleWrap(reference - state);
        } else {
            error = reference - state;
        }
        // forward euler integration
        integral += error * timer.seconds();
        derivative = (error - lastError) / timer.seconds();

        double output = (error * Kp) + (integral * Ki) + (derivative * Kd);

        timer.reset();

        return output;
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

}
