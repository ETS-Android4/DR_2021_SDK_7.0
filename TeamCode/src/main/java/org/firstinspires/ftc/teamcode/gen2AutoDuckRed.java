package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="gen2AutoDuckRed")


public class gen2AutoDuckRed extends LinearOpMode
{
    BNO055IMU imu;

    public DcMotor intake1 = null;
    public DcMotor intake2 = null;
    public DcMotor duckSpinnerLeft = null;
    public  DcMotor duckSpinnerRight = null;
    public Servo baseRight = null;
    public Servo  armRight = null;
    public Servo  bucketRight = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);

        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        duckSpinnerLeft = hardwareMap.dcMotor.get("duckSpinnerLeft");
        duckSpinnerRight = hardwareMap.dcMotor.get("duckSpinnerRight");
        baseRight = hardwareMap.servo.get("baseRight");
        armRight = hardwareMap.servo.get("armRight");
        bucketRight = hardwareMap.servo.get("bucketRight");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        DemoBotDriveMecanum drive = new DemoBotDriveMecanum();
        DcMotor[] motors = new DcMotor[4];
        {
            motors[0] = robot.motorLF;
            motors[1] = robot.motorLB;
            motors[2] = robot.motorRF;
            motors[3] = robot.motorRB;

        }

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime whatever = new ElapsedTime();

        waitForStart();

        bucketRight.setPosition(.17);

        drive.encoderDrive(1000, driveStyle.STRAFE_RIGHT, 1, motors);

        drive.OrientationDrive(90, 1, motors, imu);

        drive.encoderDrive(1000, driveStyle.STRAFE_RIGHT, 1, motors);

        drive.encoderDrive(1000, driveStyle.FORWARD, 1, motors);

        duckSpinnerLeft.setPower(1);
        duckSpinnerRight.setPower(1);

        sleep(5000);

        duckSpinnerLeft.setPower(0);
        duckSpinnerRight.setPower(0);

        drive.encoderDrive(1000, driveStyle.BACKWARD, 1, motors);

        armRight.setPosition(.1);

        drive.encoderDrive(1000, driveStyle.STRAFE_LEFT, 1, motors);

        drive.OrientationDrive(-90, 1, motors, imu);

        drive.encoderDrive(1000, driveStyle.STRAFE_RIGHT, 1, motors);

        //raise arm and place
        baseRight.setPosition(.6);

        sleep(1000);

        armRight.setPosition(.55);

        sleep(1000);

        //bring arm down
        armRight.setPosition(.1);

        sleep(1000);

        baseRight.setPosition(.23);

        sleep(1000);

        armRight.setPosition(0);

        drive.encoderDrive(1000, driveStyle.STRAFE_LEFT, 1, motors);

        drive.encoderDrive(1000, driveStyle.BACKWARD, 1, motors);






        //drive.timeDrive(100,1,driveStyle.FORWARD,motors);

        //sleep(10000);


    }


}
//robot.wobble.setTargetPosition(upPosition);
//                robot.wobble.setPower(0.7);
//                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
