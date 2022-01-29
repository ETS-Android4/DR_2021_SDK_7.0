package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="gen2AutoDuckBlueWithCam")
@Disabled

public class gen2AutoDuckBlueWithCam extends LinearOpMode
{
    BNO055IMU imu;

    public DcMotor intake1 = null;
    public DcMotor intake2 = null;
    public DcMotor duckSpinnerLeft = null;
    public  DcMotor duckSpinnerRight = null;
    public Servo baseRight = null;
    public Servo  armRight = null;
    public Servo  bucketRight = null;

    public DcMotorEx motorLF = null;
    public DcMotorEx motorLB = null;
    public DcMotorEx motorRF = null;
    public DcMotorEx motorRB = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        motorLF = hardwareMap.get(DcMotorEx.class, "motorLF");
        motorLB = hardwareMap.get(DcMotorEx.class, "motorLB");
        motorRF = hardwareMap.get(DcMotorEx.class, "motorRF");
        motorRB = hardwareMap.get(DcMotorEx.class, "motorRB");


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

        CopyDemoBotDriveMecanum drive = new CopyDemoBotDriveMecanum();
        DcMotorEx[] motors = new DcMotorEx[4];
        {
            motors[0] = motorLF;
            motors[1] = motorLB;
            motors[2] = motorRF;
            motors[3] = motorRB;

        }

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime whatever = new ElapsedTime();

        waitForStart();

        bucketRight.setPosition(.17);

        drive.timeDrive(500, 1, driveStyled.STRAFE_RIGHT, motors);

        drive.OrientationDrive(90, 1, motors, imu);

        drive.timeDrive(500, 1, driveStyled.STRAFE_LEFT, motors);

        duckSpinnerLeft.setPower(1);
        duckSpinnerRight.setPower(1);

        drive.timeDrive(500, 1, driveStyled.FORWARD_LEFT, motors);

        sleep(5000);

        duckSpinnerLeft.setPower(0);
        duckSpinnerRight.setPower(0);

        drive.encoderDrive(1500, driveStyled.BACKWARD, 1, motors);

        drive.timeDrive(500, 1, driveStyled.STRAFE_RIGHT, motors);

        drive.OrientationDrive(180, 1, motors, imu);

        armRight.setPosition(.1);

        drive.timeDrive(500, 1, driveStyled.FORWARD, motors);

        baseRight.setPosition(.6);

        drive.encoderDrive(2000, driveStyled.BACKWARD, 1, motors);

        armRight.setPosition(.55);

        sleep(1000);

        armRight.setPosition(.1);

        drive.timeDrive(500, 1, driveStyled.FORWARD, motors);

        baseRight.setPosition(.23);

        drive.OrientationDrive(150, 1, motors, imu);

        drive.timeDrive(3000, 1, driveStyled.FORWARD, motors);
    }


}
//robot.wobble.setTargetPosition(upPosition);
//                robot.wobble.setPower(0.7);
//                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
