package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.pinkCode.ContourPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="driveClassTest")
//@Disabled

public class driveClassTest extends LinearOpMode
{
    BNO055IMU imu;
    
    int i = 0;

    public DcMotorEx motorLF = null;
    public DcMotorEx motorLB = null;
    public DcMotorEx motorRF = null;
    public DcMotorEx motorRB = null;
    
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(hardwareMap);

        motorLF = hardwareMap.get(DcMotorEx.class, "motorLF");
        motorLB = hardwareMap.get(DcMotorEx.class, "motorLB");
        motorRF = hardwareMap.get(DcMotorEx.class, "motorRF");
        motorRB = hardwareMap.get(DcMotorEx.class, "motorRB");
        
        driveClass drive = new driveClass();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        DcMotorEx[] motors = new DcMotorEx[4];
        {
            motors[0] = motorLF;
            motors[1] = motorLB;
            motors[2] = motorRF;
            motors[3] = motorRB;

        }

        waitForStart();
        
        drive.drive(0, 1, 0, 0, motors, imu);
            
        sleep(1000);
        
        while(i < 1000)
        i++;
        drive.drive(90, 1, 270, .5, motors, imu);
    }
}
