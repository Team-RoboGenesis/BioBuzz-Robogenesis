package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "IMU test")
public class IMUTest extends LinearOpMode {
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Get the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        // 2. Define how the REV Control/Expansion Hub is mounted on your robot
        // Adjust LogoDirection and USBDirection to match your physical build
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logo, usb);

        // 3. Initialize the IMU with these parameters
        imu.initialize(new IMU.Parameters(orientation));

        waitForStart();

        while (opModeIsActive()) {
            // 4. Read the robot's orientation angles
            YawPitchRollAngles robotAngles = imu.getRobotYawPitchRollAngles();

            // 5. Extract the yaw (heading rotation around the Z-axis) in degrees
            double yaw = robotAngles.getYaw(AngleUnit.DEGREES);
            double pitch = robotAngles.getPitch(AngleUnit.DEGREES);
            double roll = robotAngles.getRoll(AngleUnit.DEGREES);

            // Display on Driver Station telemetry
            telemetry.addData("Yaw (Heading)", "%.2f degrees", yaw);
            telemetry.addData("Pitch", "%.2f degrees", pitch);
            telemetry.addData("Roll", "%.2f degrees", roll);
            telemetry.update();
        }
    }

}
