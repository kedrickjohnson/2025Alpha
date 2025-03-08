package frc.robot.subsystems;

//import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera driveCamera = new PhotonCamera("driveCamera");

    public VisionSubsystem() {

    }

   // public List<PhotonPipelineResult> getResults() {
  //      return driveCamera.getAllUnreadResults();
    //}
}
