package frc.robot.subsystems.vision;

public class VisionIOLimelight implements VisionIO {

  private final String m_limelightName;

  public VisionIOLimelight(String limelightName) {
    this.m_limelightName = limelightName;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.seesTarget = LimelightHelpers.getTV(m_limelightName);
    inputs.tX = LimelightHelpers.getTX(m_limelightName);
    inputs.tY = LimelightHelpers.getTY(m_limelightName);
    inputs.aprilTagId = LimelightHelpers.getFiducialID(m_limelightName);
  }
}
