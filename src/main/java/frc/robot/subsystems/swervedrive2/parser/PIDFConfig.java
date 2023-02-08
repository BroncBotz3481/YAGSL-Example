package frc.robot.subsystems.swervedrive2.parser;

public class PIDFConfig
{

  /**
   * PIDF Values and integral zone.
   */
  public double kP, kI, kD, kF, IZ;

  /**
   * PIDF Config constructor to contain the values.
   *
   * @param p  P gain.
   * @param i  I gain.
   * @param d  D gain.
   * @param f  F gain.
   * @param iz Intergral zone.
   */
  public PIDFConfig(double p, double i, double d, double f, double iz)
  {
    kP = p;
    kI = i;
    kD = d;
    kF = f;
    IZ = iz;
  }

  /**
   * PIDF Config constructor to contain the values.
   *
   * @param p P gain.
   * @param i I gain.
   * @param d D gain.
   * @param f F gain.
   */
  public PIDFConfig(double p, double i, double d, double f)
  {
    this(p, i, d, f, 0);
  }

  /**
   * PIDF Config constructor to contain the values.
   *
   * @param p P gain.
   * @param i I gain.
   * @param d D gain.
   */
  public PIDFConfig(double p, double i, double d)
  {
    this(p, i, d, 0, 0);
  }

  /**
   * PIDF Config constructor to contain the values.
   *
   * @param p P gain.
   * @param d D gain.
   */
  public PIDFConfig(double p, double d)
  {
    this(p, 0, d, 0, 0);
  }
}
