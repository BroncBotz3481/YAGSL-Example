// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swervelib.math.codeorange;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

@SuppressWarnings({"ParameterName", "InterfaceTypeParameterName"})
public interface KalmanTypeFilter<States extends Num, Inputs extends Num, Outputs extends Num>
{

  Matrix<States, States> getP();

  void setP(Matrix<States, States> newP);

  double getP(int i, int j);

  Matrix<States, N1> getXhat();

  void setXhat(Matrix<States, N1> xHat);

  double getXhat(int i);

  void setXhat(int i, double value);

  void reset();

  void predict(Matrix<Inputs, N1> u, double dtSeconds);

  void correct(Matrix<Inputs, N1> u, Matrix<Outputs, N1> y);
}
