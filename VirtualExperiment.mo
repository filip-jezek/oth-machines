within ;
package VirtualExperiment
  extends Modelica.Icons.Package;

  package Examples
    extends Modelica.Icons.ExamplesPackage;

    model DCEE
      extends Modelica.Icons.Example;
      VirtualExperiment.machines.DCEE fmu
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(table=[0,0,0,0; 0,
            100,0,0; 0.5,100,0,0; 2.5,100,100,0; 4,100,100,0; 4,100,100,100; 5,
            100,100,100; 5,100,100,0; 6,80,100,0; 8,80,100,0; 8,80,100,80; 10,
            80,100,80])
        annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
    equation
      connect(combiTimeTable.y[1], fmu.vE)
        annotation (Line(points={{-39,0},{-12,0}}, color={0,0,127}));
      connect(combiTimeTable.y[2], fmu.vA) annotation (Line(points={{-39,0},{-20,
              0},{-20,6},{-12,6}}, color={0,0,127}));
      connect(combiTimeTable.y[3], fmu.tauL) annotation (Line(points={{-39,0},{
              -20,0},{-20,-20},{20,-20},{20,0},{12,0}}, color={0,0,127}));
      annotation (experiment(
          StopTime=10,
          Interval=0.001,
          __Dymola_fixedstepsize=0.001,
          __Dymola_Algorithm="Euler"), __Dymola_experimentFlags(Advanced(
            InlineMethod=2,
            InlineOrder=2,
            InlineFixedStep=0.001)));
    end DCEE;
  end Examples;

  package machines
    extends Modelica.Icons.Package;

    model DCEE "DC with electrical excitation"
      extends Modelica.Electrical.Machines.Icons.Drive;
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Voltage VaNominal=100 "Nominal armature voltage";
      parameter Modelica.SIunits.Voltage VeNominal=100 "Nominal excitation voltage";
      parameter Modelica.SIunits.Torque tauNominal=63.66 "Nominal torque";
      Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_ElectricalExcited dcee(
        VaNominal=dceeData.VaNominal,
        IaNominal=dceeData.IaNominal,
        wNominal=dceeData.wNominal,
        TaNominal=dceeData.TaNominal,
        Ra=dceeData.Ra,
        TaRef=dceeData.TaRef,
        La=dceeData.La,
        Jr=10*dceeData.Jr,
        useSupport=false,
        Js=dceeData.Js,
        frictionParameters=dceeData.frictionParameters,
        phiMechanical(fixed=true),
        wMechanical(fixed=true),
        coreParameters=dceeData.coreParameters,
        strayLoadParameters=dceeData.strayLoadParameters,
        brushParameters=dceeData.brushParameters,
        IeNominal=dceeData.IeNominal,
        Re=dceeData.Re,
        TeRef=dceeData.TeRef,
        Le=dceeData.Le,
        sigmae=dceeData.sigmae,
        TaOperational=293.15,
        alpha20a=dceeData.alpha20a,
        ia(fixed=true),
        alpha20e=dceeData.alpha20e,
        TeOperational=293.15,
        ie(fixed=true))
        annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage armatureVoltage annotation (
          Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-70,60})));
      Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
            transformation(
            origin={-70,30},
            extent={{-10,-10},{10,10}},
            rotation=0)));
      Modelica.Electrical.Analog.Sources.SignalVoltage excitationVoltage
        annotation (Placement(transformation(
            origin={-70,0},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Electrical.Analog.Basic.Ground groundExcitation annotation (
          Placement(transformation(
            origin={-70,-30},
            extent={{-10,-10},{10,10}},
            rotation=0)));
      parameter
        Modelica.Electrical.Machines.Utilities.ParameterRecords.DcElectricalExcitedData
        dceeData annotation (Placement(transformation(extent={{-30,-42},{-10,-22}})));
      Modelica.Electrical.Analog.Sensors.MultiSensor multiSensorE
        annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
      Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
        annotation (Placement(transformation(extent={{-60,0},{-40,20}})));
      Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensorM
        annotation (Placement(transformation(extent={{0,-10},{20,10}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque
        annotation (Placement(transformation(extent={{50,-10},{30,10}})));
      Modelica.Blocks.Math.Gain gainTorque(k=-tauNominal/100)
        annotation (Placement(transformation(extent={{90,-10},{70,10}})));
      Modelica.Blocks.Interfaces.RealInput vA
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Modelica.Blocks.Interfaces.RealInput vE
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Modelica.Blocks.Interfaces.RealInput tauL
        annotation (Placement(transformation(extent={{140,-20},{100,20}})));
      Modelica.Blocks.Interfaces.RealOutput iA
        annotation (Placement(transformation(extent={{100,70},{120,90}})));
      Modelica.Blocks.Interfaces.RealOutput iE
        annotation (Placement(transformation(extent={{100,-90},{120,-70}})));
      Modelica.Blocks.Math.UnitConversions.To_rpm
                                to_rpm
        annotation (Placement(transformation(extent={{70,-50},{90,-30}})));
      Modelica.Blocks.Interfaces.RealOutput speed
        annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
    equation
      connect(excitationVoltage.n, groundExcitation.p)
        annotation (Line(points={{-70,-10},{-70,-20}}, color={0,0,255}));
      connect(armatureVoltage.n, ground.p)
        annotation (Line(points={{-70,50},{-70,40}}, color={0,0,255}));
      connect(armatureVoltage.p, multiSensorE.pc)
        annotation (Line(points={{-70,70},{-40,70}}, color={0,0,255}));
      connect(multiSensorE.pc, multiSensorE.pv)
        annotation (Line(points={{-40,70},{-40,80},{-30,80}}, color={0,0,255}));
      connect(multiSensorE.nc, dcee.pin_ap)
        annotation (Line(points={{-20,70},{-14,70},{-14,10}},
                                                        color={0,0,255}));
      connect(dcee.pin_an, ground.p)
        annotation (Line(points={{-26,10},{-26,40},{-70,40}},
                                                            color={0,0,255}));
      connect(ground.p, multiSensorE.nv)
        annotation (Line(points={{-70,40},{-30,40},{-30,60}}, color={0,0,255}));
      connect(excitationVoltage.p, currentSensor.p)
        annotation (Line(points={{-70,10},{-60,10}}, color={0,0,255}));
      connect(currentSensor.n, dcee.pin_ep)
        annotation (Line(points={{-40,10},{-40,6},{-30,6}}, color={0,0,255}));
      connect(groundExcitation.p, dcee.pin_en) annotation (Line(points={{-70,-20},{-40,
              -20},{-40,-6},{-30,-6}}, color={0,0,255}));
      connect(dcee.flange, multiSensorM.flange_a)
        annotation (Line(points={{-10,0},{0,0}}, color={0,0,0}));
      connect(multiSensorM.flange_b, torque.flange)
        annotation (Line(points={{20,0},{30,0}}, color={0,0,0}));
      connect(gainTorque.y, torque.tau)
        annotation (Line(points={{69,0},{52,0}}, color={0,0,127}));
      connect(armatureVoltage.v, vA)
        annotation (Line(points={{-82,60},{-120,60}}, color={0,0,127}));
      connect(vE, excitationVoltage.v)
        annotation (Line(points={{-120,0},{-82,0}}, color={0,0,127}));
      connect(gainTorque.u, tauL)
        annotation (Line(points={{92,0},{120,0}}, color={0,0,127}));
      connect(multiSensorE.i, iA) annotation (Line(points={{-36,59},{-36,50},{80,50},
              {80,80},{110,80}}, color={0,0,127}));
      connect(currentSensor.i, iE)
        annotation (Line(points={{-50,-1},{-50,-80},{110,-80}}, color={0,0,127}));
      connect(to_rpm.y, speed)
        annotation (Line(points={{91,-40},{110,-40}}, color={0,0,127}));
      connect(multiSensorM.w, to_rpm.u)
        annotation (Line(points={{16,-11},{16,-40},{68,-40}}, color={0,0,127}));
      annotation (experiment(
          StopTime=86400,
          Interval=0.001,
          Tolerance=1e-06,
          __Dymola_fixedstepsize=0.001,
          __Dymola_Algorithm="Euler"),
        Icon(graphics={
            Text(
              extent={{-100,80},{-60,40}},
              lineColor={28,108,200},
              textString="vA"),
            Text(
              extent={{-100,20},{-60,-20}},
              lineColor={28,108,200},
              textString="vE"),
            Text(
              extent={{60,100},{100,60}},
              lineColor={28,108,200},
              textString="iA"),
            Text(
              extent={{60,-60},{100,-100}},
              lineColor={28,108,200},
              textString="iE")}));
    end DCEE;
    annotation (Icon(graphics={Polygon(
            points={{-80,80},{-80,2},{-18,2},{-18,-24},{-24,-24},{0,-40},{24,
                -24},{18,-24},{18,2},{80,2},{80,80},{-80,80}},
            lineColor={28,108,200},
            fillColor={0,0,255},
            fillPattern=FillPattern.Solid), Polygon(
            points={{-80,-2},{-80,-80},{80,-80},{80,-2},{20,-2},{20,-22},{30,
                -22},{0,-42},{-30,-22},{-20,-22},{-20,-2},{-80,-2}},
            lineColor={238,46,47},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid)}));
  end machines;

  model TestRig
    machines.DCEE dCEE
      annotation (Placement(transformation(extent={{-10,-6},{10,14}})));
    Modelica.Blocks.Sources.Constant vA_const(k=100)
      annotation (Placement(transformation(extent={{-60,0},{-40,20}})));
    Modelica.Blocks.Sources.Constant vE_const(k=100)
      annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
    Modelica.Blocks.Sources.Constant tauL_const(k=100)
      annotation (Placement(transformation(extent={{40,-6},{20,14}})));
  equation
    connect(dCEE.tauL, tauL_const.y)
      annotation (Line(points={{12,4},{19,4}}, color={0,0,127}));
    connect(dCEE.vA, vA_const.y)
      annotation (Line(points={{-12,10},{-39,10}}, color={0,0,127}));
    connect(vE_const.y, dCEE.vE) annotation (Line(points={{-39,-30},{-18,-30},{
            -18,4},{-12,4}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Bitmap(
            extent={{-100,-100},{100,100}},
            imageSource=
                "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",

            fileName="modelica://VirtualExperiment/bodylight.png")}), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end TestRig;
  annotation (uses(Modelica(version="3.2.3"), Modelica_DeviceDrivers(version="1.7.0"),
      Complex(version="3.2.3")));
end VirtualExperiment;
