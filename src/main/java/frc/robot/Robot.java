// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Jack-in-the-box prop.
 *
 * Sequence when the PIR sees motion:
 *   IDLE -> CRANKING (crank spins, lid/jack still down)
 *        -> POPPED   (crank stops, piston extends, jack pops out)
 *        -> RESETTING (piston retracts, short settle time)
 *        -> IDLE (cooldown before the next trigger)
 */
public class Robot extends TimedRobot {

  // ---- Hardware IDs ----
  private static final int CRANK_CAN_ID = 1;        // SPARK Flex CAN ID (change to match your motor)
  private static final int PCM_CAN_ID = 2;          // CTRE PCM CAN ID (default is 0)
  private static final int PISTON_FORWARD_CHANNEL = 0; // PCM channel for one side of the piston
  private static final int PISTON_REVERSE_CHANNEL = 1; // PCM channel for the other side
  private static final int PIR_DIO_PORT = 0;
  private static final int LED_PWM_PORT = 0;       // roboRIO PWM port the strip's DI wire is plugged into
  private static final int LED_LENGTH = 60;        // number of LEDs on the strip (CHANGE to match yours)
  private static final int CRANK_CURRENT_LIMIT_AMPS = 40;

  // Default timings (seconds). These can be changed live from Elastic.
  private static final double DEFAULT_CRANK_MIN_TIME_SEC = 1.0; // shortest possible crank time before the pop
  private static final double DEFAULT_CRANK_MAX_TIME_SEC = 5.0; // longest possible crank time before the pop
  private static final double DEFAULT_CRANK_RPM = 60.0;         // crank speed in RPM (at the crank, after any gearing)
  private static final double CRANK_GEAR_RATIO = 1.0;           // motor rotations per crank rotation (e.g. 5.0 for a 5:1 reduction)

  // Velocity PID defaults. Units: output (-1..1) per motor RPM of error.
  private static final double VORTEX_FREE_SPEED_RPM = 6784.0;
  private static final double DEFAULT_KP = 0.0002;
  private static final double DEFAULT_KI = 0.0;
  private static final double DEFAULT_KD = 0.0;
  private static final double DEFAULT_KFF = 1.0 / VORTEX_FREE_SPEED_RPM; // feedforward does most of the work
  private static final double DEFAULT_POPPED_TIME_SEC = 3.0;    // how long the jack stays out
  private static final double DEFAULT_RESET_DELAY_SEC = 0.5;    // settle time after retracting
  private static final double DEFAULT_COOLDOWN_SEC = 2.5;

  private static final boolean DETECTED_WHEN_TRUE = true;

  // Flip these two if the jack moves the wrong way
  private static final Value POPPED_OUT = Value.kForward;
  private static final Value RETRACTED = Value.kReverse;

  private enum State { IDLE, CRANKING, POPPED, RESETTING }

  private final DigitalInput irsensor = new DigitalInput(PIR_DIO_PORT);

  // One double-acting piston on the CTRE Pneumatics Control Module
  private final DoubleSolenoid m_jack = new DoubleSolenoid(
      PCM_CAN_ID, PneumaticsModuleType.CTREPCM, PISTON_FORWARD_CHANNEL, PISTON_REVERSE_CHANNEL);

  // Compressor on the PCM (digital pressure switch mode)
  private final Compressor m_compressor = new Compressor(PCM_CAN_ID, PneumaticsModuleType.CTREPCM);

  // NEO Vortex on a SPARK Flex, spins the crank on the side
  private final SparkFlex m_crank = new SparkFlex(CRANK_CAN_ID, MotorType.kBrushless);
  private final SparkClosedLoopController m_crankPid = m_crank.getClosedLoopController();

  // Last PID gains sent to the SPARK, so we only reconfigure when Elastic values change
  private double m_lastP = Double.NaN, m_lastI, m_lastD, m_lastFF;

  // 12V addressable LED strip (DI / BI / GND / +12V) on a roboRIO PWM port
  private final AddressableLED m_led = new AddressableLED(LED_PWM_PORT);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LED_LENGTH);

  CommandXboxController controller = new CommandXboxController(0);

  // Elastic choosers
  private final SendableChooser<Boolean> m_ledMode = new SendableChooser<>();
  private final SendableChooser<LEDPattern> m_manualPattern = new SendableChooser<>();
  private final SendableChooser<LEDPattern> m_idlePattern = new SendableChooser<>();
  private final SendableChooser<LEDPattern> m_crankingPattern = new SendableChooser<>();
  private final SendableChooser<LEDPattern> m_poppedPattern = new SendableChooser<>();
  private final SendableChooser<LEDPattern> m_resettingPattern = new SendableChooser<>();

  private State state = State.IDLE;
  private double stateStartTime = 0;
  private double m_currentCrankTime = DEFAULT_CRANK_MIN_TIME_SEC; // picked at random each trigger
  private double lastSequenceEndTime = -1000; // far in the past so the first trigger works

  public Robot() {
    // Compressor: runs off the PCM's pressure switch input
    m_compressor.enableDigital();

    // Crank motor configuration
    SparkFlexConfig crankConfig = new SparkFlexConfig();
    crankConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CRANK_CURRENT_LIMIT_AMPS)
        .inverted(false); // set true if the crank spins the wrong direction
    crankConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // Vortex built-in sensor
        .pid(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD)
        .velocityFF(DEFAULT_KFF)
        .outputRange(-1.0, 1.0);
    m_crank.configure(crankConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Start the LED strip
    m_led.setLength(LED_LENGTH);
    m_led.setData(m_ledBuffer);
    m_led.start();

    // Mode: true = follow the prop state, false = manual override
    m_ledMode.setDefaultOption("Auto (follow state)", true);
    m_ledMode.addOption("Manual", false);
    SmartDashboard.putData("LED Mode", m_ledMode);

    // Pattern pickers (second argument is the pre-selected pattern)
    setupPatternChooser(m_manualPattern, "Solid Violet");
    setupPatternChooser(m_idlePattern, "Breath Blue");
    setupPatternChooser(m_crankingPattern, "Solid Yellow");
    setupPatternChooser(m_poppedPattern, "Rainbow");
    setupPatternChooser(m_resettingPattern, "Solid Orange");

    SmartDashboard.putData("LED Manual Pattern", m_manualPattern);
    SmartDashboard.putData("LED Idle", m_idlePattern);
    SmartDashboard.putData("LED Cranking", m_crankingPattern);
    SmartDashboard.putData("LED Popped", m_poppedPattern);
    SmartDashboard.putData("LED Resetting", m_resettingPattern);

    // Timing / speed values you can edit in Elastic
    SmartDashboard.setDefaultBoolean("PIR Enabled", false); // set to true once the PIR is wired up
    SmartDashboard.setDefaultNumber("Crank Min Time (s)", DEFAULT_CRANK_MIN_TIME_SEC);
    SmartDashboard.setDefaultNumber("Crank Max Time (s)", DEFAULT_CRANK_MAX_TIME_SEC);
    SmartDashboard.setDefaultNumber("Crank Speed (RPM)", DEFAULT_CRANK_RPM);
    SmartDashboard.setDefaultNumber("Crank kP", DEFAULT_KP);
    SmartDashboard.setDefaultNumber("Crank kI", DEFAULT_KI);
    SmartDashboard.setDefaultNumber("Crank kD", DEFAULT_KD);
    SmartDashboard.setDefaultNumber("Crank kFF", DEFAULT_KFF);
    SmartDashboard.setDefaultNumber("Popped Time (s)", DEFAULT_POPPED_TIME_SEC);
    SmartDashboard.setDefaultNumber("Reset Delay (s)", DEFAULT_RESET_DELAY_SEC);
    SmartDashboard.setDefaultNumber("Trigger Cooldown (s)", DEFAULT_COOLDOWN_SEC);
  }

  /** Fills a chooser with LED patterns. defaultName is the pre-selected one. */
  private void setupPatternChooser(SendableChooser<LEDPattern> chooser, String defaultName) {
    Object[][] patterns = {
      // Solid colors
      {"Solid Hot Pink", LEDPattern.solid(Color.kHotPink)},
      {"Solid Dark Red", LEDPattern.solid(Color.kDarkRed)},
      {"Solid Red", LEDPattern.solid(Color.kRed)},
      {"Solid Orange", LEDPattern.solid(Color.kOrange)},
      {"Solid Gold", LEDPattern.solid(Color.kGold)},
      {"Solid Yellow", LEDPattern.solid(Color.kYellow)},
      {"Solid Lime", LEDPattern.solid(Color.kLime)},
      {"Solid Green", LEDPattern.solid(Color.kGreen)},
      {"Solid Aqua", LEDPattern.solid(Color.kAqua)},
      {"Solid Blue", LEDPattern.solid(Color.kBlue)},
      {"Solid Violet", LEDPattern.solid(Color.kViolet)},
      {"Solid White", LEDPattern.solid(Color.kWhite)},
      {"Off (Black)", LEDPattern.solid(Color.kBlack)},

      // Breathing
      {"Breath Red", LEDPattern.solid(Color.kRed).breathe(Seconds.of(2))},
      {"Breath Orange", LEDPattern.solid(Color.kOrange).breathe(Seconds.of(2))},
      {"Breath Blue", LEDPattern.solid(Color.kBlue).breathe(Seconds.of(2))},
      {"Breath Violet", LEDPattern.solid(Color.kViolet).breathe(Seconds.of(2))},
      {"Breath White", LEDPattern.solid(Color.kWhite).breathe(Seconds.of(2))},

      // Strobes
      {"Strobe Red", LEDPattern.solid(Color.kRed).blink(Seconds.of(0.1))},
      {"Strobe Orange", LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.1))},
      {"Strobe Blue", LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.1))},
      {"Strobe White", LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.1))},

      // Rainbows
      {"Rainbow", LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(50))},
      {"Rainbow (Slow)", LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(15))},
    };

    for (Object[] p : patterns) {
      String name = (String) p[0];
      LEDPattern value = (LEDPattern) p[1];
      if (name.equals(defaultName)) {
        chooser.setDefaultOption(name, value);
      } else {
        chooser.addOption(name, value);
      }
    }
  }

  /** Re-sends the PID gains to the SPARK Flex, but only if the Elastic values changed. */
  private void updateCrankPid() {
    double p = SmartDashboard.getNumber("Crank kP", DEFAULT_KP);
    double i = SmartDashboard.getNumber("Crank kI", DEFAULT_KI);
    double d = SmartDashboard.getNumber("Crank kD", DEFAULT_KD);
    double ff = SmartDashboard.getNumber("Crank kFF", DEFAULT_KFF);

    if (p == m_lastP && i == m_lastI && d == m_lastD && ff == m_lastFF) {
      return;
    }
    m_lastP = p;
    m_lastI = i;
    m_lastD = d;
    m_lastFF = ff;

    SparkFlexConfig pidConfig = new SparkFlexConfig();
    pidConfig.closedLoop.pid(p, i, d).velocityFF(ff);
    // Only touches the gains; leaves everything else on the SPARK alone
    m_crank.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setState(State newState) {
    state = newState;
    stateStartTime = Timer.getFPGATimestamp();
  }

  /** Decides which pattern the LED strip should show right now. */
  private void updateLeds() {
    boolean auto = m_ledMode.getSelected();
    LEDPattern pattern;

    if (!auto) {
      pattern = m_manualPattern.getSelected();
    } else {
      switch (state) {
        case CRANKING:  pattern = m_crankingPattern.getSelected(); break;
        case POPPED:    pattern = m_poppedPattern.getSelected(); break;
        case RESETTING: pattern = m_resettingPattern.getSelected(); break;
        case IDLE:
        default:        pattern = m_idlePattern.getSelected(); break;
      }
    }

    if (pattern != null) {
      // Animated patterns (breathe, strobe, rainbow) must be re-applied every loop
      pattern.applyTo(m_ledBuffer);
      m_led.setData(m_ledBuffer);
    }
  }

  @Override
  public void robotPeriodic() {
    // Runs in every mode, so the LEDs work even while disabled
    updateLeds();

    SmartDashboard.putString("Prop State", state.toString());
    SmartDashboard.putBoolean("PIR Raw", irsensor.get());
    updateCrankPid(); // live PID tuning from Elastic
    SmartDashboard.putNumber("Crank RPM (actual)", m_crank.getEncoder().getVelocity() / CRANK_GEAR_RATIO);
    SmartDashboard.putNumber("Crank Output", m_crank.getAppliedOutput());
    SmartDashboard.putNumber("Crank Current (A)", m_crank.getOutputCurrent());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_crank.stopMotor();
    m_jack.set(RETRACTED);
    lastSequenceEndTime = -1000;
    setState(State.IDLE);
  }

  @Override
  public void teleopPeriodic() {
    double now = Timer.getFPGATimestamp();
    boolean pirDetected = (irsensor.get() == DETECTED_WHEN_TRUE)
        && SmartDashboard.getBoolean("PIR Enabled", true);
    boolean buttonPressed = controller.a().getAsBoolean(); // A button triggers the jack
    boolean detected = pirDetected || buttonPressed;
    SmartDashboard.putBoolean("Button Pressed", buttonPressed);
    double elapsed = now - stateStartTime;

    double crankMin = SmartDashboard.getNumber("Crank Min Time (s)", DEFAULT_CRANK_MIN_TIME_SEC);
    double crankMax = SmartDashboard.getNumber("Crank Max Time (s)", DEFAULT_CRANK_MAX_TIME_SEC);
    double crankRpm = SmartDashboard.getNumber("Crank Speed (RPM)", DEFAULT_CRANK_RPM);
    double poppedTime = SmartDashboard.getNumber("Popped Time (s)", DEFAULT_POPPED_TIME_SEC);
    double resetDelay = SmartDashboard.getNumber("Reset Delay (s)", DEFAULT_RESET_DELAY_SEC);
    double cooldown = SmartDashboard.getNumber("Trigger Cooldown (s)", DEFAULT_COOLDOWN_SEC);

    // Keep the target within the motor's real capability no matter what is typed into Elastic
    double motorRpm = crankRpm * CRANK_GEAR_RATIO;
    motorRpm = Math.max(-VORTEX_FREE_SPEED_RPM, Math.min(VORTEX_FREE_SPEED_RPM, motorRpm));

    double cooldownLeft = Math.max(0, cooldown - (now - lastSequenceEndTime));
    SmartDashboard.putNumber("Cooldown Left (s)", state == State.IDLE ? cooldownLeft : 0);

    switch (state) {
      case IDLE:
        if (detected && cooldownLeft <= 0) {
          // Pick a fresh random crank time for this pop (swap min/max if entered backwards)
          double lo = Math.max(0, Math.min(crankMin, crankMax));
          double hi = Math.max(0, Math.max(crankMin, crankMax));
          m_currentCrankTime = lo + Math.random() * (hi - lo);
          SmartDashboard.putNumber("Crank Time This Pop (s)", m_currentCrankTime);
          setState(State.CRANKING);
        }
        break;

      case CRANKING:
        // Crank spins "Pop goes the weasel" style until it's time to pop.
        // Closed-loop velocity control holds the speed steady even under load.
        m_crankPid.setReference(motorRpm, ControlType.kVelocity);
        SmartDashboard.putNumber("Crank RPM (target)", crankRpm);
        if (elapsed >= m_currentCrankTime) {
          m_crank.stopMotor();
          m_jack.set(POPPED_OUT);
          setState(State.POPPED);
        }
        break;

      case POPPED:
        if (elapsed >= poppedTime) {
          m_jack.set(RETRACTED);
          setState(State.RESETTING);
        }
        break;

      case RESETTING:
        if (elapsed >= resetDelay) {
          lastSequenceEndTime = now; // cooldown starts now
          setState(State.IDLE);
        }
        break;
    }
  }

  @Override
  public void disabledInit() {
    // Never leave the crank spinning when the robot is disabled
    m_crank.stopMotor();
    m_jack.set(RETRACTED);
    setState(State.IDLE);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}