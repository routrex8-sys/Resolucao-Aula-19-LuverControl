package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Atividade 2 (Aula 19) - TimedRobot + WPILib.
 *
 * Desafio 1: autonomo que coleta 3 algas -> 2 na NET e 1 no PROCESSOR.
 * Desafio 2: botao A (segurar) leva o robo ate X=7.8, Y=4.0, 60 graus.
 *
 * Tudo que depende do SEU projeto (drivetrain, odometria, mecanismos) esta
 * na secao "PONTE COM O SEU PROJETO". Adapte so aquela parte.
 */
public class Robot extends TimedRobot {

  // ===================== CONSTANTES =====================
  private static final double MAX_VEL_LINEAR = 3.0; // m/s
  private static final double MAX_VEL_ANGULAR = 4.0; // rad/s
  private static final double TOL_POS = 0.05; // m
  private static final double TOL_ANG = Math.toRadians(2.0);

  // Desafio 2
  private static final Pose2d POSE_DESAFIO_2 = new Pose2d(7.8, 4.0, Rotation2d.fromDegrees(60));

  // Desafio 1 - AJUSTE estas poses com as coordenadas reais da sua arena/simulacao
  private static final Pose2d POSE_INICIAL = new Pose2d(7.2, 1.0, new Rotation2d());
  private static final Pose2d ALGA_1 = new Pose2d(7.0, 2.5, Rotation2d.fromDegrees(90)); // AJUSTAR
  private static final Pose2d ALGA_2 = new Pose2d(8.0, 2.5, Rotation2d.fromDegrees(90)); // AJUSTAR
  private static final Pose2d ALGA_3 = new Pose2d(9.0, 2.5, Rotation2d.fromDegrees(90)); // AJUSTAR
  private static final Pose2d POSE_NET = new Pose2d(7.6, 6.0, Rotation2d.fromDegrees(90)); // AJUSTAR
  private static final Pose2d POSE_PROCESSOR = new Pose2d(11.5, 1.0, Rotation2d.fromDegrees(-90)); // AJUSTAR

  // ===================== HARDWARE / CONTROLE =====================
  private final XboxController controle = new XboxController(0);

  private final PIDController pidX = new PIDController(2.0, 0, 0);
  private final PIDController pidY = new PIDController(2.0, 0, 0);
  private final PIDController pidTheta = new PIDController(4.0, 0, 0);

  // ===================== AUTONOMO (maquina de estados) =====================
  private enum Acao { COLETAR, NET, PROCESSOR }

  /** Cada passo: ir ate a pose e depois executar a acao. */
  private record Passo(Pose2d pose, Acao acao) {}

  private final Passo[] sequencia = {
    new Passo(ALGA_1, Acao.COLETAR),
    new Passo(POSE_NET, Acao.NET),
    new Passo(ALGA_2, Acao.COLETAR),
    new Passo(POSE_NET, Acao.NET),
    new Passo(ALGA_3, Acao.COLETAR),
    new Passo(POSE_PROCESSOR, Acao.PROCESSOR),
  };

  private enum Fase { IR_ATE_POSE, EXECUTAR_ACAO, FIM }

  private int passoAtual;
  private Fase fase;
  private final Timer timerAcao = new Timer();
  private static final double TEMPO_MAX_ACAO = 2.5; // s, evita travar o autonomo

  public Robot() {
    pidTheta.enableContinuousInput(-Math.PI, Math.PI);
    pidX.setTolerance(TOL_POS);
    pidY.setTolerance(TOL_POS);
    pidTheta.setTolerance(TOL_ANG);
  }

  // ===================== AUTONOMO =====================
  @Override
  public void autonomousInit() {
    resetarPose(POSE_INICIAL);
    passoAtual = 0;
    fase = Fase.IR_ATE_POSE;
  }

  @Override
  public void autonomousPeriodic() {
    if (fase == Fase.FIM || passoAtual >= sequencia.length) {
      pararRobo();
      pararMecanismos();
      return;
    }

    Passo passo = sequencia[passoAtual];

    switch (fase) {
      case IR_ATE_POSE -> {
        if (irParaPose(passo.pose())) {
          pararRobo();
          timerAcao.restart();
          fase = Fase.EXECUTAR_ACAO;
        }
      }
      case EXECUTAR_ACAO -> {
        pararRobo();
        boolean concluida = executarAcao(passo.acao());
        if (concluida || timerAcao.hasElapsed(TEMPO_MAX_ACAO)) {
          pararMecanismos();
          passoAtual++;
          fase = (passoAtual >= sequencia.length) ? Fase.FIM : Fase.IR_ATE_POSE;
        }
      }
      default -> {}
    }
  }

  /** Retorna true quando a acao terminou. */
  private boolean executarAcao(Acao acao) {
    switch (acao) {
      case COLETAR -> {
        coletarAlga();
        return temAlga();
      }
      case NET -> {
        pontuarNet();
        return !temAlga();
      }
      case PROCESSOR -> {
        pontuarProcessor();
        return !temAlga();
      }
    }
    return true;
  }

  // ===================== TELEOP =====================
  @Override
  public void teleopPeriodic() {
    if (controle.getAButton()) {
      // Desafio 2: segure A para navegar ate a pose de destino
      irParaPose(POSE_DESAFIO_2);
    } else {
      pidX.reset();
      pidY.reset();
      pidTheta.reset();
      double vx = -MathUtil.applyDeadband(controle.getLeftY(), 0.1) * MAX_VEL_LINEAR;
      double vy = -MathUtil.applyDeadband(controle.getLeftX(), 0.1) * MAX_VEL_LINEAR;
      double w = -MathUtil.applyDeadband(controle.getRightX(), 0.1) * MAX_VEL_ANGULAR;
      dirigirCampo(vx, vy, w);
    }
  }

  // ===================== NAVEGACAO ATE UMA POSE =====================
  /**
   * Usa a pose atual (odometria) como referencia e comanda velocidades
   * field-relative ate o alvo. Funciona de qualquer posicao/orientacao inicial.
   *
   * @return true quando chegou (posicao e angulo dentro da tolerancia)
   */
  private boolean irParaPose(Pose2d alvo) {
    Pose2d atual = getPose();

    double vx = pidX.calculate(atual.getX(), alvo.getX());
    double vy = pidY.calculate(atual.getY(), alvo.getY());
    double w =
        pidTheta.calculate(atual.getRotation().getRadians(), alvo.getRotation().getRadians());

    // limita a velocidade linear total, mantendo a direcao
    double vLinear = Math.hypot(vx, vy);
    if (vLinear > MAX_VEL_LINEAR) {
      vx *= MAX_VEL_LINEAR / vLinear;
      vy *= MAX_VEL_LINEAR / vLinear;
    }
    w = MathUtil.clamp(w, -MAX_VEL_ANGULAR, MAX_VEL_ANGULAR);

    boolean chegou =
        atual.getTranslation().getDistance(alvo.getTranslation()) < TOL_POS
            && Math.abs(MathUtil.angleModulus(
                    alvo.getRotation().getRadians() - atual.getRotation().getRadians()))
                < TOL_ANG;

    if (chegou) {
      dirigirCampo(0, 0, 0);
    } else {
      dirigirCampo(vx, vy, w);
    }
    return chegou;
  }

  private void pararRobo() {
    dirigirCampo(0, 0, 0);
  }

  // =====================================================================
  //                 PONTE COM O SEU PROJETO (ADAPTAR AQUI)
  // =====================================================================

  /** Pose atual do robo na arena (odometria/simulacao). */
  private Pose2d getPose() {
    
    return new Pose2d();
  }

  /** Reseta a odometria para a pose informada. */
  private void resetarPose(Pose2d pose) {
    
  }

  /** Dirige o chassi com velocidades FIELD-RELATIVE (m/s, m/s, rad/s). */
  private void dirigirCampo(double vx, double vy, double omega) {
    
    // Se o seu metodo for robot-relative, use:
    // ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getPose().getRotation())
  }

  /** Liga o mecanismo que coleta a alga. */
  private void coletarAlga() {
   
  }

  /** Posiciona/ejeta para pontuar na NET. */
  private void pontuarNet() {
    
  }

  /** Posiciona/ejeta para pontuar no PROCESSOR. */
  private void pontuarProcessor() {
    
  }

  /** true se o robo esta segurando uma alga. */
  private boolean temAlga() {
    
    return false;
  }

  /** Para os mecanismos (motores do intake/braco/lancador). */
  private void pararMecanismos() {
   
  }
}