#include "obr/rescue_exit_mission.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{
void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}
bool close(double a, double b) { return std::abs(a - b) < 1e-8; }

struct Fixture
{
    RescueExitMission mission;
    CameraLineSnapshot bottom;
    ForwardLineSnapshot forward;
    Esp32TelemetrySnapshot telemetry;
    std::chrono::steady_clock::time_point now{};
    RescueExitOutput last;
    Fixture()
    {
        telemetry.serialOpen = telemetry.sensorFresh = telemetry.mpuOk = telemetry.motorSleepPinHigh = true;
        telemetry.lastSensorAgeMs = 0;
        telemetry.leftEncoderRate = telemetry.rightEncoderRate = 0;
        telemetry.yawZDeg = telemetry.gyroZDegPerSec = 0;
        telemetry.esp32UptimeMs = 1;
        forward.sourceFresh = forward.exitAnalysisActive = true;
        forward.exitRunSequence = 7;
        bottom.sourceFresh = bottom.normalSteeringValid = bottom.exitLineUnbranched = true;
        bottom.lineControlSource = "fusion";
        bottom.lineFollowerLeftPower = 0.72;
        bottom.lineFollowerRightPower = 0.69;
        mission.setReferenceHeading(-config::kRescueExitFromEntryYawDegrees);
    }
    RescueExitOutput tick(int ms = 50, bool frame = true)
    {
        now += std::chrono::milliseconds(ms);
        telemetry.esp32UptimeMs += ms;
        if (frame)
        {
            bottom.lineTimestamp += ms / 1000.0;
            forward.timestamp += ms / 1000.0;
            ++bottom.lineSequence;
            ++forward.sequence;
        }
        last = mission.update(bottom, forward, {}, telemetry, 7, now);
        require(std::isfinite(last.leftPower) && std::abs(last.leftPower) <= 1.0 &&
                std::isfinite(last.rightPower) && std::abs(last.rightPower) <= 1.0,
                "Potências devem ser finitas e limitadas");
        return last;
    }
    void travel(double leftCm, double rightCm)
    {
        telemetry.leftEncoderCount += std::llround(leftCm * config::kEncoderCountsPerCentimeter);
        telemetry.rightEncoderCount += std::llround(rightCm * config::kEncoderCountsPerCentimeter);
    }
    void guidance(double angle)
    {
        auto& candidate = forward.exitCandidates[2];
        candidate.visible = candidate.guidanceValid = true;
        candidate.score = 0.5;
        candidate.guidanceAngleDegrees = angle;
        candidate.entryDepthNormalized = 0.95;
    }
    void finishWallReposition()
    {
        require(tick(config::kRescueExitWallReverseMs).status.phase == "rescue_exit_wall_advance_starting" &&
                close(last.leftPower, 0.0), "A ré deve encerrar com saída zero antes da inversão");
        tick();
        require(close(last.leftPower, config::kRescueExitExplorationPower) &&
                close(last.rightPower, config::kRescueExitExplorationPower), "O novo avanço deve usar a potência configurada");
        require(tick(config::kRescueExitWallAdvanceMs).status.phase == "rescue_exit_zeroing_yaw" &&
                close(last.leftPower, 0.0) && close(last.rightPower, 0.0),
                "O novo avanço deve encerrar parado antes do zero de yaw");
    }
    void enableFront()
    {
        tick();
        tick();
        travel(config::kRescueExitCrossingCm + 0.1, config::kRescueExitCrossingCm + 0.1);
        require(tick().status.phase == "rescue_exit_wall_reverse", "Os 60 cm devem iniciar a ré na parede");
        finishWallReposition();
        require(tick(config::kRescueExitYawZeroSettleMs).status.phase == "rescue_exit_left_turn_starting",
                "A pausa deve registrar o zero local e preparar o giro");
        tick();
        telemetry.yawZDeg -= config::kRescueExitLeftTurnDegrees;
        tick();
        tick(config::kTurn90SettleMs + 1);
        require(last.status.phase == "rescue_exit_front_guidance_starting", "O segundo giro deve liberar a busca");
    }
};

void testCompletedRescueRoute()
{
    Fixture f;
    f.mission.startCompletedRescueRoute();
    f.telemetry.yawZDeg = 12.0;
    require(f.tick().status.phase == "rescue_exit_remembered_entry" &&
                f.last.leftPower > 0.0 && !f.last.completed,
            "O resgate concluído deve avançar antes do giro, sem aceitar Fusion.");
    f.travel(config::kRescueCompletedEntryAdvanceCm - 0.1,
             config::kRescueCompletedEntryAdvanceCm - 0.1);
    require(f.tick().status.phase == "rescue_exit_remembered_entry",
            "O avanço de entrada deve respeitar a distância configurada.");
    f.travel(0.2, 0.2);
    require(close(f.tick().leftPower, 0.0),
            "O robô deve parar para estabilizar antes do giro.");
    require(f.tick(config::kRescueDistanceSettleMs + 1).status.phase ==
                "rescue_exit_remembered_turn_starting" &&
                close(f.last.leftPower, 0.0),
            "A conclusão da entrada deve manter os motores parados neste ciclo.");

    const double targetYaw = std::remainder(
        12.0 + config::kRescueCompletedRightTurnDegrees, 360.0);
    require(f.tick().status.phase == "rescue_exit_direct_turning" &&
                close(f.last.status.exitHeadingDegrees, targetYaw),
            "O giro deve apontar 45 graus à direita do heading da entrada.");
    f.telemetry.yawZDeg = targetYaw;
    f.tick();
    require(f.tick(config::kTurn90SettleMs + 1).status.phase ==
                "rescue_exit_initial_straight" &&
                close(f.last.status.exitAdvanceCm, 0.0),
            "A segunda reta deve começar a medir distância após o giro.");
    f.travel(config::kRescueCompletedStraightCm - 0.1,
             config::kRescueCompletedStraightCm - 0.1);
    require(f.tick().status.phase == "rescue_exit_initial_straight",
            "O Fusion não deve assumir antes dos 30 cm configurados.");
    f.travel(0.2, 0.2);
    require(!f.tick().completed, "Um frame Fusion não confirma a saída.");
    for (int i = 1; i < config::kRescueExitAcquisitionFrames; ++i) f.tick();
    require(f.last.completed && f.last.status.phase == "rescue_exit_acquired",
            "A rota curta deve entregar a linha após os frames Fusion exigidos.");
}


void testNormalSequenceAndDistanceGate()
{
    Fixture f;
    f.bottom.normalSteeringValid = false;
    f.guidance(120.0);
    require(f.tick().status.phase == "rescue_exit_crossing_starting", "Alinhado, deve iniciar a travessia");
    f.tick();
    f.travel(config::kRescueExitCrossingCm + 5.0, config::kRescueExitCrossingCm - 1.0);
    require(f.tick().status.phase == "rescue_exit_initial_straight" &&
            close(f.last.leftPower, config::kRescueExitExplorationPower) &&
            close(f.last.rightPower, config::kRescueExitExplorationPower),
            "A menor roda limita a travessia e a visão não pode corrigir nessa etapa");
    f.travel(0.0, 1.1);
    require(f.tick().status.phase == "rescue_exit_wall_reverse", "Após 60 cm deve iniciar a ré na parede");
    f.finishWallReposition();
    require(f.tick(config::kRescueExitYawZeroSettleMs).status.phase == "rescue_exit_left_turn_starting",
            "Depois da pausa deve preparar o giro");
    require(f.tick().status.phase == "rescue_exit_left_turning" &&
            f.last.leftPower < 0.0 && f.last.rightPower > 0.0,
            "O segundo giro deve ser à esquerda");
    require(close(f.last.status.exitHeadingDegrees, -config::kRescueExitLeftTurnDegrees), "O segundo alvo usa o zero local da parede");
    f.telemetry.yawZDeg = -config::kRescueExitLeftTurnDegrees;
    f.tick();
    f.tick(config::kTurn90SettleMs + 1);
    require(f.last.status.phase == "rescue_exit_front_guidance_starting" &&
            close(f.last.status.exitAdvanceCm, 0.0), "O giro não deve entrar na distância da busca");
    f.tick();
    require(f.last.status.phase == "rescue_exit_front_guidance" && f.last.leftPower > f.last.rightPower,
            "Após o giro, a busca deve corrigir imediatamente pelo Fusion frontal");
}

void testSilverReferenceAndTolerance()
{
    for (const auto headings : {std::pair<double,double>{170.0, 170.0}, {-170.0, 170.0}, {0.0, -100.0}})
    {
        Fixture f;
        f.mission.setReferenceHeading(headings.first);
        f.telemetry.yawZDeg = headings.second;
        const double target = std::remainder(headings.first + config::kRescueExitFromEntryYawDegrees, 360.0);
        const double error = std::remainder(target - headings.second, 360.0);
        f.tick();
        require(close(f.last.status.exitHeadingDegrees, target), "O alvo deve usar o yaw salvo no prata");
        require(error > 0.0 ? f.last.leftPower > 0.0 : f.last.leftPower < 0.0,
                "O alinhamento deve escolher o menor giro até o alvo absoluto");
        f.travel(50.0, 50.0);
        f.telemetry.yawZDeg = target;
        f.tick();
        f.tick(config::kTurn90SettleMs + 1);
        require(f.last.status.phase == "rescue_exit_crossing_starting" &&
                close(f.last.status.exitAdvanceCm, 0.0), "Pulsos durante o giro não contam como travessia");
    }
    Fixture inside;
    inside.mission.setReferenceHeading(0.0);
    inside.telemetry.yawZDeg = config::kRescueExitFromEntryYawDegrees -
                              config::kRescueExitFromEntryYawToleranceDegrees;
    require(inside.tick().status.phase == "rescue_exit_crossing_starting", "Deve aceitar a tolerância ampla");
}

void testSensorlessTimedSequenceAndRecovery()
{
    Fixture f;
    f.mission.reset();
    f.telemetry.sensorFresh = f.telemetry.mpuOk = false;
    f.telemetry.yawZDeg = std::numeric_limits<double>::quiet_NaN();
    f.forward.exitAnalysisActive = false;
    f.forward.cameraObscured = true;
    f.bottom.sourceFresh = false;
    require(f.tick().leftPower > 0.0 && f.last.rightPower < 0.0, "Sem IMU, o giro deve usar tempo");
    require(f.tick(config::kRescueExitTimedQuarterTurnMs).status.phase == "rescue_exit_crossing_starting",
            "O giro temporizado deve encerrar sem falha");
    require(f.tick().leftPower > 0.0 && f.last.rightPower > 0.0, "A câmera obstruída não bloqueia a travessia");
    require(f.tick(config::kRescueExitCrossingTimeoutMs).status.phase == "rescue_exit_wall_reverse",
            "Sem encoders, o prazo deve encerrar a travessia");
    f.finishWallReposition();
    require(f.tick(config::kRescueExitYawZeroSettleMs).status.phase == "rescue_exit_left_turn_starting",
            "Sem IMU, zerar a referência não deve bloquear a sequência");
    require(f.tick().leftPower < 0.0 && f.last.rightPower > 0.0, "Sem IMU, o segundo giro é à esquerda");
    require(f.tick(config::kRescueExitTimedQuarterTurnMs).status.phase == "rescue_exit_front_guidance_starting",
            "O segundo giro deve liberar a busca");
    f.tick();
    require(!f.last.failed && f.last.leftPower > 0.0, "A busca deve continuar por tempo sem câmera");
    require(f.tick(config::kRescueExitNormalSearchTimeoutMs).status.phase == "rescue_exit_waiting_line" &&
            !f.last.failed && close(f.last.leftPower, 0.0), "Busca sem visão deve parar sem falhar a missão");
    f.bottom.sourceFresh = true;
    for (int i = 0; i < config::kRescueExitAcquisitionFrames; ++i) f.tick();
    require(f.last.completed, "A CAM0 recuperada deve concluir mesmo após o prazo da busca");
}

void testTurnTimeoutAndImuLossDoNotAbort()
{
    for (const bool loseImu : {false, true})
    {
        Fixture f;
        f.mission.setReferenceHeading(0.0);
        require(f.tick().status.phase == "rescue_exit_direct_turning", "Deve iniciar o alinhamento");
        if (loseImu) f.telemetry.mpuOk = false;
        require(f.tick(config::kRescueExitNormalTurnTimeoutMs).status.phase == "rescue_exit_crossing_starting" &&
                !f.last.failed, "Timeout ou perda da IMU devem liberar a próxima etapa");
    }
    Fixture partial;
    partial.mission.setReferenceHeading(0.0);
    partial.tick();
    partial.tick(config::kRescueExitTimedQuarterTurnMs - 10);
    partial.telemetry.mpuOk = false;
    require(partial.tick(10).status.phase == "rescue_exit_crossing_starting",
            "Perder a IMU não deve reiniciar o prazo do giro temporizado");
}

void testFusionValidationAndConsecutiveHandoff()
{
    Fixture f;
    f.bottom.normalSteeringValid = false;
    f.enableFront();
    f.guidance(120.0);
    f.tick();
    require(f.last.leftPower > f.last.rightPower, "Fusion válido deve corrigir à direita");
    f.guidance(60.0);
    f.tick();
    require(f.last.leftPower < f.last.rightPower, "Fusion válido deve corrigir à esquerda");
    f.forward.cameraObscured = true;
    f.tick();
    require(close(f.last.leftPower, f.last.rightPower), "Câmera obstruída não pode fornecer correção");
    f.forward.cameraObscured = false;
    f.forward.exitRunSequence = 6;
    f.tick();
    require(close(f.last.leftPower, f.last.rightPower), "IPC de outra execução não pode fornecer correção");
    f.forward.exitRunSequence = 7;
    f.guidance(std::numeric_limits<double>::quiet_NaN());
    f.tick();
    require(close(f.last.leftPower, f.last.rightPower), "Ângulo inválido não pode fornecer correção");
    f.bottom.normalSteeringValid = true;
    f.tick();
    for (int i = 0; i < 5; ++i) require(!f.tick(50, false).completed, "Frames repetidos não confirmam a CAM0");
    f.bottom.normalSteeringValid = false;
    f.tick();
    f.bottom.normalSteeringValid = true;
    for (int i = 0; i < config::kRescueExitAcquisitionFrames - 1; ++i)
        require(!f.tick().completed, "A confirmação deve exigir frames consecutivos");
    require(f.tick().completed && close(f.last.leftPower, f.bottom.lineFollowerLeftPower),
            "A CAM0 deve assumir com suas potências");
}

void testCompletedRouteStillStopsOnFailures()
{
    Fixture camera;
    camera.mission.startCompletedRescueRoute();
    camera.forward.cameraObscured = true;
    require(camera.tick().failed && close(camera.last.leftPower, 0.0), "Reentrada conserva a proteção visual");
    Fixture sensor;
    sensor.mission.startCompletedRescueRoute();
    sensor.telemetry.sensorFresh = false;
    require(close(sensor.tick().leftPower, 0.0), "Reentrada deve parar com sensor ausente");
    require(sensor.tick(config::kRescueExitSensorTimeoutMs).failed, "Reentrada conserva o timeout dos sensores");
}

void testLocalEmergencyAndStalledCrossing()
{
    Fixture f;
    f.mission.setReferenceHeading(0.0);
    f.tick();
    f.telemetry.emergencyStopActive = true;
    f.tick();
    require(close(f.last.leftPower, 0.0) && close(f.last.rightPower, 0.0) && !f.last.failed,
            "Emergência local deve zerar ambos os comandos mesmo na sequência temporizada");

    Fixture stalled;
    stalled.bottom.normalSteeringValid = false;
    stalled.tick();
    stalled.tick();
    require(stalled.tick(config::kRescueExitCrossingTimeoutMs).status.phase == "rescue_exit_wall_reverse" &&
            !stalled.last.failed, "Encoders sem progresso não devem abortar a travessia normal");

    Fixture reboot;
    reboot.tick();
    reboot.tick();
    reboot.telemetry.esp32UptimeMs = 0;
    reboot.telemetry.leftEncoderCount = reboot.telemetry.rightEncoderCount = 100000;
    reboot.tick(1);
    require(!reboot.last.failed && close(reboot.last.status.exitAdvanceCm, 0.0),
            "Reinício da ESP32 não pode falhar a missão nem somar o salto dos encoders");
}

void testWallYawZeroAfterSettling()
{
    Fixture f;
    f.bottom.normalSteeringValid = false;
    f.tick();
    f.tick();
    f.telemetry.yawZDeg = 170.0;
    f.travel(config::kRescueExitCrossingCm + 0.1, config::kRescueExitCrossingCm + 0.1);
    require(f.tick().status.phase == "rescue_exit_wall_reverse", "Deve reposicionar antes de registrar o zero");
    f.finishWallReposition();
    f.telemetry.yawZDeg = -170.0;
    const auto waiting = f.tick(config::kRescueExitYawZeroSettleMs - 1);
    require(waiting.status.phase == "rescue_exit_zeroing_yaw" && close(waiting.leftPower, 0.0),
            "O zero não pode ser registrado antes da pausa terminar");
    require(f.tick(1).status.phase == "rescue_exit_left_turn_starting" &&
            close(f.last.status.exitHeadingDegrees, -config::kRescueExitLeftTurnDegrees),
            "O alvo deve ser negativo no referencial local zerado depois de estabilizar");
    f.tick();
    require(f.last.leftPower < 0.0 && f.last.rightPower > 0.0, "O alvo local deve comandar a esquerda");
    f.telemetry.yawZDeg = std::remainder(-170.0 - config::kRescueExitLeftTurnDegrees, 360.0);
    f.tick();
    f.tick(config::kTurn90SettleMs + 1);
    require(f.last.status.phase == "rescue_exit_front_guidance_starting" && !f.last.failed,
            "A conversão do yaw global com passagem por 180 graus deve concluir o giro local");
}

void testWallRepositionDeadlinesAndEmergency()
{
    Fixture f;
    f.bottom.normalSteeringValid = false;
    f.tick();
    f.tick();
    f.travel(config::kRescueExitCrossingCm + 0.1, config::kRescueExitCrossingCm + 0.1);
    f.tick();
    f.telemetry.sensorFresh = f.telemetry.mpuOk = false;
    f.forward.cameraObscured = true;
    f.tick(config::kRescueExitWallReverseMs - 1);
    require(f.last.status.phase == "rescue_exit_wall_reverse" &&
            close(f.last.leftPower, -config::kRescueExitExplorationPower) &&
            close(f.last.rightPower, -config::kRescueExitExplorationPower) && !f.last.failed,
            "A ré deve durar um segundo mesmo com câmera e sensores inválidos");
    require(f.tick(1).status.phase == "rescue_exit_wall_advance_starting", "A ré deve encerrar no prazo exato");
    f.tick(config::kRescueExitWallAdvanceMs - 1);
    require(f.last.status.phase == "rescue_exit_wall_advance" && f.last.leftPower > 0.0 && !f.last.failed,
            "O novo avanço deve durar dois segundos antes de registrar o zero");
    require(f.tick(1).status.phase == "rescue_exit_zeroing_yaw" && close(f.last.leftPower, 0.0),
            "O prazo deve encerrar o avanço sem falhar a missão");

    for (const bool duringReverse : {true, false})
    {
        Fixture emergency;
        emergency.tick();
        emergency.tick();
        emergency.travel(config::kRescueExitCrossingCm + 0.1, config::kRescueExitCrossingCm + 0.1);
        emergency.tick();
        if (!duringReverse) emergency.tick(config::kRescueExitWallReverseMs);
        emergency.telemetry.emergencyStopActive = true;
        emergency.tick();
        require(close(emergency.last.leftPower, 0.0) && close(emergency.last.rightPower, 0.0),
                "Emergência deve zerar ambos os motores durante a ré e o novo avanço");
    }
}
}

int main()
{
    try
    {
        testNormalSequenceAndDistanceGate();
        testSilverReferenceAndTolerance();
        testSensorlessTimedSequenceAndRecovery();
        testTurnTimeoutAndImuLossDoNotAbort();
        testFusionValidationAndConsecutiveHandoff();
        testCompletedRescueRoute();
        testCompletedRouteStillStopsOnFailures();
        testLocalEmergencyAndStalledCrossing();
        testWallYawZeroAfterSettling();
        testWallRepositionDeadlinesAndEmergency();
        std::cout << "rescue_exit_mission_test: OK\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "rescue_exit_mission_test: " << error.what() << '\n';
        return 1;
    }
}
