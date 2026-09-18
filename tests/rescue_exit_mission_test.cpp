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
    void enableFront()
    {
        tick();
        travel(config::kRescueExitFrontGuidanceStartCm + 0.1,
               config::kRescueExitFrontGuidanceStartCm + 0.1);
        tick();
    }
};

void testDistanceGateAndFrontCorrections()
{
    Fixture f;
    f.bottom.normalSteeringValid = false;
    f.guidance(120.0);
    require(!f.mission.requiresRescueZoneDetection(), "Saída fixa não precisa classificar corners");
    f.tick();
    for (int i = 0; i < 5; ++i) f.tick();
    require(!f.last.completed && close(f.last.leftPower, 0.75) && close(f.last.rightPower, 0.75),
            "Fusion não pode controlar antes dos 23 cm");
    f.travel(config::kRescueExitFrontGuidanceStartCm + 5.0,
             config::kRescueExitFrontGuidanceStartCm - 1.0);
    f.tick();
    require(f.last.status.phase == "rescue_exit_initial_straight", "A menor roda deve limitar os 23 cm");
    f.travel(0.0, 1.1);
    f.tick();
    require(f.last.status.phase == "rescue_exit_front_guidance" && f.last.leftPower > f.last.rightPower,
            "Fusion frontal deve corrigir à direita após o limiar");
    f.guidance(60.0);
    f.tick();
    require(f.last.leftPower < f.last.rightPower, "Fusion deve corrigir à esquerda");
    f.forward.exitCandidates = {};
    f.tick();
    require(close(f.last.leftPower, 0.75) && close(f.last.rightPower, 0.75),
            "Imagem sem Fusion deve cancelar a correção imediatamente");
    f.guidance(120.0);
    f.tick();
    require(!f.last.completed && f.last.status.phase == "rescue_exit_front_guidance" &&
            f.last.leftPower > f.last.rightPower, "Fusion deve voltar a corrigir quando reaparecer");
}

void testEncoderThresholdAtConfiguredDistance()
{
    require(close(config::kRescueExitFrontGuidanceStartCm, 23.0),
            "A saída normal deve liberar o Fusion frontal após 23 cm");
    Fixture f;
    f.bottom.normalSteeringValid = false;
    f.guidance(120.0);
    f.tick();
    const auto thresholdCounts = static_cast<long long>(std::ceil(
        config::kRescueExitFrontGuidanceStartCm * config::kEncoderCountsPerCentimeter));
    f.telemetry.leftEncoderCount = thresholdCounts + 100;
    f.telemetry.rightEncoderCount = thresholdCounts - 1;
    require(f.tick().status.phase == "rescue_exit_initial_straight",
            "Um pulso antes dos 23 cm ainda deve manter a reta");
    ++f.telemetry.rightEncoderCount;
    require(f.tick().status.phase == "rescue_exit_front_guidance" &&
            f.last.leftPower > f.last.rightPower,
            "O primeiro pulso que atinge os 23 cm deve habilitar a correção");
}

void testCandidateValidationAndProximity()
{
    Fixture f;
    f.bottom.normalSteeringValid = false;
    f.enableFront();
    f.guidance(120.0);
    f.forward.exitCandidates[2].entryDepthNormalized = 0.2;
    f.tick();
    require(f.last.leftPower > f.last.rightPower && f.last.status.exitGuidanceState == "CAM1",
            "Fusion distante deve corrigir após os 23 cm, sem exigir proximidade de 85%");
    f.forward.exitCandidates[2].entryDepthNormalized = 0.95;
    f.tick();
    require(f.last.leftPower > f.last.rightPower, "Fita próxima deve liberar correção");
    for (int invalid = 0; invalid < 4; ++invalid)
    {
        f.guidance(120.0);
        auto& candidate = f.forward.exitCandidates[2];
        candidate.blockedByColor = invalid == 0;
        candidate.grayNoiseLikely = invalid == 1;
        candidate.guidanceValid = invalid != 2;
        if (invalid == 3) candidate.guidanceAngleDegrees = std::numeric_limits<double>::quiet_NaN();
        f.tick();
        require(close(f.last.leftPower, f.last.rightPower), "Fusion inválido deve manter a reta");
    }
    f.forward.exitCandidates = {};
    f.guidance(120.0);
    f.forward.exitCandidates[0] = f.forward.exitCandidates[2];
    f.forward.exitCandidates[0].score = 0.9;
    f.forward.exitCandidates[0].guidanceAngleDegrees = 60.0;
    f.tick();
    require(f.last.leftPower < f.last.rightPower && f.last.status.exitSector == 0 &&
            close(f.last.status.exitConfidence, 0.9), "Controle e diagnóstico devem usar a melhor candidata válida");
    f.guidance(91.0);
    f.forward.exitCandidates[0] = {};
    f.tick();
    require(close(f.last.leftPower, f.last.rightPower), "Zona morta deve preservar a reta");
    f.forward.exitCandidates = {};
    f.travel(40.0, 40.0);
    f.tick();
    require(!f.last.failed && f.last.status.exitAdvanceCm > 60.0 && f.last.leftPower > 0.0,
            "Ausência de fita após 60 cm não deve iniciar busca ou retorno");
}

void testBottomConfirmationUsesNewConsecutiveFrames()
{
    Fixture f;
    f.enableFront();
    f.tick(50, false);
    f.tick(50, false);
    require(!f.last.completed, "IPC repetido não pode confirmar a CAM0");
    f.tick();
    f.bottom.normalSteeringValid = false;
    f.tick();
    f.bottom.normalSteeringValid = true;
    f.bottom.exitLineUnbranched = false;
    for (int i = 0; i < config::kRescueExitAcquisitionFrames - 1; ++i)
        require(!f.tick().completed, "CAM0 precisa de quatro frames consecutivos após perda");
    require(f.tick().completed && close(f.last.leftPower, f.bottom.lineFollowerLeftPower) &&
            close(f.last.rightPower, f.bottom.lineFollowerRightPower), "Handoff deve aplicar a CAM0 sem pausa");
}

void testFallbackDistanceLimitAndBlocker()
{
    Fixture f;
    f.bottom.lineControlSource = "gap-forward";
    f.bottom.normalSteeringValid = false;
    f.enableFront();
    require(f.last.status.exitGuidanceState == "STRAIGHT_NO_FUSION" &&
            f.last.status.exitBottomBlocker == "SOURCE_gap-forward",
            "O diagnóstico deve distinguir fallback de Fusion inferior válido");
    f.travel(30.0, 30.0);
    require(!f.tick().failed && f.last.leftPower > 0.0,
            "O fallback deve continuar reto dentro do limite configurado");
    f.guidance(120.0);
    f.tick();
    require(close(f.last.status.exitFallbackAdvanceCm, 0.0), "Fusion atual deve reiniciar o limite sem alvo");
    f.forward.exitCandidates = {};
    f.travel(config::kRescueExitFallbackMaximumAdvanceCm + 0.1,
             config::kRescueExitFallbackMaximumAdvanceCm + 0.1);
    require(f.tick().failed && close(f.last.leftPower, 0.0) && close(f.last.rightPower, 0.0) &&
            f.last.status.exitGuidanceState == "STOPPED",
            "Sem Fusion de nenhuma câmera, o limite deve parar o robô sem buscar outro yaw");
}

// Simula um giro próximo do alvo cuja correção não produz movimento.
// A saída deve avançar após dois segundos, mas nunca com a IMU inválida.
void testTurnCorrectionDeadlineContinuesExit()
{
    for (const bool loseImu : {false, true})
    {
        Fixture f;
        f.mission.setReferenceHeading(0.0);
        require(f.tick().status.phase == "rescue_exit_direct_turning",
                "A saída deve iniciar o giro");
        f.telemetry.yawZDeg = config::kRescueExitFromEntryYawDegrees -
            config::kRescueExitFromEntryYawToleranceDegrees - 1.0;
        f.telemetry.gyroZDegPerSec = 100.0;
        f.tick();
        f.telemetry.gyroZDegPerSec = 0.0;
        require(f.tick(config::kTurn90SettleMs + 1).status.phase ==
                    "rescue_exit_direct_turning",
                "O erro residual deve iniciar correção");
        const auto correctionStartedAt = f.now;
        // Ultrapassa o antigo limite de três pulsos sem mudar o yaw.
        for (int pulse = 0; pulse < 4; ++pulse)
        {
            require(!f.tick(config::kTurn90CorrectionPulseMs + 1).failed,
                    "Correção insuficiente não deve abortar a saída");
            require(!f.tick(config::kTurn90SettleMs + 1).failed,
                    "O limite de pulsos não deve abortar a saída");
        }
        const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            f.now - correctionStartedAt).count();
        require(f.tick(config::kRescueExitTurnCorrectionMaximumMs -
                           static_cast<int>(elapsedMs) - 1).status.phase ==
                    "rescue_exit_direct_turning",
                "Antes dos dois segundos, a correção deve permanecer ativa");
        if (loseImu) f.telemetry.mpuOk = false;
        const auto result = f.tick(1);
        if (loseImu)
            require(result.leftPower == 0.0 && result.rightPower == 0.0 &&
                        result.status.phase != "rescue_exit_initial_straight",
                    "O prazo não pode liberar avanço com IMU inválida");
        else
            require(!result.failed && result.status.phase == "rescue_exit_initial_straight" &&
                        result.leftPower > 0.0 && result.rightPower > 0.0,
                    "Após o prazo de correção, a saída deve seguir para a reta");
    }
}

void testTurnExcludedAndEntryReference()
{
    Fixture f;
    f.mission.reset();
    f.telemetry.yawZDeg = 170.0;
    f.mission.setReferenceHeading(170.0);
    const double target = std::remainder(170.0 + config::kRescueExitFromEntryYawDegrees, 360.0);
    require(f.tick().status.phase == "rescue_exit_direct_turning", "A referência de entrada deve definir o alvo");
    f.travel(50.0, 50.0);
    f.telemetry.yawZDeg = target;
    f.tick();
    f.tick(config::kTurn90SettleMs + 1);
    require(f.last.status.phase == "rescue_exit_initial_straight" && close(f.last.status.exitAdvanceCm, 0.0) &&
            close(f.last.status.exitHeadingDegrees, target), "Giro não pode entrar na distância da reta");
    f.travel(10.0, 10.0);
    f.tick();
    require(f.last.status.exitAdvanceCm < 11.0, "Distância deve começar após o giro");
    f.mission.reset();
    f.mission.setReferenceHeading(f.telemetry.yawZDeg);
    f.tick();
    require(close(f.last.status.exitAdvanceCm, 0.0) && !f.last.completed, "Nova execução deve limpar a distância e o handoff");
}

void testAbsoluteExitHeadingAndShortestTurn()
{
    for (const auto& headings : {std::pair<double, double>{0.0, 0.0},
                                 {0.0, 90.0}, {170.0, 170.0}, {-170.0, 170.0},
                                 {170.0, -100.0}})
    {
        Fixture f;
        f.mission.reset();
        f.mission.setReferenceHeading(headings.first);
        f.telemetry.yawZDeg = headings.second;
        const double target = std::remainder(
            headings.first + config::kRescueExitFromEntryYawDegrees, 360.0);
        const double turn = std::remainder(target - headings.second, 360.0);
        f.tick();
        require(close(f.last.status.exitHeadingDegrees, target),
                "O alvo deve depender da entrada salva e não do yaw atual");
        require(turn > 0.0 ? (f.last.leftPower > 0.0 && f.last.rightPower < 0.0)
                           : (f.last.leftPower < 0.0 && f.last.rightPower > 0.0),
                "A IMU deve escolher o menor giro à esquerda ou à direita");
        f.telemetry.yawZDeg = target;
        f.tick();
        require(f.tick(config::kTurn90SettleMs + 1).status.phase == "rescue_exit_initial_straight",
                "O giro deve terminar no mesmo alvo absoluto da saída");
    }
}

void testMissingEntryHeadingStopsSafely()
{
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()})
    {
        Fixture f;
        f.mission.reset();
        f.mission.setReferenceHeading(invalid);
        f.telemetry.yawZDeg = 90.0;
        require(f.tick().failed && close(f.last.leftPower, 0.0) && close(f.last.rightPower, 0.0),
                "Referência inválida deve falhar com motores parados, sem usar o yaw atual");
        require(f.tick().failed, "A falha deve permanecer até uma nova execução");
    }
    Fixture missing;
    missing.mission.reset();
    require(missing.tick().failed, "Referência ausente também deve impedir a saída normal");
}

void testNormalExitUsesDedicatedTolerance()
{
    Fixture inside;
    inside.mission.setReferenceHeading(0.0);
    inside.telemetry.yawZDeg = config::kRescueExitFromEntryYawDegrees -
                              config::kRescueExitFromEntryYawToleranceDegrees;
    require(inside.tick().status.phase == "rescue_exit_initial_straight",
            "Dentro da tolerância própria, a saída normal já pode iniciar a reta");
    Fixture outside;
    outside.mission.setReferenceHeading(0.0);
    outside.telemetry.yawZDeg = config::kRescueExitFromEntryYawDegrees -
                               config::kRescueExitFromEntryYawToleranceDegrees - 0.1;
    require(outside.tick().status.phase == "rescue_exit_direct_turning",
            "Fora da tolerância própria, a saída normal deve alinhar o yaw");
}

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

void testFailuresAndRecovery()
{
    Fixture camera;
    camera.bottom.normalSteeringValid = false;
    camera.enableFront();
    camera.guidance(120.0);
    camera.tick();
    camera.forward.ageMs = config::kRescueExitForwardStatusTimeoutMs + 1;
    require(camera.tick().status.phase == "rescue_exit_waiting_camera" && close(camera.last.leftPower, 0.0),
            "Imagem vencida deve zerar motores");
    camera.forward.ageMs = 0.0;
    camera.forward.exitCandidates = {};
    require(camera.tick().leftPower > 0.0, "Imagem atual sem linha deve permitir retomada reta");
    camera.forward.exitRunSequence = 6;
    require(close(camera.tick().leftPower, 0.0), "Execução incorreta não pode guiar motores");
    require(camera.tick(config::kRescueExitCameraRecoveryTimeoutMs).failed, "Câmera ausente deve falhar após recuperação");

    Fixture sensor;
    sensor.tick();
    sensor.telemetry.sensorFresh = false;
    require(sensor.tick().status.phase == "rescue_exit_waiting_sensors" && close(sensor.last.leftPower, 0.0),
            "Sensor inválido deve parar imediatamente");
    sensor.telemetry.sensorFresh = true;
    require(sensor.tick().leftPower > 0.0, "Sensor recuperado deve permitir retomada");
    sensor.telemetry.sensorFresh = false;
    sensor.tick();
    require(sensor.tick(config::kRescueExitSensorTimeoutMs).failed, "Sensor ausente deve falhar no timeout");

    Fixture wall;
    wall.tick();
    wall.forward.cameraObscured = true;
    require(wall.tick().failed && close(wall.last.leftPower, 0.0), "Obstrução deve falhar sem buscar outro yaw");
    Fixture stall;
    stall.tick();
    require(stall.tick(config::kRescueDistanceStallTimeoutMs).failed, "Rodas sem progresso devem parar");
    Fixture reboot;
    reboot.tick();
    reboot.telemetry.esp32UptimeMs = 0;
    require(reboot.tick(1).failed, "Reinício do ESP32 deve interromper a saída");
    Fixture timeout;
    timeout.tick();
    require(timeout.tick(config::kRescueExitTotalTimeoutMs).failed, "Timeout total deve permanecer ativo");
}
}

int main()
{
    try
    {
        testDistanceGateAndFrontCorrections();
        testEncoderThresholdAtConfiguredDistance();
        testCandidateValidationAndProximity();
        testBottomConfirmationUsesNewConsecutiveFrames();
        testFallbackDistanceLimitAndBlocker();
        testTurnCorrectionDeadlineContinuesExit();
        testTurnExcludedAndEntryReference();
        testAbsoluteExitHeadingAndShortestTurn();
        testMissingEntryHeadingStopsSafely();
        testNormalExitUsesDedicatedTolerance();
        testCompletedRescueRoute();
        testFailuresAndRecovery();
        std::cout << "rescue_exit_mission_test: OK\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "rescue_exit_mission_test: " << error.what() << '\n';
        return 1;
    }
}
