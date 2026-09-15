#include "obr/rescue_exit_mission.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

struct Fixture
{
    RescueExitMission mission;
    CameraLineSnapshot bottom;
    ForwardLineSnapshot forward;
    RescueZoneSnapshot zones;
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
        bottom.sourceFresh = bottom.silverClassifierFresh = true;
    }
    RescueExitOutput tick(int ms = 50, bool frame = true)
    {
        now += std::chrono::milliseconds(ms);
        telemetry.esp32UptimeMs += ms;
        if (frame)
        {
            forward.timestamp += ms / 1000.0;
            bottom.lineTimestamp += ms / 1000.0;
            ++bottom.silverSequence;
            ++bottom.lineSequence;
            ++forward.sequence;
        }
        last = mission.update(bottom, forward, zones, telemetry, 7, now);
        require(std::abs(last.leftPower) <= 1 && std::abs(last.rightPower) <= 1, "Potências fora da faixa");
        return last;
    }
    void candidate(double angle = 0)
    {
        forward.exitCandidates[2] = {
            true, angle, 0.3, 2, 1, true, true,
            std::clamp(90.0 + angle, 0.0, 180.0)};
    }
    void approach()
    {
        // O heading inicial representa o corner vermelho proibido na estratégia geométrica.
        candidate(30.0);
        for (int i = 0; i < 12; ++i)
            if (tick().leftPower > 0 && last.rightPower > 0)
            {
                candidate();
                return;
            }
        throw std::runtime_error("Não iniciou aproximação");
    }
    void black()
    {
        bottom.normalSteeringValid = bottom.exitLineUnbranched = true;
        bottom.lineControlSource = "fusion";
        bottom.lineFollowerLeftPower = 0.72;
        bottom.lineFollowerRightPower = 0.69;
    }
    void advanceBottomValidation()
    {
        const long long counts = std::llround(
            config::kRescueExitBottomValidationAdvanceCm *
            config::kEncoderCountsPerCentimeter);
        telemetry.leftEncoderCount += counts;
        telemetry.rightEncoderCount += counts;
    }
    void finishLineEntry()
    {
        for (int frame = 0; frame < 20; ++frame)
        {
            const auto result = tick();
            if (result.status.phase == "rescue_exit_line_entry" &&
                result.leftPower > 0.0 && result.rightPower > 0.0)
            {
                const long long counts = std::llround(
                    config::kRescueExitLineEntryAdvanceCm *
                    config::kEncoderCountsPerCentimeter);
                telemetry.leftEncoderCount += counts;
                telemetry.rightEncoderCount += counts;
            }
            if (result.status.phase == "rescue_exit_line_seeking") return;
        }
        throw std::runtime_error("A reta de 3 cm não concluiu");
    }
    void finishInitialLinePivot()
    {
        const auto first = tick();
        require(first.status.phase == "rescue_exit_line_seeking" &&
                    first.leftPower * first.rightPower < 0.0,
                "O primeiro pivô não chegou aos motores");
        const auto second = tick(config::kRescueExitLineSearchSideMs);
        require(second.leftPower * second.rightPower < 0.0 &&
                    first.leftPower * second.leftPower < 0.0,
                "O segundo pivô não começou continuamente");
    }
};

void testFreshAcquisitionAndSilverPriority()
{
    Fixture f;
    require(f.mission.requiresRescueZoneDetection(),
            "Detector colorido não iniciou ativo para classificar os corners");
    f.approach();
    require(!f.mission.requiresRescueZoneDetection(),
            "Detector colorido separado permaneceu ativo durante o avanço do corner");
    f.black();
    f.finishLineEntry();
    f.finishInitialLinePivot();
    for (int i = 0; i < 3; ++i) require(!f.tick(1, false).completed, "Contou frame repetido");
    for (int i = 1; i < config::kRescueExitAcquisitionFrames; ++i)
        require(!f.tick().completed, "Transferiu antes de verificar a prata");
    require(!f.tick().completed &&
                f.last.status.phase == "rescue_exit_validating_bottom",
            "Fusion inferior encerrou a saída antes da janela da prata");
    f.advanceBottomValidation();
    require(f.tick().completed && f.last.leftPower == f.bottom.lineFollowerLeftPower,
            "Não transferiu após validar a janela física da prata");
    Fixture silver;
    silver.approach();
    silver.black();
    silver.bottom.courseMarkerConfirmed = true;
    silver.bottom.courseMarker = CourseMarker::Gray;
    require(!silver.tick().completed &&
                silver.last.status.phase == "rescue_exit_corner_backing",
            "Prata não iniciou o retorno ao ponto de referência");
    require(silver.last.status.exitRejections.find("permanente") != std::string::npos, "Prata não bloqueou direção");
    require(silver.tick().status.phase != "rescue_exit_checking_silver",
            "Prata persistente impediu o retorno ao centro");
    require(silver.tick().status.phase != "rescue_exit_checking_silver",
            "Prata persistente impediu a busca do próximo corner");
    for (int frame = 0; frame < 5; ++frame)
    {
        const auto output = silver.tick();
        require(!(output.leftPower > 0 && output.rightPower > 0),
                "Prata persistente permitiu um novo avanço");
    }
}

void testBranchedFusionWaitsForSingleLineAndClassifierStop()
{
    Fixture f;
    f.approach();
    f.black();
    f.bottom.exitLineUnbranched = false;
    f.bottom.lineNearDetected = true;
    require(f.tick().status.phase == "rescue_exit_line_seeking" &&
                f.last.leftPower == 0.0 && f.last.rightPower == 0.0,
            "Fusion de uma bifurcação guiou as rodas antes dos pivôs");
    f.bottom.exitLineUnbranched = true;
    f.finishLineEntry();
    f.finishInitialLinePivot();
    for (int i = 0; i < config::kRescueExitAcquisitionFrames; ++i)
        require(!f.tick().completed, "Transferiu antes da janela da prata");
    require(f.last.status.phase == "rescue_exit_validating_bottom",
            "Faixa única não iniciou a validação após quatro frames");
    f.bottom.exitLineUnbranched = false;
    f.advanceBottomValidation();
    require(f.tick().completed, "T durante a validação cancelou a saída após 25 cm");

    Fixture gap;
    gap.approach();
    gap.black();
    gap.finishLineEntry();
    gap.finishInitialLinePivot();
    for (int i = 0; i < config::kRescueExitAcquisitionFrames; ++i) gap.tick();
    require(gap.last.status.phase == "rescue_exit_validating_bottom",
            "Fusion normal não iniciou a validação antes do GAP");
    gap.bottom.lineControlSource = "fusion-gap-reacquire";
    gap.bottom.normalSteeringValid = false;
    gap.bottom.lineFollowerLeftPower = gap.bottom.lineFollowerRightPower = 0.0;
    require(gap.tick().leftPower == 0.0 && !gap.last.completed,
            "GAP sem potência NORMAL movimentou o robô");
    gap.bottom.lineFollowerLeftPower = gap.bottom.lineFollowerRightPower = 0.75;
    gap.advanceBottomValidation();
    require(gap.tick().completed,
            "Fusion de reacquisição no GAP cancelou a saída após 25 cm");

    Fixture invalid;
    invalid.approach();
    invalid.black();
    invalid.bottom.normalSteeringValid = false;
    for (int i = 0; i < 5; ++i) require(!invalid.tick().completed, "Aceitou Fusion sem direção válida");
    invalid.bottom.silverClassifierFresh = false;
    require(invalid.tick(300).leftPower > 0 && !invalid.last.failed,
            "Oscilação curta da CAM0 bloqueou uma candidata FAR/MID");
}

void testObscuredReturnsFromCurrentCorner()
{
    for (double advance : {8.0, 55.0})
    {
        Fixture f;
        f.approach();
        const long long counts = std::llround(advance * config::kEncoderCountsPerCentimeter);
        f.telemetry.leftEncoderCount += counts;
        f.telemetry.rightEncoderCount += counts;
        f.forward.cameraObscured = true;
        auto output = f.tick();
        require(output.leftPower == 0 &&
                    output.status.phase == "rescue_exit_corner_backing",
                "Obstrução não iniciou o retorno ao centro");
        require(f.tick().leftPower == -config::kRescueExitExplorationPower,
                "Ré para o próximo yaw não usou a potência configurada");
    }
}

void testLostCandidateResumesGeometryExploration()
{
    Fixture f;
    f.approach();
    f.forward.exitCandidates = {};
    require(f.tick(200).leftPower > 0, "Não reteve a última curva durante oscilação curta");
    const auto resumed = f.tick(101);
    require(resumed.status.phase == "rescue_exit_corner_exploring" &&
                resumed.leftPower == config::kRescueExitExplorationPower &&
                resumed.rightPower == config::kRescueExitExplorationPower,
            "Fusion perdida não devolveu o controle ao avanço reto do corner");
    const long long counts = std::llround(20.0 * config::kEncoderCountsPerCentimeter);
    f.telemetry.leftEncoderCount += counts;
    f.telemetry.rightEncoderCount += counts;
    require(f.tick().status.phase == "rescue_exit_exploring" &&
                f.last.leftPower > 0 && f.last.rightPower > 0,
            "Corner geométrico desistiu antes de avançar ao menos 20 cm");
}

void testSlowForwardCadenceAndScoreOscillation()
{
    Fixture f;
    f.tick(1);
    f.candidate();
    f.forward.sourceFresh = false;
    f.forward.ageMs = 160.0;
    f.tick(160);
    f.forward.exitCandidates[2].score = 0.18;
    f.tick(160);
    f.forward.exitCandidates[2].score = 0.74;
    const auto acquired = f.tick(160);
    require(acquired.status.phase == "rescue_exit_corner_exploring" &&
                acquired.leftPower > 0 && acquired.rightPower > 0,
            "Cadência de 160 ms ou score oscilante bloqueou a Fusion frontal");
}

void testScanStartsBoundedActiveExploration()
{
    Fixture f;
    bool sawExploration = false;
    bool sawFinalScan = false;
    for (int tick = 0; tick < 2000; ++tick)
    {
        const auto output = f.tick();
        if (output.status.phase == "rescue_exit_turning" ||
            output.status.phase == "rescue_exit_exploration_turning")
        {
            require(output.leftPower == 0.0 ||
                    (std::abs(output.leftPower) == config::kRescueExitTurnPower &&
                     output.rightPower == -output.leftPower),
                    "Giro mudou a potência do resgate");
            if (output.leftPower != 0.0)
                f.telemetry.yawZDeg = std::remainder(
                    f.telemetry.yawZDeg + (output.leftPower > 0.0 ? 10.0 : -10.0),
                    360.0);
        }
        if (output.status.phase == "rescue_exit_exploring")
        {
            sawExploration = true;
            require(output.leftPower == config::kRescueExitExplorationPower &&
                        output.rightPower == config::kRescueExitExplorationPower,
                    "Exploração não usou 0,80 em linha reta");
            const long long counts = std::llround(
                10.0 * config::kEncoderCountsPerCentimeter);
            f.telemetry.leftEncoderCount += counts;
            f.telemetry.rightEncoderCount += counts;
        }
        sawFinalScan = sawFinalScan || output.status.exitRound == 2;
        require(output.status.exitExplorationAttempt <=
                    config::kRescueExitExplorationMaximumAttempts,
                "Exploração ultrapassou três tentativas");
        require(output.status.exitExplorationAdvanceCm <=
                    config::kRescueExitExplorationTotalCm + 1.0,
                "Exploração ultrapassou 60 cm");
        if (sawFinalScan) break;
    }
    require(sawExploration && sawFinalScan,
            "Varredura vazia não explorou antes da rodada final");
}

void testCandidateStopsActiveScan()
{
    Fixture f;
    RescueExitOutput output;
    for (int tick = 0; tick < 60; ++tick)
    {
        output = f.tick();
        if (output.status.phase == "rescue_exit_geometry_turning" ||
            output.status.phase == "rescue_exit_turning") break;
    }
    require((output.status.phase == "rescue_exit_geometry_turning" ||
             output.status.phase == "rescue_exit_turning") &&
                output.leftPower > 0 && output.rightPower < 0,
            "Busca geométrica não iniciou o giro usado pelo teste");
    f.candidate(30.0);
    output = f.tick();
    require(output.status.phase == "rescue_exit_searching" &&
                output.leftPower == 0 && output.rightPower == 0,
            "Primeira candidata não interrompeu imediatamente o giro");
    f.tick();
    output = f.tick();
    require(output.status.phase == "rescue_exit_corner_exploring" &&
            output.leftPower > 0 && output.rightPower > 0,
            "Terceiro frame não retomou imediatamente o avanço reto do corner");
}

void testPartialFusionGuidesExploration()
{
    Fixture f;
    f.tick(1);
    f.candidate(30.0);
    f.tick(160);
    f.forward.exitCandidates = {};
    f.tick(config::kRescueExitCandidateReviewMs);

    for (int tick = 0; tick < 1000; ++tick)
    {
        const auto output = f.tick();
        if ((output.status.phase == "rescue_exit_turning" ||
             output.status.phase == "rescue_exit_exploration_turning") &&
            output.leftPower != 0.0)
        {
            f.telemetry.yawZDeg = std::remainder(
                f.telemetry.yawZDeg + (output.leftPower > 0.0 ? 10.0 : -10.0),
                360.0);
        }
        if (output.status.phase == "rescue_exit_exploring")
        {
            require(ImuTurnController::angularDistanceDegrees(
                        output.status.exitExplorationHeadingDegrees, 30.0) <= 1.0,
                    "Pista Fusion parcial não definiu o heading da exploração");
            return;
        }
    }
    throw std::runtime_error("Pista Fusion parcial não iniciou exploração");
}

void testBottomFusionTakesOverDuringExploration()
{
    Fixture f;
    for (int tick = 0; tick < 1000; ++tick)
    {
        const auto output = f.tick();
        if ((output.status.phase == "rescue_exit_turning" ||
             output.status.phase == "rescue_exit_exploration_turning") &&
            output.leftPower != 0.0)
        {
            f.telemetry.yawZDeg = std::remainder(
                f.telemetry.yawZDeg + (output.leftPower > 0.0 ? 10.0 : -10.0),
                360.0);
        }
        if (output.status.phase == "rescue_exit_exploring")
        {
            f.black();
            RescueExitOutput acquired;
            for (int frame = 0; frame < config::kRescueExitAcquisitionFrames; ++frame)
                acquired = f.tick();
            require(!acquired.completed &&
                        acquired.status.phase == "rescue_exit_validating_bottom",
                    "Fusion inferior encerrou a exploração antes da janela da prata");
            f.advanceBottomValidation();
            acquired = f.tick();
            require(acquired.completed &&
                        acquired.leftPower == f.bottom.lineFollowerLeftPower &&
                        acquired.rightPower == f.bottom.lineFollowerRightPower,
                    "Fusion inferior não assumiu durante a exploração");
            return;
        }
    }
    throw std::runtime_error("Teste não alcançou a exploração ativa");
}

void testCandidateIsReviewedBeforeAnotherTurn()
{
    Fixture f;
    f.tick(1);
    f.candidate();
    require(f.tick(160).leftPower == 0, "Primeira evidência não parou a busca");
    f.forward.exitCandidates = {};
    f.forward.exitCandidates[4] = {true, 40, 1.0, 2, 1, true, true, 130.0};
    require(f.tick(160).leftPower == 0 && f.last.rightPower == 0,
            "Outra direção roubou a candidata durante a revisão");
    f.forward.exitCandidates = {};
    for (int frame = 0; frame < 2; ++frame)
    {
        const auto waiting = f.tick(160);
        require(waiting.status.phase == "rescue_exit_searching" &&
                    waiting.leftPower == 0 && waiting.rightPower == 0,
                "Frame perdido iniciou outro giro antes de revisar a candidata");
    }
    f.candidate();
    require(f.tick(160).leftPower == 0, "Segundo frame consistente iniciou avanço cedo");
    const auto acquired = f.tick(160);
    require(acquired.status.phase == "rescue_exit_corner_exploring" &&
            acquired.leftPower > 0 && acquired.rightPower > 0,
            "Candidata reobservada não retomou o avanço no terceiro frame");

    Fixture expired;
    expired.tick(1);
    expired.candidate();
    expired.tick(160);
    expired.forward.exitCandidates = {};
    expired.tick(config::kRescueExitCandidateReviewMs);
    for (int frame = 0; frame < config::kRescueExitCandidateFrames; ++frame)
        expired.tick(160);
    require(expired.last.status.phase == "rescue_exit_searching",
            "Revisão expirada impediu a retomada da varredura");
    require(expired.tick().status.phase == "rescue_exit_turning",
            "Varredura não retomou após imagens vazias novas");
}

void testHeadingWrapRejectsSameEntrance()
{
    Fixture f;
    f.telemetry.yawZDeg = 359;
    f.approach();
    f.bottom.courseMarkerConfirmed = true;
    f.bottom.courseMarker = CourseMarker::Gray;
    f.tick();
    f.bottom.courseMarkerConfirmed = false;
    f.telemetry.yawZDeg = 1;
    for (int i = 0; i < 6; ++i)
    {
        const auto output = f.tick();
        require(!(output.leftPower > 0 && output.rightPower > 0), "Revisitou prata ao cruzar zero graus");
    }
}

void testSilverConeAndStraightGeometryAdvance()
{
    Fixture silver;
    silver.approach();
    silver.bottom.courseMarkerConfirmed = true;
    silver.bottom.courseMarker = CourseMarker::Gray;
    require(silver.tick().status.exitRejections.find("±35") != std::string::npos,
            "Entrada prata não recebeu o cone amplo");

    Fixture curve;
    curve.approach();
    curve.forward.exitCandidates[2].guidanceAngleDegrees = 55.0;
    curve.forward.exitCandidates[2].entryAngleDegrees = 150.0;
    curve.forward.exitCandidates[2].entryOffsetNormalized = 0.60;
    const auto distant = curve.tick();
    require(distant.leftPower == config::kRescueExitExplorationPower &&
                distant.rightPower == config::kRescueExitExplorationPower,
            "Fusion frontal desviou o avanço reto do corner distante");
    curve.forward.exitCandidates[2].entryDepthNormalized = 0.90;
    const auto near = curve.tick();
    require(near.leftPower == config::kRescueExitExplorationPower &&
                near.rightPower == config::kRescueExitExplorationPower,
            "Fusion frontal desviou o avanço reto do corner próximo");
}

void testNoisyFrontBlackRejectsCurrentYawBeforeFusion()
{
    Fixture noisy;
    noisy.approach();
    noisy.telemetry.yawZDeg = noisy.last.status.exitExplorationHeadingDegrees;
    noisy.black();
    noisy.forward.exitCandidates[2] = {};
    noisy.forward.exitCandidates[2].txDegrees = 0.0;
    noisy.forward.exitCandidates[2].grayNoiseLikely = true;
    const auto rejected = noisy.tick();
    require(rejected.status.phase == "rescue_exit_corner_backing" &&
                rejected.leftPower == 0.0 && rejected.rightPower == 0.0,
            "CAM1 ruidosa não interrompeu o avanço antes do Fusion da CAM0");
    require(rejected.status.exitRejections.find("permanente") != std::string::npos,
            "CAM1 ruidosa não descartou o yaw atual");

    Fixture confirmed;
    confirmed.approach();
    confirmed.black();
    require(confirmed.tick().status.phase == "rescue_exit_line_entry",
            "Fusion único não iniciou a reta de 3 cm");
    confirmed.forward.exitCandidates[2] = {};
    confirmed.forward.exitCandidates[2].txDegrees = 0.0;
    confirmed.forward.exitCandidates[2].grayNoiseLikely = true;
    require(confirmed.tick().status.phase == "rescue_exit_line_entry",
            "CAM1 ruidosa rejeitou uma saída já confirmada pela CAM0");

    Fixture solid;
    solid.approach();
    solid.telemetry.yawZDeg = solid.last.status.exitExplorationHeadingDegrees;
    solid.black();
    require(solid.tick().status.phase != "rescue_exit_corner_backing",
            "Linha preta contínua foi confundida com prata na CAM1");
}

void testNearBlackSearchesYawSideUntilCam0Fusion()
{
    Fixture f;
    f.approach();
    f.bottom.lineNearDetected = true;
    f.forward.exitCandidates[2].txDegrees = 8.0;
    f.forward.exitCandidates[2].guidanceAngleDegrees = 115.0;
    const auto started = f.tick();
    require(started.status.phase == "rescue_exit_line_seeking" &&
                started.leftPower == 0.0 && started.rightPower == 0.0,
            "Preto próximo sem Fusion não iniciou os pivôs parado");
    const auto turning = f.tick();
    require(turning.status.phase == "rescue_exit_line_seeking" &&
                turning.leftPower == config::kRescueExitLineSearchTurnPower &&
                turning.rightPower == -config::kRescueExitLineSearchTurnPower,
            "Yaw +100° não iniciou o pivô pela direita com potência 0,75");
    const auto otherSide = f.tick(config::kRescueExitLineSearchSideMs);
    require(otherSide.leftPower < 0.0 && otherSide.rightPower > 0.0,
            "Segundo pivô não começou continuamente após 650 ms");

    f.black();
    f.finishLineEntry();
    f.finishInitialLinePivot();
    const auto fusion = f.tick();
    require(fusion.status.phase == "rescue_exit_line_seek_fusion" &&
                fusion.leftPower == f.bottom.lineFollowerLeftPower &&
                fusion.rightPower == f.bottom.lineFollowerRightPower,
            "Primeiro Fusion não interrompeu o giro e guiou as rodas");
    for (int frame = 1; frame < config::kRescueExitAcquisitionFrames; ++frame)
        f.tick();
    require(f.last.status.phase == "rescue_exit_validating_bottom",
            "Pivôs não preservaram os quatro frames de confirmação");
}

void testFailuresStayStopped()
{
    for (int failure = 0; failure < 4; ++failure)
    {
        Fixture f;
        f.approach();
        if (failure == 0) f.telemetry.emergencyStopActive = true;
        if (failure == 1) f.telemetry.sensorFresh = false;
        if (failure == 2) f.forward.exitAnalysisActive = false;
        if (failure == 3) f.forward.exitRunSequence = 6;
        require(f.tick().leftPower == 0, "Falha não zerou motores");
        require(f.tick(2000).failed, "Falha persistente não encerrou");
    }
    Fixture stalled;
    stalled.approach();
    require(stalled.tick(config::kRescueDistanceStallTimeoutMs).status.phase ==
                "rescue_exit_corner_backing",
            "Travamento não iniciou o retorno para outro yaw");
    Fixture total;
    total.tick();
    require(total.tick(config::kRescueExitTotalTimeoutMs).failed, "Tempo total não encerrou");
}

void testExitBehind()
{
    Fixture behind;
    bool reached = false;
    for (int tick = 0; tick < 500; ++tick)
    {
        const double angle = std::remainder(180.0 - behind.telemetry.yawZDeg, 360.0);
        behind.forward.exitCandidates = {};
        if (std::abs(angle) < 10.0) behind.candidate(angle);
        const auto output = behind.tick();
        if (output.leftPower > 0 && output.rightPower < 0)
            behind.telemetry.yawZDeg = std::remainder(behind.telemetry.yawZDeg + 10.0, 360.0);
        if (output.leftPower > 0 && output.rightPower > 0)
        {
            reached = true;
            break;
        }
        require(!output.failed, "Busca encerrou antes de observar a saída atrás");
    }
    require(reached, "Não tentou a candidata atrás do robô");
}

void testGeometryStraightAndHandoff()
{
    Fixture tracking;
    tracking.approach();
    tracking.candidate(29.0);
    const auto curve = tracking.tick();
    require(curve.status.phase == "rescue_exit_corner_exploring" &&
                curve.leftPower == config::kRescueExitExplorationPower &&
                curve.rightPower == config::kRescueExitExplorationPower,
            "CAM1 desviou o corner geométrico distante");
    tracking.forward.exitCandidates[2].entryDepthNormalized = 0.90;
    const auto nearRight = tracking.tick();
    require(nearRight.leftPower == config::kRescueExitExplorationPower &&
                nearRight.rightPower == config::kRescueExitExplorationPower,
            "CAM1 desviou o corner geométrico próximo à direita");

    Fixture straight;
    straight.approach();
    const auto centered = straight.tick();
    require(centered.leftPower == config::kRescueExitExplorationPower &&
                centered.rightPower == config::kRescueExitExplorationPower,
            "Corner centralizado não manteve o avanço reto");

    Fixture left;
    left.approach();
    left.candidate(30.0);
    left.forward.exitCandidates[2].guidanceAngleDegrees = 61.0;
    left.forward.exitCandidates[2].entryDepthNormalized = 0.90;
    const auto leftCurve = left.tick();
    require(leftCurve.leftPower == config::kRescueExitExplorationPower &&
                leftCurve.rightPower == config::kRescueExitExplorationPower,
            "CAM1 desviou o corner geométrico próximo à esquerda");

    Fixture handoff;
    handoff.approach();
    handoff.black();
    handoff.finishLineEntry();
    handoff.finishInitialLinePivot();
    for (int i = 1; i < config::kRescueExitAcquisitionFrames; ++i)
        require(!handoff.tick().completed, "CAM0 assumiu antes da janela da prata");
    require(!handoff.tick().completed,
            "CAM0 encerrou a saída antes da janela física da prata");
    handoff.advanceBottomValidation();
    require(handoff.tick().completed,
            "CAM0 não assumiu após a janela física da prata");

    Fixture obscured;
    obscured.approach();
    obscured.forward.cameraObscured = true;
    require(obscured.tick().status.phase == "rescue_exit_corner_backing" &&
            obscured.last.leftPower == 0 && obscured.last.rightPower == 0,
            "Obstrução não iniciou o retorno seguro do corner");

    Fixture silver;
    silver.approach();
    silver.bottom.silverCandidateDetected = true;
    require(silver.tick(config::kRescueDistanceStallTimeoutMs).leftPower == 0, "Indício de prata não freou");
    silver.bottom.silverCandidateDetected = false;
    require(silver.tick().leftPower > 0 && !silver.last.failed, "Espera deliberada foi tratada como travamento");
}

}

int main()
{
    try
    {
        testFreshAcquisitionAndSilverPriority();
        testBranchedFusionWaitsForSingleLineAndClassifierStop();
        testObscuredReturnsFromCurrentCorner();
        testLostCandidateResumesGeometryExploration();
        testSlowForwardCadenceAndScoreOscillation();
        testScanStartsBoundedActiveExploration();
        testCandidateStopsActiveScan();
        testPartialFusionGuidesExploration();
        testBottomFusionTakesOverDuringExploration();
        testCandidateIsReviewedBeforeAnotherTurn();
        testHeadingWrapRejectsSameEntrance();
        testSilverConeAndStraightGeometryAdvance();
        testNoisyFrontBlackRejectsCurrentYawBeforeFusion();
        testNearBlackSearchesYawSideUntilCam0Fusion();
        testFailuresStayStopped();
        testExitBehind();
        testGeometryStraightAndHandoff();
        std::cout << "rescue_exit_mission_test: OK\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
