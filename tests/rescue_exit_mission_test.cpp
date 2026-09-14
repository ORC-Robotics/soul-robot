#include "obr/rescue_exit_mission.h"
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
        last = mission.update(bottom, forward, telemetry, 7, now);
        require(std::abs(last.leftPower) <= 1 && std::abs(last.rightPower) <= 1, "Potências fora da faixa");
        return last;
    }
    void candidate(double angle = 0)
    {
        forward.exitCandidates[2] = {true, angle, 0.3, 2, 1, true};
    }
    void approach()
    {
        candidate();
        for (int i = 0; i < 12; ++i)
            if (tick().leftPower > 0 && last.rightPower > 0) return;
        throw std::runtime_error("Não iniciou aproximação");
    }
    void black()
    {
        bottom.normalSteeringValid = bottom.exitLineUnbranched = true;
        bottom.lineControlSource = "fusion";
    }
};

void testFreshAcquisitionAndSilverPriority()
{
    Fixture f;
    f.approach();
    f.black();
    for (int i = 0; i < 3; ++i) require(!f.tick().completed, "Confirmou cedo demais");
    for (int i = 0; i < 3; ++i) require(!f.tick(1, false).completed, "Contou frame repetido");
    require(f.tick().completed && f.last.leftPower == 0, "Não confirmou quatro imagens novas");
    Fixture silver;
    silver.approach();
    silver.black();
    for (int i = 0; i < 3; ++i) silver.tick();
    silver.bottom.courseMarkerConfirmed = true;
    silver.bottom.courseMarker = CourseMarker::Gray;
    require(!silver.tick().completed && silver.last.status.phase == "rescue_exit_rejected", "Prata perdeu prioridade");
    require(silver.last.status.exitRejections.find("permanente") != std::string::npos, "Prata não bloqueou direção");
}

void testInvalidGeometryAndClassifierStop()
{
    Fixture f;
    f.approach();
    f.black();
    f.bottom.exitLineUnbranched = false;
    for (int i = 0; i < 5; ++i) require(!f.tick().completed, "Aceitou bifurcação");
    f.bottom.silverClassifierFresh = false;
    require(f.tick().leftPower == 0 && !f.last.failed, "Não parou sem classificador");
    require(f.tick(2000).failed, "Não encerrou após perda de sensor");
}

void testObscuredReverseIsLimitedToTravel()
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
        require(output.leftPower == 0 && output.status.phase == "rescue_exit_rejected", "Obstrução não parou");
        const double target = output.status.targetDistanceCm;
        require(target <= 40 && target <= advance + 1, "Ré ultrapassa a tentativa");
        require(f.tick().leftPower == -config::kRescuePostDepositReversePower, "Ré não reutiliza potência");
        f.telemetry.leftEncoderCount -= std::llround(target * config::kEncoderCountsPerCentimeter) + 1;
        f.telemetry.rightEncoderCount -= std::llround(target * config::kEncoderCountsPerCentimeter) + 1;
        require(f.tick().leftPower == 0, "Ré não freou ao alcançar a distância");
        f.tick(config::kRescueDistanceSettleMs);
        require(f.last.status.exitReverseCm >= target, "Diagnóstico perdeu a distância recuada");
    }
}

void testLostCandidateStopsAndDoesNotSwitch()
{
    Fixture f;
    f.approach();
    f.forward.exitCandidates = {};
    f.forward.exitCandidates[4] = {true, 30, 1.0, 2, 1, true};
    require(f.tick().leftPower == 0, "Continuou sem o alvo travado");
    require(f.tick(500).status.phase == "rescue_exit_rejected", "Não rejeitou alvo perdido");
}

void testScanTwoRoundsAndPower()
{
    Fixture f;
    bool sawSecond = false;
    int movingTicks = 0;
    for (int tick = 0; tick < 2000; ++tick)
    {
        const auto output = f.tick();
        sawSecond = sawSecond || output.status.exitRound == 2;
        if (output.leftPower > 0)
        {
            require(output.leftPower == config::kRescueSearchTurnPower && output.rightPower == -output.leftPower,
                    "Giro mudou a potência do resgate");
            f.telemetry.yawZDeg = std::remainder(f.telemetry.yawZDeg + 10.0, 360.0);
            ++movingTicks;
        }
        if (output.failed)
        {
            require(sawSecond && movingTicks == 72, "Não executou exatamente duas varreduras");
            require(output.status.exitLastFailure == "Duas rodadas sem saída válida", "Encerrou por falha inesperada");
            return;
        }
    }
    throw std::runtime_error("Varredura não terminou");
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

void testFailuresStayStopped()
{
    for (int failure = 0; failure < 4; ++failure)
    {
        Fixture f;
        f.approach();
        if (failure == 0) f.telemetry.emergencyStopActive = true;
        if (failure == 1) f.telemetry.sensorFresh = false;
        if (failure == 2) f.forward.sourceFresh = false;
        if (failure == 3) f.forward.exitRunSequence = 6;
        require(f.tick().leftPower == 0, "Falha não zerou motores");
        require(f.tick(2000).failed, "Falha persistente não encerrou");
    }
    Fixture stalled;
    stalled.approach();
    require(stalled.tick(config::kRescueDistanceStallTimeoutMs).failed, "Travamento não encerrou");
    Fixture total;
    total.tick();
    require(total.tick(config::kRescueExitTotalTimeoutMs).failed, "Tempo total não encerrou");
}

void testExitBehindAndOneTemporaryRetry()
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

    Fixture retry;
    int attempts = 0;
    for (int tick = 0; tick < 2000; ++tick)
    {
        const double angle = std::remainder(-retry.telemetry.yawZDeg, 360.0);
        retry.forward.exitCandidates = {};
        if (std::abs(angle) < 10) retry.candidate(angle);
        retry.forward.cameraObscured = retry.last.status.phase == "rescue_exit_candidate";
        const auto output = retry.tick();
        if (output.status.phase == "rescue_exit_candidate") ++attempts;
        if (output.leftPower > 0 && output.rightPower < 0)
            retry.telemetry.yawZDeg = std::remainder(retry.telemetry.yawZDeg + 10.0, 360.0);
        if (output.failed)
        {
            require(attempts == 2 && output.status.exitRound == 2,
                    "Rejeição temporária deve liberar exatamente uma nova tentativa");
            return;
        }
    }
    throw std::runtime_error("Busca repetiu rejeições temporárias indefinidamente");
}

void testNearConfidenceDoesNotRegressAndSilverWaitIsNotStall()
{
    Fixture f;
    f.approach();
    f.forward.exitCandidates[2].nearestBand = 2;
    f.tick();
    f.forward.exitCandidates[2].nearestBand = 0;
    f.forward.exitCandidates[2].depthBands = 1;
    f.forward.exitCandidates[2].tapeValid = false;
    require(f.tick().leftPower == 0, "Aproximação voltou a aceitar uma massa distante depois do NEAR");
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
        testInvalidGeometryAndClassifierStop();
        testObscuredReverseIsLimitedToTravel();
        testLostCandidateStopsAndDoesNotSwitch();
        testScanTwoRoundsAndPower();
        testHeadingWrapRejectsSameEntrance();
        testFailuresStayStopped();
        testExitBehindAndOneTemporaryRetry();
        testNearConfidenceDoesNotRegressAndSilverWaitIsNotStall();
        std::cout << "rescue_exit_mission_test: OK\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
