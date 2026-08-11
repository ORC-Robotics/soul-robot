#pragma once

// Monitora somente a saúde do processo de captura da câmera.
// Este módulo não interpreta pixels nem participa de decisões de movimento.
class CameraMonitor
{
public:
    bool ready() const;
};
