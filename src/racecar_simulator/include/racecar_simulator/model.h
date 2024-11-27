#ifndef MODELS_H
#define MODELS_H

#include <stdexcept>
#include <cmath>
#include "model_input.h"
#include "model_state.h"

// 모델 인터페이스
class IModel {
public:
    virtual ~IModel() {}
    virtual void update(IState& state) = 0;
};

// ODEModel 클래스
class ODEModel : public IModel {
public:
    void update(IState& state) override;
};

// PacejkaModel 클래스
class PacejkaModel : public IModel {
public:
    void update(IState& state) override;
};

#endif // MODELS_H
