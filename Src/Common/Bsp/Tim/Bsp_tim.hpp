#pragma once

#include "Soc.hpp"
#include HAL_INCLUDE
#include "Singleton.hpp"

class Tim : public Singleton<Tim> {
public:
    /**
     * @brief tim register callback
     */
    void registerCallback(TIM_HandleTypeDef *_htim, HAL_TIM_CallbackIDTypeDef _callbackID,
                          pTIM_CallbackTypeDef _pCallback);

private:
    Tim() = default;
    friend class Singleton<Tim>;
};
