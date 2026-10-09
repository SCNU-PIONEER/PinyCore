#include "Bsp_tim.hpp"

void Tim::registerCallback(TIM_HandleTypeDef *_htim, HAL_TIM_CallbackIDTypeDef _callbackID,
                           pTIM_CallbackTypeDef _pCallback)
{
    HAL_TIM_RegisterCallback(_htim, _callbackID, _pCallback);
}
