#pragma once

class Pid {
  public:
    virtual float calc(float ref, float cur)=0;
    virtual void reset()=0;
};
