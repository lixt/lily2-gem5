/*
 * Copyright (C) 2014 DSP Group, Institute of Microelectronics, Tsinghua University.
 * All rights reserved.
 */

#ifndef __CPU_HYBRID_RESOURCES_BIMODAL_HH__
#define __CPU_HYBRID_RESOURCES_BIMODAL_HH__


class Bimodal
{
  public:
    // Constructor.
    Bimodal (void) : state (0) {}

  public:
    // Returns 1 indicates the bimodal gives a positive selection.
    bool getAdopt (void) const
    {
        switch (state) {
            case 0x0: return false;
            case 0x1: return false;
            case 0x2: return true;
            case 0x3: return true;
            default : assert (0);
        }
    }

    // Sets the state according to the feedback.
    void setAtopt (bool adopt)
    {
        switch (state) {
            case 0x0: state = adopt ? 0x1 : 0x0; break;
            case 0x1: state = adopt ? 0x2 : 0x0; break;
            case 0x2: state = adopt ? 0x3 : 0x2; break;
            case 0x3: state = adopt ? 0x2 : 0x3; break;
            default : assert (0);
        }
    }

  private:
    int state;
};

#endif // __CPU_HYBRID_RESOURCES_BIMODAL_HH__
