#pragma once
#include "daisy_seed.h"

#define KNOB_BLEND_PIN 28
#define KNOB_STEREO_PIN 24
#define KNOB_FEEDBACK_PIN 18
#define KNOB_FILTER_PIN 16
#define KNOB_START_L_PIN 25
#define KNOB_LENGTH_L_PIN 23
#define KNOB_SPEED_L_PIN 22
#define KNOB_FREEZE_L_PIN 21
#define KNOB_START_R_PIN 20
#define KNOB_LENGTH_R_PIN 19
#define KNOB_SPEED_R_PIN 17
#define KNOB_FREEZE_R_PIN 15

namespace slooper
{
    using namespace daisy;

    // The minimum difference in parameter value to be registered.
    constexpr float kMinValueDelta{0.003f};
    // The minimum difference in parameter value to be considered picked up.
    constexpr float kMinPickupValueDelta{0.01f};
    // The trigger threshold value.
    constexpr float kTriggerThres{0.3f};

    DaisySeed seed;

    enum Knob
    {
        KNOB_BLEND,
        KNOB_STEREO,
        KNOB_FEEDBACK,
        KNOB_FILTER,
        KNOB_START_L,
        KNOB_LENGTH_L,
        KNOB_SPEED_L,
        KNOB_FREEZE_L,
        KNOB_START_R,
        KNOB_LENGTH_R,
        KNOB_SPEED_R,
        KNOB_FREEZE_R,
        KNOB_LAST,
    };

    AnalogControl controls[Knob::KNOB_LAST]{};
    Parameter knobs[Knob::KNOB_LAST]{};

    inline void InitHw()
    {
        seed.Configure();
        seed.Init(true);
        seed.SetAudioBlockSize(48);
        float blockRate = seed.AudioSampleRate() / (float)seed.AudioBlockSize();

        short knobPins[KNOB_LAST] = {
            KNOB_BLEND_PIN,
            KNOB_STEREO_PIN,
            KNOB_FEEDBACK_PIN,
            KNOB_FILTER_PIN,
            KNOB_START_L_PIN,
            KNOB_LENGTH_L_PIN,
            KNOB_SPEED_L_PIN,
            KNOB_FREEZE_L_PIN,
            KNOB_START_R_PIN,
            KNOB_LENGTH_R_PIN,
            KNOB_SPEED_R_PIN,
            KNOB_FREEZE_R_PIN,
        };

        AdcChannelConfig knobInit[KNOB_LAST];
        for (short i = 0; i < KNOB_LAST; i++)
        {
            knobInit[i].InitSingle(seed.GetPin(knobPins[i]));
        }
        seed.adc.Init(knobInit, KNOB_LAST);

        for (short i = 0; i < KNOB_LAST; i++)
        {
            controls[i].Init(seed.adc.GetPtr(i), blockRate, true);
            controls[i].SetCoeff(1.f); // No slew;
            knobs[i].Init(controls[i], 0.0f, 1.0f, Parameter::LINEAR);
        }
    }

    void StartAudio(AudioHandle::AudioCallback cb)
    {
        seed.StartAudio(cb);
    }

    void StartAdc()
    {
        seed.adc.Start();
    }

    float GetAudioSampleRate()
    {
        return seed.AudioSampleRate();
    }

    inline void ProcessControls()
    {
        for (short i = 0; i < KNOB_LAST; i++)
        {
            controls[i].Process();
        }
    }
}