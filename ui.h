#pragma once

#include "hw.h"
#include "slooper.h"
#include "wreath/head.h"
#include "Utility/dsp.h"
#include <string>

namespace slooper
{
    using namespace daisy;
    using namespace daisysp;
    using namespace wreath;

    constexpr float kMaxMsHoldForTrigger{300.f};
    constexpr float kMaxGain{5.f};
    constexpr float kMaxFilterValue{1500.f};
    constexpr float kMaxRateSlew{10.f};

    enum Channel
    {
        LEFT,
        RIGHT,
        BOTH,
        SETTINGS,
    };
    Channel prevChannel{Channel::BOTH};
    Channel currentChannel{Channel::BOTH};

    float values[12]{};

    bool startUp{true};
    bool first{true};
    bool buffering{};

    float Map(float value, float aMin, float aMax, float bMin, float bMax)
    {
        float k = std::abs(bMax - bMin) / std::abs(aMax - aMin) * (bMax > bMin ? 1 : -1);

        return bMin + k * (value - aMin);
    }

    inline void ProcessParameter(short idx, float value)
    {
        switch (idx)
        {
        case KNOB_BLEND:
            looper.dryWetMix = value;
            looper.feedbackOnly = value == 0.f;
            break;
        case KNOB_STEREO:
            looper.stereoWidth = value;
            if (value == 0.f)
            {
                looper.mustRetrigger = true;
            }
            break;
        case KNOB_FEEDBACK:
            looper.feedback = value;
            break;
        case KNOB_FILTER:
            looper.SetFilterValue(Map(value, 0.f, 1.f, 0.f, kMaxFilterValue));
            break;
        case KNOB_START_L:
        case KNOB_START_R:
            {
                /*
                if (KNOB_START_L == idx)
                {
                    looper.leftFeedbackPath = value;
                }
                else
                {
                    looper.rightFeedbackPath = value;
                }
                */
                Channel channel = KNOB_START_L == idx ? Channel::LEFT : Channel::RIGHT;
                looper.SetLoopStart(channel, Map(value, 0.f, 1.f, 0.f, looper.GetBufferSamples(channel) - 1));
            }
            break;
        case KNOB_LENGTH_L:
        case KNOB_LENGTH_R:
            {
                Channel channel = KNOB_LENGTH_L == idx ? Channel::LEFT : Channel::RIGHT;

                // Backwards, from buffer's length to 50ms.
                if (value <= 0.35f)
                {
                    looper.SetLoopLength(channel, Map(value, 0.f, 0.35f, looper.GetBufferSamples(channel), kMinSamplesForFlanger));
                    looper.SetDirection(channel, Direction::BACKWARDS);
                }
                // Backwards, from 50ms to 1ms (grains).
                else if (value < 0.47f)
                {
                    looper.SetLoopLength(channel, Map(value, 0.35f, 0.47f, kMinSamplesForFlanger, kMinSamplesForTone));
                    looper.SetDirection(channel, Direction::BACKWARDS);
                }
                // Forward, from 1ms to 50ms (grains).
                else if (value >= 0.53f && value < 0.65f)
                {
                    looper.SetLoopLength(channel, Map(value, 0.53f, 0.65f, kMinSamplesForTone, kMinSamplesForFlanger));
                    looper.SetDirection(channel, Direction::FORWARD);
                }
                // Forward, from 50ms to buffer's length.
                else if (value >= 0.65f)
                {
                    looper.SetLoopLength(channel, Map(value, 0.65f, 1.f, kMinSamplesForFlanger, looper.GetBufferSamples(channel)));
                    looper.SetDirection(channel, Direction::FORWARD);
                }
                // Center dead zone.
                else
                {
                    looper.SetLoopLength(channel, Map(value, 0.47f, 0.53f, kMinLoopLengthSamples, kMinLoopLengthSamples));
                    looper.SetDirection(channel, Direction::FORWARD);
                }

                // Refresh the rate parameter if the note mode changed.
                static StereoLooper::NoteMode noteModeLeft{};
                if (noteModeLeft != looper.noteModeLeft)
                {
                    ProcessParameter(KNOB_SPEED_L, values[KNOB_SPEED_L]);
                }
                noteModeLeft = looper.noteModeLeft;

                static StereoLooper::NoteMode noteModeRight{};
                if (noteModeRight != looper.noteModeRight)
                {
                    ProcessParameter(KNOB_SPEED_R, values[KNOB_SPEED_R]);
                }
                noteModeRight = looper.noteModeRight;
            }
            break;
        case KNOB_SPEED_L:
        case KNOB_SPEED_R:
            {
                Channel channel = KNOB_SPEED_L == idx ? Channel::LEFT : Channel::RIGHT;
                StereoLooper::NoteMode noteMode = KNOB_SPEED_L == idx ? looper.noteModeLeft : looper.noteModeRight;

                if (StereoLooper::NoteMode::NOTE == noteMode)
                {
                    // In "note" mode, the rate knob sets the pitch, with 4
                    // octaves span.
                    value = std::floor(Map(value, 0.f, 1.f, -24, 24)) - 24;
                    value = std::pow(2.f, value / 12);
                }
                else if (StereoLooper::NoteMode::FLANGER == noteMode)
                {
                    // In "note" mode, the rate knob sets the pitch, with 4
                    // octaves span.
                    value = Map(value, 0.f, 1.f, -24, 24);
                    value = std::pow(2.f, value / 12);
                }
                else
                {
                    if (value < 0.45f)
                    {
                        value = Map(value, 0.f, 0.45f, kMinSpeedMult, 1.f);
                    }
                    else if (value > 0.55f)
                    {
                        value = Map(value, 0.55f, 1.f, 1.f, kMaxSpeedMult);
                    }
                    // Center dead zone.
                    else
                    {
                        value = Map(value, 0.45f, 0.55f, 1.f, 1.f);
                    }
                }
                looper.SetReadRate(channel, value);
            }
            break;
        case KNOB_FREEZE_L:
        case KNOB_FREEZE_R:
            {
                Channel channel = KNOB_FREEZE_L == idx ? Channel::LEFT : Channel::RIGHT;
                looper.SetFreeze(channel, value);
            }
            break;

        default:
            break;
        }
    }

    inline void ProcessKnob(int idx)
    {
        float value = knobs[idx].Process();
        // Handle range limits.
        if (value < kMinValueDelta)
        {
            value = 0.f;
        }
        else if (value > 1 - kMinValueDelta)
        {
            value = 1.f;
        }

        // Process the parameter only if it actually changed.
        if (std::abs(values[idx] - value) > kMinValueDelta)
        {
            ProcessParameter(idx, value);

            values[idx] = value;
        }
    }

    inline void ProcessUi()
    {
        if (looper.IsStartingUp())
        {
            if (startUp)
            {
                startUp = false;

                // Init parameters.
                values[KNOB_BLEND] = knobs[KNOB_BLEND].Process();
                ProcessParameter(KNOB_BLEND, values[KNOB_BLEND]);
                looper.inputGain = 1.f;
                looper.filterType = StereoLooper::FilterType::BP;
                looper.filterLevel = 0.75f;
                looper.rateSlew = 0.f;
                looper.crossedFeedback = true;
                looper.leftFeedbackPath = 0.f;
                looper.rightFeedbackPath = 1.f;
                looper.SetLoopSync(Channel::LEFT, false);
                looper.SetLoopSync(Channel::RIGHT, true);
                looper.SetDegradation(0.25f);
            }

            return;
        }

        // The looper is buffering.
        if (looper.IsBuffering())
        {
            ProcessKnob(KNOB_LENGTH_L);

            // Stop buffering.
            if (values[KNOB_LENGTH_L] < 1.f)
            {
                looper.mustStopBuffering = true;
            }

            return;
        }

        // The looper is ready, do some configuration before starting.
        if (looper.IsReady())
        {
            // Init all the parameters with the relative knobs position.
            for (size_t i = 0; i < KNOB_LAST; i++)
            {
                values[i] = knobs[i].Process();
                if (values[i] < kMinValueDelta)
                {
                    values[i] = 0.f;
                }
                else if (values[i] > 1 - kMinValueDelta)
                {
                    values[i] = 1.f;
                }
                ProcessParameter(i, values[i]);
            }

            looper.Start();

            return;
        }

        if (looper.IsRunning())
        {
            ProcessKnob(KNOB_BLEND);
            ProcessKnob(KNOB_STEREO);
            ProcessKnob(KNOB_FEEDBACK);
            ProcessKnob(KNOB_FILTER);

            ProcessKnob(KNOB_START_L);
            ProcessKnob(KNOB_LENGTH_L);
            ProcessKnob(KNOB_SPEED_L);
            ProcessKnob(KNOB_FREEZE_L);

            ProcessKnob(KNOB_START_R);
            ProcessKnob(KNOB_LENGTH_R);
            ProcessKnob(KNOB_SPEED_R);
            ProcessKnob(KNOB_FREEZE_R);
        }

        first = false;
    }
}