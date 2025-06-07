#pragma once
#include "Sequencer.h"
#include <vector>

class SequencerManager {
public:
    SequencerManager(size_t numVoices) : voices(numVoices) {}

    Sequencer& getVoice(size_t idx) { return voices.at(idx); }
    const Sequencer& getVoice(size_t idx) const { return voices.at(idx); }
    size_t getNumVoices() const { return voices.size(); }

    // Advance all voices (example: for polyphonic clocking)
    void advanceAll(uint8_t step, int mm_distance,
                    bool button16, bool button17, bool button18,
                    int selectedStepForEdit) {
        for (auto& voice : voices) {
            voice.advanceStep(step, mm_distance, button16, button17, button18, selectedStepForEdit);
        }
    }

    // Add more management functions as needed (e.g., set per-voice outputs, parameters, etc.)

private:
    std::vector<Sequencer> voices;
};