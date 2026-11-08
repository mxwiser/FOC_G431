#pragma once
#include <stdint.h>

// Fixed-storage note sequencer; no HAL, allocation, or clock dependency.
// The caller supplies the sine function and invokes next() once per ADC frame.
namespace music_tone {
const unsigned capacity = 16u;
const float max_amplitude = 2.0f; // A peak, independent of ordinary IQ limits.
const float min_frequency = 80.0f, max_frequency = 1000.0f;
const float two_pi = 6.2831853071795864769f;
struct Note { float step, amplitude; uint32_t samples, edge; };
struct Player {
    Note notes[capacity], current;
    unsigned read, write, count;
    uint32_t position, accepted, completed;
    float phase;
    bool running;
    // Live MIDI tone: a persistent oscillator that bypasses the note queue.
    bool live;
    float live_hz, live_amp, live_current, live_phase;
};

inline bool make_note(float hz, float amps, float ms, float dt, Note &note)
{
    // Ordered comparisons also reject NaN and infinity before integer casts.
    if (!(dt > 0.0f && dt <= 0.001f) ||
        !(amps >= 0.0f && amps <= max_amplitude) ||
        !(ms >= 10.0f && ms <= 2000.0f) ||
        !((hz == 0.0f && amps == 0.0f) ||
          (hz >= min_frequency && hz <= max_frequency && hz * dt <= 1.0f / 12.0f))) return false;
    const float samples = ms * 0.001f / dt;
    if (!(samples >= 2.0f && samples <= 20000000.0f)) return false;
    note.step = two_pi * hz * dt;
    note.amplitude = amps;
    note.samples = static_cast<uint32_t>(samples + 0.5f);
    note.edge = static_cast<uint32_t>(0.005f / dt + 0.5f);
    if (!note.edge) note.edge = 1u;
    if (note.edge > (note.samples - 1u) / 2u) note.edge = (note.samples - 1u) / 2u;
    return true;
}
inline void clear(Player &p)
{
    p.read = p.write = p.count = 0u;
    p.position = 0u; p.phase = 0.0f; p.running = false;
    p.live = false; p.live_hz = p.live_amp = p.live_current = p.live_phase = 0.0f;
    // Keep lifetime ACK counters so a cancelled note cannot look newly accepted.
}
inline bool enqueue(Player &p, const Note &note)
{
    if (p.count == capacity) return false;
    p.notes[p.write] = note;
    p.write = (p.write + 1u) % capacity;
    ++p.count; ++p.accepted;
    return true;
}
inline bool next(Player &p, float &phase, float &amplitude)
{
    if (p.running && p.position == p.current.samples) {
        ++p.completed; p.running = false;
    }
    if (!p.running) {
        if (!p.count) return false;
        p.current = p.notes[p.read];
        p.read = (p.read + 1u) % capacity; --p.count;
        p.position = 0u; p.phase = 0.0f; p.running = true;
    }
    float envelope = 1.0f;
    if (p.position < p.current.edge)
        envelope = static_cast<float>(p.position) / p.current.edge;
    const uint32_t left = p.current.samples - 1u - p.position;
    if (left < p.current.edge) envelope = static_cast<float>(left) / p.current.edge;
    phase = p.phase; amplitude = p.current.amplitude * envelope;
    p.phase += p.current.step;
    if (p.phase >= two_pi) p.phase -= two_pi;
    ++p.position;
    return true;
}

inline bool valid_live(float hz, float amps, float dt)
{
    if (!(dt > 0.0f && dt <= 0.001f)) return false;
    if (!(amps >= 0.0f && amps <= max_amplitude)) return false;
    return (hz == 0.0f && amps == 0.0f) ||
        (hz >= min_frequency && hz <= max_frequency && hz * dt <= 1.0f / 12.0f);
}

inline void set_live(Player &p, float hz, float amps)
{
    if (!p.live) {
        // Switching from the queue to live drops any pending notes.
        p.read = p.write = p.count = 0u;
        p.running = false;
        p.live_phase = 0.0f;
        p.live_current = 0.0f;
    }
    p.live = true;
    if (hz > 0.0f) p.live_hz = hz;
    p.live_amp = amps;
}

// Persistent oscillator for live MIDI. Keeps phase across pitch changes and
// ramps amplitude over ~10 ms so note edges do not click. Returns true even
// while silent so the caller stays in MUSIC until a stop or heartbeat timeout.
inline bool next_live(Player &p, float dt, float &phase, float &amplitude)
{
    if (!p.live) return false;
    const float step = max_amplitude * dt / 0.01f;
    const float error = p.live_amp - p.live_current;
    if (error > step) p.live_current += step;
    else if (error < -step) p.live_current -= step;
    else p.live_current = p.live_amp;
    phase = p.live_phase; amplitude = p.live_current;
    p.live_phase += two_pi * p.live_hz * dt;
    if (p.live_phase >= two_pi) p.live_phase -= two_pi;
    return true;
}
} // namespace music_tone
