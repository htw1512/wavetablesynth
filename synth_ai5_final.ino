#include <Arduino.h>
#include <I2S.h>
#include <MIDI.h>
#include <math.h>

// #define ENABLE_SERIAL_DEBUGGING

// --- Pin Definitionen --- (ALLE WIEDER AKTIV)
const int I2S_BCLK_PIN = 20; const int I2S_LRCLK_PIN = 21; const int I2S_DOUT_PIN = 22;
const int MUX_SIG_PIN = A0; const int MUX_S0_PIN = 2; const int MUX_S1_PIN = 3; const int MUX_S2_PIN = 4; const int MUX_S3_PIN = 5;
const int MUX_CHANNELS = 16; // <<< Lesen bis C15 (PM Amount)
const int MIDI_RX_PIN = 1;
const int LFO_WAVE_SWITCH_PIN = 11; // <<< Aktiv
const int OCTAVE_SWITCH_PIN = 10;   // <<< Aktiv
const int PITCH_BEND_PIN = A1;      // <<< Aktiv
const int MOD_WHEEL_PIN  = A2;      // <<< Aktiv
const int FILTER_TYPE_SWITCH_PIN = 12; // <<< Aktiv

// --- Audio Einstellungen ---
const int SAMPLE_RATE = 44100; const int WAVETABLE_SIZE = 512; const float PI_F = 3.141592653589793f;

// --- Synthesizer Einstellungen --- (MONO + Alle Features + Gain von funktionierender Basis)
const int NUM_VOICES = 1;
const float ADC_MAX_VAL = 4095.0f; const float SMOOTHING_FACTOR = 0.05f; const float ENVELOPE_IDLE_THRESHOLD = 0.001f;
const float LFO_MIN_RATE = 0.1f; const float LFO_MAX_RATE = 25.0f; const float MAX_VIBRATO_SEMITONES = 1.0f;
const float VCF_MIN_CUTOFF = 40.0f; const float VCF_MAX_CUTOFF = 18000.0f; const float VCF_MIN_Q = 0.707f;
const float VCF_MAX_Q = 3.5f; // Dein Wert
const float PITCH_MOD_FACTOR_APPROX = 0.057762265f;
const float NOISE_SCALE_FACTOR = 0.4f; // Dein Wert
const float OSC_MIX_SCALE = 0.20f; // <<< Leicht reduziert wegen PM/Noise Addition
const float PRE_FILTER_SCALE = 0.8f; // Dein Wert
const float FINAL_AMP_SCALE = 0.8f; // <<< Leicht reduziert zur Sicherheit
const float BROWN_NOISE_POLE = 0.985f; const float BROWN_NOISE_GAIN_COMP = 4.5f;
const float DC_BLOCKER_POLE = 0.995f;
const float MIN_ATTACK_TIME = 0.005f; // Dein Wert
const float MAX_ATTACK_TIME_SECONDS = 2.0f; // Dein Wert
const float MAX_PM_AMOUNT_FACTOR = 80.0f; // PM Skalierung
const float DENORMAL_THRESHOLD = 1e-18f;
const float MAX_PITCH_BEND_SEMITONES = 2.0f;
const float MOD_WHEEL_FILTER_MOD_HZ = 3000.0f;

// --- LFO Wellenform Typen ---
enum LfoWaveform { LFO_WAVE_SINE, LFO_WAVE_SQUARE, LFO_WAVE_SAW };
// --- Oktavzustände ---
enum OctaveState { OCTAVE_NORMAL, OCTAVE_MINUS_1, OCTAVE_MINUS_2 };
// --- Filtertypen ---
enum FilterType { FILTER_LP, FILTER_HP, FILTER_BP };

// --- Potentiometer Mapping Konstanten (Alle wieder aktiv) ---
const int POT_OSC_SINE_AMP=0; const int POT_OSC_TRI_AMP=1; const int POT_OSC_SAW_AMP=2; const int POT_OSC_SQUARE_AMP=3;
const int POT_ENV_ATTACK=4; const int POT_ENV_DECAY=5; const int POT_ENV_SUSTAIN=6; const int POT_ENV_RELEASE=7;
const int POT_LFO_DEPTH=8; const int POT_LFO_RATE=9;
const int POT_VCF_CUTOFF=10; const int POT_VCF_RESONANCE=11;
const int POT_NOISE_LEVEL=12;
const int POT_TREMOLO_AMOUNT=13;
const int POT_NOISE_COLOR=14;
const int POT_PM_AMOUNT=15;

// --- ADSR Array Indizes ---
const int ATTACK_IDX=0; const int DECAY_IDX=1; const int SUSTAIN_IDX=2; const int RELEASE_IDX=3;

// --- Wavetables ---
int16_t sineTable[WAVETABLE_SIZE]; int16_t squareTable[WAVETABLE_SIZE]; int16_t sawTable[WAVETABLE_SIZE]; int16_t triangleTable[WAVETABLE_SIZE];

// --- Globale Variablen (Alle wieder aktiv) ---
int potRawValues[MUX_CHANNELS]; float waveAmps[4]; float smoothedWaveAmp0=0.0f, smoothedWaveAmp1=0.0f, smoothedWaveAmp2=0.0f, smoothedWaveAmp3=0.0f; float adsrParams[4];
float currentLfoRate=0.0f, currentLfoDepth=0.0f; float smoothedLfoRate=0.0f, smoothedLfoDepth=0.0f; float lfoPhaseInc=0.0f;
float currentVcfCutoff=0.0f, currentVcfResonance=0.0f; float smoothedVcfCutoff=0.0f, smoothedVcfResonance=0.0f;
float filter_b0=1.0f, filter_b1=0.0f, filter_b2=0.0f; float filter_a1=0.0f, filter_a2=0.0f;
float currentNoiseLevel=0.0f; float smoothedNoiseLevel=0.0f;
float currentTremoloAmount=0.0f; float smoothedTremoloAmount=0.0f;
float currentNoiseColor=0.0f; float smoothedNoiseColor=0.0f;
float currentPMAmount=0.0f; float smoothedPMAmount=0.0f;
float dcBlockerLastIn=0.0f; float dcBlockerLastOut=0.0f; float brownNoiseFilterState=0.0f;
LfoWaveform selectedLfoWaveform = LFO_WAVE_SINE;
OctaveState selectedOctaveState = OCTAVE_NORMAL;
FilterType selectedFilterType = FILTER_LP;
bool lastOctaveSwitchState = HIGH; bool lastLfoWaveSwitchState = HIGH; bool lastFilterTypeSwitchState = HIGH;
unsigned long lastOctaveDebounceTime = 0; unsigned long lastLfoDebounceTime = 0; unsigned long lastFilterTypeDebounceTime = 0;
float currentPitchBend = 0.0f; float smoothedPitchBend = 0.0f;
float currentModWheel = 0.0f; float smoothedModWheel = 0.0f;
unsigned long lastPotReadTime = 0;

// --- Voice Struktur ---
enum EnvelopeState { IDLE, ATTACK, DECAY, SUSTAIN, RELEASE };
struct Voice { bool active; int note; float frequency; float phase; float lfoPhase; EnvelopeState envState; float envLevel; float releaseStartLevel; unsigned long envStartTime; float filter_x1, filter_x2, filter_y1, filter_y2; };
Voice voice;

// --- I2S & MIDI Objekte ---
I2S i2s(OUTPUT, I2S_BCLK_PIN, I2S_DOUT_PIN, I2S_LRCLK_PIN); MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

// --- Forward Declarations ---
float midiToFreq(byte note); void updateEnvelope(Voice& v); void processAndWriteAudio(); void readControls();
int16_t getInterpolatedSample(const int16_t* table, float phase); float getLfoValue(float lfoPhase); void calculateFilterCoefficients(float cutoffHz, float Q); void selectMuxChannel(byte channel); void handleNoteOn(byte channel, byte note, byte velocity); void handleNoteOff(byte channel, byte note, byte velocity); void generateWavetables(); float applyDCBlocker(float sample); void resetVoice(Voice& v); void updateOctaveState(); void updateLfoWaveformState(); void updateFilterTypeState();

// --- Debounce Zeit Konstante ---
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long POT_READ_INTERVAL_MS = 20;

// --- Helper Funktionen Implementierung ---

void selectMuxChannel(byte channel) {
  digitalWrite(MUX_S0_PIN, bitRead(channel, 0)); digitalWrite(MUX_S1_PIN, bitRead(channel, 1));
  digitalWrite(MUX_S2_PIN, bitRead(channel, 2)); digitalWrite(MUX_S3_PIN, bitRead(channel, 3));
  delayMicroseconds(5);
}

int16_t getInterpolatedSample(const int16_t* table, float phase) {
  phase = fmodf(phase, (float)WAVETABLE_SIZE); if (phase < 0.0f) phase += WAVETABLE_SIZE;
  int index1 = (int)floorf(phase); int index2 = (index1 + 1) % WAVETABLE_SIZE;
  float frac = phase - floorf(phase); int16_t val1 = table[index1]; int16_t val2 = table[index2];
  return (int16_t)(val1 + (val2 - val1) * frac);
}

float getLfoValue(float lfoPhase) {
  const int16_t* currentTable;
  switch (selectedLfoWaveform) {
    case LFO_WAVE_SQUARE: currentTable = squareTable; break;
    case LFO_WAVE_SAW: currentTable = sawTable; break;
    case LFO_WAVE_SINE: default: currentTable = sineTable; break;
  }
  int16_t sample = getInterpolatedSample(currentTable, lfoPhase);
  return (float)sample / 32767.0f;
}

void calculateFilterCoefficients(float cutoffHz, float Q) {
  cutoffHz = max(10.0f, min(cutoffHz, (float)SAMPLE_RATE * 0.49f)); Q = max(0.5f, min(Q, VCF_MAX_Q));
  float w0=2.0f*PI_F*cutoffHz/(float)SAMPLE_RATE; float cos_w0=cosf(w0); float sin_w0=sinf(w0); float alpha=sin_w0/(2.0f*Q);
  float a0_raw=1.0f+alpha; float a1_raw=-2.0f*cos_w0; float a2_raw=1.0f-alpha;
  float b0_raw, b1_raw, b2_raw;
  switch (selectedFilterType) {
    case FILTER_HP: b0_raw=(1.0f+cos_w0)/2.0f; b1_raw=-(1.0f+cos_w0); b2_raw=(1.0f+cos_w0)/2.0f; break;
    case FILTER_BP: b0_raw=alpha; b1_raw=0.0f; b2_raw=-alpha; break;
    case FILTER_LP: default: b0_raw=(1.0f-cos_w0)/2.0f; b1_raw=1.0f-cos_w0; b2_raw=(1.0f-cos_w0)/2.0f; break;
  }
  if(fabsf(a0_raw)<1e-9f) a0_raw=1e-9f; float inv_a0=1.0f/a0_raw;
  filter_b0=b0_raw*inv_a0; filter_b1=b1_raw*inv_a0; filter_b2=b2_raw*inv_a0; filter_a1=a1_raw*inv_a0; filter_a2=a2_raw*inv_a0;
}

// --- readControls liest jetzt alle 16 Potis + A1/A2 ---
void readControls() {
  for(int i=0; i<MUX_CHANNELS; i++) { // Liest 0-15
      selectMuxChannel(i); delayMicroseconds(10); potRawValues[i]=analogRead(MUX_SIG_PIN);
  }
  waveAmps[0]=potRawValues[POT_OSC_SINE_AMP]/ADC_MAX_VAL; waveAmps[1]=potRawValues[POT_OSC_TRI_AMP]/ADC_MAX_VAL; waveAmps[2]=potRawValues[POT_OSC_SAW_AMP]/ADC_MAX_VAL; waveAmps[3]=potRawValues[POT_OSC_SQUARE_AMP]/ADC_MAX_VAL;
  adsrParams[ATTACK_IDX] = MIN_ATTACK_TIME + (potRawValues[POT_ENV_ATTACK] / ADC_MAX_VAL) * MAX_ATTACK_TIME_SECONDS; // Linear Attack
  adsrParams[DECAY_IDX]=powf(potRawValues[POT_ENV_DECAY]/ADC_MAX_VAL,2.0f)*5.0f;
  adsrParams[SUSTAIN_IDX]=potRawValues[POT_ENV_SUSTAIN]/ADC_MAX_VAL;
  adsrParams[RELEASE_IDX]=powf(potRawValues[POT_ENV_RELEASE]/ADC_MAX_VAL,2.0f)*5.0f;
  currentLfoDepth=potRawValues[POT_LFO_DEPTH]/ADC_MAX_VAL;
  float rawLfoRateNorm=potRawValues[POT_LFO_RATE]/ADC_MAX_VAL; currentLfoRate=LFO_MIN_RATE+rawLfoRateNorm*(LFO_MAX_RATE-LFO_MIN_RATE);
  float rawCutoffNorm = potRawValues[POT_VCF_CUTOFF] / ADC_MAX_VAL;
  float rawResonanceNorm = potRawValues[POT_VCF_RESONANCE] / ADC_MAX_VAL;
  currentVcfCutoff = VCF_MIN_CUTOFF * powf(VCF_MAX_CUTOFF / VCF_MIN_CUTOFF, rawCutoffNorm);
  currentVcfResonance = VCF_MIN_Q + rawResonanceNorm * (VCF_MAX_Q - VCF_MIN_Q);
  currentNoiseLevel = potRawValues[POT_NOISE_LEVEL] / ADC_MAX_VAL;
  currentTremoloAmount = potRawValues[POT_TREMOLO_AMOUNT] / ADC_MAX_VAL;
  currentNoiseColor = potRawValues[POT_NOISE_COLOR] / ADC_MAX_VAL;
  currentPMAmount = (potRawValues[POT_PM_AMOUNT] / ADC_MAX_VAL) * MAX_PM_AMOUNT_FACTOR; // <<< Wieder aktiv
  int rawPitchBend = analogRead(PITCH_BEND_PIN); int rawModWheel = analogRead(MOD_WHEEL_PIN);
  currentPitchBend = ((float)rawPitchBend / ADC_MAX_VAL - 0.5f) * 2.0f;
  currentModWheel = (float)rawModWheel / ADC_MAX_VAL;
}

float midiToFreq(byte note) { if(note>127)return 0.0f; return 440.0f*powf(2.0f,(note-69.0f)/12.0f); }

void updateEnvelope(Voice& v) {
    if(v.envState==IDLE){if(v.envLevel>0.0f)v.envLevel=0.0f;return;} unsigned long cT=micros(); float dT=(cT-v.envStartTime)*1.0e-6f; if(dT<0)dT=0;
    switch(v.envState){case ATTACK:{float aT=adsrParams[ATTACK_IDX]; if(aT>1e-6f)v.envLevel=min(1.0f,dT/aT); else v.envLevel=1.0f; if(v.envLevel>=1.0f){v.envLevel=1.0f;v.envState=DECAY;v.envStartTime=cT;} break;} case DECAY:{float dTi=adsrParams[DECAY_IDX],sL=adsrParams[SUSTAIN_IDX]; if(dTi>0.001f)v.envLevel=sL+(1.0f-sL)*(1.0f-min(1.0f,dT/dTi)); else v.envLevel=sL; if(v.envLevel<=(sL+1e-5f)){v.envLevel=sL;v.envState=SUSTAIN; if(sL<=ENVELOPE_IDLE_THRESHOLD)resetVoice(v);} break;} case SUSTAIN:{if(adsrParams[SUSTAIN_IDX]<=ENVELOPE_IDLE_THRESHOLD) resetVoice(v); else v.envLevel=adsrParams[SUSTAIN_IDX]; break;} case RELEASE:{float rT=adsrParams[RELEASE_IDX],sL=v.releaseStartLevel; if(rT>0.001f){float p=min(1.0f,dT/rT);v.envLevel=sL*(1.0f-p);} else v.envLevel=0.0f; v.envLevel=max(0.0f,v.envLevel); if(v.envLevel<=ENVELOPE_IDLE_THRESHOLD) resetVoice(v); break;} case IDLE: if(v.envLevel>0.0f)v.envLevel=0.0f; v.active=false; break;}
    if (fabsf(v.envLevel) < DENORMAL_THRESHOLD) { v.envLevel = 0.0f; if (v.envState != IDLE && v.envState != ATTACK) { v.envState = IDLE; v.active = false; } }
}

float applyDCBlocker(float sample) {
    float fO=sample-dcBlockerLastIn+DC_BLOCKER_POLE*dcBlockerLastOut; dcBlockerLastIn=sample;
    if(isnan(fO)||isinf(fO)){fO=dcBlockerLastOut;dcBlockerLastIn=0.0f;dcBlockerLastOut=0.0f;} else {dcBlockerLastOut=fO;} return fO;
}

void updateOctaveState() {
    bool currentSwitchState = (digitalRead(OCTAVE_SWITCH_PIN) == LOW);
    if (currentSwitchState && !lastOctaveSwitchState) {
        switch (selectedOctaveState) {
            case OCTAVE_NORMAL: selectedOctaveState = OCTAVE_MINUS_1; break;
            case OCTAVE_MINUS_1: selectedOctaveState = OCTAVE_MINUS_2; break;
            case OCTAVE_MINUS_2: selectedOctaveState = OCTAVE_NORMAL; break;
        }
    }
    lastOctaveSwitchState = currentSwitchState;
}

void updateLfoWaveformState() {
    unsigned long currentTime = millis();
    if (currentTime - lastLfoDebounceTime > DEBOUNCE_DELAY) {
        bool currentSwitchState = (digitalRead(LFO_WAVE_SWITCH_PIN) == LOW);
        if (currentSwitchState && !lastLfoWaveSwitchState) {
            switch (selectedLfoWaveform) {
                case LFO_WAVE_SINE:   selectedLfoWaveform = LFO_WAVE_SQUARE; break;
                case LFO_WAVE_SQUARE: selectedLfoWaveform = LFO_WAVE_SAW;    break;
                case LFO_WAVE_SAW:    selectedLfoWaveform = LFO_WAVE_SINE;   break;
            }
            lastLfoDebounceTime = currentTime;
        }
        lastLfoWaveSwitchState = currentSwitchState;
    }
}

void updateFilterTypeState() {
    unsigned long currentTime = millis();
    if (currentTime - lastFilterTypeDebounceTime > DEBOUNCE_DELAY) {
        bool currentSwitchState = (digitalRead(FILTER_TYPE_SWITCH_PIN) == LOW);
        if (currentSwitchState && !lastFilterTypeSwitchState) {
            switch (selectedFilterType) {
                case FILTER_LP: selectedFilterType = FILTER_HP; break;
                case FILTER_HP: selectedFilterType = FILTER_BP; break;
                case FILTER_BP: selectedFilterType = FILTER_LP; break;
            }
            calculateFilterCoefficients(smoothedVcfCutoff, smoothedVcfResonance);
            lastFilterTypeDebounceTime = currentTime;
        }
        lastFilterTypeSwitchState = currentSwitchState;
    }
}

// --- processAndWriteAudio (MONO - Alle Features aktiv) ---
void processAndWriteAudio() {
    float voiceOutput = 0.0f;
    if (voice.envState != IDLE) {
        updateEnvelope(voice);
        if (voice.envState != IDLE && voice.envLevel > ENVELOPE_IDLE_THRESHOLD) {
            float octaveMultiplier = 1.0f;
            if (selectedOctaveState == OCTAVE_MINUS_1) { octaveMultiplier = 0.5f; } else if (selectedOctaveState == OCTAVE_MINUS_2) { octaveMultiplier = 0.25f; }
            float effectiveFrequency = voice.frequency * octaveMultiplier;
            float bendSemitones = smoothedPitchBend * MAX_PITCH_BEND_SEMITONES;
            float pitchBendFactor = 1.0f + PITCH_MOD_FACTOR_APPROX * bendSemitones;
            float frequencyAfterBend = effectiveFrequency * pitchBendFactor;
            float lfoValue = getLfoValue(voice.lfoPhase);
            float finalLfoDepth = smoothedLfoDepth * smoothedModWheel;
            float pitchModSemitones = lfoValue * finalLfoDepth * MAX_VIBRATO_SEMITONES;
            float pitchModFactor = 1.0f + PITCH_MOD_FACTOR_APPROX * pitchModSemitones;
            float modulatedFreq = frequencyAfterBend * pitchModFactor;
            float currentPhaseInc = modulatedFreq * WAVETABLE_SIZE / SAMPLE_RATE;

            // --- Phasenmodulation wieder aktiv ---
            float phaseOffset = 0.0f;
            if (smoothedPMAmount > 0.01f) { // <<< Verwendet smoothedPMAmount
                 phaseOffset = lfoValue * smoothedPMAmount;
            }
            float lookupPhase = voice.phase + phaseOffset; // <<< Modulierte Phase

            // Oszillator Mix
            float oscSample = 0.0f;
            if (smoothedWaveAmp0 > 0.001f) oscSample += getInterpolatedSample(sineTable, lookupPhase) * smoothedWaveAmp0;
            if (smoothedWaveAmp1 > 0.001f) oscSample += getInterpolatedSample(triangleTable, lookupPhase) * smoothedWaveAmp1;
            if (smoothedWaveAmp2 > 0.001f) oscSample += getInterpolatedSample(sawTable, lookupPhase) * smoothedWaveAmp2;
            if (smoothedWaveAmp3 > 0.001f) oscSample += getInterpolatedSample(squareTable, lookupPhase) * smoothedWaveAmp3;
            oscSample *= OSC_MIX_SCALE; // <<< 0.18f

            // Noise mit Farbwechsel
            float noiseSample = 0.0f;
            if (smoothedNoiseLevel > 0.001f) {
                 float whiteNoise=(float)random(-32768,32767)/32767.0f;
                 float brownNoiseRaw=BROWN_NOISE_POLE*brownNoiseFilterState+whiteNoise*(1.0f-BROWN_NOISE_POLE);
                 brownNoiseFilterState=brownNoiseRaw; float brownNoise=brownNoiseRaw*BROWN_NOISE_GAIN_COMP;
                 if(smoothedNoiseColor < 0.5f){ noiseSample=whiteNoise*32767.0f; } else { noiseSample=brownNoise*32767.0f; }
                 noiseSample*=smoothedNoiseLevel*NOISE_SCALE_FACTOR; // <<< 0.3f
             }
            float sampleBeforeDC = oscSample + noiseSample;
            float sampleBlocked = applyDCBlocker(sampleBeforeDC);
            float voiceSamplePreFilter = sampleBlocked * PRE_FILTER_SCALE; // <<< 0.7f

            // Filter
            float x0=voiceSamplePreFilter;
            float y0=filter_b0*x0+filter_b1*voice.filter_x1+filter_b2*voice.filter_x2-filter_a1*voice.filter_y1-filter_a2*voice.filter_y2; // Max Q = 2.5f
            voice.filter_x2=voice.filter_x1; voice.filter_x1=x0; voice.filter_y2=voice.filter_y1;
            if(isnan(y0)||isinf(y0)){y0=voice.filter_y2;voice.filter_x1=0.0f;voice.filter_x2=0.0f;voice.filter_y1=0.0f;voice.filter_y2=0.0f;}else{voice.filter_y1=y0;}
            float voiceSampleFiltered=y0;

            // Tremolo
            float voiceSampleTremolo;
            if (smoothedTremoloAmount > 0.001f) {
                float lfoNorm01=lfoValue*0.5f+0.5f; float tremoloFactor=1.0f-smoothedTremoloAmount*(1.0f-lfoNorm01);
                voiceSampleTremolo=voiceSampleFiltered*tremoloFactor;
            } else {voiceSampleTremolo=voiceSampleFiltered;}

            float voiceSampleFinal = voiceSampleTremolo * voice.envLevel * FINAL_AMP_SCALE; // <<< 0.7f
            if (isnan(voiceSampleFinal)||isinf(voiceSampleFinal)) voiceSampleFinal = 0.0f;
            voiceOutput = voiceSampleFinal;

            voice.phase += currentPhaseInc; voice.phase=fmodf(voice.phase,(float)WAVETABLE_SIZE); if(voice.phase<0.0f)voice.phase+=WAVETABLE_SIZE;
            voice.lfoPhase += lfoPhaseInc; voice.lfoPhase=fmodf(voice.lfoPhase,(float)WAVETABLE_SIZE); if(voice.lfoPhase<0.0f)voice.lfoPhase+=WAVETABLE_SIZE;
        }
    }

    // Nachbearbeitung
    if (isnan(voiceOutput)||isinf(voiceOutput)) voiceOutput = 0.0f;
    const float softClipAmount = 32767.0f * 1.0f;
    float finalSampleSoftClipped = tanhf(voiceOutput / softClipAmount) * softClipAmount;
    if(isnan(finalSampleSoftClipped)){finalSampleSoftClipped=(voiceOutput>0.0f)?32767.0f:-32768.0f;}
    float finalSampleClipped = max(-32768.0f, min(32767.0f, finalSampleSoftClipped));
    int16_t finalSampleInt16 = (int16_t)finalSampleClipped;
    int32_t packedSample = ((int32_t)finalSampleInt16 << 16) | (finalSampleInt16 & 0xFFFF);
    i2s.write(packedSample);
}

void resetVoice(Voice& v) {
    v.active=false; v.note=-1; v.frequency=0.0f; v.phase=0.0f; v.lfoPhase=0.0f;
    v.envState=IDLE; v.envLevel=0.0f; v.releaseStartLevel=0.0f; v.envStartTime=0;
    v.filter_x1=0.0f; v.filter_x2=0.0f; v.filter_y1=0.0f; v.filter_y2=0.0f;
}

void handleNoteOn(byte channel, byte note, byte velocity) {
    resetVoice(voice);
    voice.active=true; voice.note=note; voice.frequency=midiToFreq(note);
    voice.envState=ATTACK; voice.envStartTime=micros(); voice.envLevel=0.0f;
}

void handleNoteOff(byte channel, byte note, byte velocity) {
    if(voice.active && voice.note==note && voice.envState!=RELEASE && voice.envState!=IDLE){
        voice.releaseStartLevel=voice.envLevel; voice.envState=RELEASE;
        voice.envStartTime=micros(); voice.active=false;
    }
}

void generateWavetables() {
    for(int i=0;i<WAVETABLE_SIZE;++i){float ph=(float)i/WAVETABLE_SIZE;float ang=ph*2.0f*PI_F;sineTable[i]=(int16_t)(sinf(ang)*32767.0f);squareTable[i]=(ph<0.5f)?32767:-32768;sawTable[i]=(int16_t)((1.0f-2.0f*ph)*32767.0f);triangleTable[i]=(int16_t)((2.0f*fabsf(2.0f*(ph-floorf(ph+0.5f)))-1.0f)*32767.0f);}
}

void setup() {
    // Serial.begin(115200); Serial.println("Synth Setup - MONO - All Features Active v3 (Corrected)");
    pinMode(LFO_WAVE_SWITCH_PIN, INPUT_PULLUP); pinMode(OCTAVE_SWITCH_PIN, INPUT_PULLUP);
    pinMode(PITCH_BEND_PIN, INPUT); pinMode(MOD_WHEEL_PIN, INPUT);
    pinMode(FILTER_TYPE_SWITCH_PIN, INPUT_PULLUP);

    randomSeed(analogRead(A1));
    pinMode(MUX_S0_PIN,OUTPUT); pinMode(MUX_S1_PIN,OUTPUT); pinMode(MUX_S2_PIN,OUTPUT); pinMode(MUX_S3_PIN,OUTPUT);
    analogReadResolution(12); generateWavetables();
    resetVoice(voice);
    dcBlockerLastIn=0.0f; dcBlockerLastOut=0.0f; brownNoiseFilterState=0.0f;
    Serial1.setRX(MIDI_RX_PIN); MIDI.begin(MIDI_CHANNEL_OMNI);
    MIDI.setHandleNoteOn(handleNoteOn); MIDI.setHandleNoteOff(handleNoteOff); Serial1.begin(31250);
    if (!i2s.begin(SAMPLE_RATE)) { while(1); }
    readControls(); // Liest jetzt alle 16 Potis + A1/A2
    lastOctaveSwitchState = (digitalRead(OCTAVE_SWITCH_PIN) == LOW);
    lastLfoWaveSwitchState = (digitalRead(LFO_WAVE_SWITCH_PIN) == LOW);
    lastFilterTypeSwitchState = (digitalRead(FILTER_TYPE_SWITCH_PIN) == LOW);

    // Alle smoothed Werte initialisieren
    smoothedWaveAmp0=waveAmps[0]; smoothedWaveAmp1=waveAmps[1]; smoothedWaveAmp2=waveAmps[2]; smoothedWaveAmp3=waveAmps[3];
    smoothedLfoRate=currentLfoRate; smoothedLfoDepth=currentLfoDepth;
    smoothedVcfCutoff=currentVcfCutoff; smoothedVcfResonance=currentVcfResonance;
    smoothedNoiseLevel=currentNoiseLevel;
    smoothedTremoloAmount=currentTremoloAmount;
    smoothedNoiseColor=currentNoiseColor;
    smoothedPMAmount = currentPMAmount; // <<< Initialisiert PM
    smoothedPitchBend = currentPitchBend; smoothedModWheel = currentModWheel;
    lfoPhaseInc = smoothedLfoRate * WAVETABLE_SIZE / SAMPLE_RATE;
    calculateFilterCoefficients(smoothedVcfCutoff, smoothedVcfResonance);
    updateFilterTypeState();
    // Serial.println("Setup Complete");
}

void loop() {
    MIDI.read();
    unsigned long currentTime = millis();

    // Taster lesen
    if (currentTime - lastOctaveDebounceTime > DEBOUNCE_DELAY) { updateOctaveState(); lastOctaveDebounceTime = currentTime; }
    updateLfoWaveformState();
    updateFilterTypeState();

    // Potis und Analog Controller lesen
    if (currentTime - lastPotReadTime >= POT_READ_INTERVAL_MS) {
        lastPotReadTime = currentTime;
        readControls(); // Liest jetzt C0-C15 + A1/A2

        // Glätten
        smoothedWaveAmp0=smoothedWaveAmp0*(1.0f-SMOOTHING_FACTOR)+waveAmps[0]*SMOOTHING_FACTOR; smoothedWaveAmp1=smoothedWaveAmp1*(1.0f-SMOOTHING_FACTOR)+waveAmps[1]*SMOOTHING_FACTOR; smoothedWaveAmp2=smoothedWaveAmp2*(1.0f-SMOOTHING_FACTOR)+waveAmps[2]*SMOOTHING_FACTOR; smoothedWaveAmp3=smoothedWaveAmp3*(1.0f-SMOOTHING_FACTOR)+waveAmps[3]*SMOOTHING_FACTOR;
        smoothedLfoRate = smoothedLfoRate*(1.0f-SMOOTHING_FACTOR)+currentLfoRate*SMOOTHING_FACTOR; smoothedLfoDepth = smoothedLfoDepth*(1.0f-SMOOTHING_FACTOR)+currentLfoDepth*SMOOTHING_FACTOR;
        smoothedVcfCutoff = smoothedVcfCutoff*(1.0f-SMOOTHING_FACTOR)+currentVcfCutoff*SMOOTHING_FACTOR; smoothedVcfResonance = smoothedVcfResonance*(1.0f-SMOOTHING_FACTOR)+currentVcfResonance*SMOOTHING_FACTOR;
        smoothedNoiseLevel = smoothedNoiseLevel*(1.0f-SMOOTHING_FACTOR)+currentNoiseLevel*SMOOTHING_FACTOR;
        smoothedTremoloAmount = smoothedTremoloAmount*(1.0f-SMOOTHING_FACTOR)+currentTremoloAmount*SMOOTHING_FACTOR;
        smoothedNoiseColor = smoothedNoiseColor*(1.0f-SMOOTHING_FACTOR)+currentNoiseColor*SMOOTHING_FACTOR;
        smoothedPMAmount = smoothedPMAmount*(1.0f-SMOOTHING_FACTOR)+currentPMAmount*SMOOTHING_FACTOR; // <<< NEU
        smoothedPitchBend = smoothedPitchBend*(1.0f-SMOOTHING_FACTOR)+currentPitchBend*SMOOTHING_FACTOR;
        smoothedModWheel = smoothedModWheel*(1.0f-SMOOTHING_FACTOR)+currentModWheel*SMOOTHING_FACTOR;

        // Neuberechnung
        lfoPhaseInc = smoothedLfoRate * WAVETABLE_SIZE / SAMPLE_RATE;
        float modWheelFilterOffset = smoothedModWheel * MOD_WHEEL_FILTER_MOD_HZ;
        float finalCutoff = smoothedVcfCutoff + modWheelFilterOffset;
        finalCutoff = max(VCF_MIN_CUTOFF, min(VCF_MAX_CUTOFF, finalCutoff));
        calculateFilterCoefficients(finalCutoff, smoothedVcfResonance);
    }
    processAndWriteAudio();
}