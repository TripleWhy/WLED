/*
  WS2812FX.cpp contains all effect methods
  Harm Aldick - 2016
  www.aldick.org

  Copyright (c) 2016  Harm Aldick
  Licensed under the EUPL v. 1.2 or later
  Adapted from code originally licensed under the MIT license

  Modified heavily for WLED
*/

#include "wled.h"
#include "FX.h"
#include "fcn_declare.h"
#include <memory>

#include "effects/Akemi2dEffect.h"
#include "effects/AndroidEffect.h"
#include "effects/AuroraEffect.h"
#include "effects/BlackHole2dEffect.h"
#include "effects/BlendsEffect.h"
#include "effects/BlinkEffect.h"
#include "effects/BlinkRainbowEffect.h"
#include "effects/BlurzEffect.h"
#include "effects/BouncingBallsEffect.h"
#include "effects/BpmEffect.h"
#include "effects/BreathEffect.h"
#include "effects/CandleEffect.h"
#include "effects/ChaseColorEffect.h"
#include "effects/ChaseFlashEffect.h"
#include "effects/ChaseFlashRandomEffect.h"
#include "effects/ChaseRainbowEffect.h"
#include "effects/ChaseRainbowWhiteEffect.h"
#include "effects/ChaseRandomEffect.h"
#include "effects/ChunchunEffect.h"
#include "effects/ColoredBursts2dEffect.h"
#include "effects/ColorfulEffect.h"
#include "effects/ColorSweepEffect.h"
#include "effects/ColorSweepRandomEffect.h"
#include "effects/ColortwinkleEffect.h"
#include "effects/ColorwavesEffect.h"
#include "effects/ColorWipeEffect.h"
#include "effects/ColorWipeRandomEffect.h"
#include "effects/CometEffect.h"
#include "effects/Crazybees2dEffect.h"
#include "effects/DancingShadowsEffect.h"
#include "effects/DissolveEffect.h"
#include "effects/Distortionwaves2dEffect.h"
#include "effects/DjLightEffect.h"
#include "effects/Dna2dEffect.h"
#include "effects/DnaSpiral2dEffect.h"
#include "effects/Drift2dEffect.h"
#include "effects/Driftrose2dEffect.h"
#include "effects/DripEffect.h"
#include "effects/DynamicEffect.h"
#include "effects/effectUtils.h"
#include "effects/ExplodingFireworksEffect.h"
#include "effects/FadeEffect.h"
#include "effects/FairyEffect.h"
#include "effects/FairytwinkleEffect.h"
#include "effects/Fillnoise8Effect.h"
#include "effects/Fire2012Effect.h"
#include "effects/FireFlickerEffect.h"
#include "effects/Firenoise2dEffect.h"
#include "effects/FireworksEffect.h"
#include "effects/FlashSparkleEffect.h"
#include "effects/Floatingblobs2dEffect.h"
#include "effects/FlowEffect.h"
#include "effects/FlowStripeEffect.h"
#include "effects/FreqmapEffect.h"
#include "effects/FreqmatrixEffect.h"
#include "effects/FreqpixelsEffect.h"
#include "effects/FreqwaveEffect.h"
#include "effects/Frizzles2dEffect.h"
#include "effects/FunkyPlank2dEffect.h"
#include "effects/Gameoflife2dEffect.h"
#include "effects/Geq2dEffect.h"
#include "effects/Ghostrider2dEffect.h"
#include "effects/GlitterEffect.h"
#include "effects/GradientEffect.h"
#include "effects/GravcenterEffect.h"
#include "effects/GravcentricEffect.h"
#include "effects/GravfreqEffect.h"
#include "effects/GravimeterEffect.h"
#include "effects/HalloweenEyesEffect.h"
#include "effects/HeartbeatEffect.h"
#include "effects/Hiphotic2dEffect.h"
#include "effects/HyperSparkleEffect.h"
#include "effects/IcuEffect.h"
#include "effects/ImageEffect.h"
#include "effects/JuggleEffect.h"
#include "effects/JugglesEffect.h"
#include "effects/Julia2dEffect.h"
#include "effects/LakeEffect.h"
#include "effects/LarsonScannerEffect.h"
#include "effects/LightningEffect.h"
#include "effects/Lissajous2dEffect.h"
#include "effects/LoadingEffect.h"
#include "effects/MatripixEffect.h"
#include "effects/Matrix2dEffect.h"
#include "effects/Metaballs2dEffect.h"
#include "effects/MeteorEffect.h"
#include "effects/MidnoiseEffect.h"
#include "effects/MultiCometEffect.h"
#include "effects/MultiStrobeEffect.h"
#include "effects/Noise161Effect.h"
#include "effects/Noise162Effect.h"
#include "effects/Noise163Effect.h"
#include "effects/Noise164Effect.h"
#include "effects/Noise2dEffect.h"
#include "effects/NoisefireEffect.h"
#include "effects/NoisemeterEffect.h"
#include "effects/NoisemoveEffect.h"
#include "effects/NoisepalEffect.h"
#include "effects/Octopus2dEffect.h"
#include "effects/OscillateEffect.h"
#include "effects/PacificaEffect.h"
#include "effects/PaletteEffect.h"
#include "effects/particleEffects1d/Particle1dGeqEffect.h"
#include "effects/particleEffects1d/Particle1dsonicstreamEffect.h"
#include "effects/particleEffects1d/Particle1dsprayEffect.h"
#include "effects/particleEffects1d/ParticlebalanceEffect.h"
#include "effects/particleEffects1d/ParticlechaseEffect.h"
#include "effects/particleEffects1d/ParticledancingshadowsEffect.h"
#include "effects/particleEffects1d/ParticledripEffect.h"
#include "effects/particleEffects1d/Particlefire1dEffect.h"
#include "effects/particleEffects1d/Particlefireworks1dEffect.h"
#include "effects/particleEffects1d/ParticlehourglassEffect.h"
#include "effects/particleEffects1d/ParticlepinballEffect.h"
#include "effects/particleEffects1d/ParticlesparklerEffect.h"
#include "effects/particleEffects1d/ParticlestarburstEffect.h"
#include "effects/particleEffects2d/ParticleattractorEffect.h"
#include "effects/particleEffects2d/ParticleblobsEffect.h"
#include "effects/particleEffects2d/ParticleboxEffect.h"
#include "effects/particleEffects2d/ParticlecentergeqEffect.h"
#include "effects/particleEffects2d/ParticlefireEffect.h"
#include "effects/particleEffects2d/ParticlefireworksEffect.h"
#include "effects/particleEffects2d/ParticlegeqEffect.h"
#include "effects/particleEffects2d/ParticleghostriderEffect.h"
#include "effects/particleEffects2d/ParticleimpactEffect.h"
#include "effects/particleEffects2d/ParticleperlinEffect.h"
#include "effects/particleEffects2d/ParticlepitEffect.h"
#include "effects/particleEffects2d/ParticlesprayEffect.h"
#include "effects/particleEffects2d/ParticlevolcanoEffect.h"
#include "effects/particleEffects2d/ParticlevortexEffect.h"
#include "effects/particleEffects2d/ParticlewaterfallEffect.h"
#include "effects/PercentEffect.h"
#include "effects/PerlinmoveEffect.h"
#include "effects/PhasedEffect.h"
#include "effects/PhasedNoiseEffect.h"
#include "effects/PixelsEffect.h"
#include "effects/PixelwaveEffect.h"
#include "effects/Plasmaball2dEffect.h"
#include "effects/PlasmaEffect.h"
#include "effects/Plasmarotozoom2dEffect.h"
#include "effects/PlasmoidEffect.h"
#include "effects/PolarLights2dEffect.h"
#include "effects/PopcornEffect.h"
#include "effects/Pride2015Effect.h"
#include "effects/PuddlepeakEffect.h"
#include "effects/PuddlesEffect.h"
#include "effects/Pulser2dEffect.h"
#include "effects/RailwayEffect.h"
#include "effects/RainbowCycleEffect.h"
#include "effects/RainbowEffect.h"
#include "effects/RainEffect.h"
#include "effects/RandomChaseEffect.h"
#include "effects/RandomColorEffect.h"
#include "effects/RippleEffect.h"
#include "effects/RipplepeakEffect.h"
#include "effects/RocktavesEffect.h"
#include "effects/RollingBallsEffect.h"
#include "effects/RunningLightsEffect.h"
#include "effects/RunningRandomEffect.h"
#include "effects/ScanEffect.h"
#include "effects/Scrollingtext2dEffect.h"
#include "effects/Sindots2dEffect.h"
#include "effects/SinelonEffect.h"
#include "effects/SinewaveEffect.h"
#include "effects/Soap2dEffect.h"
#include "effects/Spaceships2dEffect.h"
#include "effects/SparkleEffect.h"
#include "effects/SpotsEffect.h"
#include "effects/SpotsFadeEffect.h"
#include "effects/Squaredswirl2dEffect.h"
#include "effects/StarburstEffect.h"
#include "effects/StaticEffect.h"
#include "effects/StaticPatternEffect.h"
#include "effects/StrobeEffect.h"
#include "effects/StrobeRainbowEffect.h"
#include "effects/Sunradiation2dEffect.h"
#include "effects/SunriseEffect.h"
#include "effects/Swirl2dEffect.h"
#include "effects/Tartan2dEffect.h"
#include "effects/TetrixEffect.h"
#include "effects/TheaterChaseEffect.h"
#include "effects/TrafficLightEffect.h"
#include "effects/TricolorChaseEffect.h"
#include "effects/TricolorFadeEffect.h"
#include "effects/TricolorWipeEffect.h"
#include "effects/TriStaticPatternEffect.h"
#include "effects/TvSimulatorEffect.h"
#include "effects/TwinklecatEffect.h"
#include "effects/TwinkleEffect.h"
#include "effects/TwinklefoxEffect.h"
#include "effects/TwinkleupEffect.h"
#include "effects/TwoDotsEffect.h"
#include "effects/WashingMachineEffect.h"
#include "effects/WaterfallEffect.h"
#include "effects/Waverly2dEffect.h"
#include "effects/WavesinsEffect.h"
#include "effects/Wavingcell2dEffect.h"

#if !(defined(WLED_DISABLE_PARTICLESYSTEM2D) && defined(WLED_DISABLE_PARTICLESYSTEM1D))
  #include "FXparticleSystem.h"
  #ifdef ESP8266
    #if !defined(WLED_DISABLE_PARTICLESYSTEM2D) && !defined(WLED_DISABLE_PARTICLESYSTEM1D)
    #error ESP8266 does not support 1D and 2D particle systems simultaneously. Please disable one of them.
    #endif
  #endif
#else
  #define WLED_PS_DONT_REPLACE_FX
#endif

 //////////////
 // DEV INFO //
 //////////////
/*
  information for FX metadata strings: https://kno.wled.ge/interfaces/json-api/#effect-metadata

  Audio Reactive: use the following code to pass usermod variables to effect

  uint8_t  *binNum = (uint8_t*)&SEGENV.aux1, *maxVol = (uint8_t*)(&SEGENV.aux1+1); // just in case assignment
  bool      samplePeak = false;
  float     FFT_MajorPeak = 1.0;
  uint8_t  *fftResult = nullptr;
  float    *fftBin = nullptr;
  um_data_t *um_data = getAudioData();
  volumeSmth    = *(float*)   um_data->u_data[0];
  volumeRaw     = *(float*)   um_data->u_data[1];
  fftResult     =  (uint8_t*) um_data->u_data[2];
  samplePeak    = *(uint8_t*) um_data->u_data[3];
  FFT_MajorPeak = *(float*)   um_data->u_data[4];
  my_magnitude  = *(float*)   um_data->u_data[5];
  maxVol        =  (uint8_t*) um_data->u_data[6];  // requires UI element (SEGMENT.customX?), changes source element
  binNum        =  (uint8_t*) um_data->u_data[7];  // requires UI element (SEGMENT.customX?), changes source element
  fftBin        =  (float*)   um_data->u_data[8];
*/


//////////////////////////////////////////////////////////////////////////////////////////
// mode data
static const char _data_RESERVED[] PROGMEM = "RSVD";

// add (or replace reserved) effect mode and data into vector
// use id==255 to find unallocated gaps (with "Reserved" data string)
// if vector size() is smaller than id (single) data is appended at the end (regardless of id)
// return the actual id used for the effect or 255 if the add failed.
uint8_t WS2812FX::addEffect(std::unique_ptr<EffectFactory>&& factory) {
  uint8_t id = factory->getEffectId();
  if (id == 255u) { // find empty slot
    for (size_t i=1; i<_effectFactories.size(); i++) if (_effectFactories[i] == nullptr) { id = i; break; }
  }
  for (size_t i = _effectFactories.size(); i < id; ++i) {
    _effectFactories.push_back(nullptr);
  }
  if (id < _effectFactories.size()) {
    if (_effectFactories[id] != nullptr) return 255; // do not overwrite an already added effect
    _effectFactories[id] = std::move(factory);
    return id;
  } else if (_effectFactories.size() < 255) { // 255 is reserved for indicating the effect wasn't added
    _effectFactories.push_back(std::move(factory));
    return _effectFactories.size() - 1;
  } else {
    return 255u; // The vector is full so return 255
  }
}

void WS2812FX::setupEffectData(size_t modeCount) {
  _effectFactories.resize(modeCount);

  addEffect(std::make_unique<EffectFactory>(StaticEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(BlinkEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(BreathEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ColorWipeEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ColorWipeRandomEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(RandomColorEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ColorSweepEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(DynamicEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(RainbowEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(RainbowCycleEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ScanEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(DualScanEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FadeEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(TheaterChaseEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(TheaterChaseRainbowEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(RunningLightsEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(SawEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(TwinkleEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(DissolveEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(DissolveRandomEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(SparkleEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FlashSparkleEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(HyperSparkleEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(StrobeEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(StrobeRainbowEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(MultiStrobeEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(BlinkRainbowEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(AndroidEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ChaseColorEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ChaseRandomEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ChaseRainbowEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ChaseFlashEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ChaseFlashRandomEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ChaseRainbowWhiteEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ColorfulEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(TrafficLightEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ColorSweepRandomEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(RunningColorEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(AuroraEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(RunningRandomEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(LarsonScannerEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(RainEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Pride2015Effect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ColorwavesEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FireworksEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(TetrixEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FireFlickerEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(GradientEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(LoadingEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FairyEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(TwoDotsEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FairytwinkleEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(RunningDualEffect::effectInformation));
  #ifdef WLED_ENABLE_GIF
  addEffect(std::make_unique<EffectFactory>(ImageEffect::effectInformation));
  #endif
  addEffect(std::make_unique<EffectFactory>(TricolorChaseEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(TricolorWipeEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(TricolorFadeEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(LightningEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(IcuEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(DualLarsonScannerEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(RandomChaseEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(OscillateEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(JuggleEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PaletteEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(BpmEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Fillnoise8Effect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Noise161Effect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Noise162Effect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Noise163Effect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Noise164Effect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ColortwinkleEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(LakeEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(MeteorEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(MeteorSmoothEffect::effectInformation)); // merged with mode_meteor
  addEffect(std::make_unique<EffectFactory>(RailwayEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(RippleEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(TwinklefoxEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(TwinklecatEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(HalloweenEyesEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(StaticPatternEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(TriStaticPatternEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(SpotsEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(SpotsFadeEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(CometEffect::effectInformation));
  #ifdef WLED_PS_DONT_REPLACE_FX
  addEffect(std::make_unique<EffectFactory>(MultiCometEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(RollingBallsEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(SparkleEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(GlitterEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(SolidGlitterEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(StarburstEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(DancingShadowsEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Fire2012Effect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ExplodingFireworksEffect::effectInformation));
  #endif
  addEffect(std::make_unique<EffectFactory>(CandleEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(BouncingBallsEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PopcornEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(DripEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(SinelonEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(SinelonDualEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(SinelonRainbowEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PopcornEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(DripEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PlasmaEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PercentEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(RippleRainbowEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(HeartbeatEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PacificaEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(CandleMultiEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(SolidGlitterEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(SunriseEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PhasedEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(TwinkleupEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(NoisepalEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(SinewaveEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PhasedNoiseEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FlowEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ChunchunEffect::effectInformation));  
  addEffect(std::make_unique<EffectFactory>(WashingMachineEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(BlendsEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(TvSimulatorEffect::effectInformation));
  //addEffect(std::make_unique<EffectFactory>(DynamicSmoothEffect::effectInformation));

  // --- 1D audio effects ---
  addEffect(std::make_unique<EffectFactory>(PixelsEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PixelwaveEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(JugglesEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(MatripixEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(GravimeterEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PlasmoidEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PuddlesEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(MidnoiseEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(NoisemeterEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FreqwaveEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FreqmatrixEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(WaterfallEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FreqpixelsEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(NoisefireEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PuddlepeakEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(NoisemoveEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PerlinmoveEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(RipplepeakEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FreqmapEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(GravcenterEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(GravcentricEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(GravfreqEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(DjLightEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(BlurzEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FlowStripeEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(WavesinsEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(RocktavesEffect::effectInformation));

  // --- 2D  effects ---
  #ifndef WLED_DISABLE_2D
  addEffect(std::make_unique<EffectFactory>(Plasmarotozoom2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Spaceships2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Crazybees2dEffect::effectInformation));

  #ifdef WLED_PS_DONT_REPLACE_FX
  addEffect(std::make_unique<EffectFactory>(Ghostrider2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Floatingblobs2dEffect::effectInformation));
  #endif

  addEffect(std::make_unique<EffectFactory>(Scrollingtext2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Driftrose2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Distortionwaves2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Geq2dEffect::effectInformation)); // audio
  addEffect(std::make_unique<EffectFactory>(Noise2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Firenoise2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Squaredswirl2dEffect::effectInformation));

  //non audio
  addEffect(std::make_unique<EffectFactory>(Dna2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Matrix2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Metaballs2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(FunkyPlank2dEffect::effectInformation)); // audio
  addEffect(std::make_unique<EffectFactory>(Pulser2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Drift2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Waverly2dEffect::effectInformation)); // audio
  addEffect(std::make_unique<EffectFactory>(Sunradiation2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ColoredBursts2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Julia2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Gameoflife2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Tartan2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(PolarLights2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Swirl2dEffect::effectInformation)); // audio
  addEffect(std::make_unique<EffectFactory>(Lissajous2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Frizzles2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Plasmaball2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Hiphotic2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Sindots2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(DnaSpiral2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(BlackHole2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Soap2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Octopus2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Wavingcell2dEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Akemi2dEffect::effectInformation)); // audio

  #ifndef WLED_DISABLE_PARTICLESYSTEM2D
  addEffect(std::make_unique<EffectFactory>(ParticlevolcanoEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticlefireEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticlefireworksEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticlevortexEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleperlinEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticlepitEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleboxEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleattractorEffect::effectInformation)); // 872 bytes
  addEffect(std::make_unique<EffectFactory>(ParticleimpactEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticlewaterfallEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticlesprayEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleGEQEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticlecenterGEQEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleghostriderEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleblobsEffect::effectInformation));
  #endif // WLED_DISABLE_PARTICLESYSTEM2D
  #endif // WLED_DISABLE_2D

  #ifndef WLED_DISABLE_PARTICLESYSTEM1D
  addEffect(std::make_unique<EffectFactory>(ParticleDripEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticlePinballEffect::effectInformation)); //potential replacement for: bouncing balls, rollingballs, popcorn
  addEffect(std::make_unique<EffectFactory>(ParticleDancingShadowsEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleFireworks1DEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleSparklerEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleHourglassEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Particle1DsprayEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleBalanceEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleChaseEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleStarburstEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Particle1dGeqEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(ParticleFire1DEffect::effectInformation));
  addEffect(std::make_unique<EffectFactory>(Particle1DsonicstreamEffect::effectInformation));
  #endif // WLED_DISABLE_PARTICLESYSTEM1D
}
