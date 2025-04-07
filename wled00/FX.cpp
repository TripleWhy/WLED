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
uint8_t WS2812FX::addEffect(const EffectInformation& effectInfo) {
  for (size_t i = _effectInfos.size(); i < effectInfo.effectId; ++i) {
    _effectInfos.push_back(nullptr);
  }
  if (effectInfo.effectId < _effectInfos.size()) {
    if (_effectInfos[effectInfo.effectId] != nullptr) return 255; // do not overwrite an already added effect
    _effectInfos[effectInfo.effectId] = &effectInfo;
    return effectInfo.effectId;
  } else if (_effectInfos.size() < 255) { // 255 is reserved for indicating the effect wasn't added
    _effectInfos.push_back(&effectInfo);
    return _effectInfos.size() - 1;
  } else {
    return 255u; // The vector is full so return 255
  }
}

void WS2812FX::setupEffectData(size_t modeCount) {
  _effectInfos.resize(modeCount);

  addEffect(StaticEffect::effectInformation);
  addEffect(BlinkEffect::effectInformation);
  addEffect(BreathEffect::effectInformation);
  addEffect(ColorWipeEffect::effectInformation);
  addEffect(ColorWipeRandomEffect::effectInformation);
  addEffect(RandomColorEffect::effectInformation);
  addEffect(ColorSweepEffect::effectInformation);
  addEffect(DynamicEffect::effectInformation);
  addEffect(RainbowEffect::effectInformation);
  addEffect(RainbowCycleEffect::effectInformation);
  addEffect(ScanEffect::effectInformation);
  //addEffect(DualScanEffect::effectInformation);
  addEffect(FadeEffect::effectInformation);
  addEffect(TheaterChaseEffect::effectInformation);
  //addEffect(TheaterChaseRainbowEffect::effectInformation);
  addEffect(RunningLightsEffect::effectInformation);
  //addEffect(SawEffect::effectInformation);
  addEffect(TwinkleEffect::effectInformation);
  addEffect(DissolveEffect::effectInformation);
  //addEffect(DissolveRandomEffect::effectInformation);
  addEffect(SparkleEffect::effectInformation);
  addEffect(FlashSparkleEffect::effectInformation);
  addEffect(HyperSparkleEffect::effectInformation);
  addEffect(StrobeEffect::effectInformation);
  addEffect(StrobeRainbowEffect::effectInformation);
  addEffect(MultiStrobeEffect::effectInformation);
  addEffect(BlinkRainbowEffect::effectInformation);
  addEffect(AndroidEffect::effectInformation);
  addEffect(ChaseColorEffect::effectInformation);
  addEffect(ChaseRandomEffect::effectInformation);
  addEffect(ChaseRainbowEffect::effectInformation);
  addEffect(ChaseFlashEffect::effectInformation);
  addEffect(ChaseFlashRandomEffect::effectInformation);
  addEffect(ChaseRainbowWhiteEffect::effectInformation);
  addEffect(ColorfulEffect::effectInformation);
  addEffect(TrafficLightEffect::effectInformation);
  addEffect(ColorSweepRandomEffect::effectInformation);
  //addEffect(RunningColorEffect::effectInformation);
  addEffect(AuroraEffect::effectInformation);
  addEffect(RunningRandomEffect::effectInformation);
  addEffect(LarsonScannerEffect::effectInformation);
  addEffect(RainEffect::effectInformation);
  addEffect(Pride2015Effect::effectInformation);
  addEffect(ColorwavesEffect::effectInformation);
  addEffect(FireworksEffect::effectInformation);
  addEffect(TetrixEffect::effectInformation);
  addEffect(FireFlickerEffect::effectInformation);
  addEffect(GradientEffect::effectInformation);
  addEffect(LoadingEffect::effectInformation);
  addEffect(FairyEffect::effectInformation);
  addEffect(TwoDotsEffect::effectInformation);
  addEffect(FairytwinkleEffect::effectInformation);
  //addEffect(RunningDualEffect::effectInformation);
  #ifdef WLED_ENABLE_GIF
  addEffect(ImageEffect::effectInformation);
  #endif
  addEffect(TricolorChaseEffect::effectInformation);
  addEffect(TricolorWipeEffect::effectInformation);
  addEffect(TricolorFadeEffect::effectInformation);
  addEffect(LightningEffect::effectInformation);
  addEffect(IcuEffect::effectInformation);
  //addEffect(DualLarsonScannerEffect::effectInformation);
  addEffect(RandomChaseEffect::effectInformation);
  addEffect(OscillateEffect::effectInformation);
  addEffect(JuggleEffect::effectInformation);
  addEffect(PaletteEffect::effectInformation);
  addEffect(BpmEffect::effectInformation);
  addEffect(Fillnoise8Effect::effectInformation);
  addEffect(Noise161Effect::effectInformation);
  addEffect(Noise162Effect::effectInformation);
  addEffect(Noise163Effect::effectInformation);
  addEffect(Noise164Effect::effectInformation);
  addEffect(ColortwinkleEffect::effectInformation);
  addEffect(LakeEffect::effectInformation);
  addEffect(MeteorEffect::effectInformation);
  //addEffect(MeteorSmoothEffect::effectInformation); // merged with mode_meteor
  addEffect(RailwayEffect::effectInformation);
  addEffect(RippleEffect::effectInformation);
  addEffect(TwinklefoxEffect::effectInformation);
  addEffect(TwinklecatEffect::effectInformation);
  addEffect(HalloweenEyesEffect::effectInformation);
  addEffect(StaticPatternEffect::effectInformation);
  addEffect(TriStaticPatternEffect::effectInformation);
  addEffect(SpotsEffect::effectInformation);
  addEffect(SpotsFadeEffect::effectInformation);
  addEffect(CometEffect::effectInformation);
  #ifdef WLED_PS_DONT_REPLACE_FX
  addEffect(MultiCometEffect::effectInformation);
  addEffect(RollingBallsEffect::effectInformation);
  addEffect(SparkleEffect::effectInformation);
  addEffect(GlitterEffect::effectInformation);
  //addEffect(SolidGlitterEffect::effectInformation);
  addEffect(StarburstEffect::effectInformation);
  addEffect(DancingShadowsEffect::effectInformation);
  addEffect(Fire2012Effect::effectInformation);
  addEffect(ExplodingFireworksEffect::effectInformation);
  #endif
  addEffect(CandleEffect::effectInformation);
  addEffect(BouncingBallsEffect::effectInformation);
  addEffect(PopcornEffect::effectInformation);
  addEffect(DripEffect::effectInformation);
  addEffect(SinelonEffect::effectInformation);
  //addEffect(SinelonDualEffect::effectInformation);
  //addEffect(SinelonRainbowEffect::effectInformation);
  addEffect(PopcornEffect::effectInformation);
  addEffect(DripEffect::effectInformation);
  addEffect(PlasmaEffect::effectInformation);
  addEffect(PercentEffect::effectInformation);
  //addEffect(RippleRainbowEffect::effectInformation);
  addEffect(HeartbeatEffect::effectInformation);
  addEffect(PacificaEffect::effectInformation);
  //addEffect(CandleMultiEffect::effectInformation);
  //addEffect(SolidGlitterEffect::effectInformation);
  addEffect(SunriseEffect::effectInformation);
  addEffect(PhasedEffect::effectInformation);
  addEffect(TwinkleupEffect::effectInformation);
  addEffect(NoisepalEffect::effectInformation);
  addEffect(SinewaveEffect::effectInformation);
  addEffect(PhasedNoiseEffect::effectInformation);
  addEffect(FlowEffect::effectInformation);
  addEffect(ChunchunEffect::effectInformation);
  addEffect(WashingMachineEffect::effectInformation);
  addEffect(BlendsEffect::effectInformation);
  addEffect(TvSimulatorEffect::effectInformation);
  //addEffect(DynamicSmoothEffect::effectInformation);

  // --- 1D audio effects ---
  addEffect(PixelsEffect::effectInformation);
  addEffect(PixelwaveEffect::effectInformation);
  addEffect(JugglesEffect::effectInformation);
  addEffect(MatripixEffect::effectInformation);
  addEffect(GravimeterEffect::effectInformation);
  addEffect(PlasmoidEffect::effectInformation);
  addEffect(PuddlesEffect::effectInformation);
  addEffect(MidnoiseEffect::effectInformation);
  addEffect(NoisemeterEffect::effectInformation);
  addEffect(FreqwaveEffect::effectInformation);
  addEffect(FreqmatrixEffect::effectInformation);
  addEffect(WaterfallEffect::effectInformation);
  addEffect(FreqpixelsEffect::effectInformation);
  addEffect(NoisefireEffect::effectInformation);
  addEffect(PuddlepeakEffect::effectInformation);
  addEffect(NoisemoveEffect::effectInformation);
  addEffect(PerlinmoveEffect::effectInformation);
  addEffect(RipplepeakEffect::effectInformation);
  addEffect(FreqmapEffect::effectInformation);
  addEffect(GravcenterEffect::effectInformation);
  addEffect(GravcentricEffect::effectInformation);
  addEffect(GravfreqEffect::effectInformation);
  addEffect(DjLightEffect::effectInformation);
  addEffect(BlurzEffect::effectInformation);
  addEffect(FlowStripeEffect::effectInformation);
  addEffect(WavesinsEffect::effectInformation);
  addEffect(RocktavesEffect::effectInformation);

  // --- 2D  effects ---
  #ifndef WLED_DISABLE_2D
  addEffect(Plasmarotozoom2dEffect::effectInformation);
  addEffect(Spaceships2dEffect::effectInformation);
  addEffect(Crazybees2dEffect::effectInformation);

  #ifdef WLED_PS_DONT_REPLACE_FX
  addEffect(Ghostrider2dEffect::effectInformation);
  addEffect(Floatingblobs2dEffect::effectInformation);
  #endif

  addEffect(Scrollingtext2dEffect::effectInformation);
  addEffect(Driftrose2dEffect::effectInformation);
  addEffect(Distortionwaves2dEffect::effectInformation);
  addEffect(Geq2dEffect::effectInformation); // audio
  addEffect(Noise2dEffect::effectInformation);
  addEffect(Firenoise2dEffect::effectInformation);
  addEffect(Squaredswirl2dEffect::effectInformation);

  //non audio
  addEffect(Dna2dEffect::effectInformation);
  addEffect(Matrix2dEffect::effectInformation);
  addEffect(Metaballs2dEffect::effectInformation);
  addEffect(FunkyPlank2dEffect::effectInformation); // audio
  addEffect(Pulser2dEffect::effectInformation);
  addEffect(Drift2dEffect::effectInformation);
  addEffect(Waverly2dEffect::effectInformation); // audio
  addEffect(Sunradiation2dEffect::effectInformation);
  addEffect(ColoredBursts2dEffect::effectInformation);
  addEffect(Julia2dEffect::effectInformation);
  addEffect(Gameoflife2dEffect::effectInformation);
  addEffect(Tartan2dEffect::effectInformation);
  addEffect(PolarLights2dEffect::effectInformation);
  addEffect(Swirl2dEffect::effectInformation); // audio
  addEffect(Lissajous2dEffect::effectInformation);
  addEffect(Frizzles2dEffect::effectInformation);
  addEffect(Plasmaball2dEffect::effectInformation);
  addEffect(Hiphotic2dEffect::effectInformation);
  addEffect(Sindots2dEffect::effectInformation);
  addEffect(DnaSpiral2dEffect::effectInformation);
  addEffect(BlackHole2dEffect::effectInformation);
  addEffect(Soap2dEffect::effectInformation);
  addEffect(Octopus2dEffect::effectInformation);
  addEffect(Wavingcell2dEffect::effectInformation);
  addEffect(Akemi2dEffect::effectInformation); // audio

  #ifndef WLED_DISABLE_PARTICLESYSTEM2D
  addEffect(ParticlevolcanoEffect::effectInformation);
  addEffect(ParticlefireEffect::effectInformation);
  addEffect(ParticlefireworksEffect::effectInformation);
  addEffect(ParticlevortexEffect::effectInformation);
  addEffect(ParticleperlinEffect::effectInformation);
  addEffect(ParticlepitEffect::effectInformation);
  addEffect(ParticleboxEffect::effectInformation);
  addEffect(ParticleattractorEffect::effectInformation); // 872 bytes
  addEffect(ParticleimpactEffect::effectInformation);
  addEffect(ParticlewaterfallEffect::effectInformation);
  addEffect(ParticlesprayEffect::effectInformation);
  addEffect(ParticleGEQEffect::effectInformation);
  addEffect(ParticlecenterGEQEffect::effectInformation);
  addEffect(ParticleghostriderEffect::effectInformation);
  addEffect(ParticleblobsEffect::effectInformation);
  #endif // WLED_DISABLE_PARTICLESYSTEM2D
  #endif // WLED_DISABLE_2D

  #ifndef WLED_DISABLE_PARTICLESYSTEM1D
  addEffect(ParticleDripEffect::effectInformation);
  addEffect(ParticlePinballEffect::effectInformation); //potential replacement for: bouncing balls, rollingballs, popcorn
  addEffect(ParticleDancingShadowsEffect::effectInformation);
  addEffect(ParticleFireworks1DEffect::effectInformation);
  addEffect(ParticleSparklerEffect::effectInformation);
  addEffect(ParticleHourglassEffect::effectInformation);
  addEffect(Particle1DsprayEffect::effectInformation);
  addEffect(ParticleBalanceEffect::effectInformation);
  addEffect(ParticleChaseEffect::effectInformation);
  addEffect(ParticleStarburstEffect::effectInformation);
  addEffect(Particle1dGeqEffect::effectInformation);
  addEffect(ParticleFire1DEffect::effectInformation);
  addEffect(Particle1DsonicstreamEffect::effectInformation);
  #endif // WLED_DISABLE_PARTICLESYSTEM1D
}
