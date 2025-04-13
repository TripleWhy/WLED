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
#include "colors.h"
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
#include "effects/particleEffects1d/Particle1dSonicStreamEffect.h"
#include "effects/particleEffects1d/Particle1dSprayEffect.h"
#include "effects/particleEffects1d/ParticleBalanceEffect.h"
#include "effects/particleEffects1d/ParticleChaseEffect.h"
#include "effects/particleEffects1d/ParticleDancingShadowsEffect.h"
#include "effects/particleEffects1d/ParticleDripEffect.h"
#include "effects/particleEffects1d/ParticleFire1dEffect.h"
#include "effects/particleEffects1d/ParticleFireworks1dEffect.h"
#include "effects/particleEffects1d/ParticleHourglassEffect.h"
#include "effects/particleEffects1d/ParticlePinballEffect.h"
#include "effects/particleEffects1d/ParticleSparklerEffect.h"
#include "effects/particleEffects1d/ParticleStarburstEffect.h"
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

static constexpr void assignEffectInfo(std::array<const EffectInformation*, MODE_COUNT>& array, const EffectInformation& info) {
  array[info.effectId] = &info;
}

static constexpr std::array<const EffectInformation*, MODE_COUNT> setupEffectData() {
  std::array<const EffectInformation*, MODE_COUNT> array{};
  assignEffectInfo(array, StaticEffect::effectInformation);
  assignEffectInfo(array, BlinkEffect::effectInformation);
  assignEffectInfo(array, BreathEffect::effectInformation);
  assignEffectInfo(array, ColorWipeEffect::effectInformation);
  assignEffectInfo(array, ColorWipeRandomEffect::effectInformation);
  assignEffectInfo(array, RandomColorEffect::effectInformation);
  assignEffectInfo(array, ColorSweepEffect::effectInformation);
  assignEffectInfo(array, DynamicEffect::effectInformation);
  assignEffectInfo(array, RainbowEffect::effectInformation);
  assignEffectInfo(array, RainbowCycleEffect::effectInformation);
  assignEffectInfo(array, ScanEffect::effectInformation);
  //assignEffectInfo(array, DualScanEffect::effectInformation);
  assignEffectInfo(array, FadeEffect::effectInformation);
  assignEffectInfo(array, TheaterChaseEffect::effectInformation);
  //assignEffectInfo(array, TheaterChaseRainbowEffect::effectInformation);
  assignEffectInfo(array, RunningLightsEffect::effectInformation);
  //assignEffectInfo(array, SawEffect::effectInformation);
  assignEffectInfo(array, TwinkleEffect::effectInformation);
  assignEffectInfo(array, DissolveEffect::effectInformation);
  //assignEffectInfo(array, DissolveRandomEffect::effectInformation);
  assignEffectInfo(array, SparkleEffect::effectInformation);
  assignEffectInfo(array, FlashSparkleEffect::effectInformation);
  assignEffectInfo(array, HyperSparkleEffect::effectInformation);
  assignEffectInfo(array, StrobeEffect::effectInformation);
  assignEffectInfo(array, StrobeRainbowEffect::effectInformation);
  assignEffectInfo(array, MultiStrobeEffect::effectInformation);
  assignEffectInfo(array, BlinkRainbowEffect::effectInformation);
  assignEffectInfo(array, AndroidEffect::effectInformation);
  assignEffectInfo(array, ChaseColorEffect::effectInformation);
  assignEffectInfo(array, ChaseRandomEffect::effectInformation);
  assignEffectInfo(array, ChaseRainbowEffect::effectInformation);
  assignEffectInfo(array, ChaseFlashEffect::effectInformation);
  assignEffectInfo(array, ChaseFlashRandomEffect::effectInformation);
  assignEffectInfo(array, ChaseRainbowWhiteEffect::effectInformation);
  assignEffectInfo(array, ColorfulEffect::effectInformation);
  assignEffectInfo(array, TrafficLightEffect::effectInformation);
  assignEffectInfo(array, ColorSweepRandomEffect::effectInformation);
  //assignEffectInfo(array, RunningColorEffect::effectInformation);
  assignEffectInfo(array, AuroraEffect::effectInformation);
  assignEffectInfo(array, RunningRandomEffect::effectInformation);
  assignEffectInfo(array, LarsonScannerEffect::effectInformation);
  assignEffectInfo(array, RainEffect::effectInformation);
  assignEffectInfo(array, Pride2015Effect::effectInformation);
  assignEffectInfo(array, ColorwavesEffect::effectInformation);
  assignEffectInfo(array, FireworksEffect::effectInformation);
  assignEffectInfo(array, TetrixEffect::effectInformation);
  assignEffectInfo(array, FireFlickerEffect::effectInformation);
  assignEffectInfo(array, GradientEffect::effectInformation);
  assignEffectInfo(array, LoadingEffect::effectInformation);
  assignEffectInfo(array, FairyEffect::effectInformation);
  assignEffectInfo(array, TwoDotsEffect::effectInformation);
  assignEffectInfo(array, FairytwinkleEffect::effectInformation);
  //assignEffectInfo(array, RunningDualEffect::effectInformation);
  #ifdef WLED_ENABLE_GIF
  assignEffectInfo(array, ImageEffect::effectInformation);
  #endif
  assignEffectInfo(array, TricolorChaseEffect::effectInformation);
  assignEffectInfo(array, TricolorWipeEffect::effectInformation);
  assignEffectInfo(array, TricolorFadeEffect::effectInformation);
  assignEffectInfo(array, LightningEffect::effectInformation);
  assignEffectInfo(array, IcuEffect::effectInformation);
  //assignEffectInfo(array, DualLarsonScannerEffect::effectInformation);
  assignEffectInfo(array, RandomChaseEffect::effectInformation);
  assignEffectInfo(array, OscillateEffect::effectInformation);
  assignEffectInfo(array, JuggleEffect::effectInformation);
  assignEffectInfo(array, PaletteEffect::effectInformation);
  assignEffectInfo(array, BpmEffect::effectInformation);
  assignEffectInfo(array, Fillnoise8Effect::effectInformation);
  assignEffectInfo(array, Noise161Effect::effectInformation);
  assignEffectInfo(array, Noise162Effect::effectInformation);
  assignEffectInfo(array, Noise163Effect::effectInformation);
  assignEffectInfo(array, Noise164Effect::effectInformation);
  assignEffectInfo(array, ColortwinkleEffect::effectInformation);
  assignEffectInfo(array, LakeEffect::effectInformation);
  assignEffectInfo(array, MeteorEffect::effectInformation);
  //assignEffectInfo(array, MeteorSmoothEffect::effectInformation); // merged with mode_meteor
  assignEffectInfo(array, RailwayEffect::effectInformation);
  assignEffectInfo(array, RippleEffect::effectInformation);
  assignEffectInfo(array, TwinklefoxEffect::effectInformation);
  assignEffectInfo(array, TwinklecatEffect::effectInformation);
  assignEffectInfo(array, HalloweenEyesEffect::effectInformation);
  assignEffectInfo(array, StaticPatternEffect::effectInformation);
  assignEffectInfo(array, TriStaticPatternEffect::effectInformation);
  assignEffectInfo(array, SpotsEffect::effectInformation);
  assignEffectInfo(array, SpotsFadeEffect::effectInformation);
  assignEffectInfo(array, CometEffect::effectInformation);
  #ifdef WLED_PS_DONT_REPLACE_FX
  assignEffectInfo(array, MultiCometEffect::effectInformation);
  assignEffectInfo(array, RollingBallsEffect::effectInformation);
  assignEffectInfo(array, SparkleEffect::effectInformation);
  assignEffectInfo(array, GlitterEffect::effectInformation);
  //assignEffectInfo(array, SolidGlitterEffect::effectInformation);
  assignEffectInfo(array, StarburstEffect::effectInformation);
  assignEffectInfo(array, DancingShadowsEffect::effectInformation);
  assignEffectInfo(array, Fire2012Effect::effectInformation);
  assignEffectInfo(array, ExplodingFireworksEffect::effectInformation);
  #endif
  assignEffectInfo(array, CandleEffect::effectInformation);
  assignEffectInfo(array, BouncingBallsEffect::effectInformation);
  assignEffectInfo(array, PopcornEffect::effectInformation);
  assignEffectInfo(array, DripEffect::effectInformation);
  assignEffectInfo(array, SinelonEffect::effectInformation);
  //assignEffectInfo(array, SinelonDualEffect::effectInformation);
  //assignEffectInfo(array, SinelonRainbowEffect::effectInformation);
  assignEffectInfo(array, PopcornEffect::effectInformation);
  assignEffectInfo(array, DripEffect::effectInformation);
  assignEffectInfo(array, PlasmaEffect::effectInformation);
  assignEffectInfo(array, PercentEffect::effectInformation);
  //assignEffectInfo(array, RippleRainbowEffect::effectInformation);
  assignEffectInfo(array, HeartbeatEffect::effectInformation);
  assignEffectInfo(array, PacificaEffect::effectInformation);
  //assignEffectInfo(array, CandleMultiEffect::effectInformation);
  //assignEffectInfo(array, SolidGlitterEffect::effectInformation);
  assignEffectInfo(array, SunriseEffect::effectInformation);
  assignEffectInfo(array, PhasedEffect::effectInformation);
  assignEffectInfo(array, TwinkleupEffect::effectInformation);
  assignEffectInfo(array, NoisepalEffect::effectInformation);
  assignEffectInfo(array, SinewaveEffect::effectInformation);
  assignEffectInfo(array, PhasedNoiseEffect::effectInformation);
  assignEffectInfo(array, FlowEffect::effectInformation);
  assignEffectInfo(array, ChunchunEffect::effectInformation);
  assignEffectInfo(array, WashingMachineEffect::effectInformation);
  assignEffectInfo(array, BlendsEffect::effectInformation);
  assignEffectInfo(array, TvSimulatorEffect::effectInformation);
  //assignEffectInfo(array, DynamicSmoothEffect::effectInformation);

  // --- 1D audio effects ---
  assignEffectInfo(array, PixelsEffect::effectInformation);
  assignEffectInfo(array, PixelwaveEffect::effectInformation);
  assignEffectInfo(array, JugglesEffect::effectInformation);
  assignEffectInfo(array, MatripixEffect::effectInformation);
  assignEffectInfo(array, GravimeterEffect::effectInformation);
  assignEffectInfo(array, PlasmoidEffect::effectInformation);
  assignEffectInfo(array, PuddlesEffect::effectInformation);
  assignEffectInfo(array, MidnoiseEffect::effectInformation);
  assignEffectInfo(array, NoisemeterEffect::effectInformation);
  assignEffectInfo(array, FreqwaveEffect::effectInformation);
  assignEffectInfo(array, FreqmatrixEffect::effectInformation);
  assignEffectInfo(array, WaterfallEffect::effectInformation);
  assignEffectInfo(array, FreqpixelsEffect::effectInformation);
  assignEffectInfo(array, NoisefireEffect::effectInformation);
  assignEffectInfo(array, PuddlepeakEffect::effectInformation);
  assignEffectInfo(array, NoisemoveEffect::effectInformation);
  assignEffectInfo(array, PerlinmoveEffect::effectInformation);
  assignEffectInfo(array, RipplepeakEffect::effectInformation);
  assignEffectInfo(array, FreqmapEffect::effectInformation);
  assignEffectInfo(array, GravcenterEffect::effectInformation);
  assignEffectInfo(array, GravcentricEffect::effectInformation);
  assignEffectInfo(array, GravfreqEffect::effectInformation);
  assignEffectInfo(array, DjLightEffect::effectInformation);
  assignEffectInfo(array, BlurzEffect::effectInformation);
  assignEffectInfo(array, FlowStripeEffect::effectInformation);
  assignEffectInfo(array, WavesinsEffect::effectInformation);
  assignEffectInfo(array, RocktavesEffect::effectInformation);

  // --- 2D  effects ---
  #ifndef WLED_DISABLE_2D
  assignEffectInfo(array, Plasmarotozoom2dEffect::effectInformation);
  assignEffectInfo(array, Spaceships2dEffect::effectInformation);
  assignEffectInfo(array, Crazybees2dEffect::effectInformation);

  #ifdef WLED_PS_DONT_REPLACE_FX
  assignEffectInfo(array, Ghostrider2dEffect::effectInformation);
  assignEffectInfo(array, Floatingblobs2dEffect::effectInformation);
  #endif

  assignEffectInfo(array, Scrollingtext2dEffect::effectInformation);
  assignEffectInfo(array, Driftrose2dEffect::effectInformation);
  assignEffectInfo(array, Distortionwaves2dEffect::effectInformation);
  assignEffectInfo(array, Geq2dEffect::effectInformation); // audio
  assignEffectInfo(array, Noise2dEffect::effectInformation);
  assignEffectInfo(array, Firenoise2dEffect::effectInformation);
  assignEffectInfo(array, Squaredswirl2dEffect::effectInformation);

  //non audio
  assignEffectInfo(array, Dna2dEffect::effectInformation);
  assignEffectInfo(array, Matrix2dEffect::effectInformation);
  assignEffectInfo(array, Metaballs2dEffect::effectInformation);
  assignEffectInfo(array, FunkyPlank2dEffect::effectInformation); // audio
  assignEffectInfo(array, Pulser2dEffect::effectInformation);
  assignEffectInfo(array, Drift2dEffect::effectInformation);
  assignEffectInfo(array, Waverly2dEffect::effectInformation); // audio
  assignEffectInfo(array, Sunradiation2dEffect::effectInformation);
  assignEffectInfo(array, ColoredBursts2dEffect::effectInformation);
  assignEffectInfo(array, Julia2dEffect::effectInformation);
  assignEffectInfo(array, Gameoflife2dEffect::effectInformation);
  assignEffectInfo(array, Tartan2dEffect::effectInformation);
  assignEffectInfo(array, PolarLights2dEffect::effectInformation);
  assignEffectInfo(array, Swirl2dEffect::effectInformation); // audio
  assignEffectInfo(array, Lissajous2dEffect::effectInformation);
  assignEffectInfo(array, Frizzles2dEffect::effectInformation);
  assignEffectInfo(array, Plasmaball2dEffect::effectInformation);
  assignEffectInfo(array, Hiphotic2dEffect::effectInformation);
  assignEffectInfo(array, Sindots2dEffect::effectInformation);
  assignEffectInfo(array, DnaSpiral2dEffect::effectInformation);
  assignEffectInfo(array, BlackHole2dEffect::effectInformation);
  assignEffectInfo(array, Soap2dEffect::effectInformation);
  assignEffectInfo(array, Octopus2dEffect::effectInformation);
  assignEffectInfo(array, Wavingcell2dEffect::effectInformation);
  assignEffectInfo(array, Akemi2dEffect::effectInformation); // audio

  #ifndef WLED_DISABLE_PARTICLESYSTEM2D
  assignEffectInfo(array, ParticlevolcanoEffect::effectInformation);
  assignEffectInfo(array, ParticlefireEffect::effectInformation);
  assignEffectInfo(array, ParticlefireworksEffect::effectInformation);
  assignEffectInfo(array, ParticlevortexEffect::effectInformation);
  assignEffectInfo(array, ParticleperlinEffect::effectInformation);
  assignEffectInfo(array, ParticlepitEffect::effectInformation);
  assignEffectInfo(array, ParticleboxEffect::effectInformation);
  assignEffectInfo(array, ParticleattractorEffect::effectInformation); // 872 bytes
  assignEffectInfo(array, ParticleimpactEffect::effectInformation);
  assignEffectInfo(array, ParticlewaterfallEffect::effectInformation);
  assignEffectInfo(array, ParticlesprayEffect::effectInformation);
  assignEffectInfo(array, ParticleGEQEffect::effectInformation);
  assignEffectInfo(array, ParticlecenterGEQEffect::effectInformation);
  assignEffectInfo(array, ParticleghostriderEffect::effectInformation);
  assignEffectInfo(array, ParticleblobsEffect::effectInformation);
  #endif // WLED_DISABLE_PARTICLESYSTEM2D
  #endif // WLED_DISABLE_2D

  #ifndef WLED_DISABLE_PARTICLESYSTEM1D
  assignEffectInfo(array, ParticleDripEffect::effectInformation);
  assignEffectInfo(array, ParticlePinballEffect::effectInformation); //potential replacement for: bouncing balls, rollingballs, popcorn
  assignEffectInfo(array, ParticleDancingShadowsEffect::effectInformation);
  assignEffectInfo(array, ParticleFireworks1dEffect::effectInformation);
  assignEffectInfo(array, ParticleSparklerEffect::effectInformation);
  assignEffectInfo(array, ParticleHourglassEffect::effectInformation);
  assignEffectInfo(array, Particle1dSprayEffect::effectInformation);
  assignEffectInfo(array, ParticleBalanceEffect::effectInformation);
  assignEffectInfo(array, ParticleChaseEffect::effectInformation);
  assignEffectInfo(array, ParticleStarburstEffect::effectInformation);
  assignEffectInfo(array, Particle1dGeqEffect::effectInformation);
  assignEffectInfo(array, ParticleFire1dEffect::effectInformation);
  assignEffectInfo(array, Particle1dSonicStreamEffect::effectInformation);
  #endif // WLED_DISABLE_PARTICLESYSTEM1D

  return array;
}

// c++20 can do this without needing a complicated function
namespace {
  constexpr std::array<const EffectInformation*, MODE_COUNT> _effectInfos PROGMEM = setupEffectData();
}

#ifdef WLED_DEBUG
void WS2812FX::printSize() {
  size_t size = 0;
  for (const Segment &seg : _segments) size += seg.getSize();
  DEBUG_PRINTF_P(PSTR("Segments: %d -> %u/%dB\n"), _segments.size(), size, SegmentMemoryManager::getUsedSpace());
  for (const Segment &seg : _segments) DEBUG_PRINTF_P(PSTR("  Seg: %d,%d [A=%d, 2D=%d, RGB=%d, W=%d, CCT=%d]\n"), seg.width(), seg.height(), seg.isActive(), seg.is2D(), seg.hasRGB(), seg.hasWhite(), seg.isCCT());
  DEBUG_PRINTF_P(PSTR("Modes: %d*%d=%uB\n"), sizeof(mode_ptr), _effectInfos.size(), (_effectInfos.size()*sizeof(mode_ptr)));
  DEBUG_PRINTF_P(PSTR("Map: %d*%d=%uB\n"), sizeof(uint16_t), (int)customMappingSize, customMappingSize*sizeof(uint16_t));
}
#endif

const EffectInformation* WS2812FX::getEffectInformation(uint8_t effectId) const {
  return _effectInfos[effectId];
}
const EffectInformation* WS2812FX::safeGetEffectInformation(uint8_t effectId) const {
  return (effectId < getModeCount()) ? _effectInfos[effectId] : nullptr;
}
const char* WS2812FX::getModeData(unsigned id) const {
  const EffectInformation* const info = safeGetEffectInformation(id); return (info != nullptr) ? info->metaData : PSTR("Solid");
}
