#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>
#import <AudioToolbox/AudioToolbox.h>
#import <mach-o/dyld.h>
#import <math.h>
#include "fishhook.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NUM_BANDS 7
static double FREQUENCIES[NUM_BANDS] = {45.0, 30.0, 60.0, 100.0, 250.0, 500.0, 1000.0}; // Band 0 is Ultra-Sub Shelf at 45Hz
static double default_gains[NUM_BANDS] = {12.0, 6.0, 2.0, -3.0, -5.0, 0.0, 1.5}; // EarthQuake preset
static double GAINS_DB[NUM_BANDS];
static double PREAMP_DB = -7.0;
static uint64_t gProcessedBufferCount = 0;

typedef struct {
    double b0, b1, b2, a1, a2;
    double x1_L, x2_L, y1_L, y2_L;
    double x1_R, x2_R, y1_R, y2_R;
} BiquadFilter64;

static BiquadFilter64 filters[NUM_BANDS];
static double gCurrentSampleRate = 44100.0;

static inline float fast_soft_clip(float x) {
    if (x > 1.2f) return 0.99f;
    if (x < -1.2f) return -0.99f;
    return x - (x * x * x) * 0.16666667f;
}

static inline double kill_denormal(double val) {
    return (fabs(val) < 1.0e-15) ? 0.0 : val;
}

static void init_low_shelf(BiquadFilter64 *f, double freq, double gainDb, double sampleRate) {
    if (fabs(gainDb) < 0.01) {
        f->b0 = 1.0; f->b1 = 0; f->b2 = 0; f->a1 = 0; f->a2 = 0;
        return;
    }
    double A = pow(10.0, gainDb / 40.0);
    double w0 = 2.0 * M_PI * freq / sampleRate;
    double cos_w0 = cos(w0);
    double sin_w0 = sin(w0);
    double alpha = sin_w0 / 2.0 * sqrt((A + 1.0/A)*(1.0/0.707 - 1.0) + 2.0);
    double beta = 2.0 * sqrt(A) * alpha;

    double b0 = A * ((A + 1.0) - (A - 1.0)*cos_w0 + beta);
    double b1 = 2.0 * A * ((A - 1.0) - (A + 1.0)*cos_w0);
    double b2 = A * ((A + 1.0) - (A - 1.0)*cos_w0 - beta);
    double a0 = (A + 1.0) + (A - 1.0)*cos_w0 + beta;
    double a1 = -2.0 * ((A - 1.0) + (A + 1.0)*cos_w0);
    double a2 = (A + 1.0) + (A - 1.0)*cos_w0 - beta;

    f->b0 = b0 / a0; f->b1 = b1 / a0; f->b2 = b2 / a0;
    f->a1 = a1 / a0; f->a2 = a2 / a0;
}

static void init_peaking(BiquadFilter64 *f, double freq, double gainDb, double sampleRate, double Q) {
    if (fabs(gainDb) < 0.01) {
        f->b0 = 1.0; f->b1 = 0; f->b2 = 0; f->a1 = 0; f->a2 = 0;
        return;
    }
    double A = pow(10.0, gainDb / 40.0);
    double omega = 2.0 * M_PI * freq / sampleRate;
    double alpha = sin(omega) / (2.0 * Q);
    double cos_w = cos(omega);

    double b0 = 1.0 + alpha * A;
    double b1 = -2.0 * cos_w;
    double b2 = 1.0 - alpha * A;
    double a0 = 1.0 + alpha / A;
    double a1 = -2.0 * cos_w;
    double a2 = 1.0 - alpha / A;

    f->b0 = b0 / a0; f->b1 = b1 / a0; f->b2 = b2 / a0;
    f->a1 = a1 / a0; f->a2 = a2 / a0;
}

static void update_biquad_single(int i, double sampleRate) {
    if (i == 0) {
        init_low_shelf(&filters[0], FREQUENCIES[0], GAINS_DB[0], sampleRate);
    } else {
        init_peaking(&filters[i], FREQUENCIES[i], GAINS_DB[i], sampleRate, 1.30);
    }
}

static void update_all_biquads(double sampleRate) {
    gCurrentSampleRate = sampleRate;
    for (int i = 0; i < NUM_BANDS; i++) {
        update_biquad_single(i, sampleRate);
    }
}

static inline double process_L(BiquadFilter64 *f, double inSample) {
    double out = f->b0 * inSample + f->b1 * f->x1_L + f->b2 * f->x2_L - f->a1 * f->y1_L - f->a2 * f->y2_L;
    f->x2_L = f->x1_L; f->x1_L = inSample;
    f->y2_L = kill_denormal(f->y1_L); f->y1_L = kill_denormal(out);
    return out;
}

static inline double process_R(BiquadFilter64 *f, double inSample) {
    double out = f->b0 * inSample + f->b1 * f->x1_R + f->b2 * f->x2_R - f->a1 * f->y1_R - f->a2 * f->y2_R;
    f->x2_R = f->x1_R; f->x1_R = inSample;
    f->y2_R = kill_denormal(f->y1_R); f->y1_R = kill_denormal(out);
    return out;
}

static void process_pcm_raw(void *data, UInt32 byteSize, UInt32 channels, BOOL isFloat, UInt32 bitsPerChannel) {
    if (!data || byteSize == 0) return;
    gProcessedBufferCount++;

    double preampFactor = pow(10.0, PREAMP_DB / 20.0);

    if (isFloat || bitsPerChannel == 32) {
        float *samples = (float *)data;
        UInt32 totalSamples = byteSize / sizeof(float);
        totalSamples -= (totalSamples % (channels > 0 ? channels : 1));

        if (channels >= 2) {
            for (UInt32 i = 0; i < totalSamples; i += channels) {
                double sL = (double)samples[i] * preampFactor;
                double sR = (double)samples[i + 1] * preampFactor;
                for (int b = 0; b < NUM_BANDS; b++) {
                    sL = process_L(&filters[b], sL);
                    sR = process_R(&filters[b], sR);
                }
                samples[i]     = fast_soft_clip((float)sL);
                samples[i + 1] = fast_soft_clip((float)sR);
            }
        } else if (channels == 1) {
            for (UInt32 i = 0; i < totalSamples; i++) {
                double sL = (double)samples[i] * preampFactor;
                for (int b = 0; b < NUM_BANDS; b++) {
                    sL = process_L(&filters[b], sL);
                }
                samples[i] = fast_soft_clip((float)sL);
            }
        }
    } else if (bitsPerChannel == 16) {
        int16_t *samples = (int16_t *)data;
        UInt32 totalSamples = byteSize / sizeof(int16_t);
        totalSamples -= (totalSamples % (channels > 0 ? channels : 1));

        if (channels >= 2) {
            for (UInt32 i = 0; i < totalSamples; i += channels) {
                double sL = ((double)samples[i] / 32768.0) * preampFactor;
                double sR = ((double)samples[i + 1] / 32768.0) * preampFactor;
                for (int b = 0; b < NUM_BANDS; b++) {
                    sL = process_L(&filters[b], sL);
                    sR = process_R(&filters[b], sR);
                }
                samples[i]     = (int16_t)(fast_soft_clip((float)sL) * 32767.0f);
                samples[i + 1] = (int16_t)(fast_soft_clip((float)sR) * 32767.0f);
            }
        }
    }
}

static void process_audio_buffer_list(AudioBufferList *ioData) {
    if (!ioData || ioData->mNumberBuffers == 0) return;
    double preampFactor = pow(10.0, PREAMP_DB / 20.0);

    if (ioData->mNumberBuffers == 2) {
        gProcessedBufferCount++;
        float *samplesL = (float *)ioData->mBuffers[0].mData;
        float *samplesR = (float *)ioData->mBuffers[1].mData;
        UInt32 totalSamples = ioData->mBuffers[0].mDataByteSize / sizeof(float);

        if (samplesL && samplesR) {
            for (UInt32 i = 0; i < totalSamples; i++) {
                double sL = (double)samplesL[i] * preampFactor;
                double sR = (double)samplesR[i] * preampFactor;

                for (int b = 0; b < NUM_BANDS; b++) {
                    sL = process_L(&filters[b], sL);
                    sR = process_R(&filters[b], sR);
                }

                samplesL[i] = fast_soft_clip((float)sL);
                samplesR[i] = fast_soft_clip((float)sR);
            }
        }
    } else if (ioData->mNumberBuffers == 1) {
        AudioBuffer buf = ioData->mBuffers[0];
        process_pcm_raw(buf.mData, buf.mDataByteSize, buf.mNumberChannels > 0 ? buf.mNumberChannels : 2, YES, 32);
    }
}

static OSStatus (*orig_AudioQueueEnqueueBuffer)(AudioQueueRef, AudioQueueBufferRef, UInt32, const AudioStreamPacketDescription *);
static OSStatus (*orig_AudioUnitRender)(AudioUnit, AudioUnitRenderActionFlags *, const AudioTimeStamp *, UInt32, UInt32, AudioBufferList *);
static OSStatus (*orig_AudioConverterFillComplexBuffer)(AudioConverterRef, AudioConverterComplexInputDataProc, void *, UInt32 *, AudioBufferList *, AudioStreamPacketDescription *);

OSStatus my_AudioQueueEnqueueBuffer(AudioQueueRef inAQ, AudioQueueBufferRef inBuffer, UInt32 inNumPacketDescs, const AudioStreamPacketDescription *inPacketDescs) {
    if (inBuffer && inBuffer->mAudioData && inBuffer->mAudioDataByteSize > 0) {
        AudioStreamBasicDescription format;
        UInt32 propSize = sizeof(format);
        OSStatus err = AudioQueueGetProperty(inAQ, kAudioQueueProperty_StreamDescription, &format, &propSize);
        if (err == noErr && format.mFormatID == kAudioFormatLinearPCM) {
            static double lastSampleRate = 0.0;
            double currentSampleRate = format.mSampleRate > 0 ? format.mSampleRate : 44100.0;
            if (lastSampleRate != currentSampleRate) {
                update_all_biquads(currentSampleRate);
                lastSampleRate = currentSampleRate;
            }
            process_pcm_raw(inBuffer->mAudioData, inBuffer->mAudioDataByteSize, format.mChannelsPerFrame, (format.mFormatFlags & kLinearPCMFormatFlagIsFloat) != 0, format.mBitsPerChannel);
        } else {
            process_pcm_raw(inBuffer->mAudioData, inBuffer->mAudioDataByteSize, 2, YES, 32);
        }
    }
    return orig_AudioQueueEnqueueBuffer(inAQ, inBuffer, inNumPacketDescs, inPacketDescs);
}

OSStatus my_AudioUnitRender(AudioUnit inUnit, AudioUnitRenderActionFlags *ioActionFlags, const AudioTimeStamp *inTimeStamp, UInt32 inBusNumber, UInt32 inNumberFrames, AudioBufferList *ioData) {
    OSStatus status = orig_AudioUnitRender(inUnit, ioActionFlags, inTimeStamp, inBusNumber, inNumberFrames, ioData);
    if (status == noErr && ioData) {
        process_audio_buffer_list(ioData);
    }
    return status;
}

OSStatus my_AudioConverterFillComplexBuffer(AudioConverterRef inAudioConverter, AudioConverterComplexInputDataProc inInputDataProc, void *inInputDataProcUserData, UInt32 *ioOutputDataPacketSize, AudioBufferList *outOutputData, AudioStreamPacketDescription *outPacketDescription) {
    OSStatus status = orig_AudioConverterFillComplexBuffer(inAudioConverter, inInputDataProc, inInputDataProcUserData, ioOutputDataPacketSize, outOutputData, outPacketDescription);
    if (status == noErr && outOutputData) {
        process_audio_buffer_list(outOutputData);
    }
    return status;
}

@interface EQManager : NSObject
+ (instancetype)shared;
- (void)setupUI;
@end

@implementation EQManager {
    UIButton *_toggleBtn;
    UIVisualEffectView *_blurContainer;
    UISlider *_sliders[NUM_BANDS];
    UILabel *_valueLabels[NUM_BANDS];
    UISlider *_preampSlider;
    UILabel *_preampLabel;
    UIButton *_presetBtn;
    UILabel *_statusLabel;
    NSTimer *_statusTimer;
    UIImpactFeedbackGenerator *_hapticLight;
    UIImpactFeedbackGenerator *_hapticMedium;
}

+ (instancetype)shared {
    static EQManager *inst = nil;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{ inst = [[EQManager alloc] init]; });
    return inst;
}

- (UIViewController *)topViewController {
    UIViewController *top = [UIApplication sharedApplication].keyWindow.rootViewController;
    while (top.presentedViewController) {
        top = top.presentedViewController;
    }
    return top;
}

- (void)triggerHaptic:(int)type {
    if (type == 0) {
        if (!_hapticLight) _hapticLight = [[UIImpactFeedbackGenerator alloc] initWithStyle:UIImpactFeedbackStyleLight];
        [_hapticLight impactOccurred];
    } else {
        if (!_hapticMedium) _hapticMedium = [[UIImpactFeedbackGenerator alloc] initWithStyle:UIImpactFeedbackStyleMedium];
        [_hapticMedium impactOccurred];
    }
}

- (void)loadSettings {
    NSUserDefaults *defs = [NSUserDefaults standardUserDefaults];
    for (int i = 0; i < NUM_BANDS; i++) {
        NSString *key = [NSString stringWithFormat:@"eq_band_%d", i];
        if ([defs objectForKey:key]) {
            GAINS_DB[i] = [defs doubleForKey:key];
        } else {
            GAINS_DB[i] = default_gains[i];
        }
    }
    if ([defs objectForKey:@"eq_preamp"]) {
        PREAMP_DB = [defs doubleForKey:@"eq_preamp"];
    }
}

- (void)applyGainsAndRefreshUI {
    for (int i = 0; i < NUM_BANDS; i++) {
        _sliders[i].value = GAINS_DB[i];
        _valueLabels[i].text = [NSString stringWithFormat:@"%+.1f", GAINS_DB[i]];
        [[NSUserDefaults standardUserDefaults] setDouble:GAINS_DB[i] forKey:[NSString stringWithFormat:@"eq_band_%d", i]];
    }
    _preampSlider.value = PREAMP_DB;
    _preampLabel.text = [NSString stringWithFormat:@"%.1f dB", PREAMP_DB];
    [[NSUserDefaults standardUserDefaults] setDouble:PREAMP_DB forKey:@"eq_preamp"];
    update_all_biquads(gCurrentSampleRate);
}

- (void)updateStatusText {
    if (gProcessedBufferCount == 0) {
        _statusLabel.text = @"🔴 Ожидание звука... (Включите трек)";
        _statusLabel.textColor = [UIColor colorWithRed:1.0 green:0.4 blue:0.4 alpha:1.0];
    } else {
        _statusLabel.text = [NSString stringWithFormat:@"⚡ EarthQuake Sub DSP (%llu buf)", gProcessedBufferCount];
        _statusLabel.textColor = [UIColor colorWithRed:0.4 green:1.0 blue:0.4 alpha:1.0];
    }
}

- (void)setupUI {
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(2.5 * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{
        UIWindow *window = [UIApplication sharedApplication].keyWindow;
        if (!window) return;

        self->_toggleBtn = [UIButton buttonWithType:UIButtonTypeSystem];
        self->_toggleBtn.frame = CGRectMake(16, 120, 46, 48);
        self->_toggleBtn.backgroundColor = [UIColor colorWithRed:0.1 green:0.1 blue:0.12 alpha:0.85];
        [self->_toggleBtn setTitle:@"🎛️" forState:UIControlStateNormal];
        self->_toggleBtn.titleLabel.font = [UIFont systemFontOfSize:22];
        self->_toggleBtn.layer.cornerRadius = 23;
        self->_toggleBtn.layer.cornerCurve = kCACornerCurveContinuous;
        self->_toggleBtn.layer.borderColor = [UIColor colorWithRed:1.0 green:0.8 blue:0.0 alpha:0.9].CGColor;
        self->_toggleBtn.layer.borderWidth = 1.5;

        UIPanGestureRecognizer *pan = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(handlePan:)];
        [self->_toggleBtn addGestureRecognizer:pan];
        [self->_toggleBtn addTarget:self action:@selector(toggleMenu) forControlEvents:UIControlEventTouchUpInside];
        [window addSubview:self->_toggleBtn];

        UIBlurEffect *blurEffect = [UIBlurEffect effectWithStyle:UIBlurEffectStyleSystemUltraThinMaterialDark];
        CGFloat menuWidth = window.bounds.size.width - 32;
        
        self->_blurContainer = [[UIVisualEffectView alloc] initWithEffect:blurEffect];
        self->_blurContainer.frame = CGRectMake(16, 100, menuWidth, 510);
        self->_blurContainer.layer.cornerRadius = 22;
        self->_blurContainer.layer.cornerCurve = kCACornerCurveContinuous;
        self->_blurContainer.layer.masksToBounds = YES;
        self->_blurContainer.layer.borderColor = [UIColor colorWithWhite:1.0 alpha:0.15].CGColor;
        self->_blurContainer.layer.borderWidth = 0.5;
        self->_blurContainer.hidden = YES;
        self->_blurContainer.alpha = 0.0;
        [window addSubview:self->_blurContainer];

        UIView *contentView = self->_blurContainer.contentView;

        UILabel *title = [[UILabel alloc] initWithFrame:CGRectMake(16, 14, 110, 22)];
        title.text = @"Parametric EQ";
        title.textColor = [UIColor colorWithRed:1.0 green:0.82 blue:0.0 alpha:1.0];
        title.font = [UIFont systemFontOfSize:15 weight:UIFontWeightBold];
        [contentView addSubview:title];

        self->_presetBtn = [UIButton buttonWithType:UIButtonTypeSystem];
        self->_presetBtn.frame = CGRectMake(menuWidth - 165, 12, 65, 28);
        [self->_presetBtn setTitle:@"Presets" forState:UIControlStateNormal];
        [self->_presetBtn setTitleColor:[UIColor whiteColor] forState:UIControlStateNormal];
        self->_presetBtn.backgroundColor = [UIColor colorWithWhite:1.0 alpha:0.12];
        self->_presetBtn.layer.cornerRadius = 8;
        self->_presetBtn.layer.cornerCurve = kCACornerCurveContinuous;
        self->_presetBtn.titleLabel.font = [UIFont systemFontOfSize:11 weight:UIFontWeightSemibold];
        [self->_presetBtn addTarget:self action:@selector(showPresetMenu) forControlEvents:UIControlEventTouchUpInside];
        [contentView addSubview:self->_presetBtn];

        UIButton *saveBtn = [UIButton buttonWithType:UIButtonTypeSystem];
        saveBtn.frame = CGRectMake(menuWidth - 95, 12, 50, 28);
        [saveBtn setTitle:@"Save" forState:UIControlStateNormal];
        [saveBtn setTitleColor:[UIColor blackColor] forState:UIControlStateNormal];
        saveBtn.backgroundColor = [UIColor colorWithRed:1.0 green:0.82 blue:0.0 alpha:1.0];
        saveBtn.layer.cornerRadius = 8;
        saveBtn.layer.cornerCurve = kCACornerCurveContinuous;
        saveBtn.titleLabel.font = [UIFont systemFontOfSize:11 weight:UIFontWeightBold];
        [saveBtn addTarget:self action:@selector(showSavePresetAlert) forControlEvents:UIControlEventTouchUpInside];
        [contentView addSubview:saveBtn];

        UIButton *closeBtn = [UIButton buttonWithType:UIButtonTypeSystem];
        closeBtn.frame = CGRectMake(menuWidth - 38, 12, 28, 28);
        [closeBtn setTitle:@"✕" forState:UIControlStateNormal];
        [closeBtn setTitleColor:[UIColor whiteColor] forState:UIControlStateNormal];
        closeBtn.backgroundColor = [UIColor colorWithRed:0.9 green:0.2 blue:0.2 alpha:0.8];
        closeBtn.layer.cornerRadius = 14;
        closeBtn.titleLabel.font = [UIFont systemFontOfSize:14 weight:UIFontWeightBold];
        [closeBtn addTarget:self action:@selector(toggleMenu) forControlEvents:UIControlEventTouchUpInside];
        [contentView addSubview:closeBtn];

        UILabel *preampTitle = [[UILabel alloc] initWithFrame:CGRectMake(16, 48, 65, 20)];
        preampTitle.text = @"Preamp:";
        preampTitle.textColor = [UIColor colorWithWhite:0.7 alpha:1.0];
        preampTitle.font = [UIFont systemFontOfSize:12 weight:UIFontWeightBold];
        [contentView addSubview:preampTitle];

        self->_preampSlider = [[UISlider alloc] initWithFrame:CGRectMake(78, 48, menuWidth - 155, 20)];
        self->_preampSlider.minimumValue = -12.0;
        self->_preampSlider.maximumValue = 0.0;
        self->_preampSlider.tintColor = [UIColor colorWithRed:1.0 green:0.82 blue:0.0 alpha:1.0];
        self->_preampSlider.value = PREAMP_DB;
        [self->_preampSlider addTarget:self action:@selector(preampChanged:) forControlEvents:UIControlEventValueChanged];
        [contentView addSubview:self->_preampSlider];

        self->_preampLabel = [[UILabel alloc] initWithFrame:CGRectMake(menuWidth - 70, 48, 56, 20)];
        self->_preampLabel.text = [NSString stringWithFormat:@"%.1f dB", PREAMP_DB];
        self->_preampLabel.textColor = [UIColor whiteColor];
        self->_preampLabel.font = [UIFont monospacedDigitSystemFontOfSize:12 weight:UIFontWeightBold];
        self->_preampLabel.textAlignment = NSTextAlignmentRight;
        [contentView addSubview:self->_preampLabel];

        UIView *line = [[UIView alloc] initWithFrame:CGRectMake(16, 76, menuWidth - 32, 0.5)];
        line.backgroundColor = [UIColor colorWithWhite:1.0 alpha:0.15];
        [contentView addSubview:line];

        UIScrollView *scroll = [[UIScrollView alloc] initWithFrame:CGRectMake(8, 82, menuWidth - 16, 380)];
        scroll.showsVerticalScrollIndicator = NO;
        [contentView addSubview:scroll];

        int y = 6;
        for (int i = 0; i < NUM_BANDS; i++) {
            UILabel *lbl = [[UILabel alloc] initWithFrame:CGRectMake(8, y, 55, 26)];
            lbl.text = (i == 0) ? @"SubShelf" : [NSString stringWithFormat:@"%.0fHz", FREQUENCIES[i]];
            lbl.textColor = (i == 0) ? [UIColor colorWithRed:1.0 green:0.82 blue:0.0 alpha:1.0] : [UIColor whiteColor];
            lbl.font = [UIFont systemFontOfSize:(i == 0 ? 11 : 13) weight:UIFontWeightSemibold];
            [scroll addSubview:lbl];

            UIButton *minusBtn = [UIButton buttonWithType:UIButtonTypeSystem];
            minusBtn.frame = CGRectMake(62, y, 28, 26);
            [minusBtn setTitle:@"-" forState:UIControlStateNormal];
            [minusBtn setTitleColor:[UIColor whiteColor] forState:UIControlStateNormal];
            minusBtn.backgroundColor = [UIColor colorWithWhite:1.0 alpha:0.12];
            minusBtn.layer.cornerRadius = 6;
            minusBtn.layer.cornerCurve = kCACornerCurveContinuous;
            minusBtn.tag = i;
            [minusBtn addTarget:self action:@selector(stepMinus:) forControlEvents:UIControlEventTouchUpInside];
            [scroll addSubview:minusBtn];

            UISlider *slider = [[UISlider alloc] initWithFrame:CGRectMake(96, y, scroll.bounds.size.width - 198, 26)];
            slider.minimumValue = -15.0;
            slider.maximumValue = +15.0;
            slider.tintColor = [UIColor colorWithRed:1.0 green:0.82 blue:0.0 alpha:1.0];
            slider.value = GAINS_DB[i];
            slider.tag = i;
            [slider addTarget:self action:@selector(sliderChanged:) forControlEvents:UIControlEventValueChanged];
            self->_sliders[i] = slider;
            [scroll addSubview:slider];

            UIButton *plusBtn = [UIButton buttonWithType:UIButtonTypeSystem];
            plusBtn.frame = CGRectMake(scroll.bounds.size.width - 94, y, 28, 26);
            [plusBtn setTitle:@"+" forState:UIControlStateNormal];
            [plusBtn setTitleColor:[UIColor whiteColor] forState:UIControlStateNormal];
            plusBtn.backgroundColor = [UIColor colorWithWhite:1.0 alpha:0.12];
            plusBtn.layer.cornerRadius = 6;
            plusBtn.layer.cornerCurve = kCACornerCurveContinuous;
            plusBtn.tag = i;
            [plusBtn addTarget:self action:@selector(stepPlus:) forControlEvents:UIControlEventTouchUpInside];
            [scroll addSubview:plusBtn];

            UILabel *valLbl = [[UILabel alloc] initWithFrame:CGRectMake(scroll.bounds.size.width - 62, y, 58, 26)];
            valLbl.text = [NSString stringWithFormat:@"%+.1f", GAINS_DB[i]];
            valLbl.textColor = [UIColor colorWithRed:1.0 green:0.82 blue:0.0 alpha:1.0];
            valLbl.font = [UIFont monospacedDigitSystemFontOfSize:13 weight:UIFontWeightBold];
            valLbl.textAlignment = NSTextAlignmentRight;
            self->_valueLabels[i] = valLbl;
            [scroll addSubview:valLbl];

            y += 45;
        }

        scroll.contentSize = CGSizeMake(scroll.bounds.size.width, y + 10);

        UIView *line2 = [[UIView alloc] initWithFrame:CGRectMake(16, 470, menuWidth - 32, 0.5)];
        line2.backgroundColor = [UIColor colorWithWhite:1.0 alpha:0.15];
        [contentView addSubview:line2];

        self->_statusLabel = [[UILabel alloc] initWithFrame:CGRectMake(16, 478, menuWidth - 32, 20)];
        self->_statusLabel.font = [UIFont systemFontOfSize:11 weight:UIFontWeightMedium];
        self->_statusLabel.textAlignment = NSTextAlignmentCenter;
        [self updateStatusText];
        [contentView addSubview:self->_statusLabel];

        self->_statusTimer = [NSTimer scheduledTimerWithTimeInterval:1.0 target:self selector:@selector(updateStatusText) userInfo:nil repeats:YES];
    });
}

- (void)handlePan:(UIPanGestureRecognizer *)pan {
    CGPoint translation = [pan translationInView:_toggleBtn.superview];
    CGFloat newX = _toggleBtn.center.x + translation.x;
    CGFloat newY = _toggleBtn.center.y + translation.y;
    
    CGFloat minX = 30, maxX = _toggleBtn.superview.bounds.size.width - 30;
    CGFloat minY = 60, maxY = _toggleBtn.superview.bounds.size.height - 60;
    
    if (newX < minX) newX = minX; if (newX > maxX) newX = maxX;
    if (newY < minY) newY = minY; if (newY > maxY) newY = maxY;

    _toggleBtn.center = CGPointMake(newX, newY);
    [pan setTranslation:CGPointZero inView:_toggleBtn.superview];
}

- (void)toggleMenu {
    [self triggerHaptic:1];
    BOOL isHidden = _blurContainer.hidden;
    
    if (isHidden) {
        _blurContainer.hidden = NO;
        [UIView animateWithDuration:0.25 animations:^{
            self->_blurContainer.alpha = 1.0;
        }];
    } else {
        [UIView animateWithDuration:0.2 animations:^{
            self->_blurContainer.alpha = 0.0;
        } completion:^(BOOL finished) {
            self->_blurContainer.hidden = YES;
        }];
    }
}

- (void)sliderChanged:(UISlider *)slider {
    int idx = (int)slider.tag;
    GAINS_DB[idx] = slider.value;
    _valueLabels[idx].text = [NSString stringWithFormat:@"%+.1f", slider.value];
    update_biquad_single(idx, gCurrentSampleRate);
    [[NSUserDefaults standardUserDefaults] setDouble:slider.value forKey:[NSString stringWithFormat:@"eq_band_%d", idx]];
}

- (void)stepMinus:(UIButton *)btn {
    [self triggerHaptic:0];
    int idx = (int)btn.tag;
    float val = _sliders[idx].value - 0.5f;
    if (val < -15.0f) val = -15.0f;
    _sliders[idx].value = val;
    [self sliderChanged:_sliders[idx]];
}

- (void)stepPlus:(UIButton *)btn {
    [self triggerHaptic:0];
    int idx = (int)btn.tag;
    float val = _sliders[idx].value + 0.5f;
    if (val > 15.0f) val = 15.0f;
    _sliders[idx].value = val;
    [self sliderChanged:_sliders[idx]];
}

- (void)preampChanged:(UISlider *)slider {
    PREAMP_DB = slider.value;
    _preampLabel.text = [NSString stringWithFormat:@"%.1f dB", PREAMP_DB];
    [[NSUserDefaults standardUserDefaults] setDouble:slider.value forKey:@"eq_preamp"];
}

- (void)showSavePresetAlert {
    [self triggerHaptic:0];
    UIAlertController *alert = [UIAlertController alertControllerWithTitle:@"Сохранить пресет" message:@"Введите название для ваших настроек:" preferredStyle:UIAlertControllerStyleAlert];
    
    [alert addTextFieldWithConfigurationHandler:^(UITextField * _Nonnull textField) {
        textField.placeholder = @"Например: Мои AirPods";
    }];

    UIAlertAction *saveAction = [UIAlertAction actionWithTitle:@"Сохранить" style:UIAlertActionStyleDefault handler:^(UIAlertAction * _Nonnull action) {
        NSString *name = alert.textFields.firstObject.text;
        if (name && name.length > 0) {
            NSMutableDictionary *presets = [[[NSUserDefaults standardUserDefaults] dictionaryForKey:@"eq_custom_presets"] mutableCopy];
            if (!presets) presets = [NSMutableDictionary dictionary];

            NSMutableArray *gainsArr = [NSMutableArray array];
            for (int i = 0; i < NUM_BANDS; i++) {
                [gainsArr addObject:@(GAINS_DB[i])];
            }

            NSDictionary *presetData = @{
                @"gains": gainsArr,
                @"preamp": @(PREAMP_DB)
            };

            [presets setObject:presetData forKey:name];
            [[NSUserDefaults standardUserDefaults] setObject:presets forKey:@"eq_custom_presets"];
            [[NSUserDefaults standardUserDefaults] synchronize];
        }
    }];

    [alert addAction:saveAction];
    [alert addAction:[UIAlertAction actionWithTitle:@"Отмена" style:UIAlertActionStyleCancel handler:nil]];
    
    [[self topViewController] presentViewController:alert animated:YES completion:nil];
}

- (void)showPresetMenu {
    [self triggerHaptic:0];
    UIAlertController *sheet = [UIAlertController alertControllerWithTitle:@"Пресеты" message:nil preferredStyle:UIAlertControllerStyleActionSheet];

    [sheet addAction:[UIAlertAction actionWithTitle:@"🔄 Flat (Сбросить в 0)" style:UIAlertActionStyleDefault handler:^(UIAlertAction * _Nonnull action) {
        for (int i = 0; i < NUM_BANDS; i++) GAINS_DB[i] = 0.0;
        PREAMP_DB = 0.0;
        [self applyGainsAndRefreshUI];
    }]];

    [sheet addAction:[UIAlertAction actionWithTitle:@"⚡ EarthQuake 20-40Hz (Макс. Глубина)" style:UIAlertActionStyleDefault handler:^(UIAlertAction * _Nonnull action) {
        double sub_gains[NUM_BANDS] = {12.0, 6.0, 2.0, -3.0, -5.0, 0.0, 1.5};
        for (int i = 0; i < NUM_BANDS; i++) GAINS_DB[i] = sub_gains[i];
        PREAMP_DB = -7.0;
        [self applyGainsAndRefreshUI];
    }]];

    [sheet addAction:[UIAlertAction actionWithTitle:@"🎧 Deep Pressure (Мягкий глубокий басс)" style:UIAlertActionStyleDefault handler:^(UIAlertAction * _Nonnull action) {
        double deep_gains[NUM_BANDS] = {9.0, 8.0, 3.0, -2.0, -4.0, 0.0, 1.0};
        for (int i = 0; i < NUM_BANDS; i++) GAINS_DB[i] = deep_gains[i];
        PREAMP_DB = -5.5;
        [self applyGainsAndRefreshUI];
    }]];

    NSDictionary *presets = [[NSUserDefaults standardUserDefaults] dictionaryForKey:@"eq_custom_presets"];
    for (NSString *presetName in presets.allKeys) {
        [sheet addAction:[UIAlertAction actionWithTitle:[NSString stringWithFormat:@"⭐ %@", presetName] style:UIAlertActionStyleDefault handler:^(UIAlertAction * _Nonnull action) {
            NSDictionary *data = presets[presetName];
            NSArray *gains = data[@"gains"];
            NSNumber *preamp = data[@"preamp"];

            if (gains && gains.count == NUM_BANDS) {
                for (int i = 0; i < NUM_BANDS; i++) {
                    GAINS_DB[i] = [gains[i] doubleValue];
                }
            }
            if (preamp) PREAMP_DB = [preamp doubleValue];
            [self applyGainsAndRefreshUI];
        }]];
    }

    if (presets && presets.count > 0) {
        [sheet addAction:[UIAlertAction actionWithTitle:@"🗑️ Удалить пресет..." style:UIAlertActionStyleDestructive handler:^(UIAlertAction * _Nonnull action) {
            [self showDeletePresetMenu];
        }]];
    }

    [sheet addAction:[UIAlertAction actionWithTitle:@"Отмена" style:UIAlertActionStyleCancel handler:nil]];
    
    sheet.popoverPresentationController.sourceView = _presetBtn;
    sheet.popoverPresentationController.sourceRect = _presetBtn.bounds;

    [[self topViewController] presentViewController:sheet animated:YES completion:nil];
}

- (void)showDeletePresetMenu {
    UIAlertController *sheet = [UIAlertController alertControllerWithTitle:@"Удалить пресет" message:@"Выберите пресет для удаления:" preferredStyle:UIAlertControllerStyleActionSheet];
    NSMutableDictionary *presets = [[[NSUserDefaults standardUserDefaults] dictionaryForKey:@"eq_custom_presets"] mutableCopy];

    for (NSString *presetName in presets.allKeys) {
        [sheet addAction:[UIAlertAction actionWithTitle:presetName style:UIAlertActionStyleDestructive handler:^(UIAlertAction * _Nonnull action) {
            [presets removeObjectForKey:presetName];
            [[NSUserDefaults standardUserDefaults] setObject:presets forKey:@"eq_custom_presets"];
            [[NSUserDefaults standardUserDefaults] synchronize];
        }]];
    }

    [sheet addAction:[UIAlertAction actionWithTitle:@"Отмена" style:UIAlertActionStyleCancel handler:nil]];
    [[self topViewController] presentViewController:sheet animated:YES completion:nil];
}
@end

__attribute__((constructor))
static void init_eq_tweak(void) {
    [[EQManager shared] loadSettings];
    
    struct rebinding rebinds[3];
    rebinds[0].name = "AudioQueueEnqueueBuffer";
    rebinds[0].replacement = (void *)(uintptr_t)my_AudioQueueEnqueueBuffer;
    rebinds[0].replaced = (void **)(uintptr_t)&orig_AudioQueueEnqueueBuffer;

    rebinds[1].name = "AudioUnitRender";
    rebinds[1].replacement = (void *)(uintptr_t)my_AudioUnitRender;
    rebinds[1].replaced = (void **)(uintptr_t)&orig_AudioUnitRender;

    rebinds[2].name = "AudioConverterFillComplexBuffer";
    rebinds[2].replacement = (void *)(uintptr_t)my_AudioConverterFillComplexBuffer;
    rebinds[2].replaced = (void **)(uintptr_t)&orig_AudioConverterFillComplexBuffer;
    
    rebind_symbols(rebinds, 3);

    [[NSNotificationCenter defaultCenter] addObserverForName:UIApplicationDidFinishLaunchingNotification object:nil queue:[NSOperationQueue mainQueue] usingBlock:^(NSNotification * _Nonnull note) {
        [[EQManager shared] setupUI];
    }];
}
