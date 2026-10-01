#import <UIKit/UIKit.h>
#import <AudioToolbox/AudioToolbox.h>
#import <mach-o/dyld.h>
#import <dlfcn.h>
#import <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define BANDS 7

static double FREQS[BANDS] = {50.0, 80.0, 160.0, 400.0, 1000.0, 2500.0, 6000.0};
static double DEFAULT_GAINS[BANDS] = {8.0, 5.0, 3.0, -2.0, -4.0, 0.0, 1.0};
static double gGains[BANDS];
static double gPreamp = -5.0;
static uint64_t gBufCount = 0;
static double gSampleRate = 44100.0;

typedef struct {
    double b0, b1, b2, a1, a2;
    double x1_L, x2_L, y1_L, y2_L;
    double x1_R, x2_R, y1_R, y2_R;
} Biquad;

static Biquad gFilters[BANDS];

static inline float soft_clip(float x) {
    if (x > 1.2f) return 0.99f;
    if (x < -1.2f) return -0.99f;
    return x - (x * x * x) * 0.16666667f;
}

static inline double kill_denormal(double v) {
    return (fabs(v) < 1.0e-15) ? 0.0 : v;
}

static void set_low_shelf(Biquad *f, double freq, double gain, double sr) {
    if (fabs(gain) < 0.01) { f->b0 = 1.0; f->b1 = 0; f->b2 = 0; f->a1 = 0; f->a2 = 0; return; }
    double A = pow(10.0, gain / 40.0);
    double w0 = 2.0 * M_PI * freq / sr;
    double cw = cos(w0), sw = sin(w0);
    double alpha = sw / 2.0 * sqrt((A + 1.0 / A) * (1.0 / 0.707 - 1.0) + 2.0);
    double beta = 2.0 * sqrt(A) * alpha;

    double a0 = (A + 1.0) + (A - 1.0) * cw + beta;
    f->b0 = (A * ((A + 1.0) - (A - 1.0) * cw + beta)) / a0;
    f->b1 = (2.0 * A * ((A - 1.0) - (A + 1.0) * cw)) / a0;
    f->b2 = (A * ((A + 1.0) - (A - 1.0) * cw - beta)) / a0;
    f->a1 = (-2.0 * ((A - 1.0) + (A + 1.0) * cw)) / a0;
    f->a2 = ((A + 1.0) + (A - 1.0) * cw - beta) / a0;
}

static void set_peaking(Biquad *f, double freq, double gain, double sr, double Q) {
    if (fabs(gain) < 0.01) { f->b0 = 1.0; f->b1 = 0; f->b2 = 0; f->a1 = 0; f->a2 = 0; return; }
    double A = pow(10.0, gain / 40.0);
    double w = 2.0 * M_PI * freq / sr;
    double alpha = sin(w) / (2.0 * Q);
    double cw = cos(w);

    double a0 = 1.0 + alpha / A;
    f->b0 = (1.0 + alpha * A) / a0;
    f->b1 = (-2.0 * cw) / a0;
    f->b2 = (1.0 - alpha * A) / a0;
    f->a1 = (-2.0 * cw) / a0;
    f->a2 = (1.0 - alpha / A) / a0;
}

static void update_filter(int i, double sr) {
    if (i == 0) set_low_shelf(&gFilters[0], FREQS[0], gGains[0], sr);
    else set_peaking(&gFilters[i], FREQS[i], gGains[i], sr, 1.30);
}

static void update_all_filters(double sr) {
    gSampleRate = sr;
    for (int i = 0; i < BANDS; i++) update_filter(i, sr);
}

static inline double process_L(Biquad *f, double in) {
    double out = f->b0 * in + f->b1 * f->x1_L + f->b2 * f->x2_L - f->a1 * f->y1_L - f->a2 * f->y2_L;
    f->x2_L = f->x1_L; f->x1_L = in;
    f->y2_L = kill_denormal(f->y1_L); f->y1_L = kill_denormal(out);
    return out;
}

static inline double process_R(Biquad *f, double in) {
    double out = f->b0 * in + f->b1 * f->x1_R + f->b2 * f->x2_R - f->a1 * f->y1_R - f->a2 * f->y2_R;
    f->x2_R = f->x1_R; f->x1_R = in;
    f->y2_R = kill_denormal(f->y1_R); f->y1_R = kill_denormal(out);
    return out;
}

static void process_pcm(void *data, UInt32 size, UInt32 ch, BOOL isFloat, UInt32 bits) {
    if (!data || !size) return;
    gBufCount++;
    double preamp = pow(10.0, gPreamp / 20.0);

    if (isFloat || bits == 32) {
        float *buf = (float *)data;
        UInt32 count = (size / sizeof(float));
        if (ch >= 2) {
            count -= (count % ch);
            for (UInt32 i = 0; i < count; i += ch) {
                double L = buf[i] * preamp, R = buf[i + 1] * preamp;
                for (int b = 0; b < BANDS; b++) {
                    L = process_L(&gFilters[b], L);
                    R = process_R(&gFilters[b], R);
                }
                buf[i] = soft_clip((float)L);
                buf[i + 1] = soft_clip((float)R);
            }
        } else if (ch == 1) {
            for (UInt32 i = 0; i < count; i++) {
                double L = buf[i] * preamp;
                for (int b = 0; b < BANDS; b++) L = process_L(&gFilters[b], L);
                buf[i] = soft_clip((float)L);
            }
        }
    } else if (bits == 16) {
        int16_t *buf = (int16_t *)data;
        UInt32 count = (size / sizeof(int16_t));
        if (ch >= 2) {
            count -= (count % ch);
            for (UInt32 i = 0; i < count; i += ch) {
                double L = (buf[i] / 32768.0) * preamp;
                double R = (buf[i + 1] / 32768.0) * preamp;
                for (int b = 0; b < BANDS; b++) {
                    L = process_L(&gFilters[b], L);
                    R = process_R(&gFilters[b], R);
                }
                buf[i] = (int16_t)(soft_clip((float)L) * 32767.0f);
                buf[i + 1] = (int16_t)(soft_clip((float)R) * 32767.0f);
            }
        }
    }
}

static void process_buffer_list(AudioBufferList *ioData) {
    if (!ioData || !ioData->mNumberBuffers) return;
    double preamp = pow(10.0, gPreamp / 20.0);

    if (ioData->mNumberBuffers == 2) {
        gBufCount++;
        float *L = (float *)ioData->mBuffers[0].mData;
        float *R = (float *)ioData->mBuffers[1].mData;
        UInt32 count = ioData->mBuffers[0].mDataByteSize / sizeof(float);
        if (L && R) {
            for (UInt32 i = 0; i < count; i++) {
                double sL = L[i] * preamp, sR = R[i] * preamp;
                for (int b = 0; b < BANDS; b++) {
                    sL = process_L(&gFilters[b], sL);
                    sR = process_R(&gFilters[b], sR);
                }
                L[i] = soft_clip((float)sL);
                R[i] = soft_clip((float)sR);
            }
        }
    } else if (ioData->mNumberBuffers == 1) {
        AudioBuffer b = ioData->mBuffers[0];
        process_pcm(b.mData, b.mDataByteSize, b.mNumberChannels ? b.mNumberChannels : 2, YES, 32);
    }
}

static OSStatus (*orig_AQEnqueue)(AudioQueueRef, AudioQueueBufferRef, UInt32, const AudioStreamPacketDescription *);
static OSStatus (*orig_AURender)(AudioUnit, AudioUnitRenderActionFlags *, const AudioTimeStamp *, UInt32, UInt32, AudioBufferList *);
static OSStatus (*orig_ACFill)(AudioConverterRef, AudioConverterComplexInputDataProc, void *, UInt32 *, AudioBufferList *, AudioStreamPacketDescription *);

static OSStatus my_AQEnqueue(AudioQueueRef aq, AudioQueueBufferRef buf, UInt32 n, const AudioStreamPacketDescription *descs) {
    if (buf && buf->mAudioData && buf->mAudioDataByteSize) {
        AudioStreamBasicDescription fmt;
        UInt32 sz = sizeof(fmt);
        if (AudioQueueGetProperty(aq, kAudioQueueProperty_StreamDescription, &fmt, &sz) == noErr && fmt.mFormatID == kAudioFormatLinearPCM) {
            static double lastSR = 0;
            double sr = fmt.mSampleRate > 0 ? fmt.mSampleRate : 44100.0;
            if (lastSR != sr) { update_all_filters(sr); lastSR = sr; }
            process_pcm(buf->mAudioData, buf->mAudioDataByteSize, fmt.mChannelsPerFrame, (fmt.mFormatFlags & kLinearPCMFormatFlagIsFloat) != 0, fmt.mBitsPerChannel);
        } else {
            process_pcm(buf->mAudioData, buf->mAudioDataByteSize, 2, YES, 32);
        }
    }
    return orig_AQEnqueue(aq, buf, n, descs);
}

static OSStatus my_AURender(AudioUnit unit, AudioUnitRenderActionFlags *flags, const AudioTimeStamp *ts, UInt32 bus, UInt32 frames, AudioBufferList *data) {
    OSStatus status = orig_AURender(unit, flags, ts, bus, frames, data);
    if (status == noErr && data) process_buffer_list(data);
    return status;
}

static OSStatus my_ACFill(AudioConverterRef conv, AudioConverterComplexInputDataProc proc, void *user, UInt32 *packets, AudioBufferList *data, AudioStreamPacketDescription *descs) {
    OSStatus status = orig_ACFill(conv, proc, user, packets, data, descs);
    if (status == noErr && data) process_buffer_list(data);
    return status;
}

static void init_hooks(void) {
    void (*MSHook)(void *, void *, void **) = (void (*)(void *, void *, void **))dlsym(RTLD_DEFAULT, "MSHookFunction");
    if (MSHook) {
        void *fn;
        if ((fn = dlsym(RTLD_DEFAULT, "AudioQueueEnqueueBuffer"))) MSHook(fn, (void *)(uintptr_t)my_AQEnqueue, (void **)(uintptr_t)&orig_AQEnqueue);
        if ((fn = dlsym(RTLD_DEFAULT, "AudioUnitRender"))) MSHook(fn, (void *)(uintptr_t)my_AURender, (void **)(uintptr_t)&orig_AURender);
        if ((fn = dlsym(RTLD_DEFAULT, "AudioConverterFillComplexBuffer"))) MSHook(fn, (void *)(uintptr_t)my_ACFill, (void **)(uintptr_t)&orig_ACFill);
    }
}

@interface YEQWindow : UIWindow
@end
@implementation YEQWindow
- (UIView *)hitTest:(CGPoint)point withEvent:(UIEvent *)event {
    UIView *v = [super hitTest:point withEvent:event];
    return (v == self || v == self.rootViewController.view) ? nil : v;
}
@end

@interface YEQVC : UIViewController
@end
@implementation YEQVC
- (BOOL)prefersStatusBarHidden { return NO; }
@end

@interface EQManager : NSObject
+ (instancetype)shared;
- (void)setupUI;
- (void)loadSettings;
@end

@implementation EQManager {
    YEQWindow *_win;
    YEQVC *_vc;
    UIButton *_btn;
    UIVisualEffectView *_blur;
    UISlider *_sliders[BANDS];
    UILabel *_labels[BANDS];
    UISlider *_preampSlider;
    UILabel *_preampLabel;
    UIButton *_presetBtn;
    UILabel *_statusLabel;
    NSTimer *_timer;
}

+ (instancetype)shared {
    static EQManager *m = nil;
    static dispatch_once_t t;
    dispatch_once(&t, ^{ m = [[EQManager alloc] init]; });
    return m;
}

- (void)loadSettings {
    NSUserDefaults *defs = [NSUserDefaults standardUserDefaults];
    for (int i = 0; i < BANDS; i++) {
        NSString *k = [NSString stringWithFormat:@"eq_band_%d", i];
        gGains[i] = [defs objectForKey:k] ? [defs doubleForKey:k] : DEFAULT_GAINS[i];
    }
    if ([defs objectForKey:@"eq_preamp"]) gPreamp = [defs doubleForKey:@"eq_preamp"];
}

- (void)applyGains {
    for (int i = 0; i < BANDS; i++) {
        _sliders[i].value = gGains[i];
        _labels[i].text = [NSString stringWithFormat:@"%+.1f", gGains[i]];
        [[NSUserDefaults standardUserDefaults] setDouble:gGains[i] forKey:[NSString stringWithFormat:@"eq_band_%d", i]];
    }
    _preampSlider.value = gPreamp;
    _preampLabel.text = [NSString stringWithFormat:@"%.1f dB", gPreamp];
    [[NSUserDefaults standardUserDefaults] setDouble:gPreamp forKey:@"eq_preamp"];
    update_all_filters(gSampleRate);
}

- (void)updateStatus {
    if (gBufCount == 0) {
        _statusLabel.text = @"🔴 Ожидание звука...";
        _statusLabel.textColor = [UIColor colorWithRed:1.0 green:0.4 blue:0.4 alpha:1.0];
    } else {
        _statusLabel.text = [NSString stringWithFormat:@"🟢 DSP Active (%llu buf)", gBufCount];
        _statusLabel.textColor = [UIColor colorWithRed:0.4 green:1.0 blue:0.4 alpha:1.0];
    }
}

- (void)setupUI {
    if (_win) return;
    _win = [[YEQWindow alloc] initWithFrame:[UIScreen mainScreen].bounds];
    if (@available(iOS 13.0, *)) {
        for (UIWindowScene *s in [UIApplication sharedApplication].connectedScenes) {
            if (s.activationState == UISceneActivationStateForegroundActive) {
                _win.windowScene = s;
                break;
            }
        }
    }
    _vc = [[YEQVC alloc] init];
    _vc.view.backgroundColor = [UIColor clearColor];
    _win.rootViewController = _vc;
    _win.windowLevel = UIWindowLevelStatusBar + 100;
    _win.hidden = NO;

    UIView *p = _vc.view;
    CGFloat w = p.bounds.size.width - 32;

    _btn = [UIButton buttonWithType:UIButtonTypeSystem];
    _btn.frame = CGRectMake(16, 120, 48, 48);
    _btn.backgroundColor = [UIColor colorWithRed:0.1 green:0.1 blue:0.12 alpha:0.85];
    [_btn setTitle:@"🎛️" forState:UIControlStateNormal];
    _btn.titleLabel.font = [UIFont systemFontOfSize:22];
    _btn.layer.cornerRadius = 24;
    _btn.layer.borderColor = [UIColor colorWithRed:1.0 green:0.8 blue:0.0 alpha:0.9].CGColor;
    _btn.layer.borderWidth = 1.5;

    UIPanGestureRecognizer *pan = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(onPan:)];
    [_btn addGestureRecognizer:pan];
    [_btn addTarget:self action:@selector(toggle) forControlEvents:UIControlEventTouchUpInside];
    [p addSubview:_btn];

    _blur = [[UIVisualEffectView alloc] initWithEffect:[UIBlurEffect effectWithStyle:UIBlurEffectStyleSystemUltraThinMaterialDark]];
    _blur.frame = CGRectMake(16, 100, w, 510);
    _blur.layer.cornerRadius = 22;
    _blur.layer.masksToBounds = YES;
    _blur.layer.borderColor = [UIColor colorWithWhite:1.0 alpha:0.15].CGColor;
    _blur.layer.borderWidth = 0.5;
    _blur.hidden = YES;
    _blur.alpha = 0.0;
    [p addSubview:_blur];

    UIView *c = _blur.contentView;

    UILabel *title = [[UILabel alloc] initWithFrame:CGRectMake(16, 14, 110, 22)];
    title.text = @"Parametric EQ";
    title.textColor = [UIColor colorWithRed:1.0 green:0.82 blue:0.0 alpha:1.0];
    title.font = [UIFont systemFontOfSize:15 weight:UIFontWeightBold];
    [c addSubview:title];

    _presetBtn = [UIButton buttonWithType:UIButtonTypeSystem];
    _presetBtn.frame = CGRectMake(w - 165, 12, 65, 28);
    [_presetBtn setTitle:@"Presets" forState:UIControlStateNormal];
    [_presetBtn setTitleColor:[UIColor whiteColor] forState:UIControlStateNormal];
    _presetBtn.backgroundColor = [UIColor colorWithWhite:1.0 alpha:0.12];
    _presetBtn.layer.cornerRadius = 8;
    _presetBtn.titleLabel.font = [UIFont systemFontOfSize:11 weight:UIFontWeightSemibold];
    [_presetBtn addTarget:self action:@selector(showPresets) forControlEvents:UIControlEventTouchUpInside];
    [c addSubview:_presetBtn];

    UIButton *saveBtn = [UIButton buttonWithType:UIButtonTypeSystem];
    saveBtn.frame = CGRectMake(w - 95, 12, 50, 28);
    [saveBtn setTitle:@"Save" forState:UIControlStateNormal];
    [saveBtn setTitleColor:[UIColor blackColor] forState:UIControlStateNormal];
    saveBtn.backgroundColor = [UIColor colorWithRed:1.0 green:0.82 blue:0.0 alpha:1.0];
    saveBtn.layer.cornerRadius = 8;
    saveBtn.titleLabel.font = [UIFont systemFontOfSize:11 weight:UIFontWeightBold];
    [saveBtn addTarget:self action:@selector(savePreset) forControlEvents:UIControlEventTouchUpInside];
    [c addSubview:saveBtn];

    UIButton *closeBtn = [UIButton buttonWithType:UIButtonTypeSystem];
    closeBtn.frame = CGRectMake(w - 38, 12, 28, 28);
    [closeBtn setTitle:@"✕" forState:UIControlStateNormal];
    [closeBtn setTitleColor:[UIColor whiteColor] forState:UIControlStateNormal];
    closeBtn.backgroundColor = [UIColor colorWithRed:0.9 green:0.2 blue:0.2 alpha:0.8];
    closeBtn.layer.cornerRadius = 14;
    closeBtn.titleLabel.font = [UIFont systemFontOfSize:14 weight:UIFontWeightBold];
    [closeBtn addTarget:self action:@selector(toggle) forControlEvents:UIControlEventTouchUpInside];
    [c addSubview:closeBtn];

    UILabel *preLabel = [[UILabel alloc] initWithFrame:CGRectMake(16, 48, 65, 20)];
    preLabel.text = @"Preamp:";
    preLabel.textColor = [UIColor colorWithWhite:0.7 alpha:1.0];
    preLabel.font = [UIFont systemFontOfSize:12 weight:UIFontWeightBold];
    [c addSubview:preLabel];

    _preampSlider = [[UISlider alloc] initWithFrame:CGRectMake(78, 48, w - 155, 20)];
    _preampSlider.minimumValue = -12.0;
    _preampSlider.maximumValue = 0.0;
    _preampSlider.tintColor = [UIColor colorWithRed:1.0 green:0.82 blue:0.0 alpha:1.0];
    _preampSlider.value = gPreamp;
    [_preampSlider addTarget:self action:@selector(onPreamp:) forControlEvents:UIControlEventValueChanged];
    [c addSubview:_preampSlider];

    _preampLabel = [[UILabel alloc] initWithFrame:CGRectMake(w - 70, 48, 56, 20)];
    _preampLabel.text = [NSString stringWithFormat:@"%.1f dB", gPreamp];
    _preampLabel.textColor = [UIColor whiteColor];
    _preampLabel.font = [UIFont monospacedDigitSystemFontOfSize:12 weight:UIFontWeightBold];
    _preampLabel.textAlignment = NSTextAlignmentRight;
    [c addSubview:_preampLabel];

    UIView *div = [[UIView alloc] initWithFrame:CGRectMake(16, 76, w - 32, 0.5)];
    div.backgroundColor = [UIColor colorWithWhite:1.0 alpha:0.15];
    [c addSubview:div];

    UIScrollView *scroll = [[UIScrollView alloc] initWithFrame:CGRectMake(8, 82, w - 16, 380)];
    scroll.showsVerticalScrollIndicator = NO;
    [c addSubview:scroll];

    int y = 6;
    for (int i = 0; i < BANDS; i++) {
        UILabel *lbl = [[UILabel alloc] initWithFrame:CGRectMake(8, y, 60, 26)];
        lbl.text = (i == 0) ? @"Sub" : (FREQS[i] >= 1000 ? [NSString stringWithFormat:@"%.1fk", FREQS[i] / 1000.0] : [NSString stringWithFormat:@"%.0f", FREQS[i]]);
        lbl.textColor = (i == 0) ? [UIColor colorWithRed:1.0 green:0.82 blue:0.0 alpha:1.0] : [UIColor whiteColor];
        lbl.font = [UIFont systemFontOfSize:12 weight:UIFontWeightSemibold];
        [scroll addSubview:lbl];

        UIButton *mBtn = [UIButton buttonWithType:UIButtonTypeSystem];
        mBtn.frame = CGRectMake(68, y, 28, 26);
        [mBtn setTitle:@"-" forState:UIControlStateNormal];
        [mBtn setTitleColor:[UIColor whiteColor] forState:UIControlStateNormal];
        mBtn.backgroundColor = [UIColor colorWithWhite:1.0 alpha:0.12];
        mBtn.layer.cornerRadius = 6;
        mBtn.tag = i;
        [mBtn addTarget:self action:@selector(onMinus:) forControlEvents:UIControlEventTouchUpInside];
        [scroll addSubview:mBtn];

        UISlider *sl = [[UISlider alloc] initWithFrame:CGRectMake(102, y, scroll.bounds.size.width - 204, 26)];
        sl.minimumValue = -15.0;
        sl.maximumValue = 15.0;
        sl.tintColor = [UIColor colorWithRed:1.0 green:0.82 blue:0.0 alpha:1.0];
        sl.value = gGains[i];
        sl.tag = i;
        [sl addTarget:self action:@selector(onSlider:) forControlEvents:UIControlEventValueChanged];
        _sliders[i] = sl;
        [scroll addSubview:sl];

        UIButton *pBtn = [UIButton buttonWithType:UIButtonTypeSystem];
        pBtn.frame = CGRectMake(scroll.bounds.size.width - 94, y, 28, 26);
        [pBtn setTitle:@"+" forState:UIControlStateNormal];
        [pBtn setTitleColor:[UIColor whiteColor] forState:UIControlStateNormal];
        pBtn.backgroundColor = [UIColor colorWithWhite:1.0 alpha:0.12];
        pBtn.layer.cornerRadius = 6;
        pBtn.tag = i;
        [pBtn addTarget:self action:@selector(onPlus:) forControlEvents:UIControlEventTouchUpInside];
        [scroll addSubview:pBtn];

        UILabel *vLbl = [[UILabel alloc] initWithFrame:CGRectMake(scroll.bounds.size.width - 62, y, 58, 26)];
        vLbl.text = [NSString stringWithFormat:@"%+.1f", gGains[i]];
        vLbl.textColor = [UIColor colorWithRed:1.0 green:0.82 blue:0.0 alpha:1.0];
        vLbl.font = [UIFont monospacedDigitSystemFontOfSize:13 weight:UIFontWeightBold];
        vLbl.textAlignment = NSTextAlignmentRight;
        _labels[i] = vLbl;
        [scroll addSubview:vLbl];

        y += 45;
    }
    scroll.contentSize = CGSizeMake(scroll.bounds.size.width, y + 10);

    UIView *div2 = [[UIView alloc] initWithFrame:CGRectMake(16, 470, w - 32, 0.5)];
    div2.backgroundColor = [UIColor colorWithWhite:1.0 alpha:0.15];
    [c addSubview:div2];

    _statusLabel = [[UILabel alloc] initWithFrame:CGRectMake(16, 478, w - 32, 20)];
    _statusLabel.font = [UIFont systemFontOfSize:11 weight:UIFontWeightMedium];
    _statusLabel.textAlignment = NSTextAlignmentCenter;
    [self updateStatus];
    [c addSubview:_statusLabel];

    _timer = [NSTimer scheduledTimerWithTimeInterval:1.0 target:self selector:@selector(updateStatus) userInfo:nil repeats:YES];
}

- (void)onPan:(UIPanGestureRecognizer *)p {
    CGPoint t = [p translationInView:_btn.superview];
    CGFloat x = fmin(fmax(_btn.center.x + t.x, 30), _btn.superview.bounds.size.width - 30);
    CGFloat y = fmin(fmax(_btn.center.y + t.y, 60), _btn.superview.bounds.size.height - 60);
    _btn.center = CGPointMake(x, y);
    [p setTranslation:CGPointZero inView:_btn.superview];
}

- (void)toggle {
    BOOL show = _blur.hidden;
    if (show) _blur.hidden = NO;
    [UIView animateWithDuration:0.2 animations:^{
        self->_blur.alpha = show ? 1.0 : 0.0;
    } completion:^(BOOL f) {
        if (!show) self->_blur.hidden = YES;
    }];
}

- (void)onSlider:(UISlider *)s {
    int i = (int)s.tag;
    gGains[i] = s.value;
    _labels[i].text = [NSString stringWithFormat:@"%+.1f", s.value];
    update_filter(i, gSampleRate);
    [[NSUserDefaults standardUserDefaults] setDouble:s.value forKey:[NSString stringWithFormat:@"eq_band_%d", i]];
}

- (void)onMinus:(UIButton *)b {
    int i = (int)b.tag;
    _sliders[i].value = fmax(_sliders[i].value - 0.5f, -15.0f);
    [self onSlider:_sliders[i]];
}

- (void)onPlus:(UIButton *)b {
    int i = (int)b.tag;
    _sliders[i].value = fmin(_sliders[i].value + 0.5f, 15.0f);
    [self onSlider:_sliders[i]];
}

- (void)onPreamp:(UISlider *)s {
    gPreamp = s.value;
    _preampLabel.text = [NSString stringWithFormat:@"%.1f dB", gPreamp];
    [[NSUserDefaults standardUserDefaults] setDouble:s.value forKey:@"eq_preamp"];
}

- (void)savePreset {
    UIAlertController *a = [UIAlertController alertControllerWithTitle:@"Сохранить пресет" message:nil preferredStyle:UIAlertControllerStyleAlert];
    [a addTextFieldWithConfigurationHandler:^(UITextField *t) { t.placeholder = @"Название"; }];
    [a addAction:[UIAlertAction actionWithTitle:@"ОК" style:UIAlertActionStyleDefault handler:^(UIAlertAction *act) {
        NSString *name = a.textFields.firstObject.text;
        if (name.length) {
            NSMutableDictionary *dict = [[[NSUserDefaults standardUserDefaults] dictionaryForKey:@"eq_presets"] mutableCopy] ?: [NSMutableDictionary dictionary];
            NSMutableArray *g = [NSMutableArray array];
            for (int i = 0; i < BANDS; i++) [g addObject:@(gGains[i])];
            dict[name] = @{@"gains": g, @"preamp": @(gPreamp)};
            [[NSUserDefaults standardUserDefaults] setObject:dict forKey:@"eq_presets"];
        }
    }]];
    [a addAction:[UIAlertAction actionWithTitle:@"Отмена" style:UIAlertActionStyleCancel handler:nil]];
    [_vc presentViewController:a animated:YES completion:nil];
}

- (void)showPresets {
    UIAlertController *s = [UIAlertController alertControllerWithTitle:@"Пресеты" message:nil preferredStyle:UIAlertControllerStyleActionSheet];
    [s addAction:[UIAlertAction actionWithTitle:@"Flat" style:UIAlertActionStyleDefault handler:^(UIAlertAction *a) {
        for (int i = 0; i < BANDS; i++) gGains[i] = 0.0;
        gPreamp = 0.0;
        [self applyGains];
    }]];
    [s addAction:[UIAlertAction actionWithTitle:@"3D Bass" style:UIAlertActionStyleDefault handler:^(UIAlertAction *a) {
        double bass[BANDS] = {8.0, 5.0, 3.0, -2.0, -4.0, 0.0, 1.0};
        for (int i = 0; i < BANDS; i++) gGains[i] = bass[i];
        gPreamp = -5.0;
        [self applyGains];
    }]];

    NSDictionary *presets = [[NSUserDefaults standardUserDefaults] dictionaryForKey:@"eq_presets"];
    for (NSString *k in presets) {
        [s addAction:[UIAlertAction actionWithTitle:k style:UIAlertActionStyleDefault handler:^(UIAlertAction *a) {
            NSArray *g = presets[k][@"gains"];
            if (g.count == BANDS) for (int i = 0; i < BANDS; i++) gGains[i] = [g[i] doubleValue];
            if (presets[k][@"preamp"]) gPreamp = [presets[k][@"preamp"] doubleValue];
            [self applyGains];
        }]];
    }
    [s addAction:[UIAlertAction actionWithTitle:@"Отмена" style:UIAlertActionStyleCancel handler:nil]];
    s.popoverPresentationController.sourceView = _presetBtn;
    [_vc presentViewController:s animated:YES completion:nil];
}
@end

__attribute__((constructor))
static void init_eq_tweak(void) {
    [[EQManager shared] loadSettings];
    [[NSNotificationCenter defaultCenter] addObserverForName:UIApplicationDidFinishLaunchingNotification object:nil queue:[NSOperationQueue mainQueue] usingBlock:^(NSNotification *n) {
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(1.5 * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{
            static dispatch_once_t once;
            dispatch_once(&once, ^{
                init_hooks();
                [[EQManager shared] setupUI];
            });
        });
    }];
}
