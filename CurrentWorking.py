import sensor, image, time, math
from pyb import Pin, Timer

DEBUG = False

INA = Pin("P0", Pin.OUT_PP)
INB = Pin("P1", Pin.OUT_PP)

servo_tim = Timer(2, freq=50)
PWM_s = servo_tim.channel(3, Timer.PWM, pin=Pin("P4"))

motor_tim = Timer(4, freq=2000)
PWM_d = motor_tim.channel(2, Timer.PWM, pin=Pin("P8"))

STEER_LEFT = int(1000 / 4.16 * 1000)
STEER_STRAIGHT = int(1500 / 4.16 * 1000)
STEER_RIGHT = int(2000 / 4.16 * 1000)
STEER_RANGE = STEER_RIGHT - STEER_STRAIGHT
STEER_DIR = -1
MAX_STEER_ANGLE_DEG = 59.0

MOTOR_BRAKE = 2880

# Keep curve speed close to the stable version, increase straight speed.
MOTOR_SLOW = 3985
MOTOR_FAST = 4450

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

W, H = sensor.width(), sensor.height()
CX = W / 2.0

LINE_THRESHOLD = [(165, 255)]

# Do not change look-ahead.
ROI_Y = int(H * 0.50)
ROI_H = H - ROI_Y
ROI_LEFT = (0, ROI_Y, W // 2, ROI_H)
ROI_RIGHT = (W // 2, ROI_Y, W // 2, ROI_H)

REF_Y_NEAR = ROI_Y + int(ROI_H * 0.80)
REF_Y_FAR = ROI_Y + int(ROI_H * 0.30)
REF_Y = REF_Y_NEAR

MIN_MAGNITUDE = 8

_lane_width_px = W * 0.55
_lane_width_valid = False
LANE_WIDTH_ALPHA = 0.08
LANE_WIDTH_MIN_PX = 20
LANE_WIDTH_MAX_PX = W * 0.92

def _update_lane_width(measured_px):
    global _lane_width_px, _lane_width_valid

    if measured_px < LANE_WIDTH_MIN_PX or measured_px > LANE_WIDTH_MAX_PX:
        return

    alpha = 0.30 if not _lane_width_valid else LANE_WIDTH_ALPHA
    _lane_width_px = _lane_width_px * (1.0 - alpha) + measured_px * alpha
    _lane_width_valid = True

CTE_WEIGHT = 0.30
HDG_WEIGHT = 0.70

# Stronger than before for continuous turns.
RAIL_FLOOR = 0.30

KP = 1.5
KD = 0.12
DERIV_ALPHA = 0.50
DEADBAND = 0.01

# Enter curve mode earlier and keep more minimum turning.
CURVE_HDG_THRESH = 0.12
MIN_CURVE_CORRECTION = 0.24

_pd_last_error = 0.0
_pd_filtered_deriv = 0.0
_pd_last_ms = time.ticks_ms()

def pd_compute(error):
    global _pd_last_error, _pd_filtered_deriv, _pd_last_ms

    if abs(error) < DEADBAND:
        error = 0.0

    now = time.ticks_ms()
    dt = max(time.ticks_diff(now, _pd_last_ms) / 1000.0, 0.001)
    _pd_last_ms = now

    raw_deriv = (error - _pd_last_error) / dt
    _pd_filtered_deriv = (DERIV_ALPHA * _pd_filtered_deriv +
                          (1.0 - DERIV_ALPHA) * raw_deriv)

    _pd_last_error = error

    p_term = KP * error
    d_term = KD * _pd_filtered_deriv

    if error != 0.0 and (p_term + d_term) * error < 0.0:
        d_term = -p_term

    return max(-1.0, min(1.0, p_term + d_term))

def reset_pd():
    global _pd_last_error, _pd_filtered_deriv, _pd_last_ms

    _pd_last_error = 0.0
    _pd_filtered_deriv = 0.0
    _pd_last_ms = time.ticks_ms()

SPEED_THRESH_START = 0.24
SPEED_THRESH_FULL = 0.65

def dynamic_speed(abs_correction):
    if abs_correction <= SPEED_THRESH_START:
        return MOTOR_FAST, "FAST"

    if abs_correction >= SPEED_THRESH_FULL:
        return MOTOR_SLOW, "SLOW"

    t = ((abs_correction - SPEED_THRESH_START) /
         (SPEED_THRESH_FULL - SPEED_THRESH_START))

    return int(MOTOR_FAST + t * (MOTOR_SLOW - MOTOR_FAST)), "MED"

_RATE_NORMAL = STEER_RANGE // 10
_RATE_TURNING = STEER_RANGE // 4
_current_pulse = STEER_STRAIGHT

def set_steering_smooth(target_pulse, turning=False):
    global _current_pulse

    rate = _RATE_TURNING if turning else _RATE_NORMAL
    delta = target_pulse - _current_pulse
    step = max(-rate, min(rate, delta))

    _current_pulse = int(max(STEER_LEFT,
                             min(STEER_RIGHT, _current_pulse + step)))

    PWM_s.pulse_width(_current_pulse)
    return _current_pulse

def set_steering(pulse_ticks):
    global _current_pulse

    _current_pulse = int(max(STEER_LEFT, min(STEER_RIGHT, pulse_ticks)))
    PWM_s.pulse_width(_current_pulse)

def _motor_set(pw):
    INA.high()
    INB.low()
    PWM_d.pulse_width(int(pw))

def drive_forward(pw):
    if not DEBUG:
        _motor_set(pw)

def drive_coast():
    INA.low()
    INB.low()
    PWM_d.pulse_width(0)

def drive_brake():
    INA.low()
    INB.low()
    PWM_d.pulse_width(0)

def kickstart():
    _motor_set(MOTOR_FAST)
    time.sleep_ms(180)
    _motor_set(MOTOR_FAST)

def safe_stop():
    drive_coast()
    time.sleep_ms(50)
    set_steering(STEER_STRAIGHT)
    drive_brake()

def get_steering_angle(target_pulse):
    deviation = target_pulse - STEER_STRAIGHT
    return (deviation / STEER_RANGE) * MAX_STEER_ANGLE_DEG * STEER_DIR

def line_x_at_y(line, target_y):
    x1, y1 = float(line.x1()), float(line.y1())
    x2, y2 = float(line.x2()), float(line.y2())

    dy = y2 - y1

    if abs(dy) < 0.5:
        return (x1 + x2) / 2.0

    t = (target_y - y1) / dy
    result = x1 + t * (x2 - x1)

    return max(0.0, min(float(W - 1), result))

def get_lane_vectors(img):
    l_obj = img.get_regression(LINE_THRESHOLD, roi=ROI_LEFT, robust=True)
    r_obj = img.get_regression(LINE_THRESHOLD, roi=ROI_RIGHT, robust=True)

    left_line = None
    right_line = None

    if l_obj is not None and l_obj.magnitude() >= MIN_MAGNITUDE:
        left_line = l_obj

    if r_obj is not None and r_obj.magnitude() >= MIN_MAGNITUDE:
        right_line = r_obj

    return left_line, right_line

def compute_lane_error(left_line, right_line):
    half_w = _lane_width_px / 2.0

    near_y = REF_Y_NEAR
    far_y = REF_Y_FAR

    if left_line is not None and right_line is not None:
        lx_near = line_x_at_y(left_line, near_y)
        rx_near = line_x_at_y(right_line, near_y)

        lx_far = line_x_at_y(left_line, far_y)
        rx_far = line_x_at_y(right_line, far_y)

        center_x = (lx_near + rx_near) / 2.0
        far_center_x = (lx_far + rx_far) / 2.0

        _update_lane_width(rx_near - lx_near)

        heading_deg = math.degrees(math.atan2(far_center_x - center_x,
                                               near_y - far_y))

        mode = "BOTH"

    elif left_line is not None:
        lx_near = line_x_at_y(left_line, near_y)
        lx_far = line_x_at_y(left_line, far_y)

        center_x = lx_near + half_w
        far_center_x = lx_far + half_w

        heading_deg = math.degrees(math.atan2(far_center_x - center_x,
                                               near_y - far_y))

        mode = "RAIL_R"

    elif right_line is not None:
        rx_near = line_x_at_y(right_line, near_y)
        rx_far = line_x_at_y(right_line, far_y)

        center_x = rx_near - half_w
        far_center_x = rx_far - half_w

        heading_deg = math.degrees(math.atan2(far_center_x - center_x,
                                               near_y - far_y))

        mode = "RAIL_L"

    else:
        return 0.0, 0.0, CX, 0.0, "LOST"

    cte_norm = max(-1.0, min(1.0, (center_x - CX) / CX))
    hdg_norm = max(-1.0, min(1.0, heading_deg / MAX_STEER_ANGLE_DEG))

    return cte_norm, hdg_norm, center_x, heading_deg, mode

def draw_debug(img, left_line, right_line, center_x, mode,
               steer_angle, speed_label, cte, hdg):

    img.draw_rectangle(ROI_LEFT, color=100)
    img.draw_rectangle(ROI_RIGHT, color=100)

    if left_line is not None:
        img.draw_line(left_line.x1(), left_line.y1(),
                      left_line.x2(), left_line.y2(),
                      color=210, thickness=2)

        img.draw_circle(int(line_x_at_y(left_line, REF_Y_NEAR)),
                        REF_Y_NEAR, 3, color=210, fill=True)

        img.draw_circle(int(line_x_at_y(left_line, REF_Y_FAR)),
                        REF_Y_FAR, 2, color=160, fill=True)

    if right_line is not None:
        img.draw_line(right_line.x1(), right_line.y1(),
                      right_line.x2(), right_line.y2(),
                      color=210, thickness=2)

        img.draw_circle(int(line_x_at_y(right_line, REF_Y_NEAR)),
                        REF_Y_NEAR, 3, color=210, fill=True)

        img.draw_circle(int(line_x_at_y(right_line, REF_Y_FAR)),
                        REF_Y_FAR, 2, color=160, fill=True)

    img.draw_circle(int(center_x), REF_Y_NEAR, 5, color=255)
    img.draw_line(int(CX), H - 1, int(CX), H - 8, color=150)

    lw_str = "W:{:.0f}px".format(_lane_width_px) if _lane_width_valid else "W:--"

    img.draw_string(0, 0, lw_str, color=255)
    img.draw_string(0, 10, mode, color=255)
    img.draw_string(0, H - 10,
                    "A:{:+.1f} {} C:{:+.2f} H:{:+.2f}".format(
                        steer_angle, speed_label, cte, hdg),
                    color=255)

def run_validation():
    print("=" * 60)
    print("  CURVE-TUNED DUAL-ROI LANE FOLLOWER | DEBUG MODE")
    print("  STEER_DIR={:+d}  KP={:.2f}  KD={:.2f}".format(
        STEER_DIR, KP, KD))
    print("=" * 60)

    set_steering(STEER_LEFT)
    time.sleep_ms(900)

    set_steering(STEER_STRAIGHT)
    time.sleep_ms(600)

    set_steering(STEER_RIGHT)
    time.sleep_ms(900)

    set_steering(STEER_STRAIGHT)
    time.sleep_ms(1500)

    _motor_set(MOTOR_FAST)
    time.sleep_ms(600)

    drive_coast()
    time.sleep_ms(600)

    test_cases = [
        (-1.00, "hard left"),
        (-0.02, "small left"),
        (0.00, "center"),
        (0.02, "small right"),
        (1.00, "hard right")
    ]

    for err, label in test_cases:
        reset_pd()
        out = pd_compute(err)
        print("err={:+.2f} ({}) correction={:+.4f}".format(
            err, label, out))

    reset_pd()

    for c in (0.0, 0.15, 0.25, 0.35, 0.50, 0.65, 1.0):
        pw, lbl = dynamic_speed(c)
        print("|correction|={:.2f} -> pw={} ({})".format(c, pw, lbl))

try:
    if DEBUG:
        run_validation()

    set_steering(STEER_STRAIGHT)

    _last_cte = 0.0
    _last_hdg = 0.0

    if not DEBUG:
        kickstart()
    else:
        drive_forward(MOTOR_FAST)

    while True:
        img = sensor.snapshot()

        left_line, right_line = get_lane_vectors(img)
        cte, hdg, center_x, heading_deg, mode = compute_lane_error(
            left_line, right_line)

        if mode == "LOST":
            cte = _last_cte * 0.70
            hdg = _last_hdg * 0.70
            center_x = CX + cte * CX

            speed_pw = MOTOR_SLOW
            speed_label = "LOST"

        else:
            _last_cte = cte
            _last_hdg = hdg

        if mode == "BOTH":
            combined_error = CTE_WEIGHT * cte + HDG_WEIGHT * hdg

        elif mode == "RAIL_L":
            rail_error = CTE_WEIGHT * cte + HDG_WEIGHT * hdg
            combined_error = min(rail_error, -RAIL_FLOOR)

        elif mode == "RAIL_R":
            rail_error = CTE_WEIGHT * cte + HDG_WEIGHT * hdg
            combined_error = max(rail_error, RAIL_FLOOR)

        else:
            combined_error = CTE_WEIGHT * cte + HDG_WEIGHT * hdg

        correction = pd_compute(combined_error)

        curve_detected = (mode in ("RAIL_L", "RAIL_R")) or (abs(hdg) > CURVE_HDG_THRESH)

        if curve_detected and abs(correction) < MIN_CURVE_CORRECTION:
            if combined_error > 0:
                correction = MIN_CURVE_CORRECTION
            elif combined_error < 0:
                correction = -MIN_CURVE_CORRECTION

        if mode != "LOST":
            speed_pw, speed_label = dynamic_speed(abs(correction))

            if curve_detected:
                speed_pw = MOTOR_SLOW
                speed_label = "TURN"

        target_pulse = STEER_STRAIGHT + int(STEER_DIR * correction * STEER_RANGE)

        is_turning = curve_detected or (abs(correction) > 0.30)

        actual_pulse = set_steering_smooth(target_pulse, turning=is_turning)
        steer_angle = get_steering_angle(actual_pulse)

        drive_forward(speed_pw)

        if DEBUG:
            draw_debug(img, left_line, right_line, center_x, mode,
                       steer_angle, speed_label, cte, hdg)

            print("{:6s} | W:{:5.1f}px | A:{:+5.1f} | CTE:{:+.3f} HDG:{:+.3f} | corr:{:+.3f} | {}".format(
                mode, _lane_width_px, steer_angle, cte, hdg, correction, speed_label))

except KeyboardInterrupt:
    safe_stop()
