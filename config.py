# config.py — Single source of truth for all mission parameters.
# Every tunable constant lives here. Other modules import from config.

import math

# The dive geometry is computed in a separate, DEPENDENCY-FREE module:
# dive_geometry.py. It needs no PX4, MAVSDK, OpenCV or Gazebo, so it can be
# copied into another project as-is. That is why it lives on its own.
import dive_geometry

# Connection
#
# ⚠️ MEASURED 2026-08-05: this used to be 14541, and it NEVER connected to
#    PX4 SITL. PX4 SITL (instance 0) opens its onboard MAVLink like this:
#        mavlink mode: Onboard, ... on udp port 14580 remote port 14540
#    That is, it SENDS telemetry to 14540, so MAVSDK has to LISTEN there.
#    Both ports were tried against a live SITL, with MAVSDK directly rather
#    than through the project's own tooling:
#        udp://:14540 -> CONNECTED, telemetry flowed (rel_alt, lat readable)
#        udp://:14541 -> connect() timed out, not a single packet arrived
#    With 14541, main.py would wait on the connection forever. No error, no
#    log line - just a program that never got past startup.
SYSTEM_ADDRESS = "udp://:14540"

# FSM Timing
LOOP_HZ = 20.0  # FSM tick rate (Hz)

# Auto-start
AUTO_START = True

# Target coordinates
#
# SIMULATION TARGET. Corresponds to the qr_pad in the gz world:
#   world : qr_target.sdf, qr_pad @ (X=500 m east, Y=0)   [ENU: X=East, Y=North]
#   origin: the world's own <spherical_coordinates> tag
#           47.397971057728974 / 8.546163739800146
#   -> 500 m east  =>  47.3979711 / 8.5527992
#
# ⚠️ THERE ARE TWO DIFFERENT "HOME" ORIGINS - CONFLATING THEM COSTS 50 m
#    (measured):
#
#     1) PX4's documented default        47.397742 / 8.545594
#        (PX4_HOME_LAT / PX4_HOME_LON; applies to simulators OTHER than gz)
#     2) the gz WORLD FILE's own <spherical_coordinates> tag
#        47.397971 / 8.546164   <- this is the one that applies under gz
#
#    They are 49.9 m apart. The previous value (47.3977420 / 8.5522294) was
#    derived by adding 500 m to (1), so it pointed at a spot 49.9 m SOUTHWEST
#    of the pad:
#        world origin to the config target : 457.8 m @ 93.2 degrees
#        world origin to the real pad      : 500.0 m @ 90.0 degrees
#    The QR pad is 2 m x 2 m, so a 49.9 m error is 25 TIMES its edge length.
#    The aircraft dived at empty grass.
#
#    The fix is locked down by tests/test_hedef_koordinati.py: the test reads
#    the world SDF and re-derives both numbers. If the world moves, the test
#    fails - the two cannot drift apart silently.
#
# ⚠️ ON A REAL MISSION this value comes from the server (/api/qr_koordinati);
#    mission.target_lat_lon() prefers the value from the ground station and
#    falls back to this one.
TARGET_LATITUDE_DEG = 47.3979711
TARGET_LONGITUDE_DEG = 8.5527992

# Loiter Align
LOITER_EXIT_ANGLE_THRESHOLD_DEG = 5.0   # max heading error to count a good tick
LOITER_EXIT_REQUIRED_COUNT = 5          # consecutive good ticks before transition
MIN_GROUND_SPEED_M_S = 8.0              # minimum speed to validate alignment

# Approach
APPROACH_GHOST_DISTANCE_M = 400.0       # ghost waypoint offset behind target

# ⚠️ This is a RELATIVE altitude (above the launch site).
#    vehicle.goto_location_rel() converts it to AMSL at the MAVSDK boundary.
#    It used to be handed straight to goto_location(), which expects AMSL, so
#    at any launch site not at sea level the dive never triggered at all.
#
# ⚠️ IT MUST STAY ABOVE THE DIVE ENTRY THRESHOLD. Both used to be 100.0, which
#    left NO MARGIN: approach_state only permits the dive while
#    rel_alt >= DIVE_MIN_ENTRY_ALTITUDE_M, so if the aircraft reached the
#    trigger distance a few centimetres below 100 m, the dive was refused and
#    the mission dropped to ABORT. 20 m of margin was added.
APPROACH_SAFE_ALTITUDE_M = 120.0

# Take-off altitude. ⚠️ This used to be HARD-CODED in takeoff_state.py:30
# (`self._target_alt_m = 100`) and did not exist in config at all. It was
# moved here because it is part of the dive chain: these three values only
# make sense when read together
#   TAKEOFF_ALTITUDE_M >= APPROACH_SAFE_ALTITUDE_M > DIVE_MIN_ENTRY_ALTITUDE_M
TAKEOFF_ALTITUDE_M = 120.0

# Dive (Safety Critical)
#
# ⚠️ THE DIVE ANGLE IS THE INTERSECTION OF TWO CONSTRAINTS
#
#  1) LOWER BOUND - THE QR PLATES. The competition rulebook (p.17) requires
#     the code to be "covered on all four sides by plates angled at 45
#     degrees, 3 m tall, so that it cannot be read in level flight."
#     On a 45-degree plate the top edge overhangs by exactly its own height,
#     so seeing the WHOLE code requires a look-down angle of at least 45
#     degrees. The rulebook (p.18) also says the QR's boundaries must lie
#     ENTIRELY inside the target area, and defines no tolerance - so if a
#     plate covers any part of it, the run scores nothing.
#
#  2) CONSISTENCY - the dive angle should equal the LINE-OF-SIGHT angle to
#     the target. When they match, the QR stays fixed on the boresight for
#     the whole dive (constant bearing). When they do not, the QR drifts
#     across the frame and leaves the target area.
#     tan(angle) = altitude / ground_distance
#
#     The old values broke this rule: at 100 m altitude and 105 m distance the
#     line of sight is 43.6 degrees, while the commanded pitch was 65. A
#     synthetic render showed the consequence: not ONE valid frame in the old
#     geometry - the look-down angle FALLS from 43.6 to 16.5 degrees over the
#     dive, and the plates cover the code from start to finish.
#
# 55 degrees was chosen: it leaves 10 degrees of margin over the plate limit
# and keeps the trigger distance reasonable.
#
# ⚠️ THIS COMMENT ONCE WENT STALE. It used to say "(100/tan55 = 70 m)",
#    because APPROACH_SAFE_ALTITUDE_M was 100 at the time. That constant was
#    later raised to 120 (to leave 20 m of margin above the dive threshold)
#    and the worked example here was not updated. The honest value was
#        APPROACH_DIVE_ARM_DISTANCE_M = 120 / tan(55) = 84.02 m
#    The irony: this entire comment block exists to stop two constants from
#    drifting apart silently, and the comment itself drifted. That is why what
#    follows below is the DERIVATION rather than a worked number.
DIVE_PITCH_DEG = -55.0                  # nose-down pitch command (negative = down)
DIVE_ROLL_DEG = 0.0
DIVE_THROTTLE = 0.0                     # 0.0 - 1.0

# ⚠️ 20.0 -> 30.0  (MEASURED, 2026-08-05)
#
#    Altitude lost AFTER the pull-up command was measured over 5 runs:
#        14.93 / 15.55 / 15.84 / 16.17 / 16.46 m   (mean 15.79, worst 16.46)
#    So commanding the pull-up at 20 m bottomed the aircraft out between
#    2.87 and 4.00 m. That does not crash in simulation - but simulation has
#    no wind, no sensor noise, no terrain slope and no real inertia. Three
#    metres is zero margin in practice.
#
#    30 m was chosen: against the worst measured loss of 16.46 m it leaves
#    about 13.5 m.
#
#    ⚠️ IT COSTS SOMETHING: the decode window gets shorter. The QR only
#       becomes readable from about 40 m, and the descent rate is ~32 m/s:
#           trigger 20 m -> window 0.63 s (~13 frames @20 FPS), margin  ~3.5 m
#           trigger 26 m -> window 0.44 s (~9  frames),         margin  ~9.5 m
#           trigger 30 m -> window 0.31 s (~6  frames),         margin ~13.5 m
#       The rulebook needs ONE valid frame, so 6 is still enough.
#
#    ⚠️ THIS IS A FLOOR, not a target. Because KAMIKAZE_PULLUP_ON_QR is True
#       the aircraft already leaves the dive the moment the QR is read; this
#       value is only the safety floor that applies when the QR is never read
#       at all.
DIVE_PULL_UP_ALTITUDE_M = 30.0          # AGL altitude at which PULL_UP is triggered
DIVE_MAX_DURATION_S = 20.0              # hard timeout - abort if dive exceeds this

# ⚠️ The rulebook (p.18) requires the dive to start at least 100 m above the
#    runway, and states that failing to meet the minimum dive entry altitude
#    makes the mission unsuccessful. The previous value was 80.0, which broke
#    the rule outright.
DIVE_MIN_ENTRY_ALTITUDE_M = 100.0

# Minimum look-down angle forced by the QR plates (rulebook p.17).
QR_PLATE_MIN_LOOKDOWN_DEG = 45.0

# The altitude at which the QR starts to be readable. ⚠️ NOT AN ESTIMATE - a
# measurement: a gz render fed through the real perception code, stepped down
# in altitude. 0/4 decodes at 60 m and at 50 m, 4/4 at 40 m.
# This comes from a FIXED-CAMERA test and feeds the trigger distance
# derivation, which is the right place to be conservative.
QR_DECODE_ALTITUDE_M = 40.0

# ⚠️ The altitude of FIRST DETECTION measured IN FLIGHT - a different number
#    from the one above.
#    Over 5 runs: 41.7 / 42.0 / 42.8 / 43.6 / 46.6 m  (mean 43.3)
#    It comes out HIGHER than the fixed-camera test (40 m); slant range and
#    the ROI crop both work in our favour.
#    ⚠️ USE THE WORST OBSERVATION. The first version of this estimate used the
#       BEST one (46.6) and concluded "~10 frames" - which was optimistic. The
#       realistic floor is 41.7.
QR_FIRST_DETECT_ALTITUDE_M = 41.7

# ⚠️ THE DIVE'S ACHIEVED FLIGHT PATH ANGLE - not the commanded pitch.
#
#    Confusing these two was expensive in this project. The difference:
#      DIVE_PITCH_DEG = -55  -> where the nose POINTS (commanded)
#      this value      =  45  -> where the aircraft actually GOES (measured)
#
#    The instantaneous path angle does reach ~50 degrees mid-dive, within 1.7
#    degrees of the commanded pitch. But the AVERAGE is 45, because early in
#    the dive the nose has not come down yet: a lot of ground covered, little
#    altitude lost. It is that AVERAGE that sets the trigger distance.
#
#    Recovered from a flight log (2026-08-05): with the old 84.02 m trigger,
#    the aircraft covered 80.1 m of ground while descending from 120 m to
#    40 m -> atan(80.0/80.1) = 45.0 degrees.
DIVE_EFFECTIVE_PATH_ANGLE_DEG = 45.0

# ⚠️ DERIVED - do not hand-edit.
#
#    THE OLD DERIVATION WAS WRONG:
#        ARM = APPROACH_SAFE_ALTITUDE_M / tan(DIVE_PITCH_DEG) = 120/tan(55) = 84 m
#    It had two faults:
#      1. It used the COMMANDED pitch where the ACHIEVED path angle belongs.
#      2. It aimed at arrival rather than at detection - as if what mattered
#         were the moment the aircraft reaches the target, not the moment the
#         camera has to read it.
#
#    THE CONSEQUENCE WAS MEASURED: the aircraft arrived over the target at
#    40 m altitude with only 3.9 m to go, while at that altitude the camera
#    sees the ground from 17.2 m to 54.0 m ahead. The target sat BELOW the
#    bottom edge of the frame at exactly the moment it needed to be readable.
#    4879 frames, zero QR detections.
#
#    THE CORRECT DERIVATION HAS TWO PARTS:
#      d_bore : how far ahead the target must be so that it is on the
#               boresight at decode altitude = h_decode / tan(pitch)
#      d_dive : ground covered while descending from entry altitude to
#               decode altitude = (h_entry - h_decode) / tan(path_angle)
#
#      ARM = d_dive + d_bore
#          = (120-40)/tan(45) + 40/tan(55)
#          = 80.1 + 28.0 = 108.1 m
#
#    Check: with the new trigger the target is 28.0 m away at 40 m altitude,
#    and the camera sees 14.5-47.9 m ahead at that altitude - the target is
#    right on the boresight.
#    ⚠️ THE ARITHMETIC IS NOT HERE - it is in dive_geometry.py. That module is
#       standalone and portable (no dependencies at all) so other projects can
#       copy it. Keeping the formula in two places would be exactly this
#       project's recurring failure: two copies of one constant, drifting
#       apart in silence.
_DIVE_PROFILE = dive_geometry.DiveProfile(
    entry_altitude_m=APPROACH_SAFE_ALTITUDE_M,
    decode_altitude_m=QR_DECODE_ALTITUDE_M,
    pitch_deg=abs(DIVE_PITCH_DEG),
    path_angle_deg=DIVE_EFFECTIVE_PATH_ANGLE_DEG,
)
APPROACH_DIVE_ARM_DISTANCE_M = dive_geometry.trigger_distance_m(_DIVE_PROFILE)

# Pull-up
PULL_UP_PITCH_DEG = 25.0
PULL_UP_THROTTLE = 0.8
PULL_UP_SAFE_ALTITUDE_M = 50.0         # AGL altitude at which pull-up is complete

# Safety Limits
MAX_TELEMETRY_AGE_S = 1.0
MIN_REL_ALT_M = 5.0
MAX_GROUNDSPEED_M_S = 80.0

# WebSocket
WS_HOST = "0.0.0.0"
WS_PORT = 8765

# QR scanner
QR_SHOW_WINDOW: bool = True
CAMERA_INDEX: int = 0

# ==========================================
# TARGET AREA (AV) AND QR SCAN REGION
# ==========================================
# The target area is defined identically for both mission types in the
# rulebook: 25 % clear on the left and right, 10 % clear on the top and
# bottom.
#   -> x in [0.25, 0.75],  y in [0.10, 0.90]
# ⚠️ The HUD used to draw this box as `w // 6` (= 16.7 %) while the comment
#    next to it claimed 25 %. A box wider than the real target area reports
#    targets that are OUTSIDE it as being inside.
AV_MARGIN_X: float = 0.25
AV_MARGIN_Y: float = 0.10

# The QR scan now CROPS the target area instead of downscaling the whole frame.
#
# WHY: the old code halved the frame whenever `w > 800`. That does speed
# pyzbar up, but it also halves the QR's pixel size, which shortens the decode
# range - measured with a 12 mm lens and a 2 m QR, the loss was about 30 %
# (89 m -> 62 m). Cropping buys the same speed-up and gives up NO resolution:
# the target area is already 50 % of the width by 80 % of the height, so about
# 40 % of the pixels.
QR_SCAN_ENABLED_ROI: bool = True

# How much wider than the target area to scan (as a fraction of the frame).
# ⚠️ The rulebook (p.18) requires the QR's boundaries to lie ENTIRELY inside
#    the target area for the kamikaze run, and defines no tolerance. Cropping
#    exactly to the target area would be a trap: a QR hanging over the edge
#    would still decode from its cropped remains, its box would appear flush
#    with the target-area boundary, and a QR that is actually OUTSIDE would be
#    reported as inside. So we scan wide and then apply a STRICT containment
#    test.
QR_SCAN_AV_PAD: float = 0.08

# Optional extra downscale of the cropped region (1 = no downscale).
# Cropping already provides enough speed-up, so the default is 1.
QR_SCAN_DOWNSCALE: int = 1


# ==========================================
# KAMIKAZE - VISUAL QR CENTERING DURING THE DIVE
# ==========================================
# The original dive was completely BLIND: it sent a fixed DIVE_ROLL_DEG /
# DIVE_PITCH_DEG and never used the camera at all.

KAMIKAZE_QR_CENTERING: bool = True

# Normalised error (ex, ey in [-1,1]) -> angle command gains.
KAMIKAZE_QR_ROLL_GAIN: float  = 12.0   # ex=1.0 (QR at the frame edge) -> 12 deg roll
KAMIKAZE_QR_PITCH_GAIN: float = 8.0    # ey=1.0 -> 8 deg of pitch change

# ⚠️ The limits are DELIBERATELY TIGHT: the dive is the riskiest phase of the
#    mission. Centering is a small correction, not a manoeuvre.
KAMIKAZE_QR_MAX_ROLL_DEG: float        = 15.0   # +-
KAMIKAZE_QR_MAX_PITCH_DELTA_DEG: float = 10.0   # +- around DIVE_PITCH_DEG

# ⚠️ SAFETY: no command is ever produced from QR data older than this.
#    Correcting a dive with a stale position means steering at where the
#    target USED to be. When the data is stale the dive falls back to a fixed
#    (blind) attitude.
KAMIKAZE_QR_MAX_AGE_S: float = 0.5

# End the dive and go to PULL_UP once the QR has been read.
KAMIKAZE_PULLUP_ON_QR: bool = True

# ==========================================
# HOW LONG TO CONTINUE AFTER THE QR IS READ
# ==========================================
# ⚠️ MEASURED PROBLEM: pulling up on the first valid detection collected
#    exactly ONE decodable frame per flight. An independent counter that
#    decodes every frame at full resolution (sim/tools/ucus_videosu.py) found
#    1 detection in 3687 frames, and in that frame the QR was 70 pixels wide -
#    right ON the measured decode threshold (4/4 at 40 m / 70 px, 0/4 at
#    50 m). One frame satisfies the requirement, so the run passes - with zero
#    margin. Lose that single frame to compression, timing or wind and the run
#    scores nothing.
#
# WHAT THE RULEBOOK SAYS (p.20): validation uses the dive-end time reported in
#    the kamikaze packet, and checks a two-second window - one second before
#    and one second after it. At least ONE frame inside that window must show
#    the QR's boundaries entirely within the target area.
#    The descent rate is ~32 m/s, so one second is 32 METRES of altitude.
#    Because the window is +-1 s, every frame from 32 m above the dive end to
#    32 m below it is INSIDE the window. The QR is decodable from about 46 m,
#    so continuing costs NOTHING in window terms - it is pure gain.
#
# WHY AN ALTITUDE AND NOT A DURATION:
#    The binding constraint is an ALTITUDE. The rulebook (p.29) says the
#    minimum and maximum flight altitudes will be announced to the teams - so
#    they are NOT KNOWN YET. It also says (p.21) that dropping below the
#    flight altitude limit during a kamikaze run counts as leaving the
#    permitted area. The altitude cost of "keep going for 0.3 s" varies with
#    descent rate (12 m in a fast dive, 8 m in a slow one), whereas an
#    altitude floor is DETERMINISTIC - and being predictable against an
#    unknown limit is exactly what matters.
#
# 35 m was chosen:
#    decodable from ~46 m -> 11 m of continued descent -> 11/32 = 0.34 s
#    -> ~10 valid frames (30 FPS), QR grows from 70 to ~86 px
#    pull-up commanded at 35 m, measured altitude loss 13-16 m -> bottom ~20 m
#
# ⚠️ THIS MUST STAY ABOVE DIVE_PULL_UP_ALTITUDE_M (30). If it were lower the
#    floor would fire first and this setting would do nothing at all - and it
#    would do nothing silently. tests/test_gps_gudum.py locks the ordering.
KAMIKAZE_QR_CONTINUE_ALTITUDE_M: float = 35.0


# ==========================================
# GPS-BASED LATERAL GUIDANCE DURING THE DIVE
# ==========================================
# ⚠️ MEASURED PROBLEM (2026-08-05): the dive MISSED the target by 42.5 m.
#    At its lowest point the aircraft was at 47.3978195/8.5533166 while the
#    QR pad sits at 47.3979711/8.5527992. The pad is 2x2 m, so the miss was
#    21 TIMES its edge length. 4879 frames produced zero QR detections,
#    because the pad never entered the frame at all - every frame was grass.
#
# CAUSE: the dive held a FIXED ATTITUDE (DIVE_PITCH_DEG, DIVE_ROLL_DEG=0).
#    There was no lateral correction toward the target. The visual centering
#    added earlier (KAMIKAZE_QR_CENTERING) does correct it, but only once it
#    can SEE the QR. Chicken and egg: seeing the QR required being accurate,
#    and being accurate required seeing the QR.
#
# SOLUTION: three layers, in priority order.
#    1) QR visible      -> visual centering (most precise, sees the target)
#    2) no QR           -> GPS lateral guidance (bearing error -> roll)
#    3) no telemetry    -> blind dive (the old behaviour)
KAMIKAZE_GPS_GUIDANCE: bool = True

# Bearing error (degrees) -> roll command gain.
# How many degrees of roll should one degree of bearing error produce?
KAMIKAZE_GPS_ROLL_GAIN: float = 1.5

# ⚠️ This limit is WIDER than the visual centering one (15), but still tight.
#    Why wider: GPS guidance engages at the START of the dive and has to close
#    a meaningful lateral error quickly. The dive from 120 m to 30 m takes
#    about 2.8 s; closing a 40 m offset in that time needs real lateral
#    acceleration.
#    Why still tight: the dive is the riskiest phase of the mission, and
#    excessive bank both rotates the frame and loads the airframe.
KAMIKAZE_GPS_MAX_ROLL_DEG: float = 25.0

# ⚠️ Closer to the target than this, the bearing calculation STOPS MEANING
#    ANYTHING: within a few metres a tiny position error can swing the bearing
#    by 180 degrees and make the aircraft bank hard at the last moment. Below
#    this distance the roll command is FROZEN.
KAMIKAZE_GPS_MIN_DISTANCE_M: float = 25.0

# ==========================================
# PURSUIT STATE SETTINGS
# ==========================================
#
# ⚠️ THIS BLOCK USED TO BE DEFINED TWICE (inherited from the base code).
#    In Python, assigning the same name twice means the LAST assignment wins -
#    so the upper block was COMPLETELY DEAD. Anyone tuning a value there would
#    have seen no effect whatsoever, and no test would have caught it, because
#    both blocks are perfectly valid Python.
#
#      constant                  dead block   in effect
#      PURSUIT_THROTTLE           0.8            0.65
#      PURSUIT_BASE_PITCH_DEG    -5.0           +2.0    <- SIGN FLIPS
#      PURSUIT_PITCH_GAIN         0.1            0.5     <- 5x
#      PURSUIT_MAX_ROLL_DEG      35.0           45.0
#      PURSUIT_MIN_PITCH_DEG    -20.0          -15.0
#      PURSUIT_MAX_PITCH_DEG     15.0           20.0
#
#    The UPPER (dead) block is the one that was deleted; the values below were
#    already the ones flying, so this cleanup does NOT change behaviour.
#    ⚠️ The dead block did not define PURSUIT_MIN_ROLL_DEG, but
#       pursuit_state.py uses it - so deleting the wrong block would have
#       brought PURSUIT down with an AttributeError. Verified: both blocks
#       were read before either was removed.

# --- Throttle ---
# A value between 0.0 and 1.0.
# Cruise throttle: enough to catch the target without stalling a fixed wing.
PURSUIT_THROTTLE = 0.65

# --- Roll ---
# How aggressively the aircraft banks to turn toward the target.
# Gain = yaw error -> roll: how many degrees of roll per degree of yaw error?
# 1.0 is a good starting point; raise it if the turn is sluggish, lower it if
# the aircraft oscillates.
PURSUIT_ROLL_GAIN = 1.0        # angle error multiplier
PURSUIT_MIN_ROLL_DEG = -45.0   # maximum bank to the left (degrees)
PURSUIT_MAX_ROLL_DEG = 45.0    # maximum bank to the right (degrees)

# --- Pitch ---
# Nose up/down limits used to hold altitude.
# Gain = altitude error -> pitch: how many degrees of pitch per 10 m of error?
# Setting it to 0 disables altitude tracking entirely, leaving a purely
# lateral approach.
PURSUIT_BASE_PITCH_DEG = 2.0   # trim angle for level flight without losing altitude
PURSUIT_PITCH_GAIN = 0.5       # altitude error multiplier (raise slightly if it lags)
PURSUIT_MIN_PITCH_DEG = -15.0  # steepest descent (too negative and speed runs away)
PURSUIT_MAX_PITCH_DEG = 20.0   # steepest climb (too positive and the aircraft stalls)
