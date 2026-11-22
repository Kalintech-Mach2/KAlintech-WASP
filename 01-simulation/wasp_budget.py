"""
WASP micro (1S brushed) - weight, thrust, hover and centre-of-gravity budget.

Run:  python3 wasp_budget.py      (standard library only)

This replaces the 165 g brushless budget in wasp_sim.py for the micro design.
Labels used in comments
  LISTING  = number from a seller or shop listing (can be wrong)
  ASSUMED  = my estimate, replace it with a measurement when the parts arrive
The big unknown is thrust per motor. Everything that depends on it is shown as a range.
Weigh every part on the 0.01 g scale and put the real numbers in below.
"""

# ---------------------------------------------------------------- 1. weight (grams)
BUDGET = [
    ("Motors x4, 8520 coreless (~5 g each)",     20.0, "LISTING"),
    ("Props x4, 65 mm",                            2.0, "ASSUMED"),
    ("Battery 1S 450 mAh (BetaFPV BT2.0)",        11.0, "ASSUMED"),
    ("ESP32-S3 SuperMini",                         2.5, "ASSUMED"),
    ("IMU MPU-6050 (GY-521)",                      2.0, "ASSUMED"),
    ("VL53L1X distance sensor",                    1.5, "ASSUMED"),
    ("4x MOSFET + 4x diode",                       0.6, "ASSUMED"),
    ("Capacitors + resistors",                     1.0, "ASSUMED"),
    ("Wire",                                       1.0, "ASSUMED"),
    ("Battery lead + connector",                   0.5, "ASSUMED"),
    ("Frame (3D printed), design target",          8.0, "ASSUMED, design to this"),
    ("Screws, foam tape, heat-shrink",             1.5, "ASSUMED"),
]
MASS_G = sum(m for _, m, _ in BUDGET)
CONTINGENCY = 0.15
LIMIT_G = 100.0          # under 100 g: no CAA Flyer ID or Operator ID needed

# ---------------------------------------------------------------- 2. thrust and hover
THRUST_PER_MOTOR_G = (20.0, 25.0, 30.0)     # ASSUMED, pessimistic / expected / optimistic. MEASURE THIS.
G_PER_WATT = (3.0, 4.0, 5.0)                # ASSUMED, bench tests of 8520 + 55 mm props quote 4 to 5
BATTERY_MAH = 450.0
USABLE = 0.80                               # do not fly the pack flat
V_BAT = 3.7
ELECTRONICS_W = 0.4                         # ESP32 + sensors, ASSUMED

# ---------------------------------------------------------------- 3. geometry for centre of gravity
MOTOR_SPACING_M = 0.070    # adjacent motor centres, 65 mm props need at least this. Update from Fusion 360.
LEVER_M = MOTOR_SPACING_M / 2

def report():
    print("WEIGHT BUDGET")
    for n, m, s in BUDGET:
        print(f"  {n:<44}{m:>6.1f} g   {s}")
    print(f"  {'TOTAL':<44}{MASS_G:>6.1f} g")
    with_c = MASS_G * (1 + CONTINGENCY)
    print(f"  with {CONTINGENCY:.0%} contingency{'':<28}{with_c:>6.1f} g   (limit {LIMIT_G:.0f} g, margin {LIMIT_G - with_c:.0f} g)\n")

    print("THRUST-TO-WEIGHT and HOVER THROTTLE   (thrust per motor is the unknown)")
    for t in THRUST_PER_MOTOR_G:
        tw = 4 * t / MASS_G
        # thrust goes roughly with voltage squared, so hover duty is about sqrt(1 / TWR)
        print(f"  {t:>4.0f} g per motor -> total {4*t:>4.0f} g, thrust-to-weight {tw:4.2f}:1, hover throttle about {100*(1/tw)**0.5:3.0f} %")
    print("  Rule of thumb: you want about 2:1 or more for crisp control and hover below ~70 %.")
    print("  If the bench shows under 25 g per motor: smaller battery (260 mAh saves ~4 g) or 55 mm props.\n")

    print("HOVER TIME   (450 mAh, 80 % usable)")
    for g in G_PER_WATT:
        p = MASS_G / g + ELECTRONICS_W
        i = p / V_BAT
        mins = BATTERY_MAH * USABLE / (i * 1000) * 60
        print(f"  {g:.0f} g/W -> {p:4.1f} W, {i:3.1f} A, about {mins:3.1f} min")
    print()

    print("CENTRE OF GRAVITY   (front/back motor pairs; motor spacing %.0f mm, update from CAD)" % (MOTOR_SPACING_M * 1000))
    print("  If the mass sits d mm off-centre the front pair and back pair must differ in thrust by:")
    for d in (1, 2, 3, 5, 10):
        diff = (d / 1000) / LEVER_M * 100
        print(f"  {d:>2} mm off -> +-{diff:4.1f} % thrust between the pairs just to hover level")
    print("  Requirement R10 uses 3 mm (about 9 %).\n")

    print("CURRENT CHECK")
    print("  8520 motor: about 1.6 to 1.8 A at full load (LISTING), stall above 5 A -> about 7 A for all four")
    print("  Battery 450 mAh x 30C = 13.5 A continuous -> OK")
    print("  MOSFET: SI2302 is rated about 2.6 to 2.9 A, so a jammed prop kills it. AO3400A is rated 5.7 A, use that.")

if __name__ == "__main__":
    report()
