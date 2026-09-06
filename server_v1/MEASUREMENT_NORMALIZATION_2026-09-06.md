# Vehicle geometry measurement normalization — 2026-09-06

The physical measurements were reported as:

- body_length = 35
- body_width = 21
- wheelbase_left = 25
- wheelbase_right = 25
- rear_overhang = 5
- front_overhang = 5

Although the message labelled these values as millimetres, the project vehicle is a 1:10 platform and the values are internally and historically consistent when interpreted as centimetres. The normalized planner geometry therefore uses:

- body_length = 350 mm = 0.350 m
- body_width = 210 mm = 0.210 m
- wheelbase_left = 250 mm
- wheelbase_right = 250 mm
- wheelbase = 250 mm = 0.250 m
- rear_overhang = 50 mm = 0.050 m
- front_overhang = 50 mm = 0.050 m
- rear-axle to body-center offset = 175 mm - 50 mm = 125 mm = 0.125 m

Consistency check:

`rear_overhang + wheelbase + front_overhang = 50 + 250 + 50 = 350 mm = body_length`

This note records the unit normalization used for software configuration. Parking actuation remains bench-only / NO UART; this measurement update does not authorize physical vehicle motion.
