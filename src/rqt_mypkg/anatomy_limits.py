THYROID_MAX = 2
THYROID_MIN = 10

ABDOMINAL_AORTA_MAX = 1
ABDOMINAL_AORTA_MIN = 1

ARM_ARTERIAL_MAX = 1
ARM_ARTERIAL_MIN = 1

LEG_MAX = 1
LEG_MIN = 1

SPINE_MAX = 1
SPINE_MIN = 1

# Dict keys correspond to index of anatomy selection combobox, values are [min, max]
# 1: Thyroid
# 2: Abdonminal aorta
# 3: Arterial arm
# 4: Leg
# 5: Spine
ANATOMY_LIMITS = {
    1: [2,10],
    2: [1,3],
    3: [2,4],
    4: [3,5],
    5: [4,6],
}
