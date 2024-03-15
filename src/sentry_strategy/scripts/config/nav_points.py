from enum import IntEnum

p1 = [
    20.59716049194336,
    5.5605181455612183,
    0.0,
    0.0,
    0.0,
    -0.892779931742495,
    0.4391848154572246,
]
p2 = [
    0.09540426731109619,
    -0.15713131427764893,
    0.0,
    0.0,
    0.0,
    0.010223278568330024,
    0.9999477409221516,
]

p3 = [
    -0.37033867835998535,
    1.885432481765747,
    0.0,
    0.0,
    0.0,
    0.018156674386186112,
    0.9998351540005153,
]

p4 = [
    -0.37033867835998535,
    1.885432481765747,
    0.0,
    0.0,
    0.0,
    0.018156674386186112,
    0.9998351540005153,
]
p5 = [
    27.09716049194336,
    3.5605181455612183,
    0.0,
    0.0,
    0.0,
    -0.352779931742495,
    0.9391848154572246,
]

p6 = [
    4.119679927825928,
    0.08305001258850098,
    0.0,
    0.0,
    0.0,
    0.04517077190885103,
    0.9989792797476625,
]

p7 = [
    27.09716049194336,
    3.5605181455612183,
    0.0,
    0.0,
    0.0,
    -0.352779931742495,
    0.9391848154572246,
]

p8 = [
    -1.3657029867172241,
    -2.409904956817627,
    0.0,
    0.0,
    0.0,
    -0.00627571322683466,
    0.9999803075178504,
]

points = [p1, p2, p3, p4, p5, p6, p7 ,p8]

class Place(IntEnum):
    red_home = 0
    blue_home = 1
    red_post = 2
    blue_post = 3
    red_island = 4
    blue_island = 5
    red_supply = 6
    blue_supply = 7
    no_place = 99