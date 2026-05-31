# /center_square_black_check.py target赋值规则
## target赋值规则：
新建变量：经过拐角=0
1. up & down : y=0.01
    - 经过拐角=1
    - 拐角:down & left : x=-0.01
    - 拐角:down & right : x=0.01
    - 经过拐角=2
    - y=-y
    - 经过拐角=3
    - x=-x
    - 经过拐角=4
    - y=-y


2. right & left : x=0.01
    - 经过拐角=1
    - 拐角:right & down : y=-0.01
    - 拐角:right & up : y=0.01
    - 经过拐角
    - x=-x
    - 经过拐角
    - y=-y
    - 经过拐角
    - x=-x
    



