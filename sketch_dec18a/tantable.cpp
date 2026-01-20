#include "tantable.h"
#include "sensordata.h"

// 计算公式: Distance(cm) * tan(2.4度) 
// tan(2.4度) ≈ 0.041911

#if ROW_INTERVAL_CM == 5
const float ROW_STEP_WIDTH[TAN_TABLE_SIZE] = {
    0.00000f,  // Row 00:   0cm * tan = 0.000 cm
    0.20955f,  // Row 01:   5cm * tan = 0.210 cm
    0.41911f,  // Row 02:  10cm * tan = 0.419 cm
    0.62866f,  // Row 03:  15cm * tan = 0.629 cm
    0.83822f,  // Row 04:  20cm * tan = 0.838 cm
    1.04777f,  // Row 05:  25cm * tan = 1.048 cm
    1.25733f,  // Row 06:  30cm * tan = 1.257 cm
    1.46688f,  // Row 07:  35cm * tan = 1.467 cm
    1.67644f,  // Row 08:  40cm * tan = 1.676 cm
    1.88599f,  // Row 09:  45cm * tan = 1.886 cm
    2.09555f,  // Row 10:  50cm * tan = 2.096 cm
    2.30510f,  // Row 11:  55cm * tan = 2.305 cm
    2.51466f,  // Row 12:  60cm * tan = 2.515 cm
    2.72421f,  // Row 13:  65cm * tan = 2.724 cm
    2.93377f,  // Row 14:  70cm * tan = 2.934 cm
    3.14332f,  // Row 15:  75cm * tan = 3.143 cm
    3.35288f,  // Row 16:  80cm * tan = 3.353 cm
    3.56244f,  // Row 17:  85cm * tan = 3.562 cm
    3.77199f,  // Row 18:  90cm * tan = 3.772 cm
    3.98155f   // Row 19:  95cm * tan = 3.982 cm
};
#elif ROW_INTERVAL_CM == 10
const float ROW_STEP_WIDTH[TAN_TABLE_SIZE] = {
    0.00000f,  // Row 00:   0cm * tan = 0.000 cm
    0.41911f,  // Row 01:  10cm * tan = 0.419 cm
    0.83822f,  // Row 02:  20cm * tan = 0.838 cm
    1.25733f,  // Row 03:  30cm * tan = 1.257 cm
    1.67644f,  // Row 04:  40cm * tan = 1.676 cm
    2.09555f,  // Row 05:  50cm * tan = 2.096 cm
    2.51466f,  // Row 06:  60cm * tan = 2.515 cm
    2.93377f,  // Row 07:  70cm * tan = 2.934 cm
    3.35288f,  // Row 08:  80cm * tan = 3.353 cm
    3.77199f,  // Row 09:  90cm * tan = 3.772 cm
    4.19110f,  // Row 10: 100cm * tan = 4.191 cm
    4.61021f,  // Row 11: 110cm * tan = 4.610 cm
    5.02932f,  // Row 12: 120cm * tan = 5.029 cm
    5.44843f,  // Row 13: 130cm * tan = 5.448 cm
    5.86754f,  // Row 14: 140cm * tan = 5.868 cm
    6.28665f,  // Row 15: 150cm * tan = 6.287 cm
    6.70576f,  // Row 16: 160cm * tan = 6.706 cm
    7.12487f,  // Row 17: 170cm * tan = 7.125 cm
    7.54398f,  // Row 18: 180cm * tan = 7.544 cm
    7.96309f   // Row 19: 190cm * tan = 7.963 cm
};
#else 
    #error "Did not specify ROW_INTERVAL_CM, or value not supported (only 50 or 100)"
#endif