#include "classifier_float.h"

typedef enum {
    CLASS_NEGATIVE_INFINITY  = 0b0000000001, // 1
    CLASS_NEGATIVE_NORMAL    = 0b0000000010, // 2
    CLASS_NEGATIVE_SUBNORMAL = 0b0000000100, // 4
    CLASS_NEGATIVE_ZERO      = 0b0000001000, // 8
    CLASS_POSITIVE_ZERO      = 0b0000010000, // 16
    CLASS_POSITIVE_SUBNORMAL = 0b0000100000, // 32
    CLASS_POSITIVE_NORMAL    = 0b0001000000, // 64
    CLASS_POSITIVE_INFINITY  = 0b0010000000, // 128
    CLASS_SIGNALING_NAN      = 0b0100000000, // 256
    CLASS_QUIET_NAN          = 0b1000000000  // 512
} e_float_classifiers;

void m_classifier_float::pn_trace(sc_trace_file* tf, int level) {
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, data_in);
    PN_TRACE(tf, data_out);
}

void m_classifier_float::proc_clk_classifier() {
    data_out.write(0);
    while(true) {
        wait();

        data_out.write(0);

        if (sign(data_in.read())) { // Negative Cases
            if (exponent(data_in.read()) == 0xFF) {
                if (mantissa(data_in.read()) == 0) {
                    data_out = CLASS_NEGATIVE_INFINITY;
                } else {
                    // Check MSB of mantissa to distinguish Quiet vs Signaling NaN
                    if (mantissa(data_in.read()).range(22, 22)) {
                        data_out = CLASS_QUIET_NAN;
                    } else {
                        data_out = CLASS_SIGNALING_NAN;
                    }
                }
            } else if (exponent(data_in.read()) == 0x00) {
                if (mantissa(data_in.read()) == 0) {
                    data_out = CLASS_NEGATIVE_ZERO;
                } else {
                    data_out = CLASS_NEGATIVE_SUBNORMAL;
                }
            } else {
                data_out = CLASS_NEGATIVE_NORMAL;
            }
        } else { // Positive Cases
            if (exponent(data_in.read()) == 0xFF) {
                if (mantissa(data_in.read()) == 0) {
                    data_out = CLASS_POSITIVE_INFINITY;
                } else {
                    if (mantissa(data_in.read()).range(22, 22)) {
                        data_out = CLASS_QUIET_NAN;
                    } else {
                        data_out = CLASS_SIGNALING_NAN;
                    }
                }
            } else if (exponent(data_in.read()) == 0x00) {
                if (mantissa(data_in.read()) == 0) {
                    data_out = CLASS_POSITIVE_ZERO;
                } else {
                    data_out = CLASS_POSITIVE_SUBNORMAL;
                }
            } else {
                data_out = CLASS_POSITIVE_NORMAL;
            }
        }
    }
}