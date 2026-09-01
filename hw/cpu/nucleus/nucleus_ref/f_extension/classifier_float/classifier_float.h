#ifndef __CLASSIFIER_FLOAT_H__
#define __CLASSIFIER_FLOAT_H__

#include <systemc.h>
#include <piconut.h>

/**
 * @fn SC_MODULE(m_classifier_float)
 * @brief Floating-point number classifier module.
 * This module inspects an incoming 32-bit floating-point number and determines 
 * its mathematical category (e.g., infinity, normal, subnormal, zero, or NaN) 
 * according to the IEEE 754 standard.
 * 
 * The 32-bit output is a one-hot bitmask where the active bit corresponds to the 
 * identified class (Bits 0 to 9).
 * @par Ports:
 * @param[in] clk Clock of the module.
 * @param[in] reset Reset of the module.
 * @param[in] data_in 32-bit raw floating-point number (IEEE 754 Single Precision) to be analyzed.
 * @param[out] data_out 32-bit output containing the 10-bit one-hot classification mask (defined by e_float_classifiers).
 */
SC_MODULE(m_classifier_float) {
    public:
        sc_in_clk PN_NAME(clk);
        sc_in<bool> PN_NAME(reset);

        sc_in<sc_uint<32>> PN_NAME(data_in);

        sc_out<sc_uint<32>> PN_NAME(data_out);

        SC_CTOR(m_classifier_float) {
            SC_CTHREAD(proc_clk_classifier, clk.pos());
            reset_signal_is(reset, true);
        }

        sc_uint<1> sign(sc_uint<32> float_in) { return float_in.range(31, 31); }
        sc_uint<8> exponent(sc_uint<32> float_in) { return float_in.range(30, 23); }
        sc_uint<23> mantissa(sc_uint<32> float_in) { return float_in.range(22, 0); }

        void pn_trace(sc_trace_file * tf, int level = 1);

        void proc_clk_classifier();
};

#endif //__CLASSIFIER_FLOAT_H__