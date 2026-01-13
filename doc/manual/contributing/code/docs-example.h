#ifndef __DOCS_EXAMPLE_H__
#define __DOCS_EXAMPLE_H__

#include <systemc.h>
#include <piconut.h>

/**
 * @fn SC_MODULE(m_module)
 * @brief Module Brief
 *
 * @par Notes
 * - Nice Notes
 *
 * @param[in]  clk                             Clock signal
 *
 * @param[out] output_singal<1>               Instruction port strobe signal
 */

SC_MODULE(m_module)
{
public:
    sc_in<bool> PN_NAME(clk);
    sc_out<bool> PN_NAME(output_signal);
};

#endif /* __DOCS_EXAMPLE_H__ */