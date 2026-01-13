#include "top.h"

void m_top::pn_trace(sc_trace_file *tf, int level)
{

   // calling trace of submodules
   if (level >= 2)
   {
      piconut->pn_trace(tf, level);
   }
   // Internal traces
}

void m_top::init_submodules()
{

   piconut = sc_new<m_piconut>("piconut");
   piconut->clk(clk);
   piconut->reset(reset);
   piconut->debug_haltrequest_in(debug_haltrequest_in);
   piconut->debug_haltrequest_ack_out(debug_haltrequest_ack_out);
   piconut->mtip_in(dummy_low);
   piconut->msip_in(dummy_low);
   piconut->meip_in(dummy_low);
}

void m_top::proc_cmb()
{
    dummy_low.write(0);
}