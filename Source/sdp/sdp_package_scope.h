///////////////////////////////////////////////////////////////////////////////
//
// Serial debug protocol package scope include file
//
///////////////////////////////////////////////////////////////////////////////

#ifndef SDP_PACKAGE_SCOPE_H_
#define SDP_PACKAGE_SCOPE_H_


#define SDP_ID_MASK     0x1FFFU
#define SDP_RW_MASK     0x2000U
#define SDP_ERROR_MASK  0x4000U


extern uint16_t sdp_data[4];


extern void sdp_data_process(void);

extern void sdp_data_link_init(void);
extern bool sdp_is_ready_to_transmit(void);
extern void sdp_transmission_start(void);

#endif
