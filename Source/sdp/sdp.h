///////////////////////////////////////////////////////////////////////////////
//
// Serial debug protocol package include file
//
///////////////////////////////////////////////////////////////////////////////

#ifndef SDP_H_
#define SDP_H_


#define SDP_NO_CHARACTER    -1



enum sdp_error_codes
{
    SDP_NO_ERROR = 0,
    SDP_CRC_ERROR,
    SDP_INEXISTENT_ID,
    SDP_VALUE_OUT_OF_RANGE,
};

struct sdp_get
{
    uint16_t id;
    uint32_t (*get_fn)(void);
};

struct sdp_set
{
    uint16_t id;
    bool (*set_fn)(uint32_t value);
};

struct sdp_store
{
    uint16_t id;
    bool (*store_fn)(uint32_t value);
};

struct sdp_fetch
{
    uint16_t id;
    uint32_t (*fetch_fn)(void);
};


// Transport layer functions (slave)
extern void sdp_slave_init(const struct sdp_get *get_array_p, uint16_t get_array_size,
                     const struct sdp_set *set_array_p, uint16_t set_array_size);


// Transport layer functions (master)
extern void sdp_master_init(const struct sdp_store *store_array_p, uint16_t store_array_size,
                     const struct sdp_fetch *fetch_array_p, uint16_t fetch_array_size);

extern bool sdp_master_read_transmit(uint16_t id);
extern bool sdp_master_write_transmit(uint16_t id);

extern enum sdp_error_codes sdp_master_error_code_last_get(void);
extern uint16_t sdp_master_transmit_error_count_get(void);
extern uint16_t sdp_master_receive_error_count_get(void);


// Data link layer functions
extern void sdp_receive_put(uint16_t c);
extern int16_t sdp_transmit_peek(void);
extern void sdp_transmit_done(void);
extern bool sdp_is_comms_lost_get(void);

#endif
