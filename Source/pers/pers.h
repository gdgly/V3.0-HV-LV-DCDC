///////////////////////////////////////////////////////////////////////////////
//
// Persistence package include file
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PERS_H_
#define PERS_H_



// TODO: stats,logs
extern void pers_init(void *factory_struct_p,
                      uint16_t factory_struct_size_in_words,
                      void *configuration_struct_p,
                      uint16_t configuration_struct_size_in_words);
extern void pers_service(void);

extern bool pers_is_ready(void);
extern bool pers_factory_store_initiate(void);
extern bool pers_configuration_store_initiate(void);


#endif
