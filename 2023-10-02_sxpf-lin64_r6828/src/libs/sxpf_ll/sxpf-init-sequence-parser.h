/*
 * sxpf-init-sequence-parser.h
 *
 *  Created on: 27.07.2015
 *      Author: troebbenack
 */

#ifndef SXPF_INIT_SEQUENCE_PARSER_H_
#define SXPF_INIT_SEQUENCE_PARSER_H_

#ifdef __cplusplus
extern "C" {
#endif

    // ----------------------------------------------------------------------------
    // - includes                                                                 -
    // ----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>

// ----------------------------------------------------------------------------
// - public defines                                                           -
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// - public types                                                             -
// ----------------------------------------------------------------------------

    typedef struct sxpf_init_sequence_alias_s {
#define ALIAS_MAX_NAME_LENGTH 128
        char name[ALIAS_MAX_NAME_LENGTH];
        uint8_t id;
    } sxpf_init_sequence_alias_t;

    typedef struct sxpf_init_sequence_event_s {
#define MAX_EVENTNAME_LENGTH 128
        uint8_t id;
        char eventname[MAX_EVENTNAME_LENGTH];
        uint32_t offset;
    } sxpf_init_sequence_event_t;

    typedef struct sxpf_init_sequence_data_s {
        char eventname[MAX_EVENTNAME_LENGTH];
        uint32_t sequenceOffset;
        uint32_t sequenceLength;
        uint8_t* sequenceData; // malloc
        int used;
        size_t atOffset; // user-wish
    } sxpf_init_sequence_data_t;

    typedef struct sxpf_init_sequence_s {
#define MAX_ALIASES_COUNT 		100
#define MAX_EVENTLIST_COUNT 	100
#define MAX_SEQUENCELIST_COUNT 	100
        const char* filename;
        uint8_t* buffer;
        size_t bufferSize;
        size_t bufferOffset;
        int lineNumber;
        int indentation;
        int fixedAddressesFound;
        uint32_t aliasesCount;
        uint32_t eventListCount;
        uint32_t sequenceListCount;
        sxpf_init_sequence_alias_t aliases[MAX_ALIASES_COUNT];
        sxpf_init_sequence_event_t eventList[MAX_EVENTLIST_COUNT];
        sxpf_init_sequence_data_t sequenceList[MAX_SEQUENCELIST_COUNT];
    } sxpf_init_sequence_t;

    // ----------------------------------------------------------------------------
    // - public functions                                                         -
    // ----------------------------------------------------------------------------

    int sxpf_init_sequence_parser_open(
        const char* filename, sxpf_init_sequence_t* seq);

    int sxpf_init_sequence_parser_close(sxpf_init_sequence_t* seq);

    void sxpf_init_sequence_parser_dump(sxpf_init_sequence_t* seq);

    uint8_t* sxpf_init_sequence_parser_readFileInBuffer(const char* filename,
        uint32_t* fileSizeInBytes);

    int sxpf_init_sequence_parser_writeBufferToFile(const char* filename,
        uint8_t* buffer, uint32_t bufferSize);

    uint8_t* sxpf_init_sequence_parser_createUpdateBinary(
        sxpf_init_sequence_t* seq, uint32_t* sizeInBytes);

    uint8_t* sxpf_init_sequence_parser_createUpdateBinary2(sxpf_init_sequence_t* seq,
        uint32_t* sizeInBytes, uint32_t sizeOfEEPROM);

    const char* sxpf_init_sequence_parser_version();

#ifdef __cplusplus
}
#endif

// ----------------------------------------------------------------------------
// - done                                                                     -
// ----------------------------------------------------------------------------

#endif /* SXPF_INIT_SEQUENCE_PARSER_H_ */
