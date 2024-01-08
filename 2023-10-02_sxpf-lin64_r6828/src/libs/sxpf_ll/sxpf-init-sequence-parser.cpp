/*
 * sxpf-init-sequence-parser.c
 *
 *  Created on: 27.07.2015
 *      Author: troebbenack
 */
#include "sxpf-init-sequence-parser.h"
#include "sxpf.h"   // for correct sxpf_decode_init_sequence() linkage
#include <stdio.h>
#include <string.h> // strlen, strncmp,...
#include <stdlib.h> // malloc, ...
#include <memory.h> // memset, ...
#include <errno.h>
#include <sys/stat.h>

 // ----------------------------------------------------------------------------
 // - private defines                                                          -
 // ----------------------------------------------------------------------------

 //#define DEBUG_READLINE
 //#define DEBUG_SPLITKEYVALUE
 //#define DEBUG_PARSING
 //#define DEBUG_STR2INT
 //#define DEBUG_ALIAS2INT
 //#define DEBUG_BINARY2
 //#define DEBUG_COLLISIONAT
 //#define DEBUG_NAMES
 //#define DEBUG_DECODE

#define CRC32_POLY                              0x04C11DB7L
#define CRC32_POLY_REVERSED                     0xEDB88320L
#define BINARY_MAGIC                            0xC0D1F1ED

#define MAX_BUFFER_SIZE (4*(1024*1024)) /* 4 MByte should be enough */
#define LOG_ERR(args, ...) \
    fprintf(stderr, "\nERROR! " args "\n\n", ##__VA_ARGS__)

#define LOG_WARN(args, ...) \
    fprintf(stderr, "\nWARNING! " args "\n\n", ##__VA_ARGS__)

#define CMD_ESCAPE                  			0x0F
#define SUB_JUMP                  			    0x00
#define SUB_RETURN                 			    0x01

// ----------------------------------------------------------------------------
// - private log-macros                                                       -
// ----------------------------------------------------------------------------

#ifdef DEBUG_READLINE
#  define TRACE1(args, ...) fprintf(stderr, "[readLine: %d] " args "\n",\
                                    __LINE__, ##__VA_ARGS__)
#else
#  define TRACE1(args, ...)
#endif

#ifdef DEBUG_SPLITKEYVALUE
#  define TRACE2(args, ...) fprintf(stderr, "[keyValue: %d] " args "\n",\
                                    __LINE__, ##__VA_ARGS__)
#else
#  define TRACE2(args, ...)
#endif

#ifdef DEBUG_PARSING
#  define TRACE3(args, ...) fprintf(stderr, "[parse: %d] " args "\n",\
                                    __LINE__, ##__VA_ARGS__)
#else
#  define TRACE3(args, ...)
#endif

#ifdef DEBUG_STR2INT
#  define TRACE4(args, ...) fprintf(stderr, "[str2Int: %d] " args "\n",\
                                    __LINE__, ##__VA_ARGS__)
#else
#  define TRACE4(args, ...)
#endif

#ifdef DEBUG_ALIAS2INT
#  define TRACE5(args, ...) fprintf(stderr, "[alias2Int: %d] " args "\n",\
                                    __LINE__, ##__VA_ARGS__)
#else
#  define TRACE5(args, ...)
#endif

#ifdef DEBUG_BINARY2
#  define TRACE6(args, ...) fprintf(stderr, "[binary2: %d] " args "\n",\
                                    __LINE__, ##__VA_ARGS__)
#else
#  define TRACE6(args, ...)
#endif

#ifdef DEBUG_COLLISIONAT
#  define TRACE7(args, ...) fprintf(stderr, "[collisionAt: %d] " args "\n",\
                                    __LINE__, ##__VA_ARGS__)
#else
#  define TRACE7(args, ...)
#endif

#ifdef DEBUG_DECODE
#  define TRACE8(args, ...) fprintf(stderr, "[decode: %d] " args "\n",\
                                    __LINE__, ##__VA_ARGS__)
#else
#  define TRACE8(args, ...)
#endif


// ----------------------------------------------------------------------------
// - private types                                                            -
// ----------------------------------------------------------------------------

typedef enum section_e {
    SECTION_NONE = 0, SECTION_ALIASES, SECTION_EVENT_LIST, SECTION_SEQUENCE,
} section_e;

typedef struct sequence_name_s {
    uint32_t offset;
    char name[MAX_EVENTNAME_LENGTH + 1];
} sequence_name_t;

// ----------------------------------------------------------------------------
// - private constants                                                        -
// ----------------------------------------------------------------------------

static const uint8_t END_OF_SEQ[1] = { 0 };

static const char* RUNTIME_BIG_ENDIAN[] = {
    ":be",
    ":big",
    ":big-endian",
    ":msb",
};

static const char* RUNTIME_LITTLE_ENDIAN[] = {
    ":le",
    ":little",
    ":little-endian",
    ":lsb",
};

// ----------------------------------------------------------------------------
// - private variables                                                        -
// ----------------------------------------------------------------------------

static uint32_t g_little_endian = 1;

// ----------------------------------------------------------------------------
// - private functions                                                        -
// ----------------------------------------------------------------------------

static uint32_t hal_bswap_32(uint32_t x)
{
    return
        ((x << 24) & 0xff000000) |
        ((x << 8) & 0x00ff0000) |
        ((x >> 8) & 0x0000ff00) |
        ((x >> 24) & 0x000000ff);
}

static uint32_t hal_utils_crc32_crc32(uint8_t value, uint32_t old_crc)
{
    uint32_t crc = old_crc, i;

    for (i = 0; i < 8; i++) {
        if ((crc ^ value) & 1)
            crc = (crc >> 1) ^ CRC32_POLY_REVERSED;
        else
            crc >>= 1;
        value >>= 1;
    }
    return crc;
}

static uint32_t hal_utils_crc32_crc32Block(const uint8_t* buffer,
    uint32_t bufferSize,
    uint32_t old_crc)
{
    uint32_t crc = old_crc;
    while (bufferSize--) {
        crc = hal_utils_crc32_crc32(*buffer++, crc);
    }
    return crc;
}

static int readFile(const char* filename, sxpf_init_sequence_t* seq)
{
    uint32_t fileSizeInBytes = 0;
    uint8_t* buffer;

    // for sure, initialize buffer data
    seq->bufferOffset = 0;
    seq->bufferSize = 0;
    seq->lineNumber = 0;
    seq->filename = filename;

    buffer = sxpf_init_sequence_parser_readFileInBuffer(filename,
        &fileSizeInBytes);
    if (!buffer) {
        return -1;
    }

    seq->buffer = buffer;
    seq->bufferSize = fileSizeInBytes;

    return 0;
}

static char* readLine(sxpf_init_sequence_t* seq)
{
    int indent = 0;
    int indentFound = 0;
    int commentFound = 0;
    int commentFound2 = 0;
    uint8_t* pBegin, * pCurrent, * pEnd;

    if (seq->bufferOffset >= seq->bufferSize) {
        TRACE1("buffer offset > buffer size");
        return NULL;
    }

    //    ' ' (0x20) space (SPC)
    //    '\t' (0x09) horizontal tab (TAB)
    //    '\n' (0x0a) newline (LF)
    //    '\v' (0x0b) vertical tab (VT)
    //    '\f' (0x0c) feed (FF)
    //    '\r' (0x0d) carriage return (CR)

    pBegin = seq->buffer + seq->bufferOffset;
    pCurrent = pBegin;
    pEnd = pBegin;

    while (seq->bufferOffset++ < seq->bufferSize) {

        char ch = *pCurrent++;

        TRACE1("ch = %x (%c)", ch, ch > 0x13 ? ch : '.');

        if (ch == '\r' || ch == '\n') {

            TRACE1("EOL found pBegin=%p pEnd=%p", pBegin, pEnd);

            // increment current line number
            seq->lineNumber++;

            // terminate current string
            *(pEnd) = '\0';

            // next char a 'line-ending'-separator -> skip it
            char next = *pCurrent;
            if (ch != next && (next == '\r' || next == '\n')) {
                pCurrent++;
                seq->bufferOffset++;
            }

            // found string
            //   -> if 'comment' or line 'empty' read next line,
            //      otherwise return read line
            if (commentFound || pBegin == pEnd) {

                TRACE1("empty line found!");

                // prepare variables to read next line
                commentFound = 0;
                commentFound2 = 0;
                indentFound = 0;
                indent = 0;
                pBegin = pCurrent;
                pEnd = pCurrent;

            }
            else {

                TRACE1("--> parse this line...");
                break;
            }

        }
        else {

            if (!indentFound) {
                if ('\t' == ch) {
                    indent += 4;
                    { // replace tab by space (to make it easier to parse
                        // setup-sequence later)
                        uint8_t* p = pCurrent - 1;
                        *p = ' ';
                    }
                }
                else if (' ' == ch) {
                    indent += 1;
                }
                else {
                    if ('#' == ch) {
                        // if line starts not with "#!" it's a comment
                        if ('!' != *pCurrent) {
                            commentFound = 1;
                        }
                    }
                    indentFound = 1;
                    pBegin = pCurrent - 1; // skip spaces and tabs
                    pEnd = pCurrent;
                }
            }
            else {
                if (!commentFound2) {
                    if ('\t' != ch && ' ' != ch && '#' != ch) {
                        pEnd = pCurrent;
                    }
                    else if ('#' == ch) {
                        commentFound2 = 1;
                    }
                }
            }

        }
    }

    if (!commentFound && pBegin != pEnd) {

        // push result values back onto stack
        TRACE1("read-line: '%s'", pBegin);
        return (char*)pBegin;

    }

    // last string was just a comment or empty, sorry
    TRACE1("end of file");
    return NULL;
}

static int splitKeyValue(const char* text, uint32_t line, char* key,
    uint32_t maxKeyLen, char* value, uint32_t maxValueLen,
    const char* delimiter)
{
    size_t i, textLen, keyLen, valueLen;
    const char* equal;

    equal = strstr(text, delimiter); // "="
    if (!equal) {
        return 0;
    }

    textLen = strlen(text);
    keyLen = equal - text;
    valueLen = textLen - (keyLen + 1);
    TRACE2("after split: keyLen=%ld, valueLen=%ld", keyLen, valueLen);

    // check length of both parts
    if (0 == keyLen || 0 == valueLen) {
        LOG_ERR("Skip line %d because of invalid key/value pair (\"%s\")!",
            line, text);
        return 0;
    }

    // remove spaces/tabs after key-end and before value-start
    // --> BUGFIX: do work value first, because 'keyLen' is changed
    for (i = keyLen + 1; i < textLen; i++, valueLen--) {
        char ch = text[i];
        TRACE2("value: ch[%ld] = %c", i, ch);
        if (ch != ' ' && ch != '\t') {
            break;
        }
    }
    for (; keyLen > 0; keyLen--) {
        char ch = text[keyLen - 1];
        TRACE2("key: ch[%ld] = %c", keyLen - 1, ch);
        if (ch != ' ' && ch != '\t') {
            break;
        }
    }
    TRACE2("after remove space/tabs: keyLen=%ld, valueLen=%ld", keyLen,
        valueLen);

    // check length of both parts (without spaces/tabs) again
    if (0 == keyLen || 0 == valueLen) {
        LOG_ERR("Skip line %d because of invalid key/value pair (\"%s\")!",
            line, text);
        return 0;
    }

    // check max key-/value-length (include needed '\0' termination)
    if (keyLen >= maxKeyLen || valueLen >= maxValueLen) {
        if (keyLen >= maxKeyLen) {
            LOG_ERR("Skip line %d because key-length exceeds %d chars "
                "(\"%s\")!", line, maxKeyLen, text);
        }
        if (valueLen >= maxValueLen) {
            LOG_ERR("Skip line %d because value-length exceeds %d chars"
                " (\"%s\")!", line, maxValueLen, text);
        }
        return 0;
    }

    // copy key-value pair
    memcpy(key, text, keyLen);
    memcpy(value, text + (textLen - valueLen), valueLen);
    key[keyLen] = '\0';
    value[valueLen] = '\0';

    // all done
    return 1;
}

static int isNumber(const char* str, size_t strLength, int base)
{
    static const char BIN_CHARS[] = { '0', '1', };
    static const char INT_CHARS[] = { '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', };
    static const char HEX_CHARS[] = { 'a', 'b', 'c', 'd', 'e', 'f', 'A', 'B',
        'C', 'D', 'E', 'F', };

    size_t i, j, len;

    for (i = 0, len = strLength; i < len; i++) {
        const char ch = str[i];
        int found = 0;
        if (2 == base) {
            for (j = 0; j < sizeof(BIN_CHARS); j++)
                if (ch == BIN_CHARS[j]) {
                    found = 1;
                    break;
                }
        }
        if (10 == base || 16 == base) {
            for (j = 0; j < sizeof(INT_CHARS); j++)
                if (ch == INT_CHARS[j]) {
                    found = 1;
                    break;
                }
        }
        if (16 == base) {
            for (j = 0; j < sizeof(HEX_CHARS); j++)
                if (ch == HEX_CHARS[j]) {
                    found = 1;
                    break;
                }
        }
        if (!found) {
            return 0;
        }
    }
    return 1;
}

static int str2Int(const char* str, size_t strLength, uint32_t* value)
{
    char* p;
    const char* v;
    int base;
    int numBytes = 1;

    TRACE4("str2Int('%s', %ld, ..)", str, strLength);

    if (strLength > 2
        && (0 == strncmp("0x", str, 2) || 0 == strncmp("0X", str, 2))) {
        v = str + 2;
        strLength -= 2;
        base = 16;
        if (strLength > 6) {
            numBytes = 4;
        }
        else if (strLength > 4) {
            numBytes = 3;
        }
        else if (strLength > 2) {
            numBytes = 2;
        }
    }
    else if (strLength > 2
        && (0 == strncmp("0b", str, 2) || 0 == strncmp("0B", str, 2))) {
        v = str + 2;
        strLength -= 2;
        base = 2;
        if (strLength > 24) {
            numBytes = 4;
        }
        else if (strLength > 16) {
            numBytes = 3;
        }
        else if (strLength > 8) {
            numBytes = 2;
        }
    }
    else {
        v = str;
        base = 10;
    }

    if (isNumber(v, strLength, base)) {
        *value = strtoul(v, &p, base);
        if (10 == base) {
            if (*value > 0xFFFFFF) {
                numBytes = 4;
            }
            else if (*value > 0xFFFF) {
                numBytes = 3;
            }
            else if (*value > 0xFF) {
                numBytes = 2;
            }
        }
        if (!g_little_endian) {
            switch (numBytes) {
            case 4: *value = hal_bswap_32(*value); break;
            case 3: *value = hal_bswap_32(*value) >> 8; break;
            case 2: *value = hal_bswap_32(*value) >> 16; break;
            default: break;
            }
        }
        return numBytes;
    }

    return 0;
}

static int aliasToInt(sxpf_init_sequence_t* seq, const char* name,
    size_t nameLength, uint32_t* value)
{
    uint32_t i;

    TRACE5("aliasToInt(..,'%s', %ld, ..)", name, nameLength);

    for (i = 0; i < seq->aliasesCount; i++) {
        if (0 == strncmp(name, seq->aliases[i].name, nameLength) &&
            (0 == seq->aliases[i].name[nameLength + 1]))
        {
            *value = seq->aliases[i].id;
            return 1;
        }
    }
    return str2Int(name, nameLength, value);
}

static int prepareNextSequence(sxpf_init_sequence_t* seq)
{
    if (seq->sequenceList[seq->sequenceListCount].sequenceData) {
        TRACE3("previous sequence list found, skip to next");
        seq->sequenceListCount++;
    }
    if (seq->sequenceListCount >= MAX_SEQUENCELIST_COUNT) {
        LOG_ERR("out of sequence-list-bounds at line %d!", seq->lineNumber);
        return -1;
    }
    return 0;
}

static int checkRuntimeOption(const char* options[], uint32_t count,
    const char* line)
{
    uint32_t i;

    TRACE3("checkRuntimeOption: count=%d", count);
    for (i = 0; i < count; i++) {
        if (0 == strcmp(line, options[i])) {
            return 1;
        }
    }

    return 0;
}

#ifdef DEBUG_NAMES
// note: only used if debugging enabled
static void hexdump(uint8_t* bytes, uint32_t length)
{
    uint32_t i;
    char ascii[17] = { 0 };

    for (i = 0; i < length; i++) {
        fprintf(stderr, "%02x ", bytes[i]);
        ascii[i % 16] = bytes[i] > 31 ? bytes[i] : '?';
        if (i % 16 == 15) {
            fprintf(stderr, "| %s\n", ascii);
            memset(ascii, 0x0, 16);
        }
    }
    fprintf(stderr, "| %s\n", ascii);
}
#endif

static uint8_t* generateNamesBinaryBlob(sxpf_init_sequence_t* seq,
    uint32_t* returnNamesLength)
{
    uint32_t i;
    uint32_t names_length = 0;
    uint8_t* names_data = NULL, * p;

    // 1st) collect size informations...
    for (i = 0; i < seq->sequenceListCount; i++) {
        uint32_t string_length;
        if (!seq->sequenceList[i].used) { continue; }
        string_length = (uint32_t)strnlen(seq->sequenceList[i].eventname,
            MAX_EVENTNAME_LENGTH);
        names_length += 1 /*tag*/ + 1 /*length*/ +
            (4 + string_length) /* value: offset + string */;
    }
    names_length += 1; // zero-termination

    // 2nd) allocate memory
    names_data = (uint8_t*)malloc(names_length);
    if (!names_data) {
        LOG_ERR("Out of memory! Can't allocate %d bytes for sequence-name "
            "binary-blob!", names_length);
        return NULL;
    }

    // clear buffer
    memset(names_data, 0x0, names_length);

    // 3th) fill blob with data
    p = names_data;
    for (i = 0; i < seq->sequenceListCount; i++) {
        size_t string_length;
        if (!seq->sequenceList[i].used) { continue; }
        string_length = strnlen(seq->sequenceList[i].eventname,
            MAX_EVENTNAME_LENGTH);
        // tag
        *p++ = 0xFE;
        // length
        *p++ = (uint8_t)(4 + string_length);
        // value: offset + string
        *(uint32_t*)p = seq->sequenceList[i].sequenceOffset;
        p += sizeof(uint32_t);
        memcpy(p, seq->sequenceList[i].eventname, string_length);
        p += string_length;
    }

#ifdef DEBUG_NAMES
    fprintf(stderr, "Dump 'Names' Binary-Blob:\n");
    hexdump(names_data, names_length);
#endif

    * returnNamesLength = names_length;
    return names_data;
}

static uint32_t parseSequenceNames(uint32_t* data32Copy, uint8_t* data,
    uint8_t* pEndOfData, uint32_t* pNamesLength,
    sequence_name_t* list, uint32_t* listCnt)
{
    uint32_t tmp;
    uint32_t names_offset = 0;
    uint8_t* p;

    // stage1: just search for tag 0xFF --> section-names
    do {
        uint32_t id, offset;

        tmp = *data32Copy++;

        if (tmp) {

            id = tmp >> 24;
            offset = tmp & 0xFFFFFF;

            if (0xFF == id) {
                TRACE8("found SECTION NAMES in event list at offset %d (0x%x)",
                    offset, offset);

                names_offset = offset;
                p = data + offset;

                // parse sequence names at first

                if (p < pEndOfData) {

                    uint8_t tag, length;

                    do {

                        uint32_t offset_length = 0;
                        uint32_t name_length = 0;

                        tag = *p++;

                        if (!tag) { break; }

                        if (p >= pEndOfData) { return 0; /* corruptData */ }

                        length = *p++;

                        while (length--) {

                            if (p >= pEndOfData) { return 0; /* corruptData */ }

                            if (offset_length < 4) {
                                list[*listCnt].offset =
                                    ((uint32_t)*p << 24) |
                                    (list[*listCnt].offset >> 8);
                                offset_length++;
                            }
                            else if (name_length < MAX_EVENTNAME_LENGTH) {
                                list[*listCnt].name[name_length] = *p;
                                name_length++;
                            }

                            p++;
                        }

                        list[*listCnt].name[name_length] = '\0';

                        TRACE8(" 0x%x = %s", list[*listCnt].offset,
                            list[*listCnt].name);

                        (*listCnt)++;

                    } while (tag && p < pEndOfData);

                    // return length of names section
                    *pNamesLength = (uint32_t)(p - (data + offset));
                }

                break;
            }
        }
    } while (tmp && (uint8_t*)data32Copy < pEndOfData);

    return names_offset;
}

static char* getSequenceName(uint32_t offset, sequence_name_t* list,
    uint32_t listCnt)
{
    uint32_t i;
    static char name[MAX_EVENTNAME_LENGTH + 1]; // WARN! used from outside!

    for (i = 0; i < listCnt; i++) {
        if (list[i].offset == offset) { return list[i].name; }
    }

    // generate a fallback-name
    sprintf(name, "[sequence-%x]", offset);
    return name;
}

static char* getSequenceNameWithOffset(uint32_t offset, sequence_name_t* list,
    uint32_t listCnt)
{
    uint32_t i;
    static char name[MAX_EVENTNAME_LENGTH + 1]; // WARN! used from outside!

    for (i = 0; i < listCnt; i++)
    {
        if (list[i].offset == offset)
        {
            size_t len = strlen(list[i].name);

            if (len > MAX_EVENTNAME_LENGTH - 16)
            {    // offset needs add. 11bytes
                LOG_WARN("sequence name to long!");
                goto fallback;
            }

            sprintf(name, "%s", list[i].name);
            if (len && ']' == list[i].name[len - 1])
            {
                sprintf(name + len - 1, "@0x%x]", offset);
            }
            return name;
        }
    }

    // generate a fallback-name
fallback:
    sprintf(name, "[sequence-%x@0x%x]", offset, offset);
    return name;
}

// ----------------------------------------------------------------------------
// - public functions                                                         -
// ----------------------------------------------------------------------------

int sxpf_init_sequence_parser_open(const char* filename,
    sxpf_init_sequence_t* seq)
{
#define MAX_KEY_LENGTH 128
#define MAX_VALUE_LENGTH 128

    const char* line = NULL;
    int result = 0;
    int versionInfoRead = 0;
    section_e section = SECTION_NONE;
    char key[MAX_KEY_LENGTH];
    char value[MAX_VALUE_LENGTH];

    // initialize structure
    memset(seq, 0x0, sizeof(sxpf_init_sequence_t));

    // load ini file into buffer
    if (readFile(filename, seq) < 0) {
        return -1;
    }

    // read line by line
    do {

        line = readLine(seq);
        if (!line) { break; }

        if (0 == strncmp("[", line, 1)) {

            char* at;
            uint32_t atOffset;

            if (!versionInfoRead)
                versionInfoError: {
                LOG_ERR("Version info sequence-string missing!");
                result = -1;
                goto exitParser;
            }
            atOffset = 0;

            // check for fixed address '@' tag
            at = (char*)strstr(line, "@");
            if (at) {
                size_t len = strlen(at + 1) - 1;
                str2Int(at + 1, len, &atOffset);
                at[0] = ']';
                at[1] = '\0';
                TRACE3("found: %s@0x%x", line, atOffset);
                if (0 == atOffset) {
                    LOG_ERR("Read offset for section '%s' (line %d) invalid "
                        "or 0!", line, seq->lineNumber);
                    result = -1;
                    goto exitParser;
                }
                seq->fixedAddressesFound = 1;
            }

            if (0 == strcmp("[aliases]", line)) {
                section = SECTION_ALIASES;
                TRACE3("found aliases-section");
            }
            else if (0 == strcmp("[event-list]", line)) {
                section = SECTION_EVENT_LIST;
                TRACE3("found event-list-section");
            }
            else { // sequence-list
                section = SECTION_SEQUENCE;
                TRACE3("found sequence-section with name '%s'", line);
                { // check for doublet/counterpart
                    uint32_t i;
                    for (i = 0; i <= seq->sequenceListCount; i++) {
                        if (!strncmp(seq->sequenceList[i].eventname, line,
                            MAX_EVENTNAME_LENGTH)) {
                            LOG_ERR("Duplicate section name '%s' found (2nd "
                                "occurrence at line %d)!", line,
                                seq->lineNumber);
                            result = -1;
                            goto exitParser;
                        }
                    }
                }
                result = prepareNextSequence(seq);
                if (result < 0) { goto exitParser; }
                TRACE3("copy sequence name");
                strncpy(seq->sequenceList[seq->sequenceListCount].eventname,
                    line, MAX_EVENTNAME_LENGTH);
                TRACE3("prepare data pointer");
                seq->sequenceList[seq->sequenceListCount].sequenceLength = 0;
                seq->sequenceList[seq->sequenceListCount].sequenceData = NULL;
                seq->sequenceList[seq->sequenceListCount].atOffset = atOffset;
                // by default: little-endian (reset previous runtime option)
                g_little_endian = 1;
            }
            continue;
        }

        switch (section)
        {
        case SECTION_NONE:

            if (0 == strncmp("#!", line, 2)) {

                if (!strstr(line, "sxproframe-init-sequence") ||
                    !strstr(line, "version=1.0")) {

                    LOG_ERR("File format error, Support only 'version=" "1.0' "
                        "of format 'sxproframe-init-sequence'!");
                    result = -2;
                    goto exitParser;
                }

                TRACE3("version: ok");
                versionInfoRead = 1;
                continue;

            }
            else {

                TRACE3("no section, but data found: '%s'", line);
                goto versionInfoError;
            }
            break;

        case SECTION_ALIASES:

            if (seq->aliasesCount >= MAX_ALIASES_COUNT) {
                LOG_ERR("out of aliases-bounds at line %d!", seq->lineNumber);
                result = -3;
                goto exitParser;
            }
            if (splitKeyValue(line, seq->lineNumber,
                seq->aliases[seq->aliasesCount].name,
                ALIAS_MAX_NAME_LENGTH, value, MAX_VALUE_LENGTH,
                "=")) {

                uint32_t tmp;

                if (!str2Int(value, strlen(value), &tmp)) {
                    LOG_ERR("value '%s' is none valid integer at line " "%d!",
                        value, seq->lineNumber);
                    result = -4;
                    goto exitParser;
                }

                seq->aliases[seq->aliasesCount].id = tmp;

                TRACE3("name=%s, id=0x%02x",
                    seq->aliases[seq->aliasesCount].name,
                    seq->aliases[seq->aliasesCount].id);
            }
            seq->aliasesCount++;
            break;

        case SECTION_EVENT_LIST:

            if (seq->eventListCount >= MAX_EVENTLIST_COUNT) {
                LOG_ERR("out of event-list-bounds at line %d!",
                    seq->lineNumber);
                result = -5;
                goto exitParser;
            }
            if (splitKeyValue(line, seq->lineNumber, key, MAX_KEY_LENGTH,
                seq->eventList[seq->eventListCount].eventname,
                MAX_EVENTNAME_LENGTH, "=")) {

                uint32_t tmp;

                if (!aliasToInt(seq, key, strlen(key), &tmp)) {
                    LOG_ERR("alias '%s' not found at line %d!", key,
                        seq->lineNumber);
                    result = -6;
                    goto exitParser;
                }

                if (tmp == 0xFF) {
                    LOG_ERR("Event-ID 255/0xFF is reserved for internal use!");
                    result = -1;
                    goto exitParser;
                }

                seq->eventList[seq->eventListCount].id = tmp;

                TRACE3("name=%s, id=0x%02x",
                    seq->eventList[seq->eventListCount].eventname,
                    seq->eventList[seq->eventListCount].id);
            }
            seq->eventListCount++;
            break;

        case SECTION_SEQUENCE: {

            int run;
            for (run = 0; run < 2; run++) {

                const char* nextSpace, * chunk = line;
                uint32_t tmp, idx = 0, len = 0, special_opcode = 0;
                int numValidBytes;

                TRACE3("run %d, sequence (at line %02d): '%s'", run,
                    seq->lineNumber, line);

                // check for special runtime options
                if (':' == line[0]) {

                    uint32_t count;

                    TRACE3("check runtime options...");

                    count = sizeof(RUNTIME_BIG_ENDIAN) / sizeof(char*);
                    if (checkRuntimeOption(RUNTIME_BIG_ENDIAN, count, line)) {
                        TRACE3("switch to big-endian");
                        g_little_endian = 0;
                        run++;
                        break;
                    }

                    count = sizeof(RUNTIME_LITTLE_ENDIAN) / sizeof(char*);
                    if (checkRuntimeOption(RUNTIME_LITTLE_ENDIAN, count, line))
                    {
                        TRACE3("switch to little-endian");
                        g_little_endian = 1;
                        run++;
                        break;
                    }

                    LOG_ERR("runtime-option '%s' at line %d unknown", line,
                        seq->lineNumber);
                    result = -7;
                    goto exitParser;
                }

                // special handling of 'JUMP' opcode
                if (0 == strncmp(line, "JUMP", 4)) {
                    uint32_t labLen = 0;
                    const char* label = line + 4;
                    // skip spaces at beginning
                    while (' ' == *label) { label++; }
                    // get length of label (without spaces at end)
                    while (label[labLen] && ' ' != label[labLen]) { labLen++; }
                    //
                    if (labLen > 28) {
                        LOG_WARN("section '%s' contains %d characters, "
                            "only 28 supported by firmware!",
                            label, labLen);
                    }
                    // calculate number of bytes needed
                    idx = 1 + 1 + 1 /*ESC+LEN+SUB*/ + labLen;
                    len = idx - 2;
                    if (1 == run) {
                        uint32_t offset, i;
                        offset = seq->sequenceList[seq->sequenceListCount].
                            sequenceLength;
                        seq->sequenceList[seq->sequenceListCount].
                            sequenceData[offset + 0] = CMD_ESCAPE;
                        seq->sequenceList[seq->sequenceListCount].
                            sequenceData[offset + 1] = len;
                        seq->sequenceList[seq->sequenceListCount].
                            sequenceData[offset + 2] = SUB_JUMP;
                        for (i = 0; i < labLen; i++) {
                            seq->sequenceList[seq->sequenceListCount].
                                sequenceData[offset + 3 + i] = label[i];
                        }
                        seq->sequenceList[seq->sequenceListCount].
                            sequenceLength += idx;
                    }
                    // remember that line is already parsed
                    special_opcode = 1;
                }

                if (0 == strncmp(line, "RETURN", 6)) {
                    // calculate number of bytes needed
                    idx = 1 + 1 + 1 /*ESC+LEN+SUB*/;
                    len = 1; // idx - 2
                    if (1 == run) {
                        uint32_t offset;
                        offset = seq->sequenceList[seq->sequenceListCount].
                            sequenceLength;
                        seq->sequenceList[seq->sequenceListCount].
                            sequenceData[offset + 0] = CMD_ESCAPE;
                        seq->sequenceList[seq->sequenceListCount].
                            sequenceData[offset + 1] = len;
                        seq->sequenceList[seq->sequenceListCount].
                            sequenceData[offset + 2] = SUB_RETURN;
                        seq->sequenceList[seq->sequenceListCount].
                            sequenceLength += idx;
                    }
                    // remember that line is already parsed
                    special_opcode = 1;
                }

                while (!special_opcode && chunk) {
                    nextSpace = strstr(chunk, " ");
                    if (nextSpace) {

                        int chunkLen = (int)(nextSpace - chunk);
                        numValidBytes = aliasToInt(seq, chunk, chunkLen, &tmp);

                        if (!numValidBytes) {
                            LOG_ERR("sequence '%s' at line %d, char-index %d "
                                "corrupt!", line, seq->lineNumber,
                                (int)(chunk - line + 1));
                            result = -7;
                            goto exitParser;
                        }

                        chunk = nextSpace + 1;
                        while (*chunk == ' ') { chunk++; }

                    }
                    else {

                        numValidBytes =
                            aliasToInt(seq, chunk, strlen(chunk), &tmp);

                        if (!numValidBytes) {
                            LOG_ERR("sequence '%s' at line %d, char-index %d "
                                "corrupt!", line, seq->lineNumber,
                                (int)(chunk - line + 1));
                            result = -7;
                            goto exitParser;
                        }

                        chunk = NULL;
                    }
                    switch (idx) {
                    case  0: /*cmd*/    break;
                    case  1: len = tmp; break;
                    default: /*data*/   break;
                    }
                    if (1 == run) {
                        uint32_t offset;
                        int i;

                        offset = seq->sequenceList[seq->sequenceListCount].
                            sequenceLength;

                        for (i = 0; i < numValidBytes; i++) {

                            uint8_t byte = (tmp >> (i << 3)) & 0xFF;

                            TRACE3(" --> 0x%02x, write to offset %d", byte,
                                offset + i);

                            seq->sequenceList[seq->sequenceListCount].
                                sequenceData[offset + i] = byte;
                            seq->sequenceList[seq->sequenceListCount].
                                sequenceLength++;
                        }

                    }
                    else {

                        TRACE3(" --> 0x%02x (numValidBytes=%d)", tmp,
                            numValidBytes);
                    }
                    idx += numValidBytes;
                };
                if (len != idx - 2) {
                    LOG_ERR("sequence '%s' at line %d corrupt! length %d not "
                        "match count %d of data-bytes!", line,
                        seq->lineNumber, len, idx - 2);
                    result = -8;
                    goto exitParser;
                }
                if (0 == run) {
                    sxpf_init_sequence_data_t* newSeq =
                        seq->sequenceList + seq->sequenceListCount;

                    uint32_t newLength = newSeq->sequenceLength + idx;

                    TRACE3("realloc to size %d", newLength);

                    newSeq->sequenceData =
                        (uint8_t*)realloc(newSeq->sequenceData, newLength);
                }
            }
            break;
        }

        default:
            LOG_ERR("parse error at line %d", seq->lineNumber);
            result = -9;
            goto exitParser;
        } // end: switch

    } while (NULL != line);

    // finish last sequence
    result = prepareNextSequence(seq);

exitParser: return result;
}

int sxpf_init_sequence_parser_close(sxpf_init_sequence_t* seq)
{
    uint32_t i;

    //printf("sxpf_init_sequence_parser_close(..)\n");

    for (i = 0; i < seq->sequenceListCount; i++) {
        if (seq->sequenceList[i].sequenceData) {
            free(seq->sequenceList[i].sequenceData);
            seq->sequenceList[i].sequenceData = NULL;
        }
    }

    if (seq->buffer) {
        free(seq->buffer);
        seq->buffer = NULL;
    }

    return 0;
}

void sxpf_init_sequence_parser_dump(sxpf_init_sequence_t* seq)
{
    uint32_t i, z;
    printf("+--------------------------------------------------------------\n");
    printf("| INIT SEQUENCE: '%s'\n", seq->filename);
    printf("+--[ aliases: count=%d ]---------------------------------------\n",
        seq->aliasesCount);
    for (i = 0; i < seq->aliasesCount; i++) {
        printf("| 0x%02x := '%s'\n", seq->aliases[i].id, seq->aliases[i].name);
    }

    printf("+--[ event-list: count=%d ]------------------------------------\n",
        seq->eventListCount);
    for (i = 0; i < seq->eventListCount; i++) {
        printf("| 0x%02x := '%s'\n", seq->eventList[i].id,
            seq->eventList[i].eventname);
    }

    printf("+--[ sequence-list: count=%d ]--------------------------------\n",
        seq->sequenceListCount);
    for (i = 0; i < seq->sequenceListCount; i++) {
        printf("| name   = '%s'\n", seq->sequenceList[i].eventname);
        printf("| length = %d\n", seq->sequenceList[i].sequenceLength);
        printf("| data   = ");
        if (seq->sequenceList[i].sequenceData) {
            for (z = 0; z < seq->sequenceList[i].sequenceLength; z++) {
                printf("0x%02x ", seq->sequenceList[i].sequenceData[z]);
                if (z % 8 == 7) {
                    printf("\n|          ");
                }
            }
        }
        else {
            printf("<null>");
        }
        printf("\n");
    }
    printf("+--[ done ]----------------------------------------------------\n");

}

/**
 * This routine reads the entire file into memory.
 */
uint8_t* sxpf_init_sequence_parser_readFileInBuffer(const char* filename,
    uint32_t* fileSizeInBytes)
{
    off_t rd;
    struct stat sb;
    uint8_t* buffer;

    // open file for read
    FILE* file = fopen(filename, "rb");
    if (!file) {
        //LOG_ERR("Can't open file '%s' for reading (%s)!", filename,
        //        strerror(errno));
        return NULL;
    }

    // get filesize
    if (stat(filename, &sb) != 0) {
        LOG_ERR("Can't determine size for file '%s' (%s)!", filename,
            strerror(errno));
        return NULL;
    }
    *fileSizeInBytes = sb.st_size;

    // protect buffer for overflow
    if (sb.st_size > MAX_BUFFER_SIZE) {
        LOG_ERR("Size of file '%s' exceeded the buffer-size of %d " "bytes. "
            "(file-size=%d)!", filename, MAX_BUFFER_SIZE, (int)sb.st_size);
        return NULL;
    }

    buffer = (uint8_t*)malloc(sb.st_size + 1);
    if (!buffer) {
        LOG_ERR("Buffer allocation of %d bytes failed!", (int)(sb.st_size + 1));
        return NULL;
    }

    // read content into buffer
    rd = (off_t)fread(buffer, sizeof(uint8_t), sb.st_size, file);
    if (rd != sb.st_size) {
        LOG_ERR("Read content of file '%s' failed! Expected %d bytes, got %d "
            "bytes.", filename, (int)sb.st_size, (int)rd);
        free(buffer);
        return NULL;
    }

    // close file
    int status = fclose(file);
    if (status != 0) {
        LOG_ERR("Closing of file '%s' failed (%s)!", filename, strerror(errno));
        free(buffer);
        return NULL;
    }

    // mark end of content with '\0'
    buffer[sb.st_size] = '\0';

    // all done
    printf("%d bytes read\n", (int)sb.st_size);

    // all done
    return buffer;
}

int sxpf_init_sequence_parser_writeBufferToFile(const char* filename,
    uint8_t* buffer,
    uint32_t bufferSize)
{
    size_t wr;
    FILE* file;

    // open file for write
    file = fopen(filename, "wb");
    if (!file) {
        fprintf(stderr, "Can't open file '%s' for writing (%s)!\n", filename,
            strerror(errno));
        return -1;
    }

    // write buffer-content to file
    wr = fwrite(buffer, sizeof(uint8_t), bufferSize, file);
    if (wr != bufferSize) {
        fprintf(stderr, "Write content to file '%s' failed! Expected %d bytes,"
            " wrote %d bytes.\n", filename, (int)bufferSize, (int)wr);
        return -2;
    }

    // close file
    int status = fclose(file);
    if (status != 0) {
        fprintf(stderr, "Closing of file '%s' failed (%s)!\n", filename,
            strerror(errno));
        return -3;
    }

    return 0;
}

static int isCollisionAt(sxpf_init_sequence_t* seq, uint32_t offset,
    uint32_t size, uint32_t me)
{
    uint32_t i, me_begin = offset, me_end = offset + size;

    // mark all used sequences
    TRACE7("  isCollisionAt: me_begin=%d, me_end=%d (exclusive)", me_begin,
        me_end);
    for (i = 0; i < seq->sequenceListCount; i++) {
        if (i == me) { TRACE6("    --> that's me"); continue; } // that's me
        if (seq->sequenceList[i].used && seq->sequenceList[i].sequenceOffset) {
            uint32_t other_begin, other_end;
            other_begin = seq->sequenceList[i].sequenceOffset;
            other_end =
                seq->sequenceList[i].sequenceOffset +
                seq->sequenceList[i].sequenceLength + sizeof(END_OF_SEQ);

            TRACE7("    --> other_begin=%d other_end=%d",
                other_begin, other_end);

            if (me_begin >= other_begin && me_begin < other_end) {
                TRACE7("    --> RUMS1!"); return i;
            }
            if (me_end - 1 >= other_begin && me_end - 1 < other_end) {
                TRACE7("    --> RUMS2!"); return i;
            }
            if (other_begin >= me_begin && other_begin < me_end) {
                TRACE7("    --> RUMS3!"); return i;
            }
            if (other_end - 1 >= me_begin && other_end - 1 < me_end) {
                TRACE7("    --> RUMS4!"); return i;
            }
        }
    }

    TRACE7("    --> NO_COLLISION");
    return -1;
}

uint8_t* sxpf_init_sequence_parser_createUpdateBinary2(sxpf_init_sequence_t* seq,
    uint32_t* sizeInBytes,
    uint32_t sizeOfEEPROM)
{
    uint32_t i, z, sizeInBytesIntern = 0, start, crc;
    uint32_t* data32 = NULL;
    uint8_t* data8 = NULL;
    uint32_t names_length = 0;
    uint32_t names_offset = 0;
    uint8_t* names_data = NULL;

    TRACE6("createUpdateBinary2");

    // Header (MAGIC/VERSION/SIZE/CRC32)
    sizeInBytesIntern += 4 * sizeof(uint32_t);

    // Event-List (for each entry + names + zero-termination)
    sizeInBytesIntern += (1 + 1 + seq->eventListCount) * sizeof(uint32_t);

    // remember start of sequences
    start = sizeInBytesIntern;

    TRACE6("start: %d", start);

    // - mark all used sequences
    // - check wanted start-offset is behind event-list
    // - update max sizeInBytes
    for (i = 0; i < seq->sequenceListCount; i++) {
        sxpf_init_sequence_data_t* ent = seq->sequenceList + i;

        if (!ent->eventname[0]) {
            LOG_ERR("eventname of sequence %d not exists", i);
            return NULL;
        }
        ent->used = 0;
        for (z = 0; z < seq->eventListCount; z++) {
            if (!seq->eventList[z].eventname[0]) {
                LOG_ERR("eventname of event %d not exists", z);
                return NULL;
            }
            if (strcmp(seq->eventList[z].eventname, ent->eventname))
                continue;

            TRACE6("mark sequence '%s' used", ent->eventname);
            if (ent->atOffset) {
                TRACE6(" --> fixed address to %d (0x%x)",
                    (int)ent->atOffset, (int)ent->atOffset);
                if (ent->atOffset < start) {
                    LOG_ERR("Offset %d for sequence '%s' collides with header "
                        "or event-list (hint: increase the offset)!",
                        (int)ent->atOffset, ent->eventname);
                    return NULL;
                }
                if (ent->atOffset + ent->sequenceLength > sizeInBytesIntern) {
                    sizeInBytesIntern =
                        (uint32_t)(ent->atOffset + ent->sequenceLength);
                    TRACE6(" --> update sizeInBytesIntern to %d",
                        sizeInBytesIntern);
                }
            }
            ent->sequenceOffset = (uint32_t)ent->atOffset;
            seq->eventList[z].offset = ent->sequenceOffset;
            ent->used = 1;
            TRACE6(" --> initialize offset to %d (0x%x)",
                seq->eventList[z].offset, seq->eventList[z].offset);
        }
    }

    // check all fixed sequences against overlapping
    for (i = 0; i < seq->sequenceListCount; i++) {
        uint32_t offset, length;
        int result;
        TRACE6("overlapping-check for '%s'", seq->sequenceList[i].eventname);
        if (!seq->sequenceList[i].used) {
            TRACE6("  --> not used");
            continue;
        }
        if (!seq->sequenceList[i].sequenceOffset) {
            TRACE6("  --> not fixed");
            continue;
        }
        offset = seq->sequenceList[i].sequenceOffset;
        length = seq->sequenceList[i].sequenceLength + sizeof(END_OF_SEQ);
        result = isCollisionAt(seq, offset, length, i);
        if (result >= 0) {
            LOG_ERR("Sequence '%s' overlaps with '%s'!",
                seq->sequenceList[i].eventname,
                seq->sequenceList[result].eventname);
            return NULL;
        }
    }

    // search position for all sequences
    for (i = 0; i < seq->sequenceListCount; i++) {
        uint32_t offset, length, found;
        found = 0;
        length = seq->sequenceList[i].sequenceLength + sizeof(END_OF_SEQ);
        TRACE6("search position for '%s' with length %d",
            seq->sequenceList[i].eventname, length);
        if (!seq->sequenceList[i].used) {
            TRACE6("  --> not used");
            continue;
        }
        if (seq->sequenceList[i].sequenceOffset) {
            TRACE6("  --> already done at offset %d (0x%x)",
                seq->sequenceList[i].sequenceOffset,
                seq->sequenceList[i].sequenceOffset);
            continue;
        }
        for (offset = start; offset < sizeOfEEPROM; offset++) {
            if (isCollisionAt(seq, offset, length, -1) < 0) {
                TRACE6(" --> found position %d (0x%x)", offset, offset);
                seq->sequenceList[i].sequenceOffset = offset;
                if (offset + length > sizeInBytesIntern) {
                    sizeInBytesIntern = offset + length;
                    TRACE6(" --> update sizeInBytesIntern to %d",
                        sizeInBytesIntern);
                }
                found = 1;
                break;
            }
        }
        if (!found) {
            LOG_ERR("Can't find any suitable position for sequence '%s' (hint: "
                "check fixed offsets)!", seq->sequenceList[i].eventname);
            return NULL;
        }
    }

    // copy offsets of all used sequences to event-list
    for (i = 0; i < seq->sequenceListCount; i++) {
        for (z = 0; z < seq->eventListCount; z++) {
            if (0 == strcmp(seq->eventList[z].eventname,
                seq->sequenceList[i].eventname)) {
                TRACE6("copy offset %d (0x%x) of '%s' to event-id %d",
                    seq->sequenceList[i].sequenceOffset,
                    seq->sequenceList[i].sequenceOffset,
                    seq->sequenceList[i].eventname, seq->eventList[z].id);
                seq->eventList[z].offset = seq->sequenceList[i].sequenceOffset;
            }
        }
    }

    // generate binary-blob of all ASCII-names of all used sequences
    names_data = generateNamesBinaryBlob(seq, &names_length);
    if (!names_data) { return NULL; }

    // search position for names
    {
        uint32_t found = 0;
        TRACE6("search position for 'names' with length %d", names_length);
        for (names_offset = start; names_offset < sizeOfEEPROM; names_offset++) {
            if (isCollisionAt(seq, names_offset, names_length, -1) < 0) {
                TRACE6(" --> found position %d (0x%x)", names_offset,
                    names_offset);
                if (names_offset + names_length > sizeInBytesIntern) {
                    sizeInBytesIntern = names_offset + names_length;
                    TRACE6(" --> update sizeInBytesIntern to %d",
                        sizeInBytesIntern);
                }
                found = 1;
                break;
            }
        }
        if (!found) {
            LOG_ERR("Can't find any suitable position for names!");
            return NULL;
        }
    }

    // check Event-List-Offsets resolved (a little paranoid: this should not
    // happen)
    for (z = 0; z < seq->eventListCount; z++) {
        if (!seq->eventList[z].offset) {
            LOG_ERR("sequence-name '%s' can not be resolved!",
                seq->eventList[z].eventname);
            return NULL;
        }
    }

    // round size up to a value multiple of 4
    // --> to be sure crc32 is already working
    sizeInBytesIntern = (sizeInBytesIntern + 3) & ~3;
    TRACE6("size of binary is after round-up: %d", sizeInBytesIntern);

    // allocate buffer
    data8 = (uint8_t*)malloc(sizeInBytesIntern);
    if (!data8) {
        LOG_ERR("out of memory");
        return NULL;
    }

    // clear buffer
    memset(data8, 0xFF, sizeInBytesIntern);

    // write header
    data32 = (uint32_t*)data8;
    *data32++ = BINARY_MAGIC;
    *data32++ = 0x00010000;
    *data32++ = sizeInBytesIntern;
    *data32++ = 0x00000000;   // left empty for CRC32

    // write event-list (with names)
    for (i = 0; i < seq->eventListCount; i++) {
        *data32++ =
            ((uint32_t)seq->eventList[i].id) << 24 | seq->eventList[i].offset;
    }
    *data32++ = ((uint32_t)0xFF << 24) | names_offset;
    *data32++ = 0x0; // 0-termination

    // write setup-sequences
    for (i = 0; i < seq->sequenceListCount; i++) {
        if (seq->sequenceList[i].used) {
            uint8_t* p = data8 + seq->sequenceList[i].sequenceOffset;
            TRACE6("  --> write sequence to offset: %d (0x%x) (size=%d)",
                (int)(p - data8), (int)(p - data8),
                seq->sequenceList[i].sequenceLength);
            memcpy(p, seq->sequenceList[i].sequenceData,
                seq->sequenceList[i].sequenceLength);
            p += seq->sequenceList[i].sequenceLength;
            TRACE6("  --> write end-sequence to offset: %d (0x%x)",
                (int)(p - data8), (int)(p - data8));
            memcpy(p, END_OF_SEQ, sizeof(END_OF_SEQ));
        }
    }

    { // write names
        uint8_t* p = data8 + names_offset;
        TRACE6("  --> write names to offset: %d (0x%x) (size=%d)",
            (int)(p - data8), (int)(p - data8), names_length);
        memcpy(p, names_data, names_length);
    }

    // generate CRC32
    TRACE6("generate crc");
    crc = hal_utils_crc32_crc32Block(data8, sizeInBytesIntern, 0x0);
    TRACE6("  --> crc is 0x%x", crc);
    ((uint32_t*)data8)[3] = crc;

    // all done
    *sizeInBytes = sizeInBytesIntern;
    return data8;

}

uint8_t* sxpf_init_sequence_parser_createUpdateBinary(sxpf_init_sequence_t* seq,
    uint32_t* sizeInBytes)
{
    uint32_t i, z, sizeInBytesIntern = 0, crc;
    uint32_t* data32 = NULL;
    uint8_t* data8 = NULL;
    uint32_t names_length = 0;
    uint32_t names_offset = 0;
    uint8_t* names_data = NULL;

    // Header (MAGIC/VERSION/SIZE/CRC32)
    sizeInBytesIntern += 4 * sizeof(uint32_t);

    // Event-List (for each entry + names + zero-termination)
    sizeInBytesIntern += (1 + 1 + seq->eventListCount) * sizeof(uint32_t);

    // Sequence-List (each sequence list length +
    //   1 Byte for Zero-Termination -> tag=0x00)
    // --> do also update the offsets of event-list to sequence-list
    // --> if sequence list is not used, skip it
    for (i = 0; i < seq->sequenceListCount; i++) {
        if (!seq->sequenceList[i].eventname[0]) {
            LOG_ERR("eventname of sequence %d not exists", i);
            return NULL;
        }
        seq->sequenceList[i].used = 0;
        for (z = 0; z < seq->eventListCount; z++) {
            if (!seq->eventList[z].eventname[0]) {
                LOG_ERR("eventname of event %d not exists", z);
                return NULL;
            }
            if (0 == strcmp(seq->eventList[z].eventname,
                seq->sequenceList[i].eventname)) {
                seq->eventList[z].offset = sizeInBytesIntern;
                seq->sequenceList[i].sequenceOffset = sizeInBytesIntern;
                seq->sequenceList[i].used = 1;
            }
        }
        if (seq->sequenceList[i].used) {
            sizeInBytesIntern += seq->sequenceList[i].sequenceLength + 1;
        }
    }

    // generate binary-blob of all ASCII-names of all used sequences
    names_data = generateNamesBinaryBlob(seq, &names_length);
    if (!names_data) { return NULL; }

    // get position for names
    {
        TRACE6("get position for 'names' with length %d", names_length);
        names_offset = sizeInBytesIntern;
        sizeInBytesIntern += names_length;
    }

    // check Event-List-Offsets resolved
    for (z = 0; z < seq->eventListCount; z++) {
        if (!seq->eventList[z].offset) {
            LOG_ERR("sequence-name '%s' can not be resolved!",
                seq->eventList[z].eventname);
            return NULL;
        }
    }

    // round size up to a value multiple of 4
    // --> to be sure crc32 is already working
    sizeInBytesIntern = (sizeInBytesIntern + 3) & ~3;

    // allocate buffer
    data8 = (uint8_t*)malloc(sizeInBytesIntern);
    if (!data8) {
        LOG_ERR("out of memory");
        return NULL;
    }

    // clear buffer
    memset(data8, 0x0, sizeInBytesIntern);

    // write header
    data32 = (uint32_t*)data8;
    *data32++ = BINARY_MAGIC;
    *data32++ = 0x00010000;
    *data32++ = sizeInBytesIntern;
    *data32++ = 0x00000000;   // left empty for CRC32

    // write event-list
    for (i = 0; i < seq->eventListCount; i++) {
        *data32++ = ((uint32_t)seq->eventList[i].id) << 24
            | seq->eventList[i].offset;
    }
    *data32++ = 0x0; // 0-termination

    { // write setup-sequences
        uint8_t* p = (uint8_t*)data32;
        for (i = 0; i < seq->sequenceListCount; i++) {
            if (seq->sequenceList[i].used) {
                memcpy(p, seq->sequenceList[i].sequenceData,
                    seq->sequenceList[i].sequenceLength);
                p += seq->sequenceList[i].sequenceLength;
                memcpy(p, END_OF_SEQ, sizeof(END_OF_SEQ));
                p += sizeof(END_OF_SEQ);
            }
        }
    }

    { // write names
        uint8_t* p = data8 + names_offset;
        TRACE6("  --> write names to offset: %d (0x%x) (size=%d)",
            (int)(p - data8), (int)(p - data8), names_length);
        memcpy(p, names_data, names_length);
    }

    // generate CRC32
    crc = hal_utils_crc32_crc32Block(data8, sizeInBytesIntern, 0x0);
    ((uint32_t*)data8)[3] = crc;

    // all done
    *sizeInBytes = sizeInBytesIntern;
    return data8;
}

int decode_init_sequence_internal(uint8_t* data, uint32_t data_size,
    char* out_buf, uint32_t buf_size, int* plasma_valid)
{
#define WRB(buf, left, fmt, ...) \
    { \
        if (out_buf != NULL) { \
            char tmp_buf[1024]; \
            int w = sprintf(tmp_buf, fmt, ##__VA_ARGS__); \
            if (w > left) { \
                LOG_ERR("Out Of Buffer"); \
                return -2; \
            } \
            memcpy(buf, tmp_buf, w); \
            buf += w; \
            left -= w; \
        } \
    } \

    uint32_t* data32 = (uint32_t*)data;
    uint8_t* p;
    uint8_t* pEndOfData;
    uint32_t size, crc, calcCrc, tmp;
    uint32_t names_offset = 0;
    uint32_t names_len = 0;
    int left = buf_size;

    sequence_name_t list[MAX_SEQUENCELIST_COUNT];
    uint32_t listCnt = 0;

    int plasma_ok = 1;

    // check the least number of bytes must exists
    if (data_size < (16 + 4)) { // 16=Header, 4=Null-Event
        LOG_ERR("Binary size of %d bytes can not be a valid init sequence!",
            data_size);
        return -1;
    }

    WRB(out_buf, left, "#! sxproframe-init-sequence, version=1.0\n");

    if (0xC0D1F1ED != *data32++ || 0x00010000 != *data32++) {
        LOG_ERR("raw data is not in init sequence format!");
        return -3;
    }

    // skip CRC32 and SIZE information
    size = *data32++;
    if (size > data_size) {
        LOG_ERR("size of header (%d) not match binary-blob size (%d)!", size,
            data_size);
        return -4;
    }
    pEndOfData = data + size;

    TRACE8("size of binary is %u (given size is %u)", size, data_size);

    crc = *data32;
    *data32 = 0x0;
    calcCrc = hal_utils_crc32_crc32Block(data, size, 0x0);
    *data32++ = crc;
    if (crc != calcCrc) {
        LOG_ERR("CRC mismatch (read=0x%08x, expected=0x%08x)!", crc, calcCrc);
        return -5;
    }

    WRB(out_buf, left, "[aliases]\n");
    WRB(out_buf, left, "# no informations in binary\n");
    WRB(out_buf, left, "\n");
    WRB(out_buf, left, "[event-list]\n");
    {
        // stage1: parse sequence names
        names_offset = parseSequenceNames(data32, data, pEndOfData, &names_len,
            list, &listCnt);

        // stage2: dump all sequence entries (without names)
        do {
            uint32_t id, offset;
            tmp = *data32++;
            if (tmp) {
                id = tmp >> 24;
                offset = tmp & 0xFFFFFF;
                if (0xFF != id) {
                    WRB(out_buf, left, "0x%x = %s\n", id,
                        getSequenceName(offset, list, listCnt));
                    TRACE8("0x%x = %s", id, getSequenceName(offset));
                }
            }
        } while (tmp && (uint8_t*)data32 < pEndOfData);
    }

    p = (uint8_t*)data32;

    // fix: '>=' not valid for empty init sequences
    if (p > pEndOfData) corruptData: {
        LOG_ERR("end of data, corrupt data at offset %d",
            (uint32_t)(p - data));
        return -6;
    }

    if (p < pEndOfData) {

        uint32_t isFixedOffset = 0;

        do {
            uint8_t tag = *p, length;
            uint32_t offset = (uint32_t)(p - data);

            // work-around for binaries with fixed offsets (there can be empty
            // 0xFF areas)
            if (0xFF == tag) { p++; isFixedOffset = 1; continue; }

            // otherwise parse sequence
            if (tag) {
                const char* name;
                // ignore section with names
                if (offset == names_offset) {
                    p += names_len;
                    continue;
                }
                if (isFixedOffset) {
                    name = getSequenceNameWithOffset(offset, list, listCnt);
                }
                else {
                    name = getSequenceName(offset, list, listCnt);
                }
                WRB(out_buf, left, "\n%s\n", name);
                TRACE8("%s", getSequenceName(offset, list, listCnt));
                isFixedOffset = 0;
            }

            do {
                tag = *p++;
                if (tag) {
                    if (p >= pEndOfData) { goto corruptData; }
                    length = *p++;
                    if (CMD_ESCAPE == tag) {
                        uint8_t sub;
                        if (p >= pEndOfData || length < 1) { goto corruptData; }
                        sub = *p++;
                        length--;
                        switch (sub) {
                        case SUB_JUMP:
                            WRB(out_buf, left, "JUMP ");
                            while (length--) {
                                if (p >= pEndOfData) { goto corruptData; }
                                WRB(out_buf, left, "%c", *p);
                                p++;
                            }
                            WRB(out_buf, left, "\n");
                            break;
                        case SUB_RETURN:
                            WRB(out_buf, left, "RETURN\n");
                            break;
                        default:
                            WRB(out_buf, left, "0x%02x %d 0x%02x ", tag, length,
                                sub);
                            if (tag == 0x0f && (sub == 0x02 || // CMD_MASKED_I2C_WRITE
                                                sub == 0x03))  // CMD_CSI2_LANE_MAP
                            {
                                plasma_ok = 0;
                            }
                            while (length--) {
                                if (p >= pEndOfData) { goto corruptData; }
                                WRB(out_buf, left, "0x%02x ", *p);
                                p++;
                            }
                            WRB(out_buf, left, "\n");
                            break;
                        }
                    }
                    else {
                        WRB(out_buf, left, "0x%02x %d ", tag, length);
                        while (length--) {
                            if (p >= pEndOfData) { goto corruptData; }
                            WRB(out_buf, left, "0x%02x ", *p);
                            p++;
                        }
                        WRB(out_buf, left, "\n");
                    }
                }
            } while (tag && p < pEndOfData);

        } while (p < pEndOfData);
    }

    // end of file mark
    if (out_buf != NULL) {
        *out_buf++ = 0x0;
    }

    if (plasma_valid != NULL)
        *plasma_valid = plasma_ok;

    // successful wrote back
    return buf_size - left;
}

int sxpf_decode_init_sequence(uint8_t* data, uint32_t data_size,
                              char* out_buf, uint32_t buf_size)
{
    return decode_init_sequence_internal(data, data_size, out_buf, buf_size, NULL);
}


int sxpf_analyze_init_sequence(uint8_t* data, uint32_t data_size,
                               sxpf_ini_props_t* ini_props)
{
    int plasma_valid;
    int ret;

    ret = decode_init_sequence_internal(data, data_size, NULL, 0, &plasma_valid);

    if (ret >= 0) {
        ini_props->eeprom_compatible = plasma_valid;
        ini_props->spi_compatible = plasma_valid;
    }

    return ret;
}
