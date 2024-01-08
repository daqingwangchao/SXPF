#ifdef _MSC_VER

#pragma warning(disable: 4103)
#pragma pack(push, 1)

#define PACKED      /* nothing */

#else

#define PACKED      __attribute__((__packed__))

#endif
