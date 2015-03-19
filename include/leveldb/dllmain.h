#ifdef _WIN32

#ifdef LEVELDB_EXPORTS
#define LEVELDB_API __declspec(dllexport)
#else
#define LEVELDB_API __declspec(dllimport)
#endif

#pragma warning(disable:4251)

#else

#define LEVELDB_API

#endif
